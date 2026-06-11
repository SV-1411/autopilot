extends RefCounted
class_name SpaceTimeAStar

# Time-indexed weighted A* global planner.
#
# WHY THIS EXISTS
#   The original VoxelAStar searched one 3D occupancy grid with every future
#   asteroid position unioned in. That collapses time: a cell a rock will pass
#   through in 2 s is treated as a wall right now, so dense belts look solid and
#   the planner refuses (false NO_PATH) before the ship even launches.
#
#   This planner checks occupancy AT THE TIME THE SHIP WOULD ARRIVE in each
#   cell. The arrival time of a node is derived from its path cost: one axis
#   move = one tick = the time to cross a voxel at cruise speed (move_time), so
#   a node with cost g is reached at t = g * move_time, which maps to predicted
#   occupancy layer floor(t / layer_dt) (clamped to the last layer). A corridor
#   is accepted if every cell is free WHEN THE SHIP GETS THERE -- threading gaps
#   that close later and crossing cells that clear before arrival.
#
# DESIGN CHOICES (and their consequences)
#   - States are CELLS, not (cell, time) pairs: closing on cells keeps the
#     search the size of plain 3D A* instead of multiplying it by the number of
#     time layers (which exhausted any reasonable expansion budget on wall-like
#     scenes). The cost: the ship cannot plan to WAIT in place for a gap to
#     open -- it only exploits gaps that are open on arrival. In practice rocks
#     keep moving, replanning runs every second, and the local planner handles
#     the rest; a genuinely wait-required corridor is reported as no-path and
#     the previous path is kept.
#   - Weighted A* (w = 7/5 = 1.4): Manhattan A* in open 3D explores a huge
#     equal-cost frontier; inflating the heuristic drives straight for the goal
#     for a near-optimal corridor at a fraction of the expansions (cost bound:
#     w * optimal). The local planner refines the corridor anyway.
#   - All-integer hot path: the heap is a PackedInt64Array of packed
#     (f, g, cell) keys -- f = 5g + 7h is exactly integer weighted cost x5.
#     closed/g-best/parent are flat packed arrays indexed by cell id. No
#     Dictionaries, no Callables, no Vector3i allocation per neighbor.
#   - Deadline: the search aborts (returns []) past deadline_usec so a replan
#     can never stall the control loop; the caller keeps the previous path.
#
# layers: Array[PackedByteArray] -- layers[i][cell_id] != 0 means blocked during
#         time slice i (built by SimWorld._build_time_layers).
# Returns cells start..goal, or [] if unreachable within budget.

const G_CAP := 4095               # max path cost in ticks (fits 20-bit pack)

static func plan(
	start: Vector3i,
	goal: Vector3i,
	layers: Array,
	dims: Vector3i,
	move_time: float,
	layer_dt: float,
	max_expansions: int = 200000,
	deadline_usec: int = 50000
) -> Array[Vector3i]:
	if start == goal:
		return [start]
	var n_layers := layers.size()
	if n_layers == 0:
		return []

	var nx := dims.x
	var ny := dims.y
	var nz := dims.z
	var yz := ny * nz
	var n_cells := nx * yz

	var start_id := start.x * yz + start.y * nz + start.z
	var goal_id := goal.x * yz + goal.y * nz + goal.z
	var gx := goal.x
	var gy := goal.y
	var gz := goal.z

	# Arrival layer for each path cost, precomputed.
	var layer_of := PackedInt32Array()
	layer_of.resize(G_CAP + 1)
	var inv := move_time / maxf(1e-9, layer_dt)
	for g in range(G_CAP + 1):
		layer_of[g] = clampi(int(float(g) * inv), 0, n_layers - 1)

	var closed := PackedByteArray()
	closed.resize(n_cells)
	var g_best := PackedInt32Array()
	g_best.resize(n_cells)
	g_best.fill(0x7FFFFFFF)
	var parent := PackedInt32Array()
	parent.resize(n_cells)
	parent.fill(-1)

	# Min-heap of int64: (f5 << 40) | (g << 20) | cell. f5 = 5g + 7h orders by
	# weighted cost (x5 to stay integer); low bits break ties deterministically.
	var heap := PackedInt64Array()
	var h0 := absi(start.x - gx) + absi(start.y - gy) + absi(start.z - gz)
	heap.push_back(((7 * h0) << 40) | start_id)
	g_best[start_id] = 0

	var t_start := Time.get_ticks_usec()
	var expansions := 0

	while heap.size() > 0:
		# -- pop min --
		var item := heap[0]
		var hs := heap.size() - 1
		var last := heap[hs]
		heap.resize(hs)
		if hs > 0:
			heap[0] = last
			var i := 0
			while true:
				var l := i * 2 + 1
				if l >= hs:
					break
				var sm := l
				var r := l + 1
				if r < hs and heap[r] < heap[l]:
					sm = r
				if heap[i] <= heap[sm]:
					break
				var tmp := heap[i]
				heap[i] = heap[sm]
				heap[sm] = tmp
				i = sm
		var cell := int(item & 0xFFFFF)
		var g := int((item >> 20) & 0xFFFFF)

		if closed[cell] != 0:
			continue
		closed[cell] = 1

		if cell == goal_id:
			return _reconstruct(parent, cell, nz, yz)

		expansions += 1
		if expansions >= max_expansions:
			return []
		if (expansions & 255) == 0 and Time.get_ticks_usec() - t_start > deadline_usec:
			return []

		var ng := g + 1
		if ng > G_CAP:
			continue
		var grid: PackedByteArray = layers[layer_of[ng]]

		var x := cell / yz
		var rem := cell % yz
		var y := rem / nz
		var z := rem % nz

		# 6 axis neighbors, inline (no arrays, no Vector3i allocation).
		for d in range(6):
			var ncell := -1
			if d == 0:
				if x + 1 < nx: ncell = cell + yz
			elif d == 1:
				if x > 0: ncell = cell - yz
			elif d == 2:
				if y + 1 < ny: ncell = cell + nz
			elif d == 3:
				if y > 0: ncell = cell - nz
			elif d == 4:
				if z + 1 < nz: ncell = cell + 1
			else:
				if z > 0: ncell = cell - 1
			if ncell < 0:
				continue
			if closed[ncell] != 0:
				continue
			if grid[ncell] != 0:
				continue
			if ng >= g_best[ncell]:
				continue
			g_best[ncell] = ng
			parent[ncell] = cell
			var hx := ncell / yz
			var hrem := ncell % yz
			var h := absi(hx - gx) + absi(hrem / nz - gy) + absi(hrem % nz - gz)
			heap.push_back(((5 * ng + 7 * h) << 40) | (ng << 20) | ncell)
			# -- sift up --
			var ci := heap.size() - 1
			while ci > 0:
				var p := (ci - 1) >> 1
				if heap[p] <= heap[ci]:
					break
				var tmp2 := heap[p]
				heap[p] = heap[ci]
				heap[ci] = tmp2
				ci = p

	return []

static func _reconstruct(parent: PackedInt32Array, goal_cell: int, nz: int, yz: int) -> Array[Vector3i]:
	var cells: Array[Vector3i] = []
	var cur := goal_cell
	while cur >= 0:
		var rem := cur % yz
		cells.push_back(Vector3i(cur / yz, rem / nz, rem % nz))
		cur = parent[cur]
	cells.reverse()
	return cells
