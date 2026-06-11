extends RefCounted
class_name SimWorld

# Headless simulation engine -- the single source of truth for the autopilot
# closed loop. It owns the ship + asteroid state and advances them with the
# SAME planners used everywhere (VoxelAStar, LocalPlanner3D, Predictor), but has
# NO dependency on the scene tree, nodes, rendering, or input.
#
# This is what makes the sim testable and benchmarkable:
#   - Main.gd drives one SimWorld and just renders its state (interactive).
#   - BatchEval.gd spins up thousands of SimWorlds headless to score the
#     autopilot over random scenarios.
#   - Scenarios save/load through to_scenario() / load_scenario().

# ---------------------------------------------------------------- world config
var bounds_min := Vector3(-90, 0, -90)
var bounds_max := Vector3(90, 60, 90)
var voxel_size := 3.0
var replan_interval := 1.0
var max_time := 60.0          # run is declared TIMEOUT after this many seconds

# Space-time planner horizon: occupancy is predicted as `plan_time_layers`
# discrete layers spaced `plan_layer_dt` seconds apart. A search tick maps to the
# layer at real_time = tick * (voxel_size / cruise_speed), clamped to the last
# layer. Together these span plan_time_layers * plan_layer_dt seconds of look-ahead.
var plan_time_layers := 16
var plan_layer_dt := 0.2
var plan_deadline_usec := 50000   # hard per-replan budget; abort keeps old path
# A/B canary: route global planning through the legacy time-collapsed union
# model. Exists so the benchmark matrix can PROVE the time-indexed planner's
# advantage every run (the union must reproduce the old false-NO_PATH rate).
var use_union_planner := false

# ------------------------------------------------------------ plan telemetry
# A planner that silently stalls or fails is a trust hole; these are reported
# by BatchEval and the HUD.
var plan_count := 0
var plan_fail_count := 0
var plan_ms_last := 0.0
var plan_ms_max := 0.0
var plan_ms_total := 0.0
# True while flying on local-planner-only guidance (replan failed and no part
# of the previous path survived validation). Cleared by the next good plan.
var degraded := false

# ---------------------------------------------------------------- ship config
var ship_radius := 1.5
var ship_mass := 8000.0
var ship_max_thrust := 250000.0
var ship_max_speed := 200.0
var ship_target_speed := 120.0
var planner_cfg := {}

# ---------------------------------------------------------------- uncertainty
# Process noise: a small random acceleration applied to every asteroid each step,
# so their true motion drifts away from the constant-velocity prediction. This is
# the stand-in for real-world tracking/process uncertainty. 0 = perfect, classic.
var noise_sigma := 0.0          # asteroid accel-noise std (m/s^2); 0 = deterministic
var seed_value := 0             # seeds the process-noise RNG (for reproducibility)

# Chance-constrained planning: when enabled, both the global and local planners
# inflate obstacles by unc_ksigma * (unc_sigma0 + unc_growth * t) to keep the ship
# outside the k-sigma uncertainty shell -- bounding collision probability.
var unc_enable := false
var unc_sigma0 := 0.0
var unc_growth := 0.0
var unc_ksigma := 3.0

var _rng: RandomNumberGenerator = null

const WAYPOINT_TOL := 3.0
const LOOKAHEAD_M := 25.0

# ---------------------------------------------------------------- scenario data
var start_pos := Vector3(-60, 10, -60)
var goal_pos := Vector3(60, 10, 60)
# asteroids: each is { "pos": Vector3, "vel": Vector3, "radius": float, "mass": float }
var asteroids: Array[Dictionary] = []

# ---------------------------------------------------------------- runtime state
var ship_pos := Vector3.ZERO
var ship_vel := Vector3.ZERO
var path: Array[Vector3] = []
var path_index := 0

var time := 0.0
var collisions := 0
var min_clearance := INF
var dv_used := 0.0
var replans := 0
var arrived := false
# EDIT / FLYING / ARRIVED / COLLISION / TIMEOUT / NO_PATH
var status := "EDIT"

var _replan_timer := 0.0
var _contact := {}

# ============================================================ derived limits
# ---------------------------------------------------------------- recording
# Flight recorder (default OFF -> zero behavior/perf change). When on, the run
# is captured as: scenario (schema v2) + timed frames (ship + every asteroid
# position) + a corridor snapshot at every replan + the final result. The JSON
# feeds tools/flight_viewer.html (Three.js) for scrub-through-time replay.
var record := false
var rec_dt := 0.1               # seconds between captured frames
var recording := {}
var _rec_timer := 0.0

# Commanded acceleration this tick (telemetry: lets the view draw the thrust
# vector -- where the autopilot is pushing and how hard).
var last_accel := Vector3.ZERO

func a_max() -> float:
	return ship_max_thrust / maxf(1.0, ship_mass)

func cruise_speed() -> float:
	return minf(ship_target_speed, ship_max_speed)

func is_terminal() -> bool:
	return status in ["ARRIVED", "COLLISION", "TIMEOUT", "NO_PATH"]

# ============================================================ run lifecycle
func reset_run() -> void:
	ship_pos = start_pos
	ship_vel = Vector3.ZERO
	path = []
	path_index = 0
	time = 0.0
	collisions = 0
	min_clearance = INF
	dv_used = 0.0
	replans = 0
	arrived = false
	_replan_timer = 0.0
	_contact = {}
	degraded = false
	_rng = RandomNumberGenerator.new()
	_rng.seed = seed_value
	plan_path()
	status = "NO_PATH" if path.is_empty() else "FLYING"
	if record:
		_rec_init()

func step(dt: float) -> void:
	if status != "FLYING":
		return

	time += dt
	var did_replan := false

	# Capture pre-step positions so collision/clearance can be measured over the
	# swept motion of this step, not just its endpoints (no tunneling at speed).
	var ship_prev := ship_pos
	var ast_prev: Array = []
	for a in asteroids:
		ast_prev.append(a["pos"])

	for a in asteroids:
		_integrate_asteroid(a, dt)

	_replan_timer += dt
	if _replan_timer >= replan_interval:
		plan_path()
		_replan_timer = 0.0
		replans += 1
		did_replan = true

	_step_ship(dt)
	_update_metrics(ship_prev, ast_prev)

	if status == "FLYING" and time >= max_time:
		status = "TIMEOUT"

	if record:
		if did_replan:
			_rec_path()
		_rec_timer += dt
		if _rec_timer >= rec_dt or is_terminal():
			_rec_frame()
			_rec_timer = 0.0

# ============================================================ asteroid motion
func _integrate_asteroid(a: Dictionary, dt: float) -> void:
	var v: Vector3 = a["vel"]
	# Process noise: random acceleration so true motion drifts from the constant-
	# velocity prediction the planners assume -- this is what makes uncertainty
	# real. The velocity kick scales with sqrt(dt) (white-noise acceleration), so
	# the accumulated drift is step-size independent: the same noise_sigma means
	# the same physical uncertainty at 30, 60, or 240 Hz.
	if noise_sigma > 0.0 and _rng != null:
		var kick := noise_sigma * sqrt(dt)
		v += Vector3(_rng.randfn(0.0, 1.0), _rng.randfn(0.0, 1.0), _rng.randfn(0.0, 1.0)) * kick
	# Position+reflection delegated to Predictor.advance: simulated truth and the
	# planners' predictions share one reflection law, by construction.
	var adv := Predictor.advance(a["pos"], v, dt, bounds_min, bounds_max)
	a["pos"] = adv["pos"]
	a["vel"] = adv["vel"]

# ============================================================ global planner
func plan_path() -> void:
	ship_pos = clamp_to_bounds(ship_pos)
	goal_pos = clamp_to_bounds(goal_pos)

	var start_cell := world_to_cell(ship_pos)
	var goal_cell := world_to_cell(goal_pos)

	# Time-aware occupancy: one predicted-occupancy layer per plan_layer_dt seconds.
	var t0 := Time.get_ticks_usec()
	var layers := _build_time_layers()
	var dims := Vector3i(cells_x(), cells_y(), cells_z())
	var move_time := voxel_size / maxf(1.0, cruise_speed())   # real seconds per voxel

	var cell_path: Array[Vector3i]
	if use_union_planner:
		cell_path = _plan_static_fallback(start_cell, goal_cell, layers, dims)
	else:
		cell_path = SpaceTimeAStar.plan(
			start_cell, goal_cell, layers, dims, move_time, plan_layer_dt,
			200000, plan_deadline_usec)

	# Telemetry: a planner that silently stalls or fails is a trust hole.
	var ms := float(Time.get_ticks_usec() - t0) / 1000.0
	plan_count += 1
	plan_ms_last = ms
	plan_ms_max = maxf(plan_ms_max, ms)
	plan_ms_total += ms

	if cell_path.is_empty():
		plan_fail_count += 1
		# Keeping a stale path silently is a trust hole: the rocks have moved
		# since it was planned. Validate what's left of it against the fresh
		# prediction; truncate at the first waypoint that is now blocked at the
		# ship's ETA, and fall back to degraded local-only guidance if nothing
		# survives.
		_guard_stale_path(layers, dims)
		return

	degraded = false
	var world_path: Array[Vector3] = []
	for c in cell_path:
		world_path.append(cell_to_world(c))
	# The grid path ends at the goal CELL CENTER -- up to half a voxel diagonal
	# (~2.6 m) from the true goal. Append the exact goal so "arrived" means the
	# goal, not its neighborhood.
	if world_path[world_path.size() - 1].distance_to(goal_pos) > 0.01:
		world_path.append(goal_pos)
	path = world_path
	path_index = 0

# On replan failure: keep only the prefix of the old path that is still
# predicted clear at the ship's estimated arrival time per waypoint. If nothing
# survives, aim straight for the goal and let the local planner do the flying
# (degraded mode, surfaced via telemetry).
func _guard_stale_path(layers: Array, dims: Vector3i) -> void:
	if path.is_empty() or path_index >= path.size():
		return
	var yz := dims.y * dims.z
	var nz := dims.z
	var cruise := maxf(1.0, cruise_speed())
	var n_layers := layers.size()
	var eta := 0.0
	var prev := ship_pos
	var valid_until := path_index
	for i in range(path_index, path.size()):
		eta += prev.distance_to(path[i]) / cruise
		prev = path[i]
		var li := clampi(int(eta / plan_layer_dt), 0, n_layers - 1)
		var c := world_to_cell(path[i])
		if (layers[li] as PackedByteArray)[c.x * yz + c.y * nz + c.z] != 0:
			break
		valid_until = i + 1
	if valid_until > path_index:
		path.resize(valid_until)   # in place: keeps the typed array
	else:
		path.clear()
		path.append(goal_pos)
		path_index = 0
		degraded = true

# Build plan_time_layers predicted-occupancy grids (flat PackedByteArray, one
# byte per cell), one every plan_layer_dt seconds. Layer i marks the cells any
# asteroid is predicted to occupy at t = i * plan_layer_dt, padded by the
# asteroid + ship radius (+ the k-sigma uncertainty shell when enabled).
# Keeping the layers SEPARATE (rather than unioning them like the old planner)
# is what lets the search pass through a cell at a time it is actually clear.
# Stamps are SPHERES (squared-radius test), not cubes: a cube blocks ~90% more
# cells than the hazard occupies, which directly causes false NO_PATH refusals.
func _build_time_layers() -> Array:
	var nx := cells_x()
	var ny := cells_y()
	var nz := cells_z()
	var yz := ny * nz
	var n_cells := nx * yz
	var layers: Array = []
	for li in range(plan_time_layers):
		var t := float(li) * plan_layer_dt
		# Uncertainty shell for this layer's look-ahead time, in metres.
		var infl := 0.0
		if unc_enable:
			infl = unc_ksigma * (unc_sigma0 + unc_growth * t)
		var grid := PackedByteArray()
		grid.resize(n_cells)   # zero-filled
		for a in asteroids:
			var r: float = a["radius"]
			var pad := int(ceil((r + ship_radius + infl) / voxel_size))
			pad = clampi(pad, 1, 8)
			var pad2 := pad * pad
			var fp: Vector3 = Predictor.predict(a["pos"], a["vel"], t, bounds_min, bounds_max)
			var c := world_to_cell(fp)
			for dx in range(-pad, pad + 1):
				var xx := c.x + dx
				if xx < 0 or xx >= nx:
					continue
				for dy in range(-pad, pad + 1):
					var yy := c.y + dy
					if yy < 0 or yy >= ny:
						continue
					var d2 := dx * dx + dy * dy
					if d2 > pad2:
						continue
					for dz in range(-pad, pad + 1):
						var zz := c.z + dz
						if zz < 0 or zz >= nz:
							continue
						if d2 + dz * dz > pad2:
							continue
						grid[xx * yz + yy * nz + zz] = 1
		layers.append(grid)
	return layers

# Time-collapsed (union) planning over the same layers -- the legacy model where
# any cell a rock EVER touches in the window is a wall. Kept as the regression
# canary: SelfTest asserts scenarios the union must refuse but the time-indexed
# planner must thread.
func _plan_static_fallback(start_cell: Vector3i, goal_cell: Vector3i,
		layers: Array, dims: Vector3i) -> Array[Vector3i]:
	var union := (layers[0] as PackedByteArray).duplicate()
	for li in range(1, layers.size()):
		var grid: PackedByteArray = layers[li]
		for i in range(union.size()):
			if grid[i] != 0:
				union[i] = 1
	var yz := dims.y * dims.z
	var nz := dims.z
	var is_free := func(c: Vector3i) -> bool:
		return cell_in_bounds(c) and union[c.x * yz + c.y * nz + c.z] == 0
	return VoxelAStar.plan(start_cell, goal_cell, is_free, 120000)

# ============================================================ ship control
func _step_ship(dt: float) -> void:
	if path.is_empty() or path_index >= path.size():
		_brake(dt)
		arrived = path_index >= path.size() and not path.is_empty()
		return

	# Waypoint advance must survive avoidance dodges: a lateral dodge >3 m means
	# the ship never passes "within tolerance" of waypoints it has clearly left
	# behind, which inflates remaining distance and falsely brakes the ship. So
	# advance past any segment whose far end the ship has passed abeam
	# (projection parameter >= 1), then apply the close-pass tolerance.
	var moved := true
	while moved:
		moved = false
		if path_index < path.size() - 1:
			var a := path[path_index]
			var ab := path[path_index + 1] - a
			var denom := ab.length_squared()
			if denom <= 1e-9 or (ship_pos - a).dot(ab) / denom >= 1.0:
				path_index += 1
				moved = true
		if path_index < path.size() and ship_pos.distance_to(path[path_index]) <= WAYPOINT_TOL:
			path_index += 1
			moved = true
	if path_index >= path.size():
		_brake(dt)
		arrived = true
		return

	var target := _lookahead_target()
	var arrival_dist := _remaining_path_distance()

	var cfg := planner_cfg.duplicate()
	cfg["unc_enable"] = unc_enable
	cfg["unc_sigma0"] = unc_sigma0
	cfg["unc_growth"] = unc_growth
	cfg["unc_ksigma"] = unc_ksigma
	var accel := LocalPlanner3D.plan(
		ship_pos, ship_vel, target, arrival_dist, asteroids,
		ship_radius, a_max(), cruise_speed(),
		bounds_min, bounds_max, cfg
	)
	_apply_accel(accel, dt)

func _apply_accel(accel: Vector3, dt: float) -> void:
	var am := a_max()
	if accel.length() > am:
		accel = accel.normalized() * am
	last_accel = accel
	# Charge Δv for the velocity change actually achieved -- thrust spent on
	# pushing past the speed cap is clipped, not delivered, and a reported fuel
	# metric must not overcount.
	var v_before := ship_vel
	ship_vel += accel * dt
	if ship_vel.length() > ship_max_speed:
		ship_vel = ship_vel.normalized() * ship_max_speed
	ship_pos += ship_vel * dt
	dv_used += (ship_vel - v_before).length()

func _brake(dt: float) -> void:
	if ship_vel.length() < 1e-3:
		ship_vel = Vector3.ZERO
		last_accel = Vector3.ZERO
		return
	var am := a_max()
	last_accel = -ship_vel.normalized() * am
	var v_before := ship_vel
	var dv := -ship_vel.normalized() * am * dt
	if dv.length() >= ship_vel.length():
		ship_vel = Vector3.ZERO
	else:
		ship_vel += dv
	ship_pos += ship_vel * dt
	# Charge only the velocity actually shed (the final partial-stop tick needs
	# less than a full a_max*dt impulse).
	dv_used += (ship_vel - v_before).length()

func _lookahead_target() -> Vector3:
	var acc := 0.0
	var prev := ship_pos
	var idx := path_index
	while idx < path.size():
		acc += prev.distance_to(path[idx])
		if acc >= LOOKAHEAD_M:
			return path[idx]
		prev = path[idx]
		idx += 1
	return path[path.size() - 1]

func _remaining_path_distance() -> float:
	if path.is_empty():
		return 0.0
	var d := 0.0
	var prev := ship_pos
	for i in range(path_index, path.size()):
		d += prev.distance_to(path[i])
		prev = path[i]
	return d

# ============================================================ metrics
func _update_metrics(ship_prev: Vector3, ast_prev: Array) -> void:
	var nearest := INF
	for i in range(asteroids.size()):
		var a := asteroids[i]
		var ar: float = a["radius"]
		# Closest approach over the step: both bodies move linearly within dt, so
		# the true minimum distance is the relative-motion segment's distance to
		# the origin.
		var rel_a: Vector3 = ship_prev - ast_prev[i]
		var rel_b: Vector3 = ship_pos - a["pos"]
		var d := LocalPlanner3D._seg_origin_dist(rel_a, rel_b) - (ship_radius + ar)
		nearest = minf(nearest, d)
		if d <= 0.0:
			if not _contact.has(i):
				_contact[i] = true
				collisions += 1
		else:
			_contact.erase(i)
	if nearest != INF:
		min_clearance = minf(min_clearance, nearest)

	if collisions > 0:
		status = "COLLISION"
	elif arrived:
		status = "ARRIVED"
	else:
		status = "FLYING"

# ============================================================ grid helpers
func cells_x() -> int: return int(ceil((bounds_max.x - bounds_min.x) / voxel_size))
func cells_y() -> int: return int(ceil((bounds_max.y - bounds_min.y) / voxel_size))
func cells_z() -> int: return int(ceil((bounds_max.z - bounds_min.z) / voxel_size))

func cell_in_bounds(c: Vector3i) -> bool:
	return (c.x >= 0 and c.x < cells_x()
		and c.y >= 0 and c.y < cells_y()
		and c.z >= 0 and c.z < cells_z())

func world_to_cell(p: Vector3) -> Vector3i:
	var lp := p - bounds_min
	return Vector3i(
		clampi(int(floor(lp.x / voxel_size)), 0, cells_x() - 1),
		clampi(int(floor(lp.y / voxel_size)), 0, cells_y() - 1),
		clampi(int(floor(lp.z / voxel_size)), 0, cells_z() - 1)
	)

func cell_to_world(c: Vector3i) -> Vector3:
	return bounds_min + Vector3(
		(float(c.x) + 0.5) * voxel_size,
		(float(c.y) + 0.5) * voxel_size,
		(float(c.z) + 0.5) * voxel_size
	)

func clamp_to_bounds(p: Vector3) -> Vector3:
	return Vector3(
		clampf(p.x, bounds_min.x, bounds_max.x),
		clampf(p.y, bounds_min.y, bounds_max.y),
		clampf(p.z, bounds_min.z, bounds_max.z)
	)

# ============================================================ scenario I/O
# A "scenario" is the editable design (start, goal, asteroids, ship params) --
# not runtime state. JSON-friendly (Vector3 -> [x,y,z]).
func to_scenario() -> Dictionary:
	var rocks: Array = []
	for a in asteroids:
		rocks.append({
			"pos": _v3_arr(a["pos"]),
			"vel": _v3_arr(a["vel"]),
			"radius": a["radius"],
			"mass": a.get("mass", 1000.0),
		})
	return {
		"version": 2,
		"start": _v3_arr(start_pos),
		"goal": _v3_arr(goal_pos),
		"bounds": {"min": _v3_arr(bounds_min), "max": _v3_arr(bounds_max)},
		"ship": {
			"radius": ship_radius,
			"mass": ship_mass,
			"max_thrust": ship_max_thrust,
			"max_speed": ship_max_speed,
			"target_speed": ship_target_speed,
		},
		# Without these a saved failure cannot be replayed faithfully -- the
		# whole point of dumping it.
		"sim": {"noise_sigma": noise_sigma, "seed": seed_value, "max_time": max_time},
		"uncertainty": {"enable": unc_enable, "sigma0": unc_sigma0,
			"growth": unc_growth, "ksigma": unc_ksigma},
		"planner_cfg": planner_cfg,
		"asteroids": rocks,
	}

func load_scenario(d: Dictionary) -> void:
	start_pos = _arr_v3(d.get("start", [-60, 10, -60]))
	goal_pos = _arr_v3(d.get("goal", [60, 10, 60]))
	if d.has("bounds"):
		bounds_min = _arr_v3(d["bounds"].get("min", _v3_arr(bounds_min)))
		bounds_max = _arr_v3(d["bounds"].get("max", _v3_arr(bounds_max)))
	if d.has("sim"):
		noise_sigma = float(d["sim"].get("noise_sigma", 0.0))
		seed_value = int(d["sim"].get("seed", 0))
		max_time = float(d["sim"].get("max_time", max_time))
	if d.has("uncertainty"):
		unc_enable = bool(d["uncertainty"].get("enable", false))
		unc_sigma0 = float(d["uncertainty"].get("sigma0", 0.0))
		unc_growth = float(d["uncertainty"].get("growth", 0.0))
		unc_ksigma = float(d["uncertainty"].get("ksigma", 3.0))
	var sh: Dictionary = d.get("ship", {})
	ship_radius = float(sh.get("radius", ship_radius))
	ship_mass = float(sh.get("mass", ship_mass))
	ship_max_thrust = float(sh.get("max_thrust", ship_max_thrust))
	ship_max_speed = float(sh.get("max_speed", ship_max_speed))
	ship_target_speed = float(sh.get("target_speed", ship_target_speed))
	if d.has("planner_cfg"):
		planner_cfg = d["planner_cfg"]
	asteroids = []
	for r in d.get("asteroids", []):
		asteroids.append({
			"pos": _arr_v3(r.get("pos", [0, 0, 0])),
			"vel": _arr_v3(r.get("vel", [0, 0, 0])),
			"radius": float(r.get("radius", 2.0)),
			"mass": float(r.get("mass", 1000.0)),
		})

static func save_to_file(path: String, scenario: Dictionary) -> bool:
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		return false
	f.store_string(JSON.stringify(scenario, "\t"))
	f.close()
	return true

# ============================================================ flight recording
func _rec_init() -> void:
	recording = {
		"version": 1,
		"scenario": to_scenario(),
		"frames": [],
		"paths": [],
	}
	_rec_timer = 0.0
	_rec_frame()
	_rec_path()

func _rec_frame() -> void:
	var ap := []
	for a in asteroids:
		var p: Vector3 = a["pos"]
		ap.append([snappedf(p.x, 0.01), snappedf(p.y, 0.01), snappedf(p.z, 0.01)])
	recording["frames"].append({
		"t": snappedf(time, 0.001),
		"ship": [snappedf(ship_pos.x, 0.01), snappedf(ship_pos.y, 0.01), snappedf(ship_pos.z, 0.01)],
		"speed": snappedf(ship_vel.length(), 0.01),
		"ast": ap,
	})

func _rec_path() -> void:
	var pts := []
	for p in path:
		pts.append([snappedf(p.x, 0.1), snappedf(p.y, 0.1), snappedf(p.z, 0.1)])
	recording["paths"].append({"t": snappedf(time, 0.001), "pts": pts})

# Result is stamped at save time so it is always the final state.
func save_recording(file_path: String) -> bool:
	if recording.is_empty():
		return false
	recording["result"] = {
		"status": status,
		"time": snappedf(time, 0.01),
		"dv_used": snappedf(dv_used, 0.1),
		"min_clearance": (-1.0 if min_clearance == INF else snappedf(min_clearance, 0.01)),
		"collisions": collisions,
		"replans": replans,
		"degraded": degraded,
	}
	var f := FileAccess.open(file_path, FileAccess.WRITE)
	if f == null:
		return false
	f.store_string(JSON.stringify(recording))
	f.close()
	return true

static func load_from_file(path: String) -> Dictionary:
	if not FileAccess.file_exists(path):
		return {}
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		return {}
	var txt := f.get_as_text()
	f.close()
	var parsed = JSON.parse_string(txt)
	return parsed if parsed is Dictionary else {}

# ============================================================ scenario gen
# Deterministic random asteroid belt (shared by Main's "B" key and BatchEval).
static func random_belt(count: int, rng: RandomNumberGenerator,
		start: Vector3, goal: Vector3,
		bmin: Vector3, bmax: Vector3) -> Array[Dictionary]:
	var rocks: Array[Dictionary] = []
	var attempts := 0
	while rocks.size() < count and attempts < count * 40:
		attempts += 1
		var pos := Vector3(
			rng.randf_range(bmin.x + 5.0, bmax.x - 5.0),
			rng.randf_range(bmin.y + 5.0, bmax.y - 5.0),
			rng.randf_range(bmin.z + 5.0, bmax.z - 5.0)
		)
		if pos.distance_to(start) < 18.0 or pos.distance_to(goal) < 18.0:
			continue
		var radius := rng.randf_range(1.5, 4.0)
		var speed := rng.randf_range(4.0, 18.0)
		var dir := Vector3(rng.randf() - 0.5, rng.randf() - 0.5, rng.randf() - 0.5)
		if dir.length() < 1e-4:
			dir = Vector3.RIGHT
		rocks.append({
			"pos": pos,
			"vel": dir.normalized() * speed,
			"radius": radius,
			"mass": radius * 500.0,
		})
	return rocks

# Saturn-ring style field: an annulus of rocks in the XZ mid-plane with
# tangential (orbit-like) motion -- inner rocks faster (Keplerian flavor),
# small velocity jitter, thin vertical spread. Crossing it is the classic
# ring-plane-crossing problem: there is no permanently empty corridor, the
# ship has to thread a gap in TIME.
static func ring_field(count: int, rng: RandomNumberGenerator,
		bmin: Vector3, bmax: Vector3) -> Array[Dictionary]:
	var rocks: Array[Dictionary] = []
	var cx := (bmin.x + bmax.x) * 0.5
	var cz := (bmin.z + bmax.z) * 0.5
	var cy := (bmin.y + bmax.y) * 0.5
	var r_in := 28.0
	var r_out := minf(bmax.x - cx, bmax.z - cz) - 4.0
	for i in range(count):
		# sqrt -> uniform density by ring area, not bunched at the centre
		var r := sqrt(rng.randf_range(r_in * r_in, r_out * r_out))
		var th := rng.randf_range(0.0, TAU)
		var pos := Vector3(cx + r * cos(th), cy + rng.randfn(0.0, 3.0), cz + r * sin(th))
		pos.y = clampf(pos.y, bmin.y + 3.0, bmax.y - 3.0)
		var tangent := Vector3(-sin(th), 0.0, cos(th))
		var speed := 16.0 * sqrt(r_in / r)
		var vel := tangent * speed + Vector3(
			rng.randfn(0.0, 0.4), rng.randfn(0.0, 0.3), rng.randfn(0.0, 0.4))
		var radius := rng.randf_range(1.0, 3.0)
		rocks.append({"pos": pos, "vel": vel, "radius": radius, "mass": radius * 500.0})
	return rocks

# ============================================================ vector <-> array
static func _v3_arr(v: Vector3) -> Array:
	return [v.x, v.y, v.z]

static func _arr_v3(a) -> Vector3:
	if a is Vector3:
		return a
	return Vector3(float(a[0]), float(a[1]), float(a[2]))
