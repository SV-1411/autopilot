extends RefCounted
class_name VoxelAStar

static func _heuristic(a: Vector3i, b: Vector3i) -> int:
	return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z)

static func _neighbors(c: Vector3i) -> Array[Vector3i]:
	return [
		Vector3i(c.x + 1, c.y, c.z),
		Vector3i(c.x - 1, c.y, c.z),
		Vector3i(c.x, c.y + 1, c.z),
		Vector3i(c.x, c.y - 1, c.z),
		Vector3i(c.x, c.y, c.z + 1),
		Vector3i(c.x, c.y, c.z - 1),
	]

static func plan(
	start: Vector3i,
	goal: Vector3i,
	is_free: Callable,
	max_expansions: int = 200000
) -> Array[Vector3i]:
	if start == goal:
		return [start]
	if not is_free.call(start) or not is_free.call(goal):
		return []

	var open: Array = []
	open.push_back({"f": _heuristic(start, goal), "g": 0, "n": start})

	var came_from := {}
	var g_score := {}
	g_score[start] = 0

	var closed := {}
	var expansions := 0

	while open.size() > 0:
		open.sort_custom(func(a, b): return a["f"] < b["f"]) 
		var current = open.pop_front()
		var node: Vector3i = current["n"]
		if closed.has(node):
			continue
		closed[node] = true

		if node == goal:
			return _reconstruct(came_from, node)

		expansions += 1
		if expansions >= max_expansions:
			break

		var base_g: int = int(current["g"])
		for nb in _neighbors(node):
			if closed.has(nb):
				continue
			if not is_free.call(nb):
				continue

			var tentative_g := base_g + 1
			var prev_g = g_score.get(nb)
			if prev_g == null or tentative_g < int(prev_g):
				came_from[nb] = node
				g_score[nb] = tentative_g
				var f := tentative_g + _heuristic(nb, goal)
				open.push_back({"f": f, "g": tentative_g, "n": nb})

	return []

static func _reconstruct(came_from: Dictionary, current: Vector3i) -> Array[Vector3i]:
	var path: Array[Vector3i] = [current]
	var cur := current
	while came_from.has(cur):
		cur = came_from[cur]
		path.push_back(cur)
	path.reverse()
	return path
