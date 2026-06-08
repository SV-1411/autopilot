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
var predict_sample_times := [0.0, 0.5, 1.0, 1.5, 2.0]
var max_time := 60.0          # run is declared TIMEOUT after this many seconds

# ---------------------------------------------------------------- ship config
var ship_radius := 1.5
var ship_mass := 8000.0
var ship_max_thrust := 250000.0
var ship_max_speed := 200.0
var ship_target_speed := 120.0
var planner_cfg := {}

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
	plan_path()
	status = "NO_PATH" if path.is_empty() else "FLYING"

func step(dt: float) -> void:
	if status != "FLYING":
		return

	time += dt

	for a in asteroids:
		_integrate_asteroid(a, dt)

	_replan_timer += dt
	if _replan_timer >= replan_interval:
		plan_path()
		_replan_timer = 0.0
		replans += 1

	_step_ship(dt)
	_update_metrics()

	if status == "FLYING" and time >= max_time:
		status = "TIMEOUT"

# ============================================================ asteroid motion
func _integrate_asteroid(a: Dictionary, dt: float) -> void:
	var p: Vector3 = a["pos"] + a["vel"] * dt
	var v: Vector3 = a["vel"]
	for axis in range(3):
		if p[axis] < bounds_min[axis]:
			p[axis] = bounds_min[axis]
			v[axis] = absf(v[axis])
		elif p[axis] > bounds_max[axis]:
			p[axis] = bounds_max[axis]
			v[axis] = -absf(v[axis])
	a["pos"] = p
	a["vel"] = v

# ============================================================ global planner
func plan_path() -> void:
	ship_pos = clamp_to_bounds(ship_pos)
	goal_pos = clamp_to_bounds(goal_pos)

	var start_cell := world_to_cell(ship_pos)
	var goal_cell := world_to_cell(goal_pos)

	var blocked := _build_predicted_occupancy()
	var is_free := func(c: Vector3i) -> bool:
		return cell_in_bounds(c) and not blocked.has(c)

	var cell_path: Array[Vector3i] = VoxelAStar.plan(start_cell, goal_cell, is_free, 120000)
	if cell_path.is_empty():
		return  # keep the previous path rather than stranding the ship

	var world_path: Array[Vector3] = []
	for c in cell_path:
		world_path.append(cell_to_world(c))
	path = world_path
	path_index = 0

func _build_predicted_occupancy() -> Dictionary:
	var blocked := {}
	for a in asteroids:
		var r: float = a["radius"]
		var pad := int(ceil((r + ship_radius) / voxel_size))
		pad = clampi(pad, 1, 6)
		for t in predict_sample_times:
			var fp: Vector3 = Predictor.predict(a["pos"], a["vel"], float(t), bounds_min, bounds_max)
			var c := world_to_cell(fp)
			for dx in range(-pad, pad + 1):
				for dy in range(-pad, pad + 1):
					for dz in range(-pad, pad + 1):
						var cc := Vector3i(c.x + dx, c.y + dy, c.z + dz)
						if cell_in_bounds(cc):
							blocked[cc] = true
	return blocked

# ============================================================ ship control
func _step_ship(dt: float) -> void:
	if path.is_empty() or path_index >= path.size():
		_brake(dt)
		arrived = path_index >= path.size() and not path.is_empty()
		return

	while path_index < path.size() and ship_pos.distance_to(path[path_index]) <= WAYPOINT_TOL:
		path_index += 1
	if path_index >= path.size():
		_brake(dt)
		arrived = true
		return

	var target := _lookahead_target()
	var arrival_dist := _remaining_path_distance()

	var accel := LocalPlanner3D.plan(
		ship_pos, ship_vel, target, arrival_dist, asteroids,
		ship_radius, a_max(), cruise_speed(),
		bounds_min, bounds_max, planner_cfg
	)
	_apply_accel(accel, dt)

func _apply_accel(accel: Vector3, dt: float) -> void:
	var am := a_max()
	if accel.length() > am:
		accel = accel.normalized() * am
	ship_vel += accel * dt
	if ship_vel.length() > ship_max_speed:
		ship_vel = ship_vel.normalized() * ship_max_speed
	ship_pos += ship_vel * dt
	dv_used += accel.length() * dt

func _brake(dt: float) -> void:
	if ship_vel.length() < 1e-3:
		ship_vel = Vector3.ZERO
		return
	var am := a_max()
	var decel := -ship_vel.normalized() * am
	var dv := decel * dt
	if dv.length() >= ship_vel.length():
		ship_vel = Vector3.ZERO
	else:
		ship_vel += dv
	ship_pos += ship_vel * dt
	dv_used += am * dt

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
func _update_metrics() -> void:
	var nearest := INF
	for i in range(asteroids.size()):
		var a := asteroids[i]
		var ap: Vector3 = a["pos"]
		var ar: float = a["radius"]
		var d := ship_pos.distance_to(ap) - (ship_radius + ar)
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
		"start": _v3_arr(start_pos),
		"goal": _v3_arr(goal_pos),
		"ship": {
			"radius": ship_radius,
			"mass": ship_mass,
			"max_thrust": ship_max_thrust,
			"max_speed": ship_max_speed,
			"target_speed": ship_target_speed,
		},
		"planner_cfg": planner_cfg,
		"asteroids": rocks,
	}

func load_scenario(d: Dictionary) -> void:
	start_pos = _arr_v3(d.get("start", [-60, 10, -60]))
	goal_pos = _arr_v3(d.get("goal", [60, 10, 60]))
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

# ============================================================ vector <-> array
static func _v3_arr(v: Vector3) -> Array:
	return [v.x, v.y, v.z]

static func _arr_v3(a) -> Vector3:
	if a is Vector3:
		return a
	return Vector3(float(a[0]), float(a[1]), float(a[2]))
