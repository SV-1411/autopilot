extends Area3D
class_name Ship

# The autonomous spacecraft.
#
# Dynamics: thrust-limited point mass. The ship cannot change velocity instantly
# -- it can only apply an acceleration up to a_max = max_thrust / mass. This is
# what turns waypoint-following into a real guidance problem (the ship has
# inertia and must plan ahead to stop or turn).
#
# Control: each tick it follows the global A* path by aiming at its current
# waypoint, but the actual thrust command comes from LocalPlanner3D, which
# bends the trajectory to avoid moving asteroids.

@export var mass_kg: float = 8000.0
@export var max_thrust_n: float = 250000.0
@export var max_speed_mps: float = 200.0
@export var ship_radius_m: float = 1.5
@export var target_speed_mps: float = 120.0

# Tunable planner weights (forwarded to LocalPlanner3D). Edit here to retune.
@export var planner_cfg: Dictionary = {}

@onready var _mesh: MeshInstance3D = $MeshInstance3D

var _base_material: Material = null
var _highlight_material: StandardMaterial3D = null

var velocity: Vector3 = Vector3.ZERO
var _running := false
var _path: Array[Vector3] = []
var _path_index := 0

# World context supplied by Main each tick.
var _obstacles: Array = []
var _bounds_min: Vector3 = Vector3(-90, 0, -90)
var _bounds_max: Vector3 = Vector3(90, 60, 90)

# Run metrics.
var _delta_v_used := 0.0      # integral of |accel| dt -- a proxy for fuel
var _arrived := false

const WAYPOINT_TOLERANCE_M := 3.0
const LOOKAHEAD_M := 25.0          # aim this far down the path, not at the next cell

func _ready() -> void:
	_base_material = _mesh.material_override
	_highlight_material = StandardMaterial3D.new()
	_highlight_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_highlight_material.albedo_color = Color(0.1, 1.0, 0.8, 1.0)
	_highlight_material.emission_enabled = true
	_highlight_material.emission = Color(0.1, 1.0, 0.8, 1.0)
	_highlight_material.emission_energy_multiplier = 2.0

func a_max() -> float:
	return max_thrust_n / maxf(1.0, mass_kg)

func cruise_speed() -> float:
	return minf(target_speed_mps, max_speed_mps)

func set_running(running: bool) -> void:
	_running = running
	if running:
		_delta_v_used = 0.0
		_arrived = false

func set_selected(selected: bool) -> void:
	if not _mesh:
		return
	_mesh.material_override = _highlight_material if selected else _base_material

func set_waypoints(points: Array[Vector3]) -> void:
	_path = points
	_path_index = 0

func get_waypoints() -> Array[Vector3]:
	return _path.duplicate()

func get_waypoint_index() -> int:
	return _path_index

func set_obstacles(obstacles: Array) -> void:
	_obstacles = obstacles

func set_world_bounds(bmin: Vector3, bmax: Vector3) -> void:
	_bounds_min = bmin
	_bounds_max = bmax

func get_delta_v_used() -> float:
	return _delta_v_used

func has_arrived() -> bool:
	return _arrived

func step_sim(dt: float) -> void:
	if not _running:
		return
	if _path.is_empty() or _path_index >= _path.size():
		_brake(dt)
		_arrived = _path_index >= _path.size() and not _path.is_empty()
		return

	# Advance through waypoints we've effectively reached.
	while _path_index < _path.size() and global_position.distance_to(_path[_path_index]) <= WAYPOINT_TOLERANCE_M:
		_path_index += 1
	if _path_index >= _path.size():
		_brake(dt)
		_arrived = true
		return

	var target := _lookahead_target()
	var arrival_dist := _remaining_path_distance()

	# The autopilot decides the thrust command.
	var accel := LocalPlanner3D.plan(
		global_position, velocity, target, arrival_dist, _obstacles,
		ship_radius_m, a_max(), cruise_speed(),
		_bounds_min, _bounds_max, planner_cfg
	)

	_apply_accel(accel, dt)

# A point ~LOOKAHEAD_M metres ahead along the path. Gives the local planner a
# stable, informative heading instead of the next 3 m waypoint.
func _lookahead_target() -> Vector3:
	var acc := 0.0
	var prev := global_position
	var idx := _path_index
	while idx < _path.size():
		acc += prev.distance_to(_path[idx])
		if acc >= LOOKAHEAD_M:
			return _path[idx]
		prev = _path[idx]
		idx += 1
	return _path[_path.size() - 1]

# Remaining distance to the final goal, measured along the path. Used to govern
# arrival speed so the ship can always brake in time.
func _remaining_path_distance() -> float:
	if _path.is_empty():
		return 0.0
	var d := 0.0
	var prev := global_position
	for i in range(_path_index, _path.size()):
		d += prev.distance_to(_path[i])
		prev = _path[i]
	return d

func _apply_accel(accel: Vector3, dt: float) -> void:
	# Clamp to thrust limit, integrate, clamp to top speed.
	if accel.length() > a_max():
		accel = accel.normalized() * a_max()
	velocity += accel * dt
	if velocity.length() > max_speed_mps:
		velocity = velocity.normalized() * max_speed_mps
	global_position += velocity * dt
	_delta_v_used += accel.length() * dt
	_face_velocity()

func _brake(dt: float) -> void:
	if velocity.length() < 1e-3:
		velocity = Vector3.ZERO
		return
	var decel := -velocity.normalized() * a_max()
	var dv := decel * dt
	if dv.length() >= velocity.length():
		velocity = Vector3.ZERO
	else:
		velocity += dv
	global_position += velocity * dt
	_delta_v_used += decel.length() * dt
	_face_velocity()

func _face_velocity() -> void:
	if velocity.length() <= 1.0:
		return
	var dir := velocity.normalized()
	# look_at fails if the facing direction is parallel to the up vector.
	var up := Vector3.UP
	if absf(dir.dot(up)) > 0.99:
		up = Vector3.FORWARD
	look_at(global_position + velocity, up)

func editor_get_position() -> Vector3:
	return global_position

func editor_set_position(p: Vector3) -> void:
	global_position = p

func editor_get_velocity() -> Vector3:
	return velocity

func editor_set_velocity(v: Vector3) -> void:
	velocity = v

func editor_get_mass_kg() -> float:
	return mass_kg

func editor_set_mass_kg(m: float) -> void:
	mass_kg = max(1.0, m)

func editor_get_radius_m() -> float:
	return ship_radius_m

func editor_set_radius_m(r: float) -> void:
	ship_radius_m = max(0.25, r)
