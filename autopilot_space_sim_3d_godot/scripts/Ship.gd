extends Area3D
class_name Ship

@export var mass_kg: float = 8000.0
@export var max_thrust_n: float = 120000.0
@export var max_speed_mps: float = 120.0
@export var ship_radius_m: float = 1.5
@export var target_speed_mps: float = 80.0

var velocity: Vector3 = Vector3.ZERO
var _running := false
var _path: Array[Vector3] = []
var _path_index := 0

func set_running(running: bool) -> void:
	_running = running

func set_waypoints(points: Array[Vector3]) -> void:
	_path = points
	_path_index = 0

func get_waypoints() -> Array[Vector3]:
	return _path.duplicate()

func get_waypoint_index() -> int:
	return _path_index

func step_sim(dt: float) -> void:
	if not _running:
		return
	if _path.is_empty() or _path_index >= _path.size():
		return

	var target: Vector3 = _path[_path_index]
	var to_target := target - global_position
	var dist := to_target.length()

	if dist <= 1.0:
		_path_index += 1
		return

	var desired_dir := to_target.normalized()
	var desired_vel := desired_dir * target_speed_mps
	var dv := desired_vel - velocity

	var max_acc: float = max_thrust_n / maxf(1.0, mass_kg)
	var acc: Vector3 = dv / maxf(0.001, dt)
	if acc.length() > max_acc:
		acc = acc.normalized() * max_acc

	velocity += acc * dt
	if velocity.length() > max_speed_mps:
		velocity = velocity.normalized() * max_speed_mps

	global_position += velocity * dt

	if velocity.length() > 0.5:
		look_at(global_position + velocity.normalized(), Vector3.UP)

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
