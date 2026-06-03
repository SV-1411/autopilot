extends Area3D
class_name Ship

@export var mass_kg: float = 8000.0
@export var max_thrust_n: float = 250000.0
@export var max_speed_mps: float = 200.0
@export var ship_radius_m: float = 1.5
@export var target_speed_mps: float = 150.0

@onready var _mesh: MeshInstance3D = $MeshInstance3D

var _base_material: Material = null
var _highlight_material: StandardMaterial3D = null

var velocity: Vector3 = Vector3.ZERO
var _running := false
var _path: Array[Vector3] = []
var _path_index := 0

func _ready() -> void:
	_base_material = _mesh.material_override
	_highlight_material = StandardMaterial3D.new()
	_highlight_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_highlight_material.albedo_color = Color(0.1, 1.0, 0.8, 1.0)
	_highlight_material.emission_enabled = true
	_highlight_material.emission = Color(0.1, 1.0, 0.8, 1.0)
	_highlight_material.emission_energy_multiplier = 2.0

func set_running(running: bool) -> void:
	_running = running

func set_selected(selected: bool) -> void:
	if not _mesh:
		return
	_mesh.material_override = _highlight_material if selected else _base_material

func set_waypoints(points: Array[Vector3]) -> void:
	_path = points
	_path_index = 0
	print("Ship: Received ", points.size(), " waypoints")

func get_waypoints() -> Array[Vector3]:
	return _path.duplicate()

func get_waypoint_index() -> int:
	return _path_index

func step_sim(dt: float) -> void:
	if not _running:
		return
	if _path.is_empty():
		print("Ship: No path")
		return
	if _path_index >= _path.size():
		print("Ship: Reached destination")
		return

	var target: Vector3 = _path[_path_index]
	var to_target := target - global_position
	var dist := to_target.length()

	print("Ship: Following waypoint ", _path_index, "/", _path.size(), " distance: ", dist)

	if dist <= 2.0:
		_path_index += 1
		print("Ship: Reached waypoint, advancing to ", _path_index)
		if _path_index >= _path.size():
			print("Ship: Reached all waypoints!")
			return
		return

	# Move toward current waypoint
	var move_dir := to_target.normalized()
	velocity = move_dir * target_speed_mps
	global_position += velocity * dt

	print("Ship: Moving at ", velocity.length(), " m/s")

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
