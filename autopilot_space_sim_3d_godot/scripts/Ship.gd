extends Area3D
class_name Ship

# Visual representation of the spacecraft. The actual flight dynamics and
# autopilot now live in SimWorld (so the interactive scene and the headless
# benchmark run identical logic). This node just holds the ship's design
# parameters (edited before a run) and renders position/orientation.

@export var mass_kg: float = 8000.0
@export var max_thrust_n: float = 250000.0
@export var max_speed_mps: float = 200.0
@export var ship_radius_m: float = 1.5
@export var target_speed_mps: float = 120.0

# Optional per-ship planner weight overrides (see LocalPlanner3D.DEFAULT_CFG).
@export var planner_cfg: Dictionary = {}

@onready var _mesh: MeshInstance3D = $MeshInstance3D

var _base_material: Material = null
var _highlight_material: StandardMaterial3D = null

func _ready() -> void:
	_base_material = _mesh.material_override
	_highlight_material = StandardMaterial3D.new()
	_highlight_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_highlight_material.albedo_color = Color(0.1, 1.0, 0.8, 1.0)
	_highlight_material.emission_enabled = true
	_highlight_material.emission = Color(0.1, 1.0, 0.8, 1.0)
	_highlight_material.emission_energy_multiplier = 2.0

func set_selected(selected: bool) -> void:
	if not _mesh:
		return
	_mesh.material_override = _highlight_material if selected else _base_material

# Point the hull along its velocity (called by Main while flying).
func face_velocity(v: Vector3) -> void:
	if v.length() <= 1.0:
		return
	var dir := v.normalized()
	var up := Vector3.UP
	if absf(dir.dot(up)) > 0.99:
		up = Vector3.FORWARD
	look_at(global_position + v, up)

func editor_get_position() -> Vector3:
	return global_position

func editor_set_position(p: Vector3) -> void:
	global_position = p

func editor_get_velocity() -> Vector3:
	return Vector3.ZERO

func editor_set_velocity(_v: Vector3) -> void:
	pass

func editor_get_mass_kg() -> float:
	return mass_kg

func editor_set_mass_kg(m: float) -> void:
	mass_kg = max(1.0, m)

func editor_get_radius_m() -> float:
	return ship_radius_m

func editor_set_radius_m(r: float) -> void:
	ship_radius_m = max(0.25, r)
