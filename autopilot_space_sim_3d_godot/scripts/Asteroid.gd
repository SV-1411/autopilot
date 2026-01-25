extends Area3D
class_name Asteroid

@export var velocity: Vector3 = Vector3.ZERO
@export var mass_kg: float = 1000.0
@export var radius_m: float = 2.0

@onready var _mesh: MeshInstance3D = $MeshInstance3D
@onready var _collider: CollisionShape3D = $CollisionShape3D

var _base_material: Material = null
var _highlight_material: StandardMaterial3D = null

func _ready() -> void:
	_base_material = _mesh.material_override
	_highlight_material = StandardMaterial3D.new()
	_highlight_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_highlight_material.albedo_color = Color(1.0, 0.95, 0.2, 1.0)
	_highlight_material.emission_enabled = true
	_highlight_material.emission = Color(1.0, 0.95, 0.2, 1.0)
	_highlight_material.emission_energy_multiplier = 2.0
	_apply_radius()

func step_sim(dt: float) -> void:
	global_position += velocity * dt

func set_radius_m(r: float) -> void:
	radius_m = max(0.25, r)
	_apply_radius()

func _apply_radius() -> void:
	if _mesh:
		_mesh.scale = Vector3.ONE * radius_m
	if _collider and _collider.shape is SphereShape3D:
		(_collider.shape as SphereShape3D).radius = radius_m

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
	return radius_m

func editor_set_radius_m(r: float) -> void:
	set_radius_m(r)

func set_selected(selected: bool) -> void:
	if not _mesh:
		return
	_mesh.material_override = _highlight_material if selected else _base_material
