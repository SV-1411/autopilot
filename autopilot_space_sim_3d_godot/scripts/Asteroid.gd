extends Area3D
class_name Asteroid

# A moving obstacle. Travels at constant velocity and bounces off the world
# bounds. Its motion is intentionally simple and deterministic so the autopilot's
# Predictor can reproduce it exactly when planning ahead.

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
	_highlight_material.albedo_color = Color(1.0, 0.3, 0.1, 1.0)
	_highlight_material.emission_enabled = true
	_highlight_material.emission = Color(1.0, 0.3, 0.1, 1.0)
	_highlight_material.emission_energy_multiplier = 2.0
	_apply_radius()

# Advance the asteroid by dt, reflecting its velocity at the world bounds.
# This must stay consistent with Predictor.predict so look-ahead is accurate.
func integrate(dt: float, bounds_min: Vector3, bounds_max: Vector3) -> void:
	var p := global_position + velocity * dt
	var v := velocity
	for axis in range(3):
		if p[axis] < bounds_min[axis]:
			p[axis] = bounds_min[axis]
			v[axis] = absf(v[axis])
		elif p[axis] > bounds_max[axis]:
			p[axis] = bounds_max[axis]
			v[axis] = -absf(v[axis])
	velocity = v
	global_position = p

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
