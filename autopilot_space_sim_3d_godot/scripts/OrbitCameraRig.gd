extends Node3D
class_name OrbitCameraRig

@export var pivot_path: NodePath
@export var camera_path: NodePath

@export var orbit_sensitivity: float = 0.01
@export var pan_sensitivity: float = 0.05
@export var zoom_sensitivity: float = 2.0

@export var min_distance: float = 8.0
@export var max_distance: float = 250.0

var _pivot: Node3D
var _camera: Camera3D
var _distance: float = 70.0

var _orbiting := false
var _panning := false
var _last_mouse: Vector2

func _ready() -> void:
	_pivot = get_node(pivot_path) as Node3D
	_camera = get_node(camera_path) as Camera3D
	_distance = clampf(_camera.position.z * -1.0 if _camera.position.z < 0.0 else _distance, min_distance, max_distance)
	# Orient the camera at the pivot immediately: until the first zoom/orbit the
	# camera otherwise keeps its scene-file orientation, which faces AWAY from
	# the world (black screen on launch).
	_update_camera_distance()

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var e := event as InputEventMouseButton
		if e.button_index == MOUSE_BUTTON_RIGHT:
			_orbiting = e.pressed
			_last_mouse = e.position
			get_viewport().set_input_as_handled()
			return
		if e.button_index == MOUSE_BUTTON_MIDDLE:
			_panning = e.pressed
			_last_mouse = e.position
			get_viewport().set_input_as_handled()
			return
		if e.button_index == MOUSE_BUTTON_WHEEL_UP and e.pressed:
			_zoom(-1.0)
			get_viewport().set_input_as_handled()
			return
		if e.button_index == MOUSE_BUTTON_WHEEL_DOWN and e.pressed:
			_zoom(1.0)
			get_viewport().set_input_as_handled()
			return

	if event is InputEventMouseMotion:
		var m := event as InputEventMouseMotion
		if _orbiting:
			_orbit(m.relative)
			get_viewport().set_input_as_handled()
			return
		if _panning:
			_pan(m.relative)
			get_viewport().set_input_as_handled()
			return

func focus(point: Vector3) -> void:
	_pivot.global_position = point

func _orbit(delta: Vector2) -> void:
	# yaw around global up
	rotate_y(-delta.x * orbit_sensitivity)
	# pitch around pivot local X
	_pivot.rotate_x(-delta.y * orbit_sensitivity)
	# clamp pitch
	var x := _pivot.rotation.x
	x = clampf(x, deg_to_rad(-80.0), deg_to_rad(-10.0))
	_pivot.rotation.x = x
	_update_camera_distance()

func _pan(delta: Vector2) -> void:
	# Pan relative to camera orientation
	var right := global_transform.basis.x
	var up := global_transform.basis.y
	_pivot.global_position += (-right * delta.x + up * delta.y) * pan_sensitivity

func _zoom(dir: float) -> void:
	_distance = clampf(_distance + dir * zoom_sensitivity, min_distance, max_distance)
	_update_camera_distance()

func _update_camera_distance() -> void:
	# camera looks at pivot; we place it along -Z of pivot in its local space
	_camera.position = Vector3(0, 0, -_distance)
	_camera.look_at(_pivot.global_position, Vector3.UP)
