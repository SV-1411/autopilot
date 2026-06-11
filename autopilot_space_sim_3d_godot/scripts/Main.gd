extends Node3D

# Interactive front-end for the autopilot.
#
# Architecture: the simulation itself lives in SimWorld (headless engine). This
# scene is a VIEW + EDITOR over it:
#   - EDIT mode: you place/select/drag asteroids, set start/goal, tune metrics,
#     and save/load scenarios. The scene nodes hold the editable design.
#   - RUN mode: a SimWorld is built from the scene, advanced each physics tick,
#     and the nodes are synced to its state for display. Stopping restores the
#     designed scenario so you can keep editing.
#
# Because RUN uses the same SimWorld as the batch evaluator (tools/BatchEval.gd),
# what you see interactively matches the benchmark exactly.

@export var ship_scene: PackedScene
@export var asteroid_scene: PackedScene

const WORLD_BOUNDS_MIN := Vector3(-90, 0, -90)
const WORLD_BOUNDS_MAX := Vector3(90, 60, 90)

const DEFAULT_PLACE_DISTANCE_M := 90.0
const PLACE_DISTANCE_STEP_M := 5.0
const NUDGE_STEP_M := 1.0
const BELT_COUNT := 18

var _running := false
var _sim: SimWorld = null
var _preview_path: Array[Vector3] = []

var _ship: Ship
var _asteroids: Array[Asteroid] = []

var _start_pos: Vector3 = Vector3(-60, 10, -60)
var _goal_pos: Vector3 = Vector3(60, 10, 60)

var _selected: Node3D = null
var _place_distance_m: float = DEFAULT_PLACE_DISTANCE_M

var _dragging := false
var _drag_start_mouse: Vector2
var _drag_start_pos: Vector3

# Saved node state to restore when a run stops.
var _restore_asteroids: Array = []

var _rng := RandomNumberGenerator.new()
var _file_dialog: FileDialog
var _file_mode := ""   # "save" or "load"

@onready var _camera: Camera3D = $CameraRig/Pivot/Camera3D
@onready var _start_marker: Node3D = $StartMarker
@onready var _goal_marker: Node3D = $GoalMarker

@onready var _run_button: Button = $UI/Panel/VBox/RunButton
@onready var _add_asteroid_button: Button = $UI/Panel/VBox/AddAsteroidButton
@onready var _mode_label: Label = $UI/Panel/VBox/ModeLabel
@onready var _selected_label: Label = $UI/Panel/VBox/SelectedLabel
@onready var _metrics_label: Label = $UI/Panel/VBox/MetricsLabel
@onready var _hint_label: Label = $UI/Panel/VBox/HintLabel
@onready var _vbox: VBoxContainer = $UI/Panel/VBox
@onready var _ui_layer: CanvasLayer = $UI

@onready var _pos_x: SpinBox = $UI/Inspector/InspectorVBox/PosGrid/PosX
@onready var _pos_y: SpinBox = $UI/Inspector/InspectorVBox/PosGrid/PosY
@onready var _pos_z: SpinBox = $UI/Inspector/InspectorVBox/PosGrid/PosZ
@onready var _vel_x: SpinBox = $UI/Inspector/InspectorVBox/VelGrid/VelX
@onready var _vel_y: SpinBox = $UI/Inspector/InspectorVBox/VelGrid/VelY
@onready var _vel_z: SpinBox = $UI/Inspector/InspectorVBox/VelGrid/VelZ
@onready var _mass: SpinBox = $UI/Inspector/InspectorVBox/Mass
@onready var _radius: SpinBox = $UI/Inspector/InspectorVBox/Radius
@onready var _apply_button: Button = $UI/Inspector/InspectorVBox/ApplyButton

var _path_line: ImmediateMesh
var _path_mesh_instance: MeshInstance3D
var _ghost_line: ImmediateMesh
var _ghost_mesh_instance: MeshInstance3D

# Realism controls (runtime-added like the save/load buttons).
var _noise_spin: SpinBox
var _unc_check: CheckBox

func _ready() -> void:
	_rng.randomize()
	_run_button.pressed.connect(_toggle_run)
	_add_asteroid_button.pressed.connect(_add_asteroid_at_cursor)
	_apply_button.pressed.connect(_apply_inspector_to_selected)

	_add_save_load_buttons()
	_add_realism_controls()
	_setup_file_dialog()

	_ship = ship_scene.instantiate() as Ship
	add_child(_ship)
	_ship.global_position = _start_pos
	_start_marker.global_position = _start_pos
	_goal_marker.global_position = _goal_pos

	_hint_label.text = "LMB: select   Shift+LMB: Goal   Ctrl+LMB: Start\nB: random belt   C: clear   Q/E: depth\nAlt+drag: move   R/F: nudge up/down\nRMB drag: orbit cam   Wheel: zoom   MMB: pan"

	_setup_environment()
	_setup_path_debug()
	_draw_bounds()
	# Spawn a demo belt immediately so a fresh launch shows the autopilot's
	# problem (and the preview corridor through it) instead of empty space.
	_generate_belt(BELT_COUNT)
	_update_ui()

func _add_save_load_buttons() -> void:
	var save_btn := Button.new()
	save_btn.text = "Save Scenario"
	save_btn.pressed.connect(_on_save_pressed)
	_vbox.add_child(save_btn)
	var load_btn := Button.new()
	load_btn.text = "Load Scenario"
	load_btn.pressed.connect(_on_load_pressed)
	_vbox.add_child(load_btn)

# Process-noise + chance-constraint controls, so the realism modes the
# benchmark exercises are also flyable interactively.
func _add_realism_controls() -> void:
	var noise_row := HBoxContainer.new()
	var noise_label := Label.new()
	noise_label.text = "Noise σ (m/s²)"
	noise_row.add_child(noise_label)
	_noise_spin = SpinBox.new()
	_noise_spin.min_value = 0.0
	_noise_spin.max_value = 5.0
	_noise_spin.step = 0.1
	_noise_spin.value = 0.0
	_noise_spin.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	noise_row.add_child(_noise_spin)
	_vbox.add_child(noise_row)

	_unc_check = CheckBox.new()
	_unc_check.text = "Chance-constrained avoidance (3σ)"
	_vbox.add_child(_unc_check)

func _setup_file_dialog() -> void:
	_file_dialog = FileDialog.new()
	_file_dialog.access = FileDialog.ACCESS_FILESYSTEM
	_file_dialog.use_native_dialog = true
	_file_dialog.add_filter("*.json", "Scenario JSON")
	_file_dialog.file_selected.connect(_on_file_selected)
	_ui_layer.add_child(_file_dialog)

# The scene file has no WorldEnvironment, so without this the viewport is a
# black void with three emissive dots in it. Build a space-like environment:
# dark-blue sky gradient, sky ambient light, glow (makes the emissive ship /
# asteroids / path read clearly), and a simple starfield dome.
func _setup_environment() -> void:
	var sky_mat := ProceduralSkyMaterial.new()
	sky_mat.sky_top_color = Color(0.015, 0.025, 0.07)
	sky_mat.sky_horizon_color = Color(0.07, 0.09, 0.18)
	sky_mat.ground_horizon_color = Color(0.07, 0.09, 0.18)
	sky_mat.ground_bottom_color = Color(0.01, 0.01, 0.03)
	var sky := Sky.new()
	sky.sky_material = sky_mat
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.ambient_light_energy = 2.5
	env.glow_enabled = true
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)

	# Brighten the floor enough to read as a ground plane against the sky.
	var floor_mat := ($Floor as MeshInstance3D).material_override as StandardMaterial3D
	if floor_mat != null:
		floor_mat.albedo_color = Color(0.13, 0.16, 0.23)

	# Starfield: a few hundred unshaded white dots on a distant upper dome.
	var star_mesh := SphereMesh.new()
	star_mesh.radius = 0.5
	star_mesh.height = 1.0
	star_mesh.radial_segments = 4
	star_mesh.rings = 2
	var star_mat := StandardMaterial3D.new()
	star_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	star_mat.albedo_color = Color(0.85, 0.88, 1.0)
	star_mesh.material = star_mat
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.mesh = star_mesh
	mm.instance_count = 350
	var srng := RandomNumberGenerator.new()
	srng.seed = 7
	for i in range(mm.instance_count):
		var dir := Vector3(srng.randfn(), absf(srng.randfn()) * 0.8 + 0.05, srng.randfn()).normalized()
		var s := srng.randf_range(0.4, 1.5)
		mm.set_instance_transform(i, Transform3D(Basis().scaled(Vector3(s, s, s)), dir * 420.0))
	var mmi := MultiMeshInstance3D.new()
	mmi.multimesh = mm
	add_child(mmi)

# Faint wireframe of the world bounds, so the playable volume is visible.
func _draw_bounds() -> void:
	var box := ImmediateMesh.new()
	var inst := MeshInstance3D.new()
	inst.mesh = box
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	m.albedo_color = Color(0.25, 0.55, 0.85, 0.25)
	inst.material_override = m
	add_child(inst)
	var lo := WORLD_BOUNDS_MIN
	var hi := WORLD_BOUNDS_MAX
	var c := [
		Vector3(lo.x, lo.y, lo.z), Vector3(hi.x, lo.y, lo.z),
		Vector3(hi.x, lo.y, hi.z), Vector3(lo.x, lo.y, hi.z),
		Vector3(lo.x, hi.y, lo.z), Vector3(hi.x, hi.y, lo.z),
		Vector3(hi.x, hi.y, hi.z), Vector3(lo.x, hi.y, hi.z),
	]
	var edges := [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]]
	box.surface_begin(Mesh.PRIMITIVE_LINES)
	for e in edges:
		box.surface_add_vertex(c[e[0]])
		box.surface_add_vertex(c[e[1]])
	box.surface_end()

func _setup_path_debug() -> void:
	_path_line = ImmediateMesh.new()
	_path_mesh_instance = MeshInstance3D.new()
	_path_mesh_instance.mesh = _path_line
	_path_mesh_instance.material_override = StandardMaterial3D.new()
	_path_mesh_instance.material_override.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_path_mesh_instance.material_override.albedo_color = Color(0.1, 1.0, 0.5, 1.0)
	add_child(_path_mesh_instance)

	# Ghost trails: where the autopilot PREDICTS each asteroid will be over the
	# next few seconds -- the planner's mental model, made visible.
	_ghost_line = ImmediateMesh.new()
	_ghost_mesh_instance = MeshInstance3D.new()
	_ghost_mesh_instance.mesh = _ghost_line
	var gm := StandardMaterial3D.new()
	gm.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	gm.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	gm.albedo_color = Color(1.0, 0.55, 0.15, 0.45)
	_ghost_mesh_instance.material_override = gm
	add_child(_ghost_mesh_instance)

# rocks: Array of {pos: Vector3, vel: Vector3} (sim dicts or editor snapshots).
func _draw_ghosts(rocks: Array) -> void:
	_ghost_line.clear_surfaces()
	if rocks.is_empty():
		return
	_ghost_line.surface_begin(Mesh.PRIMITIVE_LINES)
	for r in rocks:
		var prev: Vector3 = r["pos"]
		for k in range(1, 7):
			var t := float(k) * 0.5
			var p: Vector3 = Predictor.predict(r["pos"], r["vel"], t, WORLD_BOUNDS_MIN, WORLD_BOUNDS_MAX)
			_ghost_line.surface_add_vertex(prev)
			_ghost_line.surface_add_vertex(p)
			prev = p
	_ghost_line.surface_end()

# ============================================================ build / sync sim
# Build a SimWorld from the current editable scene.
func _build_sim() -> SimWorld:
	var sim := SimWorld.new()
	sim.bounds_min = WORLD_BOUNDS_MIN
	sim.bounds_max = WORLD_BOUNDS_MAX
	sim.start_pos = _ship.global_position
	sim.goal_pos = _goal_pos
	sim.ship_radius = _ship.ship_radius_m
	sim.ship_mass = _ship.mass_kg
	sim.ship_max_thrust = _ship.max_thrust_n
	sim.ship_max_speed = _ship.max_speed_mps
	sim.ship_target_speed = _ship.target_speed_mps
	sim.planner_cfg = _ship.planner_cfg
	if _noise_spin != null:
		sim.noise_sigma = _noise_spin.value
	if _unc_check != null and _unc_check.button_pressed:
		sim.unc_enable = true
		sim.unc_sigma0 = 0.5
		sim.unc_growth = 1.5 * sim.noise_sigma
		sim.unc_ksigma = 3.0
	var rocks: Array[Dictionary] = []
	for a in _asteroids:
		rocks.append({"pos": a.global_position, "vel": a.velocity, "radius": a.radius_m, "mass": a.mass_kg})
	sim.asteroids = rocks
	return sim

# Rebuild the green preview path (EDIT mode) using the real planner.
func _replan_preview() -> void:
	var sim := _build_sim()
	sim.ship_pos = _ship.global_position
	sim.plan_path()
	_preview_path = sim.path
	_draw_path(_preview_path)
	var rocks: Array = []
	for a in _asteroids:
		rocks.append({"pos": a.global_position, "vel": a.velocity})
	_draw_ghosts(rocks)
	_update_ui()

# ============================================================ run lifecycle
func _toggle_run() -> void:
	_running = not _running
	_run_button.text = "Stop" if _running else "Go"
	if _running:
		_start_run()
	else:
		_stop_run()
	_update_ui()

func _start_run() -> void:
	_select(null)
	# Snapshot the design so we can restore it when the run ends.
	_restore_asteroids = []
	for a in _asteroids:
		_restore_asteroids.append({"pos": a.global_position, "vel": a.velocity})
	_sim = _build_sim()
	_sim.record = true   # every interactive flight is replayable in the web viewer
	_sim.reset_run()

func _stop_run() -> void:
	# Restore the designed scenario.
	if _restore_asteroids.size() == _asteroids.size():
		for i in range(_asteroids.size()):
			_asteroids[i].global_position = _restore_asteroids[i]["pos"]
			_asteroids[i].velocity = _restore_asteroids[i]["vel"]
	_ship.global_position = _start_pos
	_sim = null
	_replan_preview()

func _physics_process(dt: float) -> void:
	if not _running or _sim == null:
		return

	_sim.step(dt)

	# Sync visuals from the engine state.
	_ship.global_position = _sim.ship_pos
	_ship.face_velocity(_sim.ship_vel)
	for i in range(min(_asteroids.size(), _sim.asteroids.size())):
		_asteroids[i].global_position = _sim.asteroids[i]["pos"]
	_draw_path(_sim.path)
	_draw_ghosts(_sim.asteroids)
	_update_ui()

	if _sim.is_terminal():
		_running = false
		_run_button.text = "Go"
		if _sim.save_recording("res://last_flight.json"):
			print("Flight recording saved: ", ProjectSettings.globalize_path("res://last_flight.json"))
			_hint_label.text = (
				"Flight recorded -> last_flight.json\n"
				+ "Open tools/flight_viewer.html in a browser\n"
				+ "and load it to replay this exact flight\n"
				+ "with a time scrubber.")
		_update_ui()

# ============================================================ input
func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("toggle_run"):
		_toggle_run()
		return
	if event.is_action_pressed("add_asteroid"):
		_add_asteroid_at_cursor()
		return

	if not _running and event is InputEventKey:
		var k := event as InputEventKey
		if not k.pressed or k.echo:
			return
		if k.keycode == KEY_Q:
			_place_distance_m = max(10.0, _place_distance_m - PLACE_DISTANCE_STEP_M)
			_update_ui()
			return
		if k.keycode == KEY_E:
			_place_distance_m = min(500.0, _place_distance_m + PLACE_DISTANCE_STEP_M)
			_update_ui()
			return
		if k.keycode == KEY_B:
			_generate_belt(BELT_COUNT)
			return
		if k.keycode == KEY_C:
			_clear_asteroids()
			return
		if _selected != null and k.keycode == KEY_R:
			_selected.global_position.y += NUDGE_STEP_M
			_populate_inspector_from_selected()
			_replan_preview()
			return
		if _selected != null and k.keycode == KEY_F:
			_selected.global_position.y -= NUDGE_STEP_M
			_populate_inspector_from_selected()
			_replan_preview()
			return

	if event.is_action_pressed("select_object"):
		_handle_click(event)
		return

	if not _running and event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_LEFT and mb.alt_pressed:
			if mb.pressed and _selected != null:
				_dragging = true
				_drag_start_mouse = mb.position
				_drag_start_pos = _selected.global_position
				get_viewport().set_input_as_handled()
				return
			if not mb.pressed:
				_dragging = false
				return

	if not _running and event is InputEventMouseMotion:
		var mm := event as InputEventMouseMotion
		if _dragging and _selected != null:
			_drag_selected(mm.position)
			get_viewport().set_input_as_handled()
			return

func _handle_click(event: InputEvent) -> void:
	if _running:
		return
	var mouse_event := event as InputEventMouseButton
	var shift_down := false
	var ctrl_down := false
	if mouse_event:
		shift_down = mouse_event.shift_pressed
		ctrl_down = mouse_event.ctrl_pressed

	var mouse_pos := get_viewport().get_mouse_position()
	var ray_origin := _camera.project_ray_origin(mouse_pos)
	var ray_dir := _camera.project_ray_normal(mouse_pos)

	var space := get_world_3d().direct_space_state
	var query := PhysicsRayQueryParameters3D.new()
	query.from = ray_origin
	query.to = ray_origin + ray_dir * 2000.0
	query.collide_with_areas = true
	query.collide_with_bodies = true
	var hit := space.intersect_ray(query)

	if shift_down:
		# Clamp: a click that misses the floor lands 90 m out along the camera
		# ray, which can be far outside the world -> the planner clamps to the
		# nearest grid edge and the corridor "runs off the screen".
		_goal_pos = _clamp_to_bounds(_hit_to_world(hit, ray_origin, ray_dir))
		_goal_marker.global_position = _goal_pos
		_replan_preview()
		return
	if ctrl_down:
		_start_pos = _clamp_to_bounds(_hit_to_world(hit, ray_origin, ray_dir))
		_ship.global_position = _start_pos
		_start_marker.global_position = _start_pos
		_replan_preview()
		return

	if hit and hit.has("collider"):
		var collider: Object = hit["collider"]
		var node: Node = collider as Node
		while node and not (node is Ship or node is Asteroid):
			node = node.get_parent()
		_select(node as Node3D)
		return

	_select(null)

func _hit_to_world(hit: Dictionary, ray_origin: Vector3, ray_dir: Vector3) -> Vector3:
	if hit and hit.has("position"):
		return hit["position"]
	return ray_origin + ray_dir * _place_distance_m

# ============================================================ asteroid editing
func _add_asteroid_at_cursor() -> void:
	if _running:
		return
	var p := _point_under_mouse_3d()
	var a := _spawn_asteroid(p, Vector3.ZERO, 2.0, 1500.0)
	_select(a)
	_replan_preview()

func _spawn_asteroid(pos: Vector3, vel: Vector3, radius: float, mass: float) -> Asteroid:
	var a := asteroid_scene.instantiate() as Asteroid
	add_child(a)
	a.global_position = _clamp_to_bounds(pos)
	a.velocity = vel
	a.mass_kg = mass
	a.set_radius_m(radius)
	_asteroids.append(a)
	return a

func _generate_belt(n: int) -> void:
	if _running:
		return
	_clear_asteroids()
	var rocks := SimWorld.random_belt(n, _rng, _start_pos, _goal_pos, WORLD_BOUNDS_MIN, WORLD_BOUNDS_MAX)
	for r in rocks:
		_spawn_asteroid(r["pos"], r["vel"], r["radius"], r["mass"])
	_select(null)
	_replan_preview()

func _clear_asteroids() -> void:
	for a in _asteroids:
		a.queue_free()
	_asteroids.clear()
	_select(null)
	_replan_preview()

func _point_under_mouse_3d() -> Vector3:
	var mouse_pos := get_viewport().get_mouse_position()
	var ray_origin := _camera.project_ray_origin(mouse_pos)
	var ray_dir := _camera.project_ray_normal(mouse_pos)
	var space := get_world_3d().direct_space_state
	var query := PhysicsRayQueryParameters3D.new()
	query.from = ray_origin
	query.to = ray_origin + ray_dir * 5000.0
	query.collide_with_areas = true
	query.collide_with_bodies = true
	var hit := space.intersect_ray(query)
	if hit and hit.has("position"):
		return hit["position"]
	return ray_origin + ray_dir * _place_distance_m

func _select(node: Node3D) -> void:
	if _selected != null and _selected.has_method("set_selected"):
		_selected.call("set_selected", false)
	_selected = node
	if _selected == null:
		_selected_label.text = "Selected: none"
		return
	_selected_label.text = "Selected: %s" % _selected.name
	if _selected.has_method("set_selected"):
		_selected.call("set_selected", true)
	$CameraRig.focus(_selected.global_position)
	_populate_inspector_from_selected()

func _drag_selected(mouse_pos: Vector2) -> void:
	var ray_origin := _camera.project_ray_origin(mouse_pos)
	var ray_dir := _camera.project_ray_normal(mouse_pos)
	var plane := Plane(-_camera.global_transform.basis.z, _drag_start_pos)
	var denom := plane.normal.dot(ray_dir)
	if abs(denom) < 1e-5:
		return
	var t := -(plane.normal.dot(ray_origin) + plane.d) / denom
	if t < 0.0:
		return
	var hit := ray_origin + ray_dir * t
	_selected.global_position = _clamp_to_bounds(hit)
	_populate_inspector_from_selected()
	_replan_preview()

func _populate_inspector_from_selected() -> void:
	if _selected == null or not _selected.has_method("editor_get_position"):
		return
	var p: Vector3 = _selected.call("editor_get_position")
	_pos_x.value = p.x
	_pos_y.value = p.y
	_pos_z.value = p.z
	if _selected.has_method("editor_get_velocity"):
		var v: Vector3 = _selected.call("editor_get_velocity")
		_vel_x.value = v.x
		_vel_y.value = v.y
		_vel_z.value = v.z
	if _selected.has_method("editor_get_mass_kg"):
		_mass.value = float(_selected.call("editor_get_mass_kg"))
	if _selected.has_method("editor_get_radius_m"):
		_radius.value = float(_selected.call("editor_get_radius_m"))

func _apply_inspector_to_selected() -> void:
	if _selected == null:
		return
	if _selected.has_method("editor_set_position"):
		_selected.call("editor_set_position", Vector3(_pos_x.value, _pos_y.value, _pos_z.value))
	if _selected.has_method("editor_set_velocity"):
		_selected.call("editor_set_velocity", Vector3(_vel_x.value, _vel_y.value, _vel_z.value))
	if _selected.has_method("editor_set_mass_kg"):
		_selected.call("editor_set_mass_kg", float(_mass.value))
	if _selected.has_method("editor_set_radius_m"):
		_selected.call("editor_set_radius_m", float(_radius.value))
	_replan_preview()

# ============================================================ save / load
func _on_save_pressed() -> void:
	if _running:
		return
	_file_mode = "save"
	_file_dialog.file_mode = FileDialog.FILE_MODE_SAVE_FILE
	_file_dialog.current_file = "scenario.json"
	_file_dialog.popup_centered_ratio(0.6)

func _on_load_pressed() -> void:
	if _running:
		return
	_file_mode = "load"
	_file_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	_file_dialog.popup_centered_ratio(0.6)

func _on_file_selected(path: String) -> void:
	if _file_mode == "save":
		var ok := SimWorld.save_to_file(path, _build_sim().to_scenario())
		print("Save scenario -> ", path, "  ok=", ok)
	elif _file_mode == "load":
		var d := SimWorld.load_from_file(path)
		if d.is_empty():
			push_error("Failed to load scenario: " + path)
			return
		_apply_scenario(d)
		print("Loaded scenario <- ", path)

func _apply_scenario(d: Dictionary) -> void:
	var tmp := SimWorld.new()
	tmp.load_scenario(d)
	_clear_asteroids()
	_start_pos = tmp.start_pos
	_goal_pos = tmp.goal_pos
	_ship.global_position = _start_pos
	_ship.ship_radius_m = tmp.ship_radius
	_ship.mass_kg = tmp.ship_mass
	_ship.max_thrust_n = tmp.ship_max_thrust
	_ship.max_speed_mps = tmp.ship_max_speed
	_ship.target_speed_mps = tmp.ship_target_speed
	_ship.planner_cfg = tmp.planner_cfg
	_start_marker.global_position = _start_pos
	_goal_marker.global_position = _goal_pos
	for r in tmp.asteroids:
		_spawn_asteroid(r["pos"], r["vel"], r["radius"], r["mass"])
	_select(null)
	_replan_preview()

# ============================================================ rendering helpers
func _clamp_to_bounds(p: Vector3) -> Vector3:
	return Vector3(
		clampf(p.x, WORLD_BOUNDS_MIN.x, WORLD_BOUNDS_MAX.x),
		clampf(p.y, WORLD_BOUNDS_MIN.y, WORLD_BOUNDS_MAX.y),
		clampf(p.z, WORLD_BOUNDS_MIN.z, WORLD_BOUNDS_MAX.z)
	)

func _draw_path(points: Array[Vector3]) -> void:
	_path_line.clear_surfaces()
	if points.size() < 2:
		return
	_path_line.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
	for p in points:
		_path_line.surface_add_vertex(p)
	_path_line.surface_end()

func _update_ui() -> void:
	_mode_label.text = "Mode: %s" % ("RUN" if _running else "EDIT")

	if _sim != null:
		var clr := "n/a" if _sim.min_clearance == INF else "%.1f m" % _sim.min_clearance
		_metrics_label.text = (
			"Metrics:\n"
			+ "- Status: %s\n" % _sim.status
			+ "- Time: %.1f s\n" % _sim.time
			+ "- Speed: %.1f m/s\n" % _sim.ship_vel.length()
			+ "- Remaining: %.1f m\n" % _sim._remaining_path_distance()
			+ "- Min clearance: %s\n" % clr
			+ "- Collisions: %d\n" % _sim.collisions
			+ "- Δv used: %.0f\n" % _sim.dv_used
			+ "- Asteroids: %d   Replans: %d\n" % [_asteroids.size(), _sim.replans]
			+ "- Plan: %.1f ms (max %.1f)   fails: %d%s\n" % [
				_sim.plan_ms_last, _sim.plan_ms_max, _sim.plan_fail_count,
				"   ⚠ DEGRADED" if _sim.degraded else ""]
			+ "- Planner: time-indexed A* + 3D DWA (swept)"
		)
	else:
		var preview_len := 0.0
		for i in range(1, _preview_path.size()):
			preview_len += _preview_path[i - 1].distance_to(_preview_path[i])
		_metrics_label.text = (
			"Metrics (EDIT):\n"
			+ "- Preview path: %.1f m\n" % preview_len
			+ "- Asteroids: %d\n" % _asteroids.size()
			+ "- Planner: time-indexed A* + 3D DWA (swept)\n"
			+ "- PlaceDist: %.1f m (Q/E)" % _place_distance_m
		)
