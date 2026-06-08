extends Node3D

# Scene orchestrator + interactive testbed.
#
# Responsibilities:
#   - Owns the world (ship, asteroids, bounds) and the EDIT/RUN mode.
#   - EDIT mode: place/select/drag asteroids, set start & goal, tune metrics.
#   - RUN mode: moves the asteroids, runs the global planner (Voxel A* over a
#     PREDICTED occupancy grid), feeds the live world to the ship's local
#     planner, detects collisions, and reports flight metrics.
#
# The autopilot is split in two layers, exactly like a real guidance system:
#   * Global planner (VoxelAStar, here)      -> a coarse, safe corridor.
#   * Local planner (LocalPlanner3D, in Ship) -> fast reactive avoidance of
#                                                moving asteroids.

@export var ship_scene: PackedScene
@export var asteroid_scene: PackedScene

const WORLD_BOUNDS_MIN := Vector3(-90, 0, -90)
const WORLD_BOUNDS_MAX := Vector3(90, 60, 90)
const WORLD_ORIGIN := WORLD_BOUNDS_MIN
const VOXEL_SIZE_M := 3.0

const REPLAN_INTERVAL_S := 1.0      # how often the global A* corridor refreshes
# Times (s) ahead at which we stamp predicted asteroid positions into the grid,
# carving a corridor around their swept path for the global planner.
const PREDICT_SAMPLE_TIMES := [0.0, 0.5, 1.0, 1.5, 2.0]

const GRID_SIZE := WORLD_BOUNDS_MAX - WORLD_BOUNDS_MIN
const GRID_CELLS_X := int(ceil(GRID_SIZE.x / VOXEL_SIZE_M))
const GRID_CELLS_Y := int(ceil(GRID_SIZE.y / VOXEL_SIZE_M))
const GRID_CELLS_Z := int(ceil(GRID_SIZE.z / VOXEL_SIZE_M))

const DEFAULT_PLACE_DISTANCE_M := 90.0
const PLACE_DISTANCE_STEP_M := 5.0
const NUDGE_STEP_M := 1.0

const BELT_COUNT := 18              # asteroids spawned by the belt generator (B)

var _running := false
var _replan_timer := 0.0
var _replans := 0

var _ship: Ship
var _asteroids: Array[Asteroid] = []

var _start_pos: Vector3 = Vector3(-60, 10, -60)
var _goal_pos: Vector3 = Vector3(60, 10, 60)

var _selected: Node3D = null
var _place_distance_m: float = DEFAULT_PLACE_DISTANCE_M

var _dragging := false
var _drag_start_mouse: Vector2
var _drag_start_pos: Vector3

# Flight metrics (per run).
var _run_time := 0.0
var _collisions := 0
var _min_clearance := INF
var _contact := {}                 # asteroid instance ids currently in contact
var _status := "EDIT"

@onready var _camera: Camera3D = $CameraRig/Pivot/Camera3D
@onready var _start_marker: Node3D = $StartMarker
@onready var _goal_marker: Node3D = $GoalMarker

@onready var _run_button: Button = $UI/Panel/VBox/RunButton
@onready var _add_asteroid_button: Button = $UI/Panel/VBox/AddAsteroidButton
@onready var _mode_label: Label = $UI/Panel/VBox/ModeLabel
@onready var _selected_label: Label = $UI/Panel/VBox/SelectedLabel
@onready var _metrics_label: Label = $UI/Panel/VBox/MetricsLabel
@onready var _hint_label: Label = $UI/Panel/VBox/HintLabel

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

func _ready() -> void:
	randomize()
	_run_button.pressed.connect(_toggle_run)
	_add_asteroid_button.pressed.connect(_add_asteroid_at_cursor)
	_apply_button.pressed.connect(_apply_inspector_to_selected)

	_ship = ship_scene.instantiate() as Ship
	add_child(_ship)
	_ship.global_position = _start_pos
	_ship.set_running(false)
	_ship.set_world_bounds(WORLD_BOUNDS_MIN, WORLD_BOUNDS_MAX)
	_start_marker.global_position = _start_pos
	_goal_marker.global_position = _goal_pos

	_hint_label.text = "LMB: select   Shift+LMB: Goal   Ctrl+LMB: Start\nB: random belt   C: clear   Q/E: depth\nAlt+drag: move   R/F: nudge selected up/down"

	_setup_path_debug()
	_update_ui()
	_plan_and_set_path()

func _setup_path_debug() -> void:
	_path_line = ImmediateMesh.new()
	_path_mesh_instance = MeshInstance3D.new()
	_path_mesh_instance.mesh = _path_line
	_path_mesh_instance.material_override = StandardMaterial3D.new()
	_path_mesh_instance.material_override.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_path_mesh_instance.material_override.albedo_color = Color(0.1, 1.0, 0.5, 1.0)
	add_child(_path_mesh_instance)

func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("toggle_run"):
		_toggle_run()
		return
	if event.is_action_pressed("add_asteroid"):
		_add_asteroid_at_cursor()
		return

	# Edit-mode helpers
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
			_plan_and_set_path()
			return
		if _selected != null and k.keycode == KEY_F:
			_selected.global_position.y -= NUDGE_STEP_M
			_populate_inspector_from_selected()
			_plan_and_set_path()
			return

	if event.is_action_pressed("select_object"):
		_handle_click(event)
		return

	# Alt+LMB drag selected object in edit mode
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
		_goal_pos = _hit_to_world(hit, ray_origin, ray_dir)
		_goal_marker.global_position = _goal_pos
		_plan_and_set_path()
		return
	if ctrl_down:
		_start_pos = _hit_to_world(hit, ray_origin, ray_dir)
		_ship.global_position = _start_pos
		_ship.velocity = Vector3.ZERO
		_start_marker.global_position = _start_pos
		_plan_and_set_path()
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

func _toggle_run() -> void:
	_running = not _running
	_ship.set_running(_running)
	_run_button.text = "Stop" if _running else "Go"
	_replan_timer = 0.0
	if _running:
		# Fresh run: reset metrics and ship to the start.
		_replans = 0
		_run_time = 0.0
		_collisions = 0
		_min_clearance = INF
		_contact.clear()
		_ship.global_position = _start_pos
		_ship.velocity = Vector3.ZERO
		_plan_and_set_path()
	_update_ui()

func _add_asteroid_at_cursor() -> void:
	if _running:
		return
	var p := _point_under_mouse_3d()
	var a := _spawn_asteroid(p, Vector3.ZERO, 2.0, 1500.0)
	_select(a)
	_plan_and_set_path()

func _spawn_asteroid(pos: Vector3, vel: Vector3, radius: float, mass: float) -> Asteroid:
	var a := asteroid_scene.instantiate() as Asteroid
	add_child(a)
	a.global_position = _clamp_to_bounds(pos)
	a.velocity = vel
	a.mass_kg = mass
	a.set_radius_m(radius)
	_asteroids.append(a)
	return a

# Scatter a field of moving asteroids (the headline test scenario).
func _generate_belt(n: int) -> void:
	if _running:
		return
	_clear_asteroids()
	var attempts := 0
	while _asteroids.size() < n and attempts < n * 40:
		attempts += 1
		var pos := Vector3(
			randf_range(WORLD_BOUNDS_MIN.x + 5.0, WORLD_BOUNDS_MAX.x - 5.0),
			randf_range(WORLD_BOUNDS_MIN.y + 5.0, WORLD_BOUNDS_MAX.y - 5.0),
			randf_range(WORLD_BOUNDS_MIN.z + 5.0, WORLD_BOUNDS_MAX.z - 5.0)
		)
		# Keep the start and goal regions clear so the run is solvable.
		if pos.distance_to(_start_pos) < 18.0 or pos.distance_to(_goal_pos) < 18.0:
			continue
		var radius := randf_range(1.5, 4.0)
		var speed := randf_range(4.0, 18.0)
		var vel := _random_unit() * speed
		_spawn_asteroid(pos, vel, radius, radius * 500.0)
	_select(null)
	_plan_and_set_path()
	_update_ui()

func _clear_asteroids() -> void:
	for a in _asteroids:
		a.queue_free()
	_asteroids.clear()
	_select(null)
	_plan_and_set_path()
	_update_ui()

func _random_unit() -> Vector3:
	var v := Vector3(randf() - 0.5, randf() - 0.5, randf() - 0.5)
	if v.length() < 1e-4:
		return Vector3.RIGHT
	return v.normalized()

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
	_plan_and_set_path()

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
	_plan_and_set_path()

func _physics_process(dt: float) -> void:
	if not _running:
		return

	_run_time += dt

	# 1. Move the asteroids (constant velocity + reflect at bounds).
	for a in _asteroids:
		a.integrate(dt, WORLD_BOUNDS_MIN, WORLD_BOUNDS_MAX)

	# 2. Hand the live world to the ship's local planner.
	_ship.set_obstacles(_obstacle_snapshot())
	_ship.set_world_bounds(WORLD_BOUNDS_MIN, WORLD_BOUNDS_MAX)

	# 3. Periodically refresh the global A* corridor against predicted occupancy.
	_replan_timer += dt
	if _replan_timer >= REPLAN_INTERVAL_S:
		_plan_and_set_path()
		_replan_timer = 0.0
		_replans += 1

	# 4. Fly the ship (reactive local avoidance happens inside step_sim).
	_ship.step_sim(dt)

	# 5. Measure safety + progress.
	_update_metrics()
	_update_ui()

func _obstacle_snapshot() -> Array:
	var out: Array = []
	for a in _asteroids:
		out.append({"pos": a.global_position, "vel": a.velocity, "radius": a.radius_m})
	return out

func _update_metrics() -> void:
	var nearest := INF
	for a in _asteroids:
		var d := _ship.global_position.distance_to(a.global_position) - (_ship.ship_radius_m + a.radius_m)
		nearest = minf(nearest, d)
		var id := a.get_instance_id()
		if d <= 0.0:
			if not _contact.has(id):
				_contact[id] = true
				_collisions += 1
		else:
			_contact.erase(id)
	if nearest != INF:
		_min_clearance = minf(_min_clearance, nearest)

	if _collisions > 0:
		_status = "COLLISION"
	elif _ship.has_arrived():
		_status = "ARRIVED"
		_running = false
		_ship.set_running(false)
		_run_button.text = "Go"
	else:
		_status = "FLYING"

func _plan_and_set_path() -> void:
	_ship.global_position = _clamp_to_bounds(_ship.global_position)
	_goal_pos = _clamp_to_bounds(_goal_pos)
	_start_marker.global_position = _ship.global_position
	_goal_marker.global_position = _goal_pos

	var start_cell := _world_to_cell(_ship.global_position)
	var goal_cell := _world_to_cell(_goal_pos)

	var blocked := _build_predicted_occupancy()
	var is_free := func(c: Vector3i) -> bool:
		return _cell_is_free(c, blocked)

	var cell_path: Array[Vector3i] = VoxelAStar.plan(start_cell, goal_cell, is_free, 120000)
	var world_path: Array[Vector3] = []
	for c in cell_path:
		world_path.append(_cell_to_world(c))

	_ship.set_waypoints(world_path)
	_draw_path(world_path)
	_update_ui()

# Stamp each asteroid's PREDICTED positions (over the look-ahead) into the grid,
# padded by the combined radius. This carves a corridor around moving rocks so
# the global path already leans away from them; the local planner does the rest.
func _build_predicted_occupancy() -> Dictionary:
	var blocked := {}
	for a in _asteroids:
		var pad := int(ceil((a.radius_m + _ship.ship_radius_m) / VOXEL_SIZE_M))
		pad = clampi(pad, 1, 6)
		for t in PREDICT_SAMPLE_TIMES:
			var fp: Vector3 = Predictor.predict(a.global_position, a.velocity, float(t), WORLD_BOUNDS_MIN, WORLD_BOUNDS_MAX)
			var c := _world_to_cell(fp)
			for dx in range(-pad, pad + 1):
				for dy in range(-pad, pad + 1):
					for dz in range(-pad, pad + 1):
						var cc := Vector3i(c.x + dx, c.y + dy, c.z + dz)
						if _cell_in_bounds(cc):
							blocked[cc] = true
	return blocked

func _cell_is_free(c: Vector3i, blocked: Dictionary) -> bool:
	if not _cell_in_bounds(c):
		return false
	if blocked.has(c):
		return false
	return true

func _cell_in_bounds(c: Vector3i) -> bool:
	return (
		c.x >= 0 and c.x < GRID_CELLS_X
		and c.y >= 0 and c.y < GRID_CELLS_Y
		and c.z >= 0 and c.z < GRID_CELLS_Z
	)

func _clamp_to_bounds(p: Vector3) -> Vector3:
	return Vector3(
		clampf(p.x, WORLD_BOUNDS_MIN.x, WORLD_BOUNDS_MAX.x),
		clampf(p.y, WORLD_BOUNDS_MIN.y, WORLD_BOUNDS_MAX.y),
		clampf(p.z, WORLD_BOUNDS_MIN.z, WORLD_BOUNDS_MAX.z)
	)

func _world_to_cell(p: Vector3) -> Vector3i:
	var lp := p - WORLD_ORIGIN
	var cx := int(floor(lp.x / VOXEL_SIZE_M))
	var cy := int(floor(lp.y / VOXEL_SIZE_M))
	var cz := int(floor(lp.z / VOXEL_SIZE_M))
	cx = clampi(cx, 0, GRID_CELLS_X - 1)
	cy = clampi(cy, 0, GRID_CELLS_Y - 1)
	cz = clampi(cz, 0, GRID_CELLS_Z - 1)
	return Vector3i(cx, cy, cz)

func _cell_to_world(c: Vector3i) -> Vector3:
	return WORLD_ORIGIN + Vector3(
		(float(c.x) + 0.5) * VOXEL_SIZE_M,
		(float(c.y) + 0.5) * VOXEL_SIZE_M,
		(float(c.z) + 0.5) * VOXEL_SIZE_M
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

	var path := _ship.get_waypoints()
	var remaining := 0.0
	if path.size() >= 2:
		var idx := _ship.get_waypoint_index()
		var prev := _ship.global_position
		for i in range(idx, path.size()):
			remaining += prev.distance_to(path[i])
			prev = path[i]

	var clr := "n/a" if _min_clearance == INF else "%.1f m" % _min_clearance
	var speed := _ship.velocity.length()

	_metrics_label.text = (
		"Metrics:\n"
		+ "- Status: %s\n" % _status
		+ "- Time: %.1f s\n" % _run_time
		+ "- Speed: %.1f m/s\n" % speed
		+ "- Remaining: %.1f m\n" % remaining
		+ "- Min clearance: %s\n" % clr
		+ "- Collisions: %d\n" % _collisions
		+ "- Δv used: %.0f\n" % _ship.get_delta_v_used()
		+ "- Asteroids: %d   Replans: %d\n" % [_asteroids.size(), _replans]
		+ "- Planner: A* corridor + 3D DWA\n"
		+ "- PlaceDist: %.1f m (Q/E)" % _place_distance_m
	)
