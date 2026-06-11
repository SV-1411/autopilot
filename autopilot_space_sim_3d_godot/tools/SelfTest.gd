extends SceneTree

# Deterministic invariant suite for the autopilot.
#
# Every defect class found in this project gets a test here so it can never
# silently return: collision tunneling, predictor/integrator drift, blind-brake
# panic, union-planner false refusals, waypoint/arrival edge cases, scenario
# persistence. Run via tools/run_headless.ps1 (which import-gates and applies a
# hard timeout). Exits 0 only if every test passes.
#
#   powershell -File tools\run_headless.ps1 -Script res://tools/SelfTest.gd

const SW := preload("res://scripts/SimWorld.gd")
const LP := preload("res://scripts/planning/LocalPlanner3D.gd")
const PR := preload("res://scripts/planning/Predictor.gd")
const STA := preload("res://scripts/planning/SpaceTimeAStar.gd")
const VA := preload("res://scripts/planning/VoxelAStar.gd")

const DT := 1.0 / 60.0

var _fails := 0

func _init() -> void:
	print("BOOT_OK")
	print("=== Autopilot self-test ===")

	_run("seg_origin_dist_basics", _t_seg_origin_dist)
	_run("empty_world_arrives", _t_empty_world)
	_run("solid_wall_refusal", _t_solid_wall)
	_run("closing_gap_4d_only", _t_closing_gap)
	_run("tunneling_caught_lookahead", _t_tunnel_lookahead)
	_run("tunneling_caught_ground_truth", _t_tunnel_metrics)
	_run("predictor_equals_integrator", _t_predictor_consistency)
	_run("least_bad_lateral_dodge", _t_lateral_dodge)
	_run("start_equals_goal", _t_start_equals_goal)
	_run("arrival_governor", _t_arrival)
	_run("goal_exactness", _t_goal_exactness)
	_run("stale_path_guard", _t_stale_path_guard)
	_run("scenario_roundtrip", _t_roundtrip)

	if _fails == 0:
		print("\nALL TESTS PASSED")
		quit(0)
	else:
		print("\n%d TEST(S) FAILED" % _fails)
		quit(1)

func _run(name: String, fn: Callable) -> void:
	var t0 := Time.get_ticks_msec()
	var err: String = fn.call()
	var ms := Time.get_ticks_msec() - t0
	if err == "":
		print("PASS  %-32s (%d ms)" % [name, ms])
	else:
		_fails += 1
		print("FAIL  %-32s (%d ms): %s" % [name, ms, err])

# ------------------------------------------------------------------ helpers

func _mk_sim(rocks: Array[Dictionary], start := Vector3(-60, 10, -60),
		goal := Vector3(60, 10, 60)) -> SimWorld:
	var sim: SimWorld = SW.new()
	sim.start_pos = start
	sim.goal_pos = goal
	sim.asteroids = rocks
	return sim

func _run_to_terminal(sim: SimWorld) -> void:
	sim.reset_run()
	var guard := int(sim.max_time / DT) + 1000
	while not sim.is_terminal() and guard > 0:
		sim.step(DT)
		guard -= 1

# A wall of static rocks across the full x=0 cross-section, optionally with a
# hole (centers within hole_r of hole_yz omitted).
func _wall(hole_yz := Vector2.INF, hole_r := 0.0) -> Array[Dictionary]:
	var rocks: Array[Dictionary] = []
	for y in range(3, 58, 9):
		for z in range(-87, 88, 9):
			if hole_yz != Vector2.INF and Vector2(y, z).distance_to(hole_yz) <= hole_r:
				continue
			rocks.append({"pos": Vector3(0, y, z), "vel": Vector3.ZERO,
				"radius": 6.0, "mass": 1000.0})
	return rocks

# ------------------------------------------------------------------ tests

func _t_seg_origin_dist() -> String:
	# Segment passing through the origin -> 0; static point -> plain distance.
	var d1: float = LP._seg_origin_dist(Vector3(-5, 0, 0), Vector3(5, 0, 0))
	if absf(d1) > 1e-5:
		return "through-origin segment gave %f, want 0" % d1
	var d2: float = LP._seg_origin_dist(Vector3(3, 4, 0), Vector3(3, 4, 0))
	if absf(d2 - 5.0) > 1e-5:
		return "degenerate segment gave %f, want 5" % d2
	var d3: float = LP._seg_origin_dist(Vector3(10, 2, 0), Vector3(-10, 2, 0))
	if absf(d3 - 2.0) > 1e-5:
		return "offset segment gave %f, want 2" % d3
	return ""

func _t_empty_world() -> String:
	var sim := _mk_sim([] as Array[Dictionary])
	_run_to_terminal(sim)
	if sim.status != "ARRIVED":
		return "status %s, want ARRIVED" % sim.status
	if sim.dv_used <= 0.0:
		return "dv_used %.1f, want > 0" % sim.dv_used
	return ""

func _t_solid_wall() -> String:
	var sim := _mk_sim(_wall())
	sim.reset_run()
	if sim.status != "NO_PATH":
		return "status %s, want NO_PATH (no gap exists)" % sim.status
	return ""

func _t_closing_gap() -> String:
	# Wall with a mid-air hole (the 2x2 block of centers around (34.5, 61.5)
	# removed -- small enough that one parked rock plugs it completely); the
	# plug rock slides away along -z within the wall plane, clearing the hole in
	# ~0.6 s and not returning inside the prediction window or before the ship
	# crosses. The union over all time layers keeps the hole blocked -> the
	# legacy 3D planner must refuse; the time-indexed planner must thread it.
	# (The hole must NOT touch the floor or ceiling, or a path sneaks around.)
	var rocks := _wall(Vector2(34.5, 61.5), 10.0)
	rocks.append({"pos": Vector3(0, 34.5, 61.5), "vel": Vector3(0, 0, -30),
		"radius": 6.0, "mass": 1000.0})
	var sim := _mk_sim(rocks)
	# A full wall forces the search to flood half the grid -- a legitimate
	# stress case, so grant a stress budget (flight default is tuned for belts).
	sim.plan_deadline_usec = 2_000_000

	var layers: Array = sim._build_time_layers()
	var dims := Vector3i(sim.cells_x(), sim.cells_y(), sim.cells_z())
	var start_cell := sim.world_to_cell(sim.start_pos)
	var goal_cell := sim.world_to_cell(sim.goal_pos)

	var union_path: Array = sim._plan_static_fallback(start_cell, goal_cell, layers, dims)
	if not union_path.is_empty():
		return "union planner found a path; scenario does not isolate the 4D case"

	# reset_run() seeds ship_pos from start_pos and plans; plan_path() alone
	# would plan from the uninitialized ship position.
	sim.reset_run()
	if sim.status == "NO_PATH":
		return "time-indexed planner refused a gap that opens in time"

	_run_to_terminal(sim)
	if sim.status != "ARRIVED":
		return "run ended %s (clearance %.2f), want ARRIVED" % [sim.status, sim.min_clearance]
	return ""

func _t_tunnel_lookahead() -> String:
	# Ship at 200 m/s; sample points 40 m apart straddle a small rock dead
	# ahead. Endpoint-only checks miss it; the swept check must not.
	var obstacles: Array = [
		{"pos": Vector3(30, 10, 0), "vel": Vector3.ZERO, "radius": 0.5}
	]
	var bmin := Vector3(-90, 0, -90)
	var bmax := Vector3(90, 60, 90)
	var frames: Array = LP._predict_frames(obstacles, 5, 0.2, bmin, bmax, false, 0, 0, 0)
	var res: Dictionary = LP._simulate(Vector3(0, 10, 0), Vector3(200, 0, 0), frames, 1.5, 0.2, 5)
	if not res["collides"]:
		return "swept look-ahead missed a rock straddled between samples"

	# Control: the same pass 15 m above the rock must NOT collide.
	var res2: Dictionary = LP._simulate(Vector3(0, 25, 0), Vector3(200, 0, 0), frames, 1.5, 0.2, 5)
	if res2["collides"]:
		return "false positive on a clear 15 m offset pass"
	return ""

func _t_tunnel_metrics() -> String:
	# Ground truth: ship crosses a small rock within ONE metrics update; the
	# swept metric must count the hit even though both endpoints are clear.
	var rocks: Array[Dictionary] = [
		{"pos": Vector3(10, 10, 0), "vel": Vector3.ZERO, "radius": 0.5, "mass": 1.0}
	]
	var sim := _mk_sim(rocks)
	sim.ship_pos = Vector3(20, 10, 0)
	sim._update_metrics(Vector3(0, 10, 0), [Vector3(10, 10, 0)])
	if sim.collisions != 1:
		return "swept ground truth counted %d collisions, want 1" % sim.collisions
	if sim.min_clearance > 0.0:
		return "min_clearance %.2f, want <= 0" % sim.min_clearance
	return ""

func _t_predictor_consistency() -> String:
	# An asteroid bouncing around the box for 30 s, integrated step by step,
	# must match the closed-form predictor at every second. This is THE
	# invariant that makes look-ahead trustworthy.
	var sim := _mk_sim([] as Array[Dictionary])
	var p0 := Vector3(0, 30, 0)
	var v0 := Vector3(37, 23, 41)
	var a := {"pos": p0, "vel": v0, "radius": 2.0, "mass": 1.0}
	var worst := 0.0
	for i in range(1, 1801):
		sim._integrate_asteroid(a, DT)
		if i % 60 == 0:
			var t := float(i) * DT
			var pred: Vector3 = PR.predict(p0, v0, t, sim.bounds_min, sim.bounds_max)
			worst = maxf(worst, (a["pos"] as Vector3).distance_to(pred))
	if worst > 0.2:
		return "integrator drifted %.3f m from prediction over 30 s (want < 0.2)" % worst
	return ""

func _t_lateral_dodge() -> String:
	# Big rock dead astern, overtaking fast; lateral space open. The planner
	# must dodge sideways, not brake into the pursuer.
	var obstacles: Array = [
		{"pos": Vector3(-40, 30, 0), "vel": Vector3(80, 0, 0), "radius": 6.0}
	]
	var accel: Vector3 = LP.plan(
		Vector3(0, 30, 0), Vector3(50, 0, 0), Vector3(80, 30, 0), 200.0,
		obstacles, 1.5, 31.25, 120.0,
		Vector3(-90, 0, -90), Vector3(90, 60, 90), {})
	var perp := sqrt(accel.y * accel.y + accel.z * accel.z)
	if perp < 0.3 * 31.25:
		return "dodge accel (%.1f, %.1f, %.1f): lateral %.1f m/s^2 too weak -- braking into pursuer?" % [
			accel.x, accel.y, accel.z, perp]
	return ""

func _t_start_equals_goal() -> String:
	var sim := _mk_sim([] as Array[Dictionary], Vector3(30, 10, 30), Vector3(30, 10, 30))
	sim.reset_run()
	if sim.status == "NO_PATH":
		return "degenerate start==goal refused"
	sim.step(DT)
	if sim.status != "ARRIVED":
		return "status %s after first step, want ARRIVED" % sim.status
	return ""

func _t_arrival() -> String:
	# Short hop: must arrive, settle, and never ping-pong past max_time.
	var sim := _mk_sim([] as Array[Dictionary], Vector3(30, 10, 30), Vector3(50, 10, 50))
	_run_to_terminal(sim)
	if sim.status != "ARRIVED":
		return "status %s, want ARRIVED" % sim.status
	if sim.time > 15.0:
		return "took %.1f s for a 28 m hop, want < 15" % sim.time
	return ""

func _t_goal_exactness() -> String:
	# The planned path must end at the exact goal, not the goal cell's center
	# (which can be off by half a voxel diagonal, ~2.6 m).
	var sim := _mk_sim([] as Array[Dictionary])
	sim.reset_run()
	if sim.path.is_empty():
		return "no path planned"
	var tail: Vector3 = sim.path[sim.path.size() - 1]
	if tail.distance_to(sim.goal_pos) > 0.01:
		return "path ends %.2f m from the goal" % tail.distance_to(sim.goal_pos)
	return ""

func _t_stale_path_guard() -> String:
	# Force total replan blockage mid-flight: the guard must drop the stale
	# path, beeline to the goal, and raise the degraded telemetry flag.
	var sim := _mk_sim([] as Array[Dictionary])
	sim.reset_run()
	if sim.status != "FLYING":
		return "setup failed: %s" % sim.status
	var dims := Vector3i(sim.cells_x(), sim.cells_y(), sim.cells_z())
	var blocked := PackedByteArray()
	blocked.resize(dims.x * dims.y * dims.z)
	blocked.fill(1)
	sim._guard_stale_path([blocked], dims)
	if not sim.degraded:
		return "guard did not enter degraded mode under total blockage"
	if sim.path.size() != 1 or sim.path[0].distance_to(sim.goal_pos) > 1e-3:
		return "degraded path is not the goal beeline (size %d)" % sim.path.size()
	# A successful plan afterwards must clear the flag.
	sim.plan_path()
	if sim.degraded:
		return "degraded flag not cleared by a successful replan"
	return ""

func _t_roundtrip() -> String:
	var rocks: Array[Dictionary] = [
		{"pos": Vector3(1, 2, 3), "vel": Vector3(-4, 5, -6), "radius": 2.5, "mass": 1250.0},
		{"pos": Vector3(-7, 8, 9), "vel": Vector3(0.5, -1.5, 2.5), "radius": 3.75, "mass": 1875.0},
	]
	var sim := _mk_sim(rocks)
	var d1: Dictionary = sim.to_scenario()
	var text := JSON.stringify(d1)
	var parsed = JSON.parse_string(text)
	if not (parsed is Dictionary):
		return "scenario JSON failed to parse"
	var sim2: SimWorld = SW.new()
	sim2.load_scenario(parsed)
	var d2: Dictionary = sim2.to_scenario()
	var s1 := JSON.stringify(d1)
	var s2 := JSON.stringify(d2)
	if s1 != s2:
		return "roundtrip mismatch:\n  a=%s\n  b=%s" % [s1, s2]
	return ""
