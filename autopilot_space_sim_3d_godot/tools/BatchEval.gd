extends SceneTree

# Headless batch evaluator for the autopilot.
#
# Runs many randomized asteroid-belt scenarios through SimWorld (no rendering)
# and reports aggregate performance: success / collision / timeout rates,
# clearance statistics, fuel (Δv), and time-to-goal. This is the benchmark
# harness -- run it after any change to see if the autopilot got better or worse,
# and later to compare the classical planner against alternatives (e.g. RL).
#
# Usage:
#   godot --headless --script res://tools/BatchEval.gd -- --runs 200 --asteroids 18 --seed 0
#   godot --headless --script res://tools/BatchEval.gd -- --runs 100 --out res://eval_results.csv
#
# Args (all optional):
#   --runs N --asteroids N --seed N --max-time S --out PATH
#   --noise SIGMA --uncertainty [--unc-sigma0 M --unc-growth M_S --unc-ksigma K]
#   --planner time|union          (union = legacy canary model)
#   --dump-failures DIR           (save every non-ARRIVED scenario as JSON)
#   --replay FILE                 (re-run one dumped scenario, exit 0 iff ARRIVED)
#   --gate-success P --gate-collisions N --gate-nopath P   (nonzero exit on violation)

# Preload by path: resolving SimWorld through the global class cache silently
# breaks (and HANGS the SceneTree) if the project was not re-imported after a
# class_name file was added. preload by path needs no cache. The run_headless.ps1
# wrapper additionally greps for BOOT_OK and kills us if it never appears.
const SimWorldScript := preload("res://scripts/SimWorld.gd")

func _init() -> void:
	print("BOOT_OK")
	var args := _parse_args(OS.get_cmdline_user_args())
	var runs := int(args.get("runs", 100))
	var n_ast := int(args.get("asteroids", 18))
	var base_seed := int(args.get("seed", 0))
	var max_time := float(args.get("max-time", 60.0))
	var out_path := str(args.get("out", ""))
	# Realism knobs: process noise on asteroids + chance-constrained planning.
	var noise := float(args.get("noise", 0.0))
	var unc := _arg_bool(args, "uncertainty")
	var unc_sigma0 := float(args.get("unc-sigma0", 0.5))
	var unc_growth := float(args.get("unc-growth", noise * 1.5))
	var unc_ksigma := float(args.get("unc-ksigma", 3.0))
	# Harness knobs.
	var planner := str(args.get("planner", "time"))        # time | union (canary)
	var dump_dir := str(args.get("dump-failures", ""))     # save non-ARRIVED scenarios here
	var replay_path := str(args.get("replay", ""))         # re-run one saved scenario
	var gate_success := float(args.get("gate-success", -1.0))   # % required, -1 = off
	var gate_collisions := int(args.get("gate-collisions", -1)) # max allowed, -1 = off
	var gate_nopath := float(args.get("gate-nopath", -1.0))     # % allowed, -1 = off

	if replay_path != "":
		quit(_replay(replay_path, planner))
		return

	var bmin := Vector3(-90, 0, -90)
	var bmax := Vector3(90, 60, 90)
	var start := Vector3(-60, 10, -60)
	var goal := Vector3(60, 10, 60)
	var dt := 1.0 / 60.0

	print("=== Autopilot batch eval ===")
	print("runs=%d  asteroids=%d  seed=%d  max_time=%.0fs  planner=%s" % [
		runs, n_ast, base_seed, max_time, planner])
	print("noise=%.2f m/s^2  uncertainty=%s%s" % [
		noise, str(unc),
		("  (sigma0=%.2f growth=%.2f k=%.1f)" % [unc_sigma0, unc_growth, unc_ksigma]) if unc else ""])

	if dump_dir != "":
		DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(dump_dir))

	var outcomes := {"ARRIVED": 0, "COLLISION": 0, "TIMEOUT": 0, "NO_PATH": 0}
	var clearances: Array = []      # min clearance per run
	var dvs: Array = []             # Δv per run (successful runs)
	var times: Array = []           # time-to-goal (successful runs)
	var rows: Array = []            # CSV rows
	var plan_ms_maxes: Array = []   # worst replan per run
	var plan_ms_means: Array = []   # mean replan per run
	var plan_fails := 0
	var degraded_runs := 0

	for i in range(runs):
		var rng := RandomNumberGenerator.new()
		rng.seed = base_seed + i

		var sim: SimWorld = SimWorldScript.new()
		sim.bounds_min = bmin
		sim.bounds_max = bmax
		sim.start_pos = start
		sim.goal_pos = goal
		sim.max_time = max_time
		sim.asteroids = SimWorld.random_belt(n_ast, rng, start, goal, bmin, bmax)
		sim.seed_value = base_seed + i + 1
		sim.noise_sigma = noise
		sim.use_union_planner = planner == "union"
		if unc:
			sim.unc_enable = true
			sim.unc_sigma0 = unc_sigma0
			sim.unc_growth = unc_growth
			sim.unc_ksigma = unc_ksigma

		# Capture the scenario BEFORE flying: asteroids mutate during the run,
		# so a dump taken afterwards would not reproduce the failure.
		var scen: Dictionary = sim.to_scenario()

		sim.reset_run()
		var guard := 0
		var guard_max := int(max_time / dt) + 1000
		while not sim.is_terminal() and guard < guard_max:
			sim.step(dt)
			guard += 1

		outcomes[sim.status] = int(outcomes.get(sim.status, 0)) + 1
		clearances.append(sim.min_clearance)
		if sim.status == "ARRIVED":
			dvs.append(sim.dv_used)
			times.append(sim.time)
		var run_plan_mean := sim.plan_ms_total / maxf(1.0, float(sim.plan_count))
		plan_ms_maxes.append(sim.plan_ms_max)
		plan_ms_means.append(run_plan_mean)
		plan_fails += sim.plan_fail_count
		if sim.degraded:
			degraded_runs += 1

		if dump_dir != "" and sim.status != "ARRIVED":
			var fpath := "%s/fail_%03d_%s.json" % [dump_dir, i, sim.status]
			SimWorld.save_to_file(fpath, scen)
			print("  dumped failing scenario: ", fpath)

		rows.append("%d,%s,%.3f,%.1f,%.2f,%d,%d,%.2f,%.2f,%s" % [
			i, sim.status, sim.min_clearance, sim.dv_used, sim.time, sim.replans,
			sim.plan_fail_count, run_plan_mean, sim.plan_ms_max, str(sim.degraded)])

		if (i + 1) % 25 == 0:
			print("  ...%d/%d done" % [i + 1, runs])

	_report(runs, outcomes, clearances, dvs, times)
	print("\nPlanner: replan mean=%.1f ms  p95(worst-per-run)=%.1f ms  failures=%d  degraded runs=%d" % [
		_mean(plan_ms_means), _percentile(plan_ms_maxes, 0.95), plan_fails, degraded_runs])

	if out_path != "":
		_write_csv(out_path, rows)
		print("Per-run CSV written to: ", out_path)

	# Gates: turn regressions into a nonzero exit so automation can refuse them.
	var exit_code := 0
	var arrived_pct := 100.0 * int(outcomes.get("ARRIVED", 0)) / runs
	var nopath_pct := 100.0 * int(outcomes.get("NO_PATH", 0)) / runs
	if gate_success >= 0.0 and arrived_pct < gate_success:
		print("GATE FAIL: success %.1f%% < required %.1f%%" % [arrived_pct, gate_success])
		exit_code = 1
	if gate_collisions >= 0 and int(outcomes.get("COLLISION", 0)) > gate_collisions:
		print("GATE FAIL: %d collisions > allowed %d" % [outcomes.get("COLLISION", 0), gate_collisions])
		exit_code = 1
	if gate_nopath >= 0.0 and nopath_pct > gate_nopath:
		print("GATE FAIL: NO_PATH %.1f%% > allowed %.1f%%" % [nopath_pct, gate_nopath])
		exit_code = 1
	if exit_code == 0 and (gate_success >= 0.0 or gate_collisions >= 0 or gate_nopath >= 0.0):
		print("ALL GATES PASSED")
	quit(exit_code)

# Re-run one dumped scenario with full detail; returns process exit code.
func _replay(path: String, planner: String) -> int:
	var d: Dictionary = SimWorld.load_from_file(path)
	if d.is_empty():
		print("REPLAY: could not load ", path)
		return 2
	var sim: SimWorld = SimWorldScript.new()
	sim.load_scenario(d)
	sim.use_union_planner = planner == "union"
	print("=== Replay: %s (planner=%s) ===" % [path, planner])
	print("asteroids=%d  noise=%.2f  unc=%s  seed=%d" % [
		sim.asteroids.size(), sim.noise_sigma, str(sim.unc_enable), sim.seed_value])
	sim.reset_run()
	var dt := 1.0 / 60.0
	var guard := int(sim.max_time / dt) + 1000
	while not sim.is_terminal() and guard > 0:
		sim.step(dt)
		guard -= 1
	print("status=%s  time=%.2fs  min_clearance=%.3f m  dv=%.1f  replans=%d  plan_fails=%d  degraded=%s" % [
		sim.status, sim.time, sim.min_clearance, sim.dv_used, sim.replans,
		sim.plan_fail_count, str(sim.degraded)])
	return 0 if sim.status == "ARRIVED" else 1

func _report(runs: int, outcomes: Dictionary, clearances: Array, dvs: Array, times: Array) -> void:
	var arrived := int(outcomes.get("ARRIVED", 0))
	var collided := int(outcomes.get("COLLISION", 0))
	var timed := int(outcomes.get("TIMEOUT", 0))
	var nopath := int(outcomes.get("NO_PATH", 0))

	print("\n--- Results over %d runs ---" % runs)
	print("Success (reached goal): %d  (%.1f%%)" % [arrived, 100.0 * arrived / runs])
	print("Collision:              %d  (%.1f%%)" % [collided, 100.0 * collided / runs])
	print("Timeout:                %d  (%.1f%%)" % [timed, 100.0 * timed / runs])
	print("No path found:          %d  (%.1f%%)" % [nopath, 100.0 * nopath / runs])

	# Exclude runs that never flew (NO_PATH leaves clearance at INF).
	var flew: Array = []
	for cl in clearances:
		if cl < INF:
			flew.append(cl)
	print("\nMin clearance to asteroids (m), %d flown runs:" % flew.size())
	print("  worst=%.2f  mean=%.2f  p10=%.2f" % [
		_min(flew), _mean(flew), _percentile(flew, 0.10)])

	if dvs.size() > 0:
		print("\nFor successful runs:")
		print("  Δv used:       mean=%.0f  min=%.0f  max=%.0f" % [_mean(dvs), _min(dvs), _max(dvs)])
		print("  time-to-goal:  mean=%.1fs  min=%.1fs  max=%.1fs" % [_mean(times), _min(times), _max(times)])

func _write_csv(path: String, rows: Array) -> void:
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f == null:
		push_error("Could not open %s for writing" % path)
		return
	f.store_line("run,status,min_clearance,dv_used,time,replans,plan_fails,plan_ms_mean,plan_ms_max,degraded")
	for r in rows:
		f.store_line(str(r))
	f.close()

# Boolean arg: bare --flag is true; an explicit value must be a truthy word.
# (bool("false") is truthy in GDScript -- naive casting silently enables flags.)
func _arg_bool(args: Dictionary, key: String) -> bool:
	if not args.has(key):
		return false
	var v = args[key]
	if v is bool:
		return v
	return str(v).to_lower() in ["1", "true", "yes", "on"]

# ---- arg parsing: --key value  (and bare --flag -> true) ----
func _parse_args(argv: PackedStringArray) -> Dictionary:
	var out := {}
	var i := 0
	while i < argv.size():
		var tok := argv[i]
		if tok.begins_with("--"):
			var key := tok.substr(2)
			if i + 1 < argv.size() and not argv[i + 1].begins_with("--"):
				out[key] = argv[i + 1]
				i += 2
			else:
				out[key] = true
				i += 1
		else:
			i += 1
	return out

# ---- small stats helpers ----
func _mean(a: Array) -> float:
	if a.is_empty():
		return 0.0
	var s := 0.0
	for x in a:
		s += float(x)
	return s / a.size()

func _min(a: Array) -> float:
	if a.is_empty():
		return 0.0
	var m := INF
	for x in a:
		m = minf(m, float(x))
	return m

func _max(a: Array) -> float:
	if a.is_empty():
		return 0.0
	var m := -INF
	for x in a:
		m = maxf(m, float(x))
	return m

func _percentile(a: Array, q: float) -> float:
	if a.is_empty():
		return 0.0
	var s := a.duplicate()
	s.sort()
	var idx := int(clampf(q * (s.size() - 1), 0, s.size() - 1))
	return float(s[idx])
