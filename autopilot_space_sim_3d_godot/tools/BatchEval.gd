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
# Args (all optional): --runs N  --asteroids N  --seed N  --max-time S  --out PATH

func _init() -> void:
	var args := _parse_args(OS.get_cmdline_user_args())
	var runs := int(args.get("runs", 100))
	var n_ast := int(args.get("asteroids", 18))
	var base_seed := int(args.get("seed", 0))
	var max_time := float(args.get("max-time", 60.0))
	var out_path := str(args.get("out", ""))

	var bmin := Vector3(-90, 0, -90)
	var bmax := Vector3(90, 60, 90)
	var start := Vector3(-60, 10, -60)
	var goal := Vector3(60, 10, 60)
	var dt := 1.0 / 60.0

	print("=== Autopilot batch eval ===")
	print("runs=%d  asteroids=%d  seed=%d  max_time=%.0fs" % [runs, n_ast, base_seed, max_time])

	var outcomes := {"ARRIVED": 0, "COLLISION": 0, "TIMEOUT": 0, "NO_PATH": 0}
	var clearances: Array = []      # min clearance per run
	var dvs: Array = []             # Δv per run (successful runs)
	var times: Array = []           # time-to-goal (successful runs)
	var rows: Array = []            # CSV rows

	for i in range(runs):
		var rng := RandomNumberGenerator.new()
		rng.seed = base_seed + i

		var sim := SimWorld.new()
		sim.bounds_min = bmin
		sim.bounds_max = bmax
		sim.start_pos = start
		sim.goal_pos = goal
		sim.max_time = max_time
		sim.asteroids = SimWorld.random_belt(n_ast, rng, start, goal, bmin, bmax)

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

		rows.append("%d,%s,%.3f,%.1f,%.2f,%d" % [
			i, sim.status, sim.min_clearance, sim.dv_used, sim.time, sim.replans])

		if (i + 1) % 25 == 0:
			print("  ...%d/%d done" % [i + 1, runs])

	_report(runs, outcomes, clearances, dvs, times)

	if out_path != "":
		_write_csv(out_path, rows)
		print("Per-run CSV written to: ", out_path)

	quit()

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
	f.store_line("run,status,min_clearance,dv_used,time,replans")
	for r in rows:
		f.store_line(str(r))
	f.close()

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
