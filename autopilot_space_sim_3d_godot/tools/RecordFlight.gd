extends SceneTree

# Record one autopilot flight to JSON for the web replay viewer.
#
# Runs a single randomized-belt scenario through SimWorld with the flight
# recorder on, and writes the recording (scenario + timed frames + corridor
# snapshots + result) for tools/flight_viewer.html (Three.js).
#
# Usage:
#   tools\run_headless.ps1 -Script res://tools/RecordFlight.gd `
#       -ToolArgs "--asteroids 26 --seed 0 --out res://last_flight.json"
#
# Args (all optional):
#   --asteroids N --seed N --max-time S --noise SIGMA --uncertainty --out PATH

const SimWorldScript := preload("res://scripts/SimWorld.gd")

func _init() -> void:
	print("BOOT_OK")
	var args := _parse_args(OS.get_cmdline_user_args())
	var n_ast := int(args.get("asteroids", 18))
	var seed_v := int(args.get("seed", 0))
	var max_time := float(args.get("max-time", 60.0))
	var noise := float(args.get("noise", 0.0))
	var unc := args.has("uncertainty")
	var out_path := str(args.get("out", "res://last_flight.json"))

	var bmin := Vector3(-90, 0, -90)
	var bmax := Vector3(90, 60, 90)
	var start := Vector3(-60, 10, -60)
	var goal := Vector3(60, 10, 60)

	var rng := RandomNumberGenerator.new()
	rng.seed = seed_v

	var sim: SimWorld = SimWorldScript.new()
	sim.bounds_min = bmin
	sim.bounds_max = bmax
	sim.start_pos = start
	sim.goal_pos = goal
	sim.max_time = max_time
	sim.asteroids = SimWorld.random_belt(n_ast, rng, start, goal, bmin, bmax)
	sim.seed_value = seed_v + 1
	sim.noise_sigma = noise
	if unc:
		sim.unc_enable = true
		sim.unc_sigma0 = 0.5
		sim.unc_growth = noise * 1.5
		sim.unc_ksigma = 3.0
	sim.record = true

	sim.reset_run()
	var dt := 1.0 / 60.0
	var guard := 0
	var guard_max := int(max_time / dt) + 1000
	while not sim.is_terminal() and guard < guard_max:
		sim.step(dt)
		guard += 1

	if sim.save_recording(out_path):
		print("status=%s  time=%.1fs  frames=%d  paths=%d" % [
			sim.status, sim.time,
			(sim.recording["frames"] as Array).size(),
			(sim.recording["paths"] as Array).size()])
		print("Recording saved: ", ProjectSettings.globalize_path(out_path))
		quit(0)
	else:
		printerr("Failed to save recording to ", out_path)
		quit(1)

func _parse_args(raw: PackedStringArray) -> Dictionary:
	var out := {}
	var i := 0
	while i < raw.size():
		var a := raw[i]
		if a.begins_with("--"):
			var key := a.substr(2)
			if i + 1 < raw.size() and not raw[i + 1].begins_with("--"):
				out[key] = raw[i + 1]
				i += 2
			else:
				out[key] = true
				i += 1
		else:
			i += 1
	return out
