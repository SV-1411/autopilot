extends RefCounted
class_name LocalPlanner3D

# 3D acceleration-sampling dynamic-window local planner with moving-obstacle
# avoidance.
#
# WHAT IT DOES
#   Given the ship's current state, a target point to head toward (the next A*
#   waypoint), and the set of asteroids (each with a known velocity), it picks
#   the best thrust (acceleration) command for this tick.
#
# HOW IT WORKS (this is the "autopilot brain")
#   1. Sample a set of candidate accelerations the ship could apply right now,
#      respecting its thrust limit (a_max) and top speed (v_max). This is the
#      "dynamic window": only physically reachable controls are considered.
#   2. For each candidate, roll the ship forward over a short horizon AND roll
#      every asteroid forward over the same horizon (using Predictor). This is
#      what makes avoidance work against *moving* obstacles, not just static
#      ones -- it reasons about where rocks WILL be.
#   3. Score each candidate trajectory by a weighted cost:
#         - progress     : how close it gets to the target
#         - clearance    : how close it comes to any (predicted) asteroid
#         - speed        : reward moving toward the target quickly
#         - effort       : a small penalty on thrust used (fuel / Δv)
#      Any candidate that is predicted to collide is rejected outright.
#   4. Return the lowest-cost acceleration.
#
# This is a classical, fully explainable controller -- no training required.
# Every weight is a tunable knob, exposed via the cfg dictionary.

const DEFAULT_CFG := {
	"horizon": 2.5,        # seconds to look ahead when scoring a candidate
	"sim_dt": 0.2,         # integration step used during look-ahead
	"ctrl_dt": 0.3,        # window over which the candidate accel is applied
	"n_dirs": 26,          # number of thrust directions sampled on a sphere
	"safe_margin": 6.0,    # metres of clearance we try to keep around the hull
	"w_goal": 1.0,         # weight: reward progress made toward the target
	"w_cross": 0.6,        # weight: penalty for drifting off the line to target
	"w_clear": 150.0,      # weight: penalty for getting within safe_margin
	"w_effort": 0.4,       # weight: penalty on thrust magnitude (fuel)
	"w_speed_cap": 5.0,    # weight: penalty for exceeding the safe arrival speed
}

# obstacles: Array of { "pos": Vector3, "vel": Vector3, "radius": float }
# Returns the chosen acceleration command (m/s^2) as a Vector3.
static func plan(
		ship_pos: Vector3,
		ship_vel: Vector3,
		target: Vector3,
		arrival_dist: float,    # remaining distance to the FINAL goal (for braking)
		obstacles: Array,
		ship_radius: float,
		a_max: float,
		v_max: float,
		bounds_min: Vector3,
		bounds_max: Vector3,
		cfg: Dictionary = {}
) -> Vector3:
	var c := DEFAULT_CFG.duplicate()
	for k in cfg.keys():
		c[k] = cfg[k]

	var candidates := _candidate_accelerations(ship_pos, ship_vel, target, a_max, int(c["n_dirs"]))

	var to_target := target - ship_pos
	var dist := to_target.length()
	var to_target_dir := to_target.normalized() if dist > 1e-5 else Vector3.ZERO
	var margin := float(c["safe_margin"])

	# Speed governor: the fastest we can go and still brake to a stop by the
	# final goal, given the thrust limit (v = sqrt(2*a*d)). Far away -> full
	# speed; close in -> the ship is forced to slow down and arrive cleanly.
	var v_cap := minf(v_max, sqrt(2.0 * a_max * maxf(0.0, arrival_dist)))

	# Predict every obstacle's position at each look-ahead step ONCE, then reuse
	# across all candidates (the prediction doesn't depend on the ship's choice).
	# This is the key performance optimisation -- without it the predictor runs
	# once per candidate per step, ~50x more work.
	var sim_dt := float(c["sim_dt"])
	var steps := int(ceil(float(c["horizon"]) / sim_dt))
	var predicted := _predict_frames(obstacles, steps, sim_dt, bounds_min, bounds_max)

	var best_cost := INF
	var best_accel := Vector3.ZERO

	for accel in candidates:
		# Velocity the ship would carry into the look-ahead after this thrust.
		var cand_v: Vector3 = ship_vel + accel * float(c["ctrl_dt"])
		if cand_v.length() > v_max:
			cand_v = cand_v.normalized() * v_max

		var result := _simulate(ship_pos, cand_v, predicted, ship_radius, sim_dt, steps)
		if result["collides"]:
			continue

		var end_pos: Vector3 = result["end_pos"]
		var min_clear: float = result["min_clear"]

		# Decompose the predicted displacement into "toward target" (along) and
		# "sideways" (perp). Rewarding along-progress (rather than end distance)
		# avoids capping speed, while perp keeps the ship on the path line.
		var end_rel := end_pos - ship_pos
		var along := end_rel.dot(to_target_dir)
		var perp := (end_rel - to_target_dir * along).length()

		var goal_cost := -along                          # more progress -> lower

		var clear_cost := 0.0
		if min_clear < margin:
			clear_cost = (margin - min_clear) / margin   # 0..1, grows as we crowd

		var effort_cost := accel.length() / a_max if a_max > 0.0 else 0.0
		var speed_over := maxf(0.0, cand_v.length() - v_cap)

		var cost := (
			float(c["w_goal"]) * goal_cost
			+ float(c["w_cross"]) * perp
			+ float(c["w_clear"]) * clear_cost
			+ float(c["w_effort"]) * effort_cost
			+ float(c["w_speed_cap"]) * speed_over
		)

		if cost < best_cost:
			best_cost = cost
			best_accel = accel

	# If every candidate was predicted to collide, brake hard along -velocity:
	# decelerating buys time and is the safest fallback.
	if best_cost == INF:
		if ship_vel.length() > 1e-3:
			best_accel = -ship_vel.normalized() * a_max
		else:
			best_accel = Vector3.ZERO

	return best_accel

# Precompute predicted obstacle positions for every look-ahead step.
# Returns Array[step] -> Array of { "p": Vector3, "sr": float } where sr is the
# pre-summed collision radius offset is handled by the caller (ship_radius).
static func _predict_frames(obstacles: Array, steps: int, dt: float,
		bounds_min: Vector3, bounds_max: Vector3) -> Array:
	var frames: Array = []
	var t := 0.0
	for s in range(steps):
		t += dt
		var frame: Array = []
		for obs in obstacles:
			frame.append({
				"p": Predictor.predict(obs["pos"], obs["vel"], t, bounds_min, bounds_max),
				"r": float(obs["radius"]),
			})
		frames.append(frame)
	return frames

# Roll the ship (constant cand_v) forward against precomputed obstacle frames;
# return whether a collision is predicted, the closest approach, and end pos.
static func _simulate(
		start_pos: Vector3,
		cand_v: Vector3,
		predicted: Array,
		ship_radius: float,
		dt: float,
		steps: int
) -> Dictionary:
	var p := start_pos
	var min_clear := INF

	for s in range(steps):
		p += cand_v * dt
		for o in predicted[s]:
			var d: float = p.distance_to(o["p"]) - (ship_radius + float(o["r"]))
			if d < min_clear:
				min_clear = d
			if d <= 0.0:
				return {"collides": true, "min_clear": d, "end_pos": p}

	return {"collides": false, "min_clear": min_clear, "end_pos": p}

static func _candidate_accelerations(
		ship_pos: Vector3,
		ship_vel: Vector3,
		target: Vector3,
		a_max: float,
		n_dirs: int
) -> Array[Vector3]:
	var candidates: Array[Vector3] = []
	candidates.append(Vector3.ZERO)  # coast

	# Uniformly distributed thrust directions on a sphere, at half and full power.
	for dir in _sphere_dirs(n_dirs):
		candidates.append(dir * a_max)
		candidates.append(dir * a_max * 0.5)

	# Always try the two "obvious" controls explicitly so they're never missed:
	# full thrust straight at the target, and a hard brake.
	var to_target := target - ship_pos
	if to_target.length() > 1e-5:
		candidates.append(to_target.normalized() * a_max)
	if ship_vel.length() > 1e-5:
		candidates.append(-ship_vel.normalized() * a_max)

	return candidates

# Evenly spaced unit vectors on a sphere (Fibonacci lattice).
static func _sphere_dirs(n: int) -> Array[Vector3]:
	var dirs: Array[Vector3] = []
	if n <= 1:
		return [Vector3.UP]
	var golden := PI * (3.0 - sqrt(5.0))
	for i in range(n):
		var y := 1.0 - (float(i) / float(n - 1)) * 2.0
		var r := sqrt(maxf(0.0, 1.0 - y * y))
		var theta := golden * float(i)
		dirs.append(Vector3(cos(theta) * r, y, sin(theta) * r))
	return dirs
