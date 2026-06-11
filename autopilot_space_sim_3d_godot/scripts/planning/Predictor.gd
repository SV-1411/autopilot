extends RefCounted
class_name Predictor

# Obstacle motion prediction.
#
# Asteroids move at constant velocity and bounce off the world bounds. To plan
# safely around them, the autopilot needs to know *where they will be* over the
# next few seconds, not just where they are now. This module provides a single
# closed-form predictor that is kept deliberately consistent with how asteroids
# actually integrate their motion (see Asteroid.gd::integrate), so the predicted
# trajectory matches reality as long as velocity is constant.
#
# Reflection is computed per-axis as a triangle wave, which is the exact
# closed-form position of a point bouncing between two walls.

static func predict(pos: Vector3, vel: Vector3, t: float,
		bounds_min: Vector3, bounds_max: Vector3) -> Vector3:
	return Vector3(
		_reflect_axis(pos.x, vel.x, t, bounds_min.x, bounds_max.x),
		_reflect_axis(pos.y, vel.y, t, bounds_min.y, bounds_max.y),
		_reflect_axis(pos.z, vel.z, t, bounds_min.z, bounds_max.z)
	)

# Advance an obstacle by dt with exact wall reflection, returning {pos, vel}.
# SimWorld's integrator MUST use this so simulated truth and predicted future
# stay identical (clamping at the wall instead of reflecting loses the overshoot
# and silently drifts truth away from prediction, bounce after bounce).
static func advance(pos: Vector3, vel: Vector3, dt: float,
		bounds_min: Vector3, bounds_max: Vector3) -> Dictionary:
	var p := pos + vel * dt
	var v := vel
	for axis in range(3):
		var lo := bounds_min[axis]
		var hi := bounds_max[axis]
		if hi <= lo:
			p[axis] = clampf(p[axis], lo, hi)
			continue
		# Fold the overshoot back into the box; each fold flips direction.
		while p[axis] < lo or p[axis] > hi:
			if p[axis] < lo:
				p[axis] = 2.0 * lo - p[axis]
			else:
				p[axis] = 2.0 * hi - p[axis]
			v[axis] = -v[axis]
	return {"pos": p, "vel": v}

static func _reflect_axis(p0: float, v: float, t: float, lo: float, hi: float) -> float:
	if hi <= lo:
		return clampf(p0, lo, hi)
	var span := hi - lo
	var x := p0 + v * t
	# Fold x into [lo, hi] using a triangle wave of period 2*span.
	var period := 2.0 * span
	var phase := fposmod(x - lo, period)
	if phase > span:
		phase = period - phase
	return lo + phase
