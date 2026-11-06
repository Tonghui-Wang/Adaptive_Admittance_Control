"""Selective Cartesian admittance with slow particle-swarm tuning.

The contact normal is force-controlled. Tangential axes track the planned
path. A unilateral spring is the environment. The swarm searches only the
normal mass and damping, on a slower clock than the joint loop, and its
velocity update points at ``pbest - x`` and ``gbest - x``.
"""

from dataclasses import dataclass

import numpy as np
import roboticstoolbox as rtb


DT = 0.01
F_STAR = 10.0
F_RAMP = 0.4
K_SOFT = 800.0
K_STIFF = 8000.0
K_HAT0 = 2000.0
APPROACH_SPEED = 0.04
ADAPT_EVERY = 25
HORIZON = 40
PSO_PARTICLES = 12
PSO_ITERS = 8
M_BOUNDS = np.array([0.4, 6.0])
B_BOUNDS = np.array([20.0, 400.0])
PSO_VMAX = np.array([0.35, 30.0])
RATE_LIMIT = np.array([0.35, 40.0])
DLS_LAMBDA = 0.05
NULL_GAIN = 1.2
QDOT_LIMIT = 1.5


@dataclass
class TangentGains:
    m: float = 1.5
    b: float = 70.0
    k: float = 350.0


@dataclass
class NormalGains:
    m: float = 1.5
    b: float = 70.0


TANGENT = TangentGains()


def desired_force(t):
    return F_STAR * np.clip(t / F_RAMP, 0.0, 1.0)


def environment_stiffness(t, duration):
    """Plant stiffness. The controller is not given this schedule."""
    return K_SOFT if t < 0.5 * duration else K_STIFF


def fit_plane(points):
    """Place a work plane on the flattest direction of the planned path.

    The outward normal is signed so the path starts in free space. The
    plane then cuts the path, and holding a normal force keeps the tool
    on that plane instead of following the path into the surface.
    """
    center = points.mean(axis=0)
    _, axes = np.linalg.eigh(np.cov(points.T))
    normal = axes[:, 0]
    normal = normal / np.linalg.norm(normal)
    offset = (points - center) @ normal
    if offset[0] < np.median(offset):
        normal = -normal
        offset = -offset
    # 1.5 cm of free space at the first sample, then the tool approaches
    # and stays on the plane under force control.
    plane_offset = float(offset[0] - 0.015)
    return normal, center + normal * plane_offset


def contact(position, normal, plane_point, stiffness):
    """Unilateral penalty. Returns (force magnitude along +normal, penetration)."""
    signed = float(normal @ (position - plane_point))
    penetration = max(0.0, -signed)
    return stiffness * penetration, penetration


def admittance_acceleration(position, velocity, p_ref, v_ref, a_ref, normal, force, force_star, penetration, normal_gains, tangent_gains):
    """Linear acceleration command.

    ``normal`` points out of the material. ``force`` is the outward contact
    force. When ``force`` is below ``force_star`` the tool accelerates into
    the surface; when the force is high it yields back out.
    """
    tangent = np.eye(3) - np.outer(normal, normal)
    error_t = tangent @ (position - p_ref)
    velocity_error_t = tangent @ (velocity - v_ref)
    accel_t = tangent @ a_ref + (
        -tangent_gains.k * error_t - tangent_gains.b * velocity_error_t
    ) / tangent_gains.m

    speed_n = float(normal @ velocity)
    if penetration <= 1e-6:
        # Free space: approach the surface at a limited speed instead of
        # integrating an unsaturated force error.
        accel_n = normal_gains.b * (-APPROACH_SPEED - speed_n) / normal_gains.m
    else:
        accel_n = ((force - force_star) - normal_gains.b * speed_n) / normal_gains.m
    return accel_t + accel_n * normal


def integrate_velocity(velocity, acceleration, dt, normal, penetration):
    updated = velocity + acceleration * dt
    speed_n = float(normal @ updated)
    if penetration <= 1e-6 and speed_n < -APPROACH_SPEED:
        updated = updated + normal * (-APPROACH_SPEED - speed_n)
    if speed_n > 0.5:
        updated = updated + normal * (0.5 - speed_n)
    return updated


def resolved_rate(jacobian_v, velocity_cmd, q, q_ref, qd_ref):
    """Damped least-squares Cartesian rate plus joint-space null motion."""
    weight = jacobian_v @ jacobian_v.T + (DLS_LAMBDA ** 2) * np.eye(3)
    jpinv = jacobian_v.T @ np.linalg.solve(weight, np.eye(3))
    null = np.eye(6) - jpinv @ jacobian_v
    qdot = jpinv @ velocity_cmd + null @ (qd_ref + NULL_GAIN * (q_ref - q))
    return np.clip(qdot, -QDOT_LIMIT, QDOT_LIMIT)


def plan(robot, via_deg, dt=DT):
    via = np.deg2rad(np.asarray(via_deg, dtype=float))
    traj = rtb.mstraj(
        viapoints=via, dt=dt, tacc=0.5, qdmax=np.full(6, 0.5)
    )
    count = len(traj.q)
    position = np.zeros((count, 3))
    speed = np.zeros((count, 3))
    for i in range(count):
        position[i] = robot.fkine(traj.q[i]).t
        speed[i] = robot.jacob0(traj.q[i])[:3, :] @ traj.qd[i]
    accel = np.gradient(speed, dt, axis=0)
    return {
        "t": np.arange(count) * dt,
        "q": traj.q,
        "qd": traj.qd,
        "qdd": traj.qdd,
        "p": position,
        "v": speed,
        "a": accel,
    }


def _sample(reference, index):
    last = len(reference["t"]) - 1
    k = min(max(index, 0), last)
    return reference["p"][k], reference["v"][k], reference["a"][k], reference["t"][k]


def rollout_cost(gains, position, velocity, reference, index, normal, plane_point, stiffness, dt, params_prev):
    """Cartesian preview. The arm is assumed to realize this acceleration."""
    trial = NormalGains(float(gains[0]), float(gains[1]))
    point = np.array(position, dtype=float, copy=True)
    speed = np.array(velocity, dtype=float, copy=True)
    force_error = 0.0
    tangent_error = 0.0
    for step in range(HORIZON):
        p_ref, v_ref, a_ref, stamp = _sample(reference, index + step)
        force, penetration = contact(point, normal, plane_point, stiffness)
        force_star = desired_force(stamp)
        accel = admittance_acceleration(
            point, speed, p_ref, v_ref, a_ref, normal, force, force_star,
            penetration, trial, TANGENT,
        )
        speed = integrate_velocity(speed, accel, dt, normal, penetration)
        point = point + speed * dt
        force_error += ((force - force_star) / F_STAR) ** 2
        tangent = (point - p_ref) - normal * float(normal @ (point - p_ref))
        tangent_error += float(tangent @ tangent) / (0.02 ** 2)
    smooth = (gains - params_prev) / np.array([1.0, 40.0])
    return force_error / HORIZON + 0.05 * tangent_error / HORIZON + 0.02 * float(smooth @ smooth)


def pso_minimize(cost, x0, lower, upper, vmax, particles, iterations, rng):
    """Standard PSO. Walls are absorbing: outward velocity is cleared."""
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    vmax = np.asarray(vmax, dtype=float)
    dimension = len(x0)
    position = np.tile(x0, (particles, 1))
    span = (upper - lower) * 0.12
    position[1:] += rng.normal(0.0, 1.0, size=(particles - 1, dimension)) * span
    position = np.clip(position, lower, upper)
    velocity = rng.uniform(-1.0, 1.0, size=position.shape) * vmax * 0.2
    personal = position.copy()
    personal_cost = np.array([cost(row) for row in position])
    best_index = int(np.argmin(personal_cost))
    global_best = personal[best_index].copy()
    global_cost = float(personal_cost[best_index])
    for generation in range(iterations):
        inertia = 0.9 - 0.5 * generation / max(iterations - 1, 1)
        for i in range(particles):
            cognitive = rng.random(dimension)
            social = rng.random(dimension)
            velocity[i] = (
                inertia * velocity[i]
                + 1.5 * cognitive * (personal[i] - position[i])
                + 1.5 * social * (global_best - position[i])
            )
            velocity[i] = np.clip(velocity[i], -vmax, vmax)
            position[i] = position[i] + velocity[i]
            outside = (position[i] <= lower) | (position[i] >= upper)
            position[i] = np.clip(position[i], lower, upper)
            velocity[i][outside] = 0.0
            value = cost(position[i])
            if value < personal_cost[i]:
                personal_cost[i] = value
                personal[i] = position[i].copy()
                if value < global_cost:
                    global_cost = float(value)
                    global_best = position[i].copy()
    return global_best, global_cost


def rate_limit(proposed, current, limit):
    return current + np.clip(proposed - current, -limit, limit)


def simulate(robot, reference, normal, plane_point, mode="adaptive", seed=7):
    """Track ``reference`` on the plane.

    ``mode`` is ``adaptive``, ``fixed``, ``heavy``, or ``light``. Heavy and
    light gains are fixed and intentionally away from the force-tracking basin.
    """
    if mode == "light":
        gains = NormalGains(0.8, 25.0)
    elif mode == "heavy":
        gains = NormalGains(5.0, 280.0)
    elif mode == "fixed":
        gains = NormalGains(1.5, 80.0)
    else:
        gains = NormalGains(1.5, 80.0)
    rng = np.random.default_rng(seed)
    count = len(reference["t"])
    duration = float(reference["t"][-1])
    q = reference["q"][0].copy()
    qdot = reference["qd"][0].copy()
    stiffness_hat = K_HAT0

    out = {key: np.zeros(count) for key in ("force", "force_des", "penetration", "m", "b", "k_hat", "k_true", "tangent")}
    out["position"] = np.zeros((count, 3))
    out["q"] = np.zeros((count, 6))

    for i, stamp in enumerate(reference["t"]):
        pose = robot.fkine(q)
        position = np.asarray(pose.t, dtype=float)
        jacobian = robot.jacob0(q)[:3, :]
        velocity = jacobian @ qdot
        stiffness = environment_stiffness(stamp, duration)
        force, penetration = contact(position, normal, plane_point, stiffness)
        force_star = desired_force(stamp)
        if penetration > 3e-3:
            measured = float(np.clip(force / penetration, 100.0, 30000.0))
            # A sudden stiffness change shows up as a large force at the
            # same penetration. Take that sample instead of creeping toward it.
            if abs(measured - stiffness_hat) > 0.4 * stiffness_hat:
                stiffness_hat = measured
            else:
                stiffness_hat = 0.8 * stiffness_hat + 0.2 * measured

        in_contact = penetration > 1e-3
        if mode == "adaptive" and i % ADAPT_EVERY == 0 and in_contact:
            current = np.array([gains.m, gains.b])

            def cost(candidate, origin=current, state=(position.copy(), velocity.copy()), start=i, hat=stiffness_hat):
                value = rollout_cost(
                    candidate, state[0], state[1], reference, start, normal,
                    plane_point, hat, DT, origin,
                )
                return value if np.isfinite(value) else 1e12

            baseline = cost(current)
            proposed, proposed_cost = pso_minimize(
                cost, current, M_BOUNDS, B_BOUNDS, PSO_VMAX,
                PSO_PARTICLES, PSO_ITERS, rng,
            )
            # A flat steady-state cost has no unique minimizer. Keep the
            # current gains unless the preview is actually better.
            if proposed_cost + 1e-3 < baseline:
                limited = rate_limit(proposed, current, RATE_LIMIT)
                gains = NormalGains(float(limited[0]), float(limited[1]))

        p_ref, v_ref, a_ref, _ = _sample(reference, i)
        accel = admittance_acceleration(
            position, velocity, p_ref, v_ref, a_ref, normal, force, force_star,
            penetration, gains, TANGENT,
        )
        velocity_cmd = integrate_velocity(velocity, accel, DT, normal, penetration)
        qdot = resolved_rate(jacobian, velocity_cmd, q, reference["q"][i], reference["qd"][i])
        q = q + qdot * DT

        tangent = (position - p_ref) - normal * float(normal @ (position - p_ref))
        out["force"][i] = force
        out["force_des"][i] = force_star
        out["penetration"][i] = penetration
        out["m"][i] = gains.m
        out["b"][i] = gains.b
        out["k_hat"][i] = stiffness_hat
        out["k_true"][i] = stiffness
        out["tangent"][i] = np.linalg.norm(tangent)
        out["position"][i] = position
        out["q"][i] = q
    out["t"] = reference["t"].copy()
    out["p_ref"] = reference["p"].copy()
    out["normal"] = normal
    out["plane_point"] = plane_point
    return out


def force_stats(result, mask):
    error = result["force"][mask] - result["force_des"][mask]
    return float(np.sqrt(np.mean(error ** 2))), float(np.std(result["force"][mask]))


def default_via_points():
    return np.array(
        [
            [10, 20, 30, 40, 50, 60],
            [-5, -10, -15, -20, -25, -30],
            [35, -20, 25, -30, 35, -50],
        ],
        dtype=float,
    )
