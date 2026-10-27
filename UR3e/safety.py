# Approximate UR3e collision checking without changing the group's collision code.
# Link centre-lines use a safety radius; the car is represented by a bounding box.

import numpy as np


def segment_hits_box(a, b, centre, dimensions, radius=0.0):
    lower = np.asarray(centre) - np.asarray(dimensions) / 2 - radius
    upper = np.asarray(centre) + np.asarray(dimensions) / 2 + radius
    direction = np.asarray(b) - np.asarray(a)
    t_min, t_max = 0.0, 1.0
    for i in range(3):
        if abs(direction[i]) < 1e-12:
            if a[i] < lower[i] or a[i] > upper[i]:
                return False
        else:
            t1 = (lower[i] - a[i]) / direction[i]
            t2 = (upper[i] - a[i]) / direction[i]
            t_min = max(t_min, min(t1, t2))
            t_max = min(t_max, max(t1, t2))
            if t_min > t_max:
                return False
    return True


def segment_distance(p1, q1, p2, q2):
    # Distance between two finite 3D line segments.
    u, v = q1 - p1, q2 - p2
    w = p1 - p2
    a, b, c = u @ u, u @ v, v @ v
    d, e = u @ w, v @ w
    denominator = a*c - b*b
    s = np.clip((b*e - c*d)/denominator, 0, 1) if denominator > 1e-12 else 0.0
    t = np.clip((b*s + e)/c, 0, 1) if c > 1e-12 else 0.0
    s = np.clip((b*t - d)/a, 0, 1) if a > 1e-12 else 0.0
    return np.linalg.norm(w + s*u - t*v)


def robot_points(robot):
    # Same joint-centre convention used by the other robot collision routines.
    poses = np.asarray(robot.fkine_all(robot.q).A, dtype=float)
    points = poses[:, :3, 3] if poses.ndim == 3 else np.asarray([poses[:3, 3]])
    tool = np.asarray(robot.fkine(robot.q).t)
    if np.linalg.norm(tool - points[-1]) > 1e-5:
        points = np.vstack((points, tool))
    return points


def check_ur3e_path(ur3e, path, car_box, other_robots=()):
    other_points = [(getattr(robot, 'name', 'robot'), robot_points(robot))
                    for robot in other_robots]
    centre, dimensions = car_box
    for step, q in enumerate(path):
        if not np.all(np.isfinite(q)) or np.any(np.abs(q) > 2*np.pi):
            raise RuntimeError('UR3e joint values out of range at step ' + str(step))

        frames = ur3e.frames(q)
        points = [frame.t for frame in frames]
        flange, tip = ur3e.sign_points(q)
        segments = [(points[i], points[i + 1], radius)
                    for i, radius in enumerate(ur3e.link_radii)]
        segments.append((flange, tip, 0.02))
        segments.append((tip, tip, ur3e.sign_radius))

        for a, b, radius in segments:
            if min(a[2], b[2]) - radius < -0.005:
                raise RuntimeError('UR3e floor collision at step ' + str(step))
            if segment_hits_box(a, b, centre, dimensions, radius + 0.025):
                raise RuntimeError('UR3e/car clearance collision at step ' + str(step))
            for name, other in other_points:
                for i in range(len(other)-1):
                    if segment_distance(a, b, other[i], other[i+1]) < radius + 0.08:
                        raise RuntimeError('UR3e too close to ' + name + ' at step ' + str(step))
