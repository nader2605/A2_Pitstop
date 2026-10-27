
# UR3e robot with pit-stop lollipop

import numpy as np
import spatialgeometry as sg
from spatialmath import SE3
from .safety import check_ur3e_path


class UR3eLollipop:
    # UR3e DH parameters
    a = [0, -0.24355, -0.21320, 0, 0, 0]
    d = [0.15185, 0, 0, 0.13105, 0.08535, 0.09210]
    alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]

    def __init__(self):
        self.base = SE3()

        # Robot starting and holding positions
        self.q_home = np.array([0.3727, -0.1523, 1.6455, 0.0061, -1.5754, 0])
        self.q_hold = np.array([0.8427, -1.2492, 1.5226, -0.2734, 0.8427, 0])
        self.q = self.q_home.copy()

        self.bar_length = 0.72
        self.sign_radius = 0.13
        self.pedestal_height = 0.62

        grey = [0.73, 0.76, 0.79, 1]
        blue = [0.15, 0.39, 0.69, 1]
        red = [0.90, 0.10, 0.09, 1]
        white = [1.0, 1.0, 1.0, 1.0]

        # Robot geometry
        self.pedestal = sg.Cuboid(
            [0.34, 0.34, 0.62],
            color=[0.38, 0.39, 0.42, 1]
        )

        self.link_radii = (0.070, 0.059, 0.051, 0.043, 0.041, 0.038)

        self.links = [
            sg.Cylinder(r, float(np.hypot(a, d)), color=grey)
            for r, a, d in zip(self.link_radii, self.a, self.d)
        ]

        self.joints = [
            sg.Cylinder(0.060 if i < 3 else 0.046, 0.055, color=blue)
            for i in range(6)
        ]

        # Lollipop geometry
        self.bar = sg.Cylinder(0.014, self.bar_length, color=grey)
        self.sign = sg.Cylinder(self.sign_radius, 0.025, color=red)
        self.white_front = sg.Cuboid([0.17, 0.035, 0.009], color=white)
        self.white_back = sg.Cuboid([0.17, 0.035, 0.009], color=white)

        self.objects = [
            self.pedestal, *self.links, *self.joints,
            self.bar, self.sign, self.white_front, self.white_back
        ]

        # Sign orientation
        R = self.frames(self.q_hold)[-1].R
        self.sign_rotation = SE3.Rt(
            R.T @ SE3.Ry(np.pi/2).R,
            np.zeros(3),
            check=False
        )

        self.update()

    def set_base(self, pose):
        self.base = pose
        self.update()

    def frames(self, q=None):
        q = np.asarray(self.q if q is None else q, dtype=float)

        if q.shape != (6,):
            raise ValueError("UR3e requires six joint angles")

        frames = [self.base]

        for angle, a, d, alpha in zip(q, self.a, self.d, self.alpha):
            T = frames[-1] * SE3.Rz(angle) * SE3(0, 0, d)
            frames.append(T * SE3(a, 0, 0) * SE3.Rx(alpha))

        return frames

    def sign_points(self, q=None):
        flange = self.frames(q)[-1]
        return flange.t, (flange * SE3(0, 0, self.bar_length)).t

    def _segment_pose(self, a, b):
        # Align cylinder between two joint positions
        z = np.asarray(b) - np.asarray(a)
        length = np.linalg.norm(z)
        if length < 1e-10:
            return SE3(*(np.asarray(a, dtype=float).tolist()))
        z = z / length

        ref = [0, 0, 1] if abs(z[2]) < 0.9 else [0, 1, 0]
        x = np.cross(ref, z)
        x /= np.linalg.norm(x)

        return SE3.Rt(
            np.column_stack((x, np.cross(z, x), z)),
            (a + b) / 2,
            check=False
        )

    def update(self):
        frames = self.frames()
        x, y = self.base.t[:2]

        self.pedestal.T = SE3(x, y, self.pedestal_height / 2).A

        for link, a, b in zip(self.links, frames[:-1], frames[1:]):
            link.T = self._segment_pose(a.t, b.t).A

        for joint, frame in zip(self.joints, frames[1:]):
            joint.T = frame.A

        flange = frames[-1]

        self.bar.T = (
            flange * SE3(0, 0, self.bar_length / 2)
        ).A

        sign_pose = (
            flange * SE3(0, 0, self.bar_length) * self.sign_rotation
        )

        self.sign.T = sign_pose.A

        for shape, offset in zip(
            [self.white_front, self.white_back],
            [0.017, -0.017]
        ):
            shape.T = (sign_pose * SE3(0, 0, offset)).A

    def add_to_env(self, env):
        self.update()

        for shape in self.objects:
            env.add(shape)

    def path_to(self, goal, count=75):
        goal = np.asarray(goal, dtype=float)

        if goal.shape != (6,):
            raise ValueError("UR3e target must contain six joint angles")

        t = np.linspace(0, 1, count)
        blend = 10*t**3 - 15*t**4 + 6*t**5

        return self.q + blend[:, None] * (goal - self.q)

    def checked_path(self, goal, car_box, other_robots=(), count=75):
        path = self.path_to(goal, count)
        check_ur3e_path(self, path, car_box, other_robots)
        return path
