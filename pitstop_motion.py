# Pit Stop motion: IK, collision checks, robot movement, lift, wheel handling
# 41013 Industrial Robotics - Lab Assignment 2

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from pitstop_config import steps, random_obstacle_count, car_footprint
from pitstop_utils import back, MotionStopped


class MotionMixin:

    # IK / safety

    def solve(self, robot, target, q0):
        solution = robot.ikine_LM(
            target,
            q0=q0,
            joint_limits=True,
            seed=0  # A button
        )

        if not solution.success:
            raise RuntimeError(robot.name + ": no IK solution")

        q = q0 + (solution.q - q0 + np.pi) % (2 * np.pi) - np.pi
        lower, upper = robot.qlim

        for i in range(len(q)):
            if not lower[i] <= q[i] <= upper[i]:
                q[i] = solution.q[i]

        return q

    def robot_name(self, robot):
        if robot is self.kuka.Robot:
            return "kuka"
        if robot is self.staubli.Robot:
            return "staubli"
        if robot is self.abb.Robot:
            return "abb"

    def guard(self, robot=None):
        self.safety.poll()
        # A requested full reset always cancels the current trajectory.
        if self.safety.reset_requested:
            raise MotionStopped
        if self.safety.state in ("RUNNING", "RECOVERING"):
            return
        # Preserve history after a stop, for A-button reverse recovery.
        if self.safety.state in ("ESTOP", "FAULT", "COLLISION"):
            self.recovery.recording = False
            self.run_requested = False
        raise MotionStopped

    # Collision interface (Lab 5 Q2 check, see collision.py)

    def collision(self, robot, q):
        pose = self.obstacle_pose if self.obstacle_active else None
        return self.collision_manager.check(robot, pose, self.obstacle_size, q)

    def path_clear(self, robot, path):
        pose = self.obstacle_pose if self.obstacle_active else None
        return self.collision_manager.path_clear(
            robot, path, pose, self.obstacle_size
        )

    def old_tyre_box(self):
        # The removed tyre becomes an obstacle once it sits on the old-tyre stand.
        car = self.car
        parent, offset = self.mounts[car.work_wheel]
        if parent != "stand_old":
            return []
        hub = self.stands["old"] * offset * car.wheel_offset.inv()
        diameter = 2 * car.wheel_centre * car.mesh_conv
        # Stop 2 cm below the outer face so the gripper releasing the tyre isn't a hit.
        height = car.wheel_outer_z * car.mesh_conv - 0.02
        return [("old tyre", hub * SE3(0, 0, height / 2), [diameter, diameter, height])]

    def all_extra_obstacles(self):
        # Everything checked besides the main obstacle: the old tyre and the random boxes.
        boxes = [(f"random box {i + 1}", pose, self.obstacle_size)
                 for i, pose in enumerate(self.random_boxes)]
        return self.old_tyre_box() + boxes

    # Carried-wheel clearance

    def carried_wheel_name(self):
        """Return the wheel currently attached to the Staubli, if there is one."""
        for name in (self.car.work_wheel, "new_wheel"):
            if self.mounts.get(name, (None, None))[0] == "staubli":
                return name
        return None

    def carried_wheel_centre(self, q):
        """Approximate the carried tyre by a sphere centred in the wheel mesh."""
        name = self.carried_wheel_name()
        if name is None:
            return None

        _, offset = self.mounts[name]
        wheel_pose = self.staubli.Robot.fkine(q) * offset
        car = self.car
        centre_local = SE3(
            car.wheel_centre * car.mesh_conv,
            car.wheel_centre * car.mesh_conv,
            car.wheel_outer_z * car.mesh_conv / 2
        )
        return (wheel_pose * centre_local).t

    def carried_wheel_clear(self, robot, q):
        """Keep a carried tyre out of the main car body while it is in transit.

        The final hub approach is intentionally allowed because the wheel must enter
        the wheel arch. Everywhere else, a conservative sphere around the tyre is
        checked against the car body's world-space bounds.
        """
        if robot is not self.staubli.Robot:
            return True

        centre = self.carried_wheel_centre(q)
        if centre is None:
            return True

        car = self.car
        hub = car.hub_world(car.work_wheel).t

        # The last part of the approach to the hub is an intentional close-contact zone.
        if np.linalg.norm(centre - hub) < 0.48:
            return True

        # Conservative body bounds derived from the W07 mesh after its scale/orientation.
        body_low = np.array([-2.40, -0.82, 0.04])
        body_high = np.array([2.70, 0.82, 1.12])
        tyre_radius = 0.31

        closest = np.minimum(np.maximum(centre, body_low), body_high)
        if np.linalg.norm(centre - closest) < tyre_radius:
            self.collision_manager.reason = "Staubli carried wheel would hit the car body"
            return False

        return True

    # Random obstacle and avoidance

    def drop_random_obstacle(self, robot, path):
        # Obstacle IN arms one drop onto a randomly chosen transit (joint move) of the
        # pit stop sequence. If no good spot is found there, it tries the next transit.
        if self.transit_index is None or self.obstacle_drop_at is None:
            return
        self.transit_index += 1
        if self.transit_index < self.obstacle_drop_at:
            return

        car = self.car
        keep_clear = [car.nut_target().t, car.wheel_target().t, car.jack_target().t,
                      *(car.wheel_target(stand).t for stand in self.stands.values())]
        others = [(r, r.q) for r in self.robot_list() if r is not robot]
        pose = self.collision_manager.random_obstacle_pose(
            robot, path, self.obstacle_size, keep_clear, others)
        if pose is None:
            return

        self.obstacle_drop_at = None
        self.obstacle_pose = pose
        self.obstacle.T = pose.A
        self.obstacle_active = True
        print(f"Obstacle dropped in the path of the {robot.name}")

        # Spawn the extra boxes at random places around this path. Keep them away from
        # the car, the stands and every robot's home pose so later moves can still finish.
        homes = [self.staubli.Robot.fkine(self.staubli_home).t]
        homes += [self.kuka.Robot.fkine(self.kuka_home).t, self.abb.Robot.fkine(self.abb_home).t]
        avoid = [car_footprint] + [(stand.t[:2] - .5, stand.t[:2] + .5) for stand in self.stands.values()]
        self.random_boxes = self.collision_manager.random_extra_obstacles(
            robot, path, self.obstacle_size, random_obstacle_count,
            keep_clear + homes, avoid, others, existing=[pose])
        for shape, box in zip(self.random_box_shapes, self.random_boxes):
            shape.T = box.A
        print(f"{len(self.random_boxes)} more boxes spawned at random places")
        self.render()

    def detour(self, robot, goal, steps):
        print(f"{robot.name}: path blocked, searching for a way around")
        pose = self.obstacle_pose if self.obstacle_active else None
        q_mid = self.collision_manager.find_detour(
            robot, robot.q, goal, pose, self.obstacle_size, steps // 2)
        if q_mid is None:
            self.safety.stop("COLLISION", self.collision_manager.reason)
            raise MotionStopped
        print(f"{robot.name}: detour found")
        first = rtb.jtraj(robot.q, q_mid, steps // 2).q
        return np.vstack([first, rtb.jtraj(q_mid, goal, steps // 2).q[1:]])

    # Robot movement

    def move_joint(self, robot, goal, steps=steps, transit=True):
        # transit=False for moves that must not detour (e.g. spinning the wheel gun).
        # Large Staubli joint moves get extra intermediate samples so the arm
        # visibly travels through the motion instead of appearing to snap.
        move_steps = steps
        if robot is self.staubli.Robot:
            max_joint_change = float(np.max(np.abs(np.asarray(goal) - robot.q)))
            smooth_steps = int(np.ceil(max_joint_change / np.deg2rad(1.5))) + 1
            move_steps = max(steps, smooth_steps)

        path = rtb.jtraj(robot.q, goal, move_steps).q
        if transit:
            self.drop_random_obstacle(robot, path)
            if not self.path_clear(robot, path):
                path = self.detour(robot, goal, move_steps)
        self.follow_path(robot, path)

    def follow_path(self, robot, path):
        self.active_robot = self.robot_name(robot)

        for q in path:
            self.guard(robot)

            # Stop on contact: halt before the step that would collide.
            if not self.path_clear(robot, [q]):
                self.safety.stop("COLLISION", self.collision_manager.reason)
                raise MotionStopped

            robot.q = q
            self.render()

        self.active_robot = None

    def move_to(self, robot, target, steps=steps):
        goal = self.solve(robot, target, robot.q)

        self.move_joint(robot, goal, steps)

    def move_line(self, robot, target, steps=steps):
        # Straight-line move: trapezoidal s(t) blends the tool position from start
        # to target (Lab 4/6), then IK at each point (Lab 3/4).
        start = robot.fkine(robot.q).t
        s = rtb.trapezoidal(0, 1, steps).q
        path = []
        q = robot.q.copy()

        for i in range(steps):
            pose = SE3.Rt(target.R, (1 - s[i]) * start + s[i] * target.t)
            q = self.solve(robot, pose, q)
            path.append(q.copy())

        self.follow_path(robot, path)

    def move_cartesian(self, robot, target, steps=steps):
        """Smoothly interpolate both tool position and orientation to target."""
        start = robot.fkine(robot.q)
        poses = rtb.ctraj(start, target, steps)
        path = []
        q = robot.q.copy()

        for pose in poses:
            q = self.solve(robot, pose, q)
            path.append(q.copy())

        self.follow_path(robot, path)

    def move_wheel_to_hub_clearance(self):
        """Carry the replacement wheel from the stand to a reachable hub approach.

        The robot is already sitting at a known-valid pose 0.2 m back from the
        replacement-wheel stand after pick_from_stand(). Do not add another retreat
        or vertical lift here: both previous versions pushed the TX90 outside its
        reachable workspace.

        Move directly to the normal 0.30 m hub approach. The IK solver is seeded from
        the robot's current joint state, so it stays on the nearby configuration
        branch instead of jumping through the old hard-coded carry pose.
        """
        staubli = self.staubli.Robot

        hub_approach = self.car.wheel_target() * back(0.30)

        # Solve one reachable end pose from the current configuration. move_to()
        # then collision-checks the interpolated joint path and detours if needed.
        self.move_to(
            staubli,
            hub_approach,
            80
        )

    def spin_tool(self, robot, angle, steps=steps):
        goal = robot.q.copy()
        lower, upper = robot.qlim[:, -1]

        if not lower <= goal[-1] + angle <= upper:
            angle = -angle

        goal[-1] += angle
        self.move_joint(robot, goal, steps, transit=False)

    # Car lift

    def set_lift(self, pitch, steps=steps):
        kuka = self.kuka.Robot
        self.active_robot = "kuka"

        for angle in np.linspace(self.car.pitch, pitch, steps):
            self.guard(kuka)

            pitch_before = self.car.pitch
            self.car.pitch = angle

            q = self.solve(
                kuka,
                self.car.jack_target(),
                kuka.q
            )

            # Stop on contact, as in follow_path.
            if not self.path_clear(kuka, [q]):
                self.car.pitch = pitch_before
                self.safety.stop("COLLISION", self.collision_manager.reason)
                raise MotionStopped

            kuka.q = q
            self.render()

        self.active_robot = None

    # Wheel handling

    def place_on_stand(self, name, stand):
        staubli = self.staubli.Robot
        target = self.car.wheel_target(self.stands[stand])

        self.move_to(staubli, target * back(0.2))
        self.move_line(staubli, target, 30)
        self.attach(name, "stand_" + stand)
        self.move_line(staubli, target * back(0.2), 30)

    def pick_from_stand(self, name, stand):
        staubli = self.staubli.Robot
        target = self.car.wheel_target(self.stands[stand])

        self.move_to(staubli, target * back(0.2))
        self.move_line(staubli, target, 30)
        self.attach(name, "staubli")
        self.move_line(staubli, target * back(0.2), 30)
