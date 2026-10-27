"""
Pit Stop Environment
41013 Industrial Robotics - Lab Assignment 2
Purpose: Swift scene of a Mercedes W07 tyre change.
    Kuka KR300   - jacks up the front of the car
    ABB IRB140   - removes and refits the wheel nut
    Staubli TX90 - swaps the front right wheel for a new tyre
"""

import numpy as np
import roboticstoolbox as rtb
import swift
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path

from KukaRobot.KukaKR300 import KukaKR300
from irb140.ABB_irb140 import ABBIRB140
from Staubli.Staubli import Staubli

Car_dir = Path(__file__).resolve().parent / "Car"

# W07.STL and Wheel.STL share units; a 39 unit wheel is a 660 mm F1 tyre
Mesh_to_m = 0.66 / 39.0

# Landmarks measured on W07.STL (mesh units, Y up, nose along +X, Z across)
Car_centre = np.array([141.2, 0.0, 500.0])  # ground, midway between axles
Front_hub_x = 239.9
Rear_hub_x = 42.5
Hub_y = 19.5
Hub_face_z = {"front": 39.0, "rear": 38.0}  # hub face offset from centreline
Jack_point = np.array([285.0, 7.0, 500.0])  # underside of the nose

# Wheel.STL: axis along +Z, inner face at z=0, nut seat at z~22, outer face z=23
Wheel_centre_xy = 19.5
Nut_seat_z = 22.0
Wheel_outer_z = 23.0
Nut_length = 0.025  # WheelNut.STL: mm, base face at z=0, 25 mm long

Car_silver = [0.75, 0.77, 0.80, 1.0]
Tyre_black = [0.08, 0.08, 0.08, 1.0]
New_tyre = [0.35, 0.05, 0.05, 1.0]  # tinted so the fresh tyre stands out
Stand_grey = [0.55, 0.55, 0.6, 1.0]
Nut_red = [0.85, 0.1, 0.1, 1.0]
Tool_grey = [0.3, 0.3, 0.3, 1.0]
Dark_grey = [0.2, 0.2, 0.2, 1.0]
Floor_grey = [0.35, 0.35, 0.38, 1.0]

# Wheel being changed: front right (car -y side, where the ABB and Staubli stand)
Work_wheel = "FR"
WHEELS = {"FL": ("front", 1), "FR": ("front", -1),
          "RL": ("rear", 1), "RR": ("rear", -1)}

# Front-end lift, pivoting about the rear axle (~70 mm at the front wheels)
Lift_pitch = np.deg2rad(1.2)

# Kuka stands off the front-left corner, not in front of the nose
Kuka_base_xy = (3.7, 1.9)

# Tools: tool frame at the tip, z pointing out of the flange
Kuka_tool_length = 0.45  # Kuka flange faces along -z6
Abb_tool_length = 0.078  # Facom 19 mm socket length
Staubli_tool_length = 0.10

# Tyre stands beside the Staubli (offsets from its base); wheels lie flat on top
Stand_height = 0.25
Stand_radius = 0.3
Stand_offsets = {"old": (-0.55, -0.1), "new": (-0.1, -0.65)}

DT = 0.02
STEPS = 50


def mesh_point(p):
    """Convert a W07.STL point (mesh units) into the car frame (metres)."""
    d = (np.asarray(p, dtype=float) - Car_centre) * Mesh_to_m
    return np.array([d[0], -d[2], d[1]])


# Car frame: x forward, y left, z up; mesh Y-up is rotated to Z-up
Car_mesh_pose = SE3.Rx(np.pi/2) * SE3(*(-Car_centre * Mesh_to_m))
Wheel_mesh_pose = SE3(-Wheel_centre_xy * Mesh_to_m, -Wheel_centre_xy * Mesh_to_m, 0)
Nut_pose = SE3(0, 0, Nut_seat_z * Mesh_to_m)  # nut mesh base on the seat
Jack_pose = SE3(mesh_point(Jack_point)) * SE3.Ry(-np.pi/2)  # tool z into the nose
Rear_pivot = mesh_point([Rear_hub_x, Hub_y, Car_centre[2]])


def hub_pose(axle, side):
    """Hub face frame in the car frame, z pointing outboard. side: +1 left, -1 right."""
    x = Front_hub_x if axle == "front" else Rear_hub_x
    p = mesh_point([x, Hub_y, Car_centre[2] - side * Hub_face_z[axle]])
    return SE3(p) * SE3.Rx(-side * np.pi/2)


def back(d):
    """Offset that backs off d metres along the tool axis."""
    return SE3(0, 0, -d)


def facing(x, y, target):
    """Base pose at (x, y) with its x axis pointing at target (x, y)."""
    return SE3(x, y, 0) * SE3.Rz(np.arctan2(target[1] - y, target[0] - x))


def add_tool(robot, tool, shapes):
    """Set robot.tool and attach shapes, given in the tool frame, to the last link."""
    robot.tool = tool
    for s in shapes:
        s.T = (tool * SE3(s.T)).A
    robot.links[-1].geometry = list(robot.links[-1].geometry) + shapes


class PitStop:
    def __init__(self, env):
        self.env = env
        self.pitch = 0.0

        # --- Robots ---
        self.kuka = KukaKR300()
        self.abb = ABBIRB140()
        self.staubli = Staubli()
        kuka, abb, staubli = self.kuka.Robot, self.abb.Robot, self.staubli.Robot

        front_hub = hub_pose(*WHEELS[Work_wheel]).t
        jack = Jack_pose.t
        kuka.base = facing(*Kuka_base_xy, jack)
        abb.base = SE3(front_hub[0] + 0.35, -1.6, 0) * SE3.Rz(np.pi/2)
        staubli.base = SE3(front_hub[0] - 0.7, -1.75, 0) * SE3.Rz(np.pi/2)  # room to swing the wheel aside
        for robot in (self.kuka, self.abb, self.staubli):
            robot.base_mesh.T = robot.Robot.base.A

        # Jack fork, wheel gun and wheel gripper
        add_tool(kuka, SE3(0, 0, -Kuka_tool_length) * SE3.Rx(np.pi), [
            sg.Cuboid([0.12, 0.12, Kuka_tool_length], pose=SE3(0, 0, -Kuka_tool_length / 2), color=Tool_grey),
            sg.Cuboid([0.03, 0.3, 0.2], pose=SE3(0, 0, -0.1), color=Dark_grey)])
        # Socket nut end at the tool tip, drive end on the flange
        add_tool(abb, SE3(0, 0, Abb_tool_length), [
            sg.Mesh(str(Path(__file__).resolve().parent / "irb140" / "Facom_NS19LA_19mm_socket.stl"),
                    pose=SE3.Rx(np.pi), scale=[0.001] * 3, color=Dark_grey)])
        add_tool(staubli, SE3(0, 0, Staubli_tool_length), [
            sg.Cylinder(0.035, Staubli_tool_length,
                        pose=SE3(0, 0, -Staubli_tool_length / 2), color=Tool_grey),
            sg.Cylinder(0.14, 0.02, pose=SE3(0, 0, -0.01), color=Dark_grey)])

        self.kuka_home = self.kuka.qr
        self.abb_home = np.array([-np.pi/2, -np.pi/2, 0, 0, np.pi/2, 0])  # parked clear of the wheel
        self.staubli_home = np.array([0, 0, np.pi/2, 0, np.pi/2, 0])
        kuka.q = self.kuka_home
        abb.q = self.abb_home
        staubli.q = self.staubli_home

        # --- Car, wheels and wheel nut ---
        self.floor = sg.Cuboid([12, 7, 0.01], pose=SE3(0.5, 0, -0.006), color=Floor_grey)
        self.car = sg.Mesh(str(Car_dir / "W07.STL"), scale=[Mesh_to_m] * 3, color=Car_silver)
        self.wheels = {name: sg.Mesh(str(Car_dir / "Wheel.STL"), scale=[Mesh_to_m] * 3, color=Tyre_black) for name in WHEELS}
        self.nut = sg.Mesh(str(Car_dir / "WheelNut.STL"), scale=[0.001] * 3, color=Nut_red)
        self.new_tyre = sg.Mesh(str(Car_dir / "Wheel.STL"), scale=[Mesh_to_m] * 3, color=New_tyre)

        # Stand tops: a wheel frame resting there has its inner face down, z up
        lx, ly = staubli.base.t[:2]
        self.stands = {name: SE3(lx + dx, ly + dy, Stand_height) for name, (dx, dy) in Stand_offsets.items()}
        self.stand_shapes = [sg.Cylinder(Stand_radius, Stand_height,
                                         pose=T * SE3(0, 0, -Stand_height / 2), color=Stand_grey) for T in self.stands.values()]

        # Each prop follows a parent pose: (parent function, offset)
        self.mounts = {"car": (self.car_pose, Car_mesh_pose)}
        for name, (axle, side) in WHEELS.items():
            self.mounts[name] = (lambda a=axle, s=side: self.car_pose() * hub_pose(a, s), Wheel_mesh_pose)
        self.mounts["nut"] = (lambda: self.hub_world(Work_wheel), Nut_pose)
        self.mounts["new_tyre"] = (lambda: self.stands["new"], Wheel_mesh_pose)
        self.props = {"car": self.car, "nut": self.nut, "new_tyre": self.new_tyre, **self.wheels}

        if env is not None:
            for shape in [self.floor, self.kuka.base_mesh, self.abb.base_mesh,
                          self.staubli.base_mesh, *self.stand_shapes, *self.props.values()]:
                env.add(shape)
            for robot in (kuka, abb, staubli):
                env.add(robot)
        self.render()

    # --- Scene state ---
    def car_pose(self):
        """Car frame in the world: pitched nose-up about the rear axle."""
        return SE3(Rear_pivot) * SE3.Ry(-self.pitch) * SE3(-Rear_pivot)

    def hub_world(self, wheel):
        return self.car_pose() * hub_pose(*WHEELS[wheel])

    def prop_pose(self, name):
        parent, offset = self.mounts[name]
        return parent() * offset

    def attach(self, name, parent):
        """Re-parent a prop without moving it."""
        self.mounts[name] = (parent, parent().inv() * self.prop_pose(name))

    def render(self):
        for name, shape in self.props.items():
            shape.T = self.prop_pose(name).A
        if self.env is not None:
            self.env.step(DT)

    # --- Motion ---
    @staticmethod
    def ee(robot):
        return lambda: robot.fkine(robot.q)

    def solve(self, robot, T, q0):
        sol = robot.ikine_LM(T, q0=q0, joint_limits=True, seed=0)
        if not sol.success:
            raise RuntimeError(f"{robot.name}: no IK solution for\n{T}")
        # Take the 2*pi equivalent closest to q0 (e.g. after spinning the wheel gun)
        q = q0 + (sol.q - q0 + np.pi) % (2 * np.pi) - np.pi
        lo, hi = robot.qlim
        return np.where((q >= lo) & (q <= hi), q, sol.q)

    def move_joint(self, robot, q_goal, steps=STEPS):
        for q in rtb.jtraj(robot.q, q_goal, steps).q:
            robot.q = q
            self.render()

    def move_to(self, robot, T, steps=STEPS):
        """Joint-space move to a pose (used for free-space moves)."""
        self.move_joint(robot, self.solve(robot, T, robot.q), steps)

    def move_line(self, robot, T, steps=STEPS):
        """Straight-line move: trapezoidal s(t) blends the position (Lab 4/6), IK at each step."""
        start = robot.fkine(robot.q).t
        for s in rtb.trapezoidal(0, 1, steps).q:
            robot.q = self.solve(robot, SE3.Rt(T.R, (1 - s) * start + s * T.t), robot.q)
            self.render()

    def spin_tool(self, robot, angle, steps=STEPS):
        """Spin the last joint (the tool axis), e.g. a wheel gun turning a nut."""
        q = robot.q.copy()
        lo, hi = robot.qlim[:, -1]
        if not lo <= q[-1] + angle <= hi:
            angle = -angle
        for qi in rtb.jtraj(q, q + np.r_[np.zeros(5), angle], steps).q:
            robot.q = qi
            self.render()

    # --- Pit stop ---
    def jack_target(self):
        return self.car_pose() * Jack_pose

    def set_lift(self, pitch, steps=STEPS):
        """Kuka holds the jack point while the car pitches with it."""
        kuka = self.kuka.Robot
        for p in np.linspace(self.pitch, pitch, steps):
            self.pitch = p
            kuka.q = self.solve(kuka, self.jack_target(), kuka.q)
            self.render()

    def nut_target(self):
        """Wheel gun tip on the nut face, pointing along the axle."""
        return self.hub_world(Work_wheel) * Nut_pose * SE3(0, 0, Nut_length) * SE3.Rx(np.pi)

    def wheel_target(self):
        """Gripper plate on the wheel's outer face, pointing along the axle."""
        return self.hub_world(Work_wheel) * SE3(0, 0, Wheel_outer_z * Mesh_to_m) * SE3.Rx(np.pi)

    def stand_target(self, stand):
        """Gripper plate on the outer face of a wheel lying on a stand."""
        return self.stands[stand] * SE3(0, 0, Wheel_outer_z * Mesh_to_m) * SE3.Rx(np.pi)

    def place_on_stand(self, name, stand):
        """Staubli lowers the wheel it holds onto a stand and lets go."""
        staubli = self.staubli.Robot
        self.move_to(staubli, self.stand_target(stand) * back(0.2))
        self.move_line(staubli, self.stand_target(stand), 30)
        self.attach(name, lambda: self.stands[stand])
        self.move_line(staubli, self.stand_target(stand) * back(0.2), 30)

    def pick_from_stand(self, name, stand):
        """Staubli grips a wheel lying on a stand and lifts it clear."""
        staubli = self.staubli.Robot
        self.move_to(staubli, self.stand_target(stand) * back(0.2))
        self.move_line(staubli, self.stand_target(stand), 30)
        self.attach(name, self.ee(staubli))
        self.move_line(staubli, self.stand_target(stand) * back(0.2), 30)

    def run(self):
        kuka, abb, staubli = self.kuka.Robot, self.abb.Robot, self.staubli.Robot
        hub = lambda: self.hub_world(Work_wheel)

        print("Kuka KR300: jacking up the front of the car")
        self.move_to(kuka, self.jack_target() * back(0.3))
        self.move_line(kuka, self.jack_target(), 30)
        self.set_lift(Lift_pitch)

        print("ABB IRB140: removing the wheel nut")
        self.move_to(abb, self.nut_target() * back(0.15))
        self.move_line(abb, self.nut_target(), 30)
        self.attach("nut", self.ee(abb))
        self.spin_tool(abb, -2 * np.pi)
        self.move_line(abb, self.nut_target() * back(0.2), 30)
        self.move_joint(abb, self.abb_home)

        print("Staubli TX90: removing the old wheel")
        self.move_to(staubli, self.wheel_target() * back(0.15))
        self.move_line(staubli, self.wheel_target(), 30)
        self.attach(Work_wheel, self.ee(staubli))
        self.move_line(staubli, self.wheel_target() * back(0.3))
        self.place_on_stand(Work_wheel, "old")

        print("Staubli TX90: fitting the new tyre")
        self.pick_from_stand("new_tyre", "new")
        self.move_to(staubli, self.wheel_target() * back(0.3))
        self.move_line(staubli, self.wheel_target())
        self.attach("new_tyre", hub)
        self.move_line(staubli, self.wheel_target() * back(0.15), 30)
        self.move_joint(staubli, self.staubli_home)

        print("ABB IRB140: refitting the wheel nut")
        self.move_to(abb, self.nut_target() * back(0.2))
        self.move_line(abb, self.nut_target(), 30)
        self.spin_tool(abb, 2 * np.pi)
        self.attach("nut", hub)
        self.move_line(abb, self.nut_target() * back(0.15), 30)
        self.move_joint(abb, self.abb_home)

        print("Kuka KR300: lowering the car")
        self.set_lift(0.0)
        self.move_line(kuka, self.jack_target() * back(0.3), 30)
        self.move_joint(kuka, self.kuka_home)
        print("Pit stop complete")


if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)
    pit_stop = PitStop(env)
    input("Press enter to start the pit stop")
    pit_stop.run()
    env.hold()
