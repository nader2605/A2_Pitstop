"""Stäubli TX90 model with its wheel gripper, and a standalone Swift joint GUI."""

from pathlib import Path
import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import swift
from swift import Slider, Button
from spatialmath import SE3

 
Robot_dir = Path(__file__).resolve().parent
Visual_dir = Robot_dir / "assets" / "staubli" / "tx90" / "visual"

Staubli_grey = [0.75, 0.75, 0.75, 1.0]
Lucas_toolLength = 0.10
tool_grey = [0.3, 0.3, 0.3, 1.0]
dark_grey = [0.2, 0.2, 0.2, 1.0]


def mesh(name, pose=SE3(), color=Staubli_grey):
    return [sg.Mesh(str(Visual_dir / f"{name}.stl"), pose=pose, color=color)]


def add_tool(robot, tool_pose, shapes):
    # Set the tool position and attach its visual parts.
    robot.tool = tool_pose

    for shape in shapes:
        shape.T = (tool_pose * SE3(shape.T)).A

    robot.links[-1].geometry = list(robot.links[-1].geometry) + shapes


class Staubli:
    def __init__(self):
        links = [
            rtb.RevoluteDH(d=.478, a=.050, alpha=-np.pi/2,
                           qlim=np.deg2rad([-180, 180])),
            rtb.RevoluteDH(a=.425, offset=-np.pi/2,
                           qlim=np.deg2rad([-130, 147.5])),
            rtb.RevoluteDH(d=.050, alpha=np.pi/2, offset=np.pi/2,
                           qlim=np.deg2rad([-145, 145])),
            rtb.RevoluteDH(d=.425, alpha=-np.pi/2,
                           qlim=np.deg2rad([-270, 270])),
            rtb.RevoluteDH(alpha=np.pi/2,
                           qlim=np.deg2rad([-115, 140])),
            rtb.RevoluteDH(d=.100,
                           qlim=np.deg2rad([-270, 270])),
        ]

        self.Robot = rtb.DHRobot(links, name="Staubli TX90")

        self.mesh_offsets = [
            SE3.Tx(-.050) * SE3.Rx(np.pi/2),
            SE3.Tx(-.425) * SE3.Rz(np.pi/2) * SE3.Rx(np.pi/2),
            SE3(),
            SE3.Ty(.425) * SE3.Rx(np.pi/2),
            SE3(),
            SE3(),
        ]

        for i, link in enumerate(self.Robot.links):
            link.geometry = mesh(f"link_{i + 1}", self.mesh_offsets[i])

        # Add wheel gripper 
        lucas_tool = SE3(0, 0, Lucas_toolLength)
        lucas_shapes = [
            sg.Cylinder(
                0.035, Lucas_toolLength,
                pose=SE3(0, 0, -Lucas_toolLength / 2),
                color=tool_grey
            ),
            sg.Cylinder(
                0.14, 0.02,
                pose=SE3(0, 0, -0.01),
                color=dark_grey
            )
        ]
        add_tool(self.Robot, lucas_tool, lucas_shapes)

        self.base_mesh = mesh("base_link", self.Robot.base, dark_grey)[0]
        self.qr = np.zeros(6)
        self.Robot.q = self.qr.copy()


def move_joint(robot, angle, joint):
    q = robot.Robot.q.copy()
    q[joint] = np.deg2rad(float(angle))
    robot.Robot.q = q


if __name__ == "__main__":
    robot = Staubli()
    env = swift.Swift()
    env.launch(realtime=True)
    env.add(robot.base_mesh)
    env.add(robot.Robot)

    sliders = []
    for i, link in enumerate(robot.Robot.links):
        lower, upper = np.rad2deg(link.qlim)
        slider = Slider(
            cb=lambda angle, j=i: move_joint(robot, angle, j),
            min=float(np.ceil(lower * 10) / 10),
            max=float(np.floor(upper * 10) / 10),
            step=.1,
            value=round(float(np.rad2deg(robot.Robot.q[i])), 1),
            desc=f"Joint {i + 1}",
            unit="deg",
        )
        sliders.append(slider)
        env.add(slider)

    reset = [False]
    env.add(Button(cb=lambda _: reset.__setitem__(0, True), desc="Reset"))

    try:
        while True:
            if reset[0]:
                reset[0] = False
                for q in rtb.jtraj(robot.Robot.q, robot.qr, 50).q:
                    robot.Robot.q = q
                    env.step(.03)
                for slider in sliders:
                    slider.value = 0
            env.step(.03)
    except KeyboardInterrupt:
        pass

