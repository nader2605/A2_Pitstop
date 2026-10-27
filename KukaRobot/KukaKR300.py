"Kuka KR300"
"Purpose: Act as car jack."


import roboticstoolbox as rtb
import numpy as np
import swift
from swift import Slider, Button
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path

#Mesh files
mesh_loc = Path(__file__).resolve().parent / "kr300_r2700_2" / "visual"
kuka_orange = [1.0, 0.36, 0.0, 1.0]
dark_grey = [0.2, 0.2, 0.2, 1.0]
lift_colour = [0.3, 0.3, 0.3, 1.0]

q_mesh = np.array([0, -np.pi/2, np.pi/2, 0, 0, 0])

# Front wing lifter end effector
lifter_stl = Path(__file__).resolve().parent / "front_wing_lifter_ASSEMBLED.stl"
lifter_scale = 0.001
lifter_flange_z = 0.450

#flipping the lifter to match the robot's flange orientation
flange = SE3.Rx(np.pi)

def mesh(name, pose=SE3(), color=kuka_orange):
    path = mesh_loc / (name + ".dae")
    if not path.exists():
        raise FileNotFoundError(f"Mesh not found: {path}")
    return [sg.Mesh(str(path), pose=pose, color=color)]

class KukaKR300:
    def __init__(self):
        links = [
            rtb.RevoluteDH(d=.645, a=0.330, alpha=-np.pi/2, qlim=[-3.2289, 3.2289]),
            rtb.RevoluteDH(d=0, a=1.150, alpha=0, qlim=[-2.4435, -0.0873]),
            rtb.RevoluteDH(d=0, a=0.115, alpha=np.pi/2, offset=-np.pi/2, qlim=[-2.0944, 2.9322]),
            rtb.RevoluteDH(d=-1.220, a=0, alpha=-np.pi/2, qlim=[-6.1087, 6.1087]),
            rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, qlim=[-2.1817, 2.1817]),
            rtb.RevoluteDH(d=-0.240, a=0, alpha=0, qlim=[-6.1087, 6.1087]),
        ]
        self.Robot = rtb.DHRobot(links, name='Kuka KR300')

        T_mesh = self.Robot.fkine_all(q_mesh)
        for i, link in enumerate(self.Robot.links):
            color = dark_grey if i == 5 else kuka_orange
            link.geometry = mesh(f"link_{i + 1}", T_mesh[i + 1].inv(), color)
        #Adding the base that the robot will be mounted on
        self.base_mesh = mesh("base_link", self.Robot.base, dark_grey)[0]

        lifter_pose = flange * SE3(0, 0, lifter_flange_z)
        self.lifter = sg.Mesh(
            str(lifter_stl),
            pose=lifter_pose,
            scale=[lifter_scale] * 3,
            color=lift_colour
        )
        last_link = self.Robot.links[-1]
        last_link.geometry = list(last_link.geometry) + [self.lifter]
        self.Robot.tool = lifter_pose

        self.qr = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
        self.Robot.q = self.qr


def move_joint(robot, angle, joint):
    q = robot.Robot.q.copy()
    q[joint] = np.deg2rad(float(angle))  # Swift may send the value as a string
    robot.Robot.q = q


if __name__ == "__main__":
    robot = KukaKR300()
    print("Kuka KR300 robot model created.")
    q_zero = np.zeros(6)
    pose = robot.Robot.fkine(q_zero)
    env = swift.Swift()
    env.launch(realtime=True)
    env.add(robot.base_mesh)
    env.add(robot.Robot)
    env.step(0.05)

    # One slider per joint (0.1 deg resolution), starting at the robot's current pose
    sliders = []
    for i in range(6):
        lower, upper = np.rad2deg(robot.Robot.links[i].qlim)
        slider = Slider(
            cb=lambda angle, j=i: move_joint(robot, angle, j),
            min=float(np.ceil(lower * 10) / 10),   # round inwards so the
            max=float(np.floor(upper * 10) / 10),  # slider stays within qlim
            step=0.1,
            value=round(float(np.rad2deg(robot.Robot.q[i])), 1),
            desc=f"Joint {i + 1}",
            unit="deg"
        )
        sliders.append(slider)
        env.add(slider)

    reset_requested = [False]
    env.add(Button(cb=lambda _: reset_requested.__setitem__(0, True), desc="reset"))

    #slider input
    try:
        while True:
            if reset_requested[0]:
                reset_requested[0] = False
                for q in rtb.jtraj(robot.Robot.q, robot.qr, 50).q:
                    robot.Robot.q = q
                    env.step(0.05)
                for slider, q in zip(sliders, robot.qr):
                    slider.value = round(float(np.rad2deg(q)), 1)
            env.step(0.05)
    except KeyboardInterrupt:
        pass