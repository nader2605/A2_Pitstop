# Pit Stop Environment
# 41013 Industrial Robotics - Lab Assignment 2

import numpy as np
import swift
from spatialmath import SE3

from KukaRobot.KukaKR300 import KukaKR300
from irb140.ABB_irb140 import ABBIRB140
from Staubli.Staubli import Staubli
from UR3e.UR3e_robot import UR3eLollipop
from Car.CarModel import CarModel
from collision import CollisionManager
from safety import Safety
from controller_estop import ControllerEstop, A_button, B_button, X_button, Start_button
from recovery import Recovery

from pitstop_config import dt, steps, Kuka_base_posXY
from pitstop_utils import facing, MotionStopped
from pitstop_scene import SceneMixin
from pitstop_attachments import AttachmentMixin
from pitstop_motion import MotionMixin
from pitstop_gui import GUIMixin
from pitstop_sequence import SequenceMixin


class PitStop(SceneMixin, AttachmentMixin, MotionMixin, GUIMixin, SequenceMixin):

    def __init__(self, env):
        self.env = env
        self.controller_estop = ControllerEstop(B_button)  # B button (1) = E-STOP
        self.safety = Safety(self.controller_estop)
        self.recovery = Recovery(self)

        self.run_requested = False
        self.reset_requested = False
        self.jog_request = None
        self.selected_robot = 0
        self.active_robot = None

        self.car = CarModel()
        self.kuka = KukaKR300()
        self.abb = ABBIRB140()
        self.staubli = Staubli()
        self.ur3e = UR3eLollipop()

        self.collision_manager = CollisionManager()

        car = self.car
        kuka = self.kuka.Robot
        abb = self.abb.Robot
        staubli = self.staubli.Robot

        # Robot positions
        front_hub = car.hub_pose("front", -1).t
        jack = car.jack_offset.t

        kuka.base = facing(Kuka_base_posXY[0], Kuka_base_posXY[1], jack)
        abb.base = SE3(front_hub[0] + 0.35, -1.6, 0) * SE3.Rz(np.pi / 2)
        staubli.base = SE3(front_hub[0] - 0.7, -1.75, 0) * SE3.Rz(np.pi / 2)

        # Lollipop marshal robot, positioned at the front of the pit box.
        self.ur3e.set_base(SE3(3.16, 1.16, 0.62))
        car_min_x = car.mesh_point([0, 0, car.centre[2]])[0]
        car_max_x = car.mesh_point([299.2, 0, car.centre[2]])[0]
        self.ur3e_car_box = (
            np.array([(car_min_x + car_max_x) / 2, 0, 0.58]),
            np.array([car_max_x - car_min_x, 1.90, 1.16])
        )

        self.kuka.base_mesh.T = kuka.base.A
        self.abb.base_mesh.T = abb.base.A
        self.staubli.base_mesh.T = staubli.base.A

        # Home positions
        self.kuka_home = self.kuka.qr.copy()
        # Hand-picked ABB park pose: arm tucked back so the Staubli can reach the wheel past it.
        self.abb_home = np.deg2rad([-90, -60, 30, 0, 90, 0])
        self.staubli_home = np.array([0, 0, np.pi / 2, 0, np.pi / 2, 0])

        kuka.q = self.kuka_home.copy()
        abb.q = self.abb_home.copy()
        staubli.q = self.staubli_home.copy()

        self.build_floor_and_stands()

        self.setup_attachments()
        self.collision_manager.extra_obstacles = self.all_extra_obstacles
        self.collision_manager.robots = [staubli, kuka, abb]

        self.build_obstacle_and_curtain()

        self.update_props()

        self.add_to_env()

        self.add_gui()
        self.render()

    def render(self):
        self.update_props()
        self.ur3e.update()
        self.recovery.record()
        self.safety.poll()

        self.status.label = f"STATE: {self.safety.state}"

        if self.safety.reason:
            self.status.label += f" | {self.safety.reason}"

        if self.env is not None:
            self.env.step(.03 if self.safety.state == "RECOVERING" else dt)

    # Reset / jog / run

    def reset_scene(self):
        # Controller E-stop recovery for the demo is intentionally simple:
        # GUI RESET restores the simulation to stage 0.
        self.kuka.Robot.q = self.kuka_home.copy()
        self.abb.Robot.q = self.abb_home.copy()
        self.staubli.Robot.q = self.staubli_home.copy()
        self.ur3e.q = self.ur3e.q_home.copy()
        self.ur3e.update()
        self.car.pitch = 0.0
        self.mounts = dict(self.initial_mounts)
        self.clear_obstacle()
        self.active_robot = None
        self.recovery.history.clear()
        self.recovery.signature = None
        self.recovery.recording = False
        self.recovery.pending = False
        self.recovery.complete = False
        self.run_requested = False
        self.jog_request = None
        self.safety.recovery_requested = False
        # Never clear a held emergency-stop input.
        if not self.controller_estop.pressed():
            self.safety.state = "READY"
            self.safety.reason = ""
        else:
            self.safety.stop("ESTOP", "Release B before starting")
        self.update_props()

    def move_lollipop(self, target):
        """Move the UR3e in checked steps; B or X interrupts each step."""
        path = self.ur3e.checked_path(
            target, self.ur3e_car_box,
            (self.kuka.Robot, self.abb.Robot, self.staubli.Robot)
        )
        self.active_robot = "ur3e"
        try:
            for q in path:
                self.guard()
                self.ur3e.q = q.copy()
                self.render()
        finally:
            self.active_robot = None

    def jog_selected(self):
        dx, dy, dz = self.jog_request
        self.jog_request = None

        robot = self.robot_list()[self.selected_robot]

        target = (
            SE3(dx, dy, dz)
            * robot.fkine(robot.q)
        )

        if not self.safety.start():
            return
        self.recovery.begin()

        try:
            self.move_to(robot, target, 25)

        except MotionStopped:
            return

        self.safety.state = "READY"

    def recover_scene(self):
        """Retrace recorded frames after B is released; does not resume AUTO."""
        if self.safety.state != "ESTOP" or self.controller_estop.pressed():
            return
        if len(self.recovery.history) <= 1:
            self.safety.reason = "No recorded recovery path; use X for full reset"
            return
        self.recovery.recording = False
        self.safety.state = "RECOVERING"
        self.safety.reason = "Recorded reverse recovery"
        try:
            self.recovery.reverse()
        except MotionStopped:
            print("Recovery interrupted; stopped in place")

    def run(self):
        if self.recovery.complete and not self.recovery.pending:
            self.safety.reason = "Cycle complete: RESET then START to recover before a new task"
            return
        if not self.safety.start():
            return

        try:
            if self.recovery.pending:
                self.recovery.reverse()
            else:
                self.recovery.begin()
                self.transit_index = 0
                try:
                    self.run_sequence()
                finally:
                    self.transit_index = None
                self.recovery.complete = True

        except MotionStopped:
            if self.recovery.pending:
                print("AUTO stopped. Release the controller E-stop, then use RESET to return to stage 0.")
            else:
                print("Stopped in place.")
            return

        self.safety.state = "READY"
        self.safety.reason = ""

        print("System READY")


if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)

    pit_stop = PitStop(env)

    print("Controller: Start = START AUTO | B = E-STOP | A = RECOVERY | X = FULL RESET")

    try:
        while True:
            pit_stop.safety.poll()

            # Start (6): begin only from READY.
            if pit_stop.controller_estop.pressed_once(Start_button) and pit_stop.safety.state == "READY":
                pit_stop.run_requested = True
                print("Controller Start: START AUTO")

            # A (0): retrace recorded motion only after B has been released.
            if pit_stop.controller_estop.pressed_once(A_button) and pit_stop.safety.state == "ESTOP":
                pit_stop.safety.recovery_requested = True

            # X (2) is polled by safety.poll(), including during robot motion.
            # Reset itself happens here once motion has stopped.

            if pit_stop.reset_requested or pit_stop.safety.reset_requested:
                pit_stop.reset_requested = False
                pit_stop.safety.reset_requested = False
                pit_stop.reset_scene()

            if pit_stop.safety.recovery_requested:
                pit_stop.safety.recovery_requested = False
                pit_stop.recover_scene()

            if (
                pit_stop.jog_request
                and
                pit_stop.safety.state == "READY"
            ):
                pit_stop.jog_selected()

            if pit_stop.run_requested or pit_stop.safety.start_requested:
                pit_stop.run_requested = False
                pit_stop.safety.start_requested = False
                pit_stop.run()

            pit_stop.render()

    except KeyboardInterrupt:
        pass
