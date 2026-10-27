# Pit Stop full task sequence
# 41013 Industrial Robotics - Lab Assignment 2

import numpy as np

from pitstop_utils import back


class SequenceMixin:

    def run_sequence(self):
        car = self.car
        kuka = self.kuka.Robot
        abb = self.abb.Robot
        staubli = self.staubli.Robot

        # The marshal signals that the pit-stop cell is active.
        print("UR3e: extending the pit-stop lollipop")
        self.move_lollipop(self.ur3e.q_hold)

        print("Kuka: lifting car")

        self.move_to(
            kuka,
            car.jack_target() * back(0.3)
        )

        self.move_line(
            kuka,
            car.jack_target(),
            30
        )

        self.set_lift(
            car.lift_pitch
        )

        print("ABB: removing wheel nut")

        self.move_to(
            abb,
            car.nut_target() * back(0.15)
        )

        self.move_line(
            abb,
            car.nut_target(),
            30
        )

        self.attach("nut", "abb")
        self.spin_tool(abb, -2 * np.pi)

        self.move_line(
            abb,
            car.nut_target() * back(0.2),
            30
        )

        self.move_joint(
            abb,
            self.abb_home
        )

        print("Staubli: removing old wheel")

        self.move_to(
            staubli,
            car.wheel_target() * back(0.15)
        )

        self.move_line(
            staubli,
            car.wheel_target(),
            30
        )

        self.attach(
            car.work_wheel,
            "staubli"
        )

        self.move_line(
            staubli,
            car.wheel_target() * back(0.3)
        )

        self.place_on_stand(
            car.work_wheel,
            "old"
        )

        print("Staubli: fitting new wheel")

        self.pick_from_stand(
            "new_wheel",
            "new"
        )

        # Carry the replacement wheel through a controlled outboard clearance route.
        # This avoids the old hard-coded joint waypoint, which could make the elbow
        # fold underneath the car even though the tool ended up at the right place.
        self.move_wheel_to_hub_clearance()

        self.move_line(
            staubli,
            car.wheel_target()
        )

        self.attach(
            "new_wheel",
            "hub_" + car.work_wheel
        )

        self.move_line(
            staubli,
            car.wheel_target() * back(0.15),
            30
        )

        self.move_joint(
            staubli,
            self.staubli_home
        )

        print("ABB: refitting wheel nut")

        self.move_to(
            abb,
            car.nut_target() * back(0.2)
        )

        self.move_line(
            abb,
            car.nut_target(),
            30
        )

        self.spin_tool(
            abb,
            2 * np.pi
        )

        self.attach(
            "nut",
            "hub_" + car.work_wheel
        )

        self.move_line(
            abb,
            car.nut_target() * back(0.15),
            30
        )

        self.move_joint(
            abb,
            self.abb_home
        )

        # Remove the sign before allowing the car to leave.
        print("UR3e: retracting the pit-stop lollipop")
        self.move_lollipop(self.ur3e.q_home)

        print("Kuka: lowering car")

        self.set_lift(0)

        self.move_line(
            kuka,
            car.jack_target() * back(0.3),
            30
        )

        self.move_joint(
            kuka,
            self.kuka_home
        )

        print("Pit stop complete")
