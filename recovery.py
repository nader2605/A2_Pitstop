"""Record scene motion and provide controlled reverse recovery."""
import numpy as np


class Recovery:
    def __init__(self, cell):
        self.cell = cell
        self.history = []
        self.recording = False
        self.pending = False
        self.complete = False
        self.signature = None

    def capture(self):
        c = self.cell
        return ([r.q.copy() for r in c.robot_list()] + [c.ur3e.q.copy()], float(c.car.pitch), dict(c.mounts))

    def record(self):
        if not self.recording or self.cell.safety.state == "RECOVERING":
            return

        state = self.capture()
        signature = (
            tuple(q.tobytes() for q in state[0]),
            state[1],
            tuple(
                (name, parent, offset.A.tobytes())
                for name, (parent, offset) in state[2].items()
            ),
        )

        if signature != self.signature:
            self.history.append(state)
            self.signature = signature

    def begin(self):
        self.history.clear()
        self.signature = None
        self.pending = False
        self.complete = False
        self.recording = True
        self.record()

    def arm_reverse(self):
        """Freeze AUTO history after an early stop so AUTO cannot resume."""
        self.recording = False
        self.pending = len(self.history) > 1
        self.cell.run_requested = False
        self.cell.jog_request = None

        if self.pending:
            self.cell.safety.reason = "Stopped before wheel removal - ACK for safe reverse recovery"

    def prepare(self):
        """Legacy GUI RESET path: prepare a recorded run for reverse recovery."""
        c = self.cell
        if not c.safety.reset():
            return

        self.recording = False
        self.pending = len(self.history) > 1
        c.run_requested = False
        c.jog_request = None

        if self.pending:
            c.safety.state = "RECOVERY_READY"
            c.safety.reason = "Hold the green robot recovery button"
            c.safety.recovery_release_required = True
        else:
            self.history.clear()
            self.signature = None
            self.complete = False

    def _changed_robot(self, start, goal):
        """Return the robot that changed between adjacent recorded frames."""
        deltas = [float(np.max(np.abs(a - b))) for a, b in zip(start[0], goal[0])]
        index = int(np.argmax(deltas))

        if deltas[index] > 1e-9:
            return self.cell.robot_list()[index] if index < len(self.cell.robot_list()) else None

        # Car pitch is driven by the KUKA even if joint change is tiny in a frame.
        if abs(start[1] - goal[1]) > 1e-9:
            return self.cell.kuka.Robot

        return None

    def reverse(self):
        """Retrace the recorded pre-wheel-removal motion under hold-to-run control."""
        c = self.cell

        if len(self.history) <= 1:
            self.pending = False
            c.safety.state = "READY"
            c.safety.reason = ""
            return

        print("Recovery: retracing recorded frames toward initial state")
        self.recording = False
        previous_name = None

        while len(self.history) > 1:
            start = self.capture()
            goal = self.history[-2]
            robot = self._changed_robot(start, goal)

            # The UR3e uses its own geometric path checker.
            if np.max(np.abs(start[0][-1] - goal[0][-1])) > 1e-9:
                from UR3e.safety import check_ur3e_path
                check_ur3e_path(c.ur3e, [goal[0][-1]], c.ur3e_car_box,
                                (c.kuka.Robot, c.abb.Robot, c.staubli.Robot))
                c.active_robot = "ur3e"
                c.guard()

            if robot is not None:
                name = c.robot_name(robot)
                c.active_robot = name

                # Controller A runs recorded reverse recovery; B and X can
                # interrupt at every safety checkpoint.
                c.guard(robot)

                robots = c.robot_list()
                goal_index = next(i for i, item in enumerate(robots) if item is robot)
                if not c.path_clear(robot, [robot.q, goal[0][goal_index]]):
                    c.safety.stop("COLLISION", "Reverse recovery path blocked")
                    c.render()
                    return

            # Adjacent history entries are individual rendered frames, so restoring
            # the previous snapshot retraces the same already-executed motion.
            for current_robot, q in zip(c.robot_list(), goal[0]):
                current_robot.q = q.copy()

            c.ur3e.q = goal[0][-1].copy()
            c.car.pitch = goal[1]
            c.mounts = dict(goal[2])
            self.history.pop()
            c.render()

        c.active_robot = None
        self.history.clear()
        self.signature = None
        self.pending = False
        self.complete = False
        self.recording = False
        c.safety.state = "READY"
        c.safety.reason = "Recovery complete - system safe and ready"
        c.select_robot(c.selected_robot)
        c.render()
        print("Recovery complete: initial scene restored safely")
