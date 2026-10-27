"""Shared safety state for the pit-stop simulation."""

class Safety:
    def __init__(self, controller=None):
        self.state = "READY"
        self.reason = ""
        self.controller = controller
        self.reset_requested = False
        self.start_requested = False
        self.recovery_requested = False

    def stop(self, state, reason):
        self.state = state
        self.reason = reason
        print(f"{state}: {reason}")

    def reset(self):
        if self.state not in ("RUNNING", "RECOVERING"):
            self.state = "READY"
            self.reason = ""
            print("System reset -> READY")
            return True
        return False

    def start(self):
        if self.state == "READY":
            self.state = "RUNNING"
            self.reason = ""
            return True
        return False

    def poll(self):
        # X is sampled during motion as well as in the main loop.
        # Defer scene mutation until the active trajectory has unwound.
        if self.controller is not None and self.controller.pressed_once(2):  # X button
            self.reset_requested = True
        # Controller E-stop uses the same latched safety state as the GUI stop.
        # Releasing the controller button does not resume motion.
        if self.controller is not None and self.controller.pressed():
            if self.state != "ESTOP":
                self.stop("ESTOP", "Controller emergency stop")
