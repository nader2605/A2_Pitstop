"""USB game-controller controls for the pit-stop simulation.

Controller mapping confirmed on the Nintendo Switch Pro Controller:
0 = A       -> SAFETY RECOVERY
1 = B       -> E-STOP
2 = X       -> FULL RESET
3 = Y       -> unused
4 = Select  -> unused
6 = Start   -> START AUTO
9 = L1      -> unused
10 = R1     -> unused

This is an educational software/controller input and is not a safety-rated
industrial emergency-stop circuit.
"""

try:
    import pygame
except ImportError:
    pygame = None


A_button = 0       # A button -> SAFETY RECOVERY
B_button = 1       # B button -> E-STOP
X_button = 2       # X button -> FULL RESET
Y_button = 3       # Y button -> unused
Select_button = 4  # Select button -> unused
Start_button = 6   # Start button -> START AUTO
L1_button = 9      # L1 button -> unused
R1_button = 10     # R1 button -> unused


class ControllerEstop:
    def __init__(self, button=B_button):
        self.button = int(button)  # B button = E-STOP
        self.joystick = None
        self._previous_buttons = {}

        if pygame is None:
            print("Controller unavailable: pygame is not installed")
            return

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("Controller unavailable: no controller detected")
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        print(f"Controller connected: {self.joystick.get_name()}")
        print("Controller controls: Start = START AUTO | B = E-STOP | A = RECOVERY | X = FULL RESET")
        print(f"Controller E-stop button: {self.button} (B)")

    def _valid_button(self, button):
        return (
            self.joystick is not None
            and 0 <= int(button) < self.joystick.get_numbuttons()
        )

    def pressed(self):
        """Return True while the E-stop button (B = 1) is held."""
        if self.joystick is None:
            return False

        pygame.event.pump()

        if not self._valid_button(self.button):
            return False

        return bool(self.joystick.get_button(self.button))

    def pressed_once(self, button):
        """Return True once on the rising edge of a controller button."""
        if self.joystick is None:
            return False

        pygame.event.pump()

        button = int(button)
        if not self._valid_button(button):
            return False

        current = bool(self.joystick.get_button(button))
        previous = self._previous_buttons.get(button, False)
        self._previous_buttons[button] = current

        return current and not previous

    def close(self):
        if pygame is not None:
            pygame.quit()
