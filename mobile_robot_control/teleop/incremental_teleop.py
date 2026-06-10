"""IncrementalTeleop — keyboard → (v, ω) commands for a mobile robot."""
import select
import sys
import termios
import threading
import tty

from .base_teleop_input import BaseTeleopInput
from .body_velocity import BodyVelocity

# How long _get_key() waits for a keypress before returning ''.
# Short enough that the input loop stays responsive; long enough to avoid
# busy-spinning and wasting CPU when the user is not pressing anything.
_KEY_READ_TIMEOUT_S = 0.1

# Save the terminal's original "cooked" mode settings at import time so
# _get_key() can always restore them, even across multiple calls.
_original_terminal_settings = termios.tcgetattr(sys.stdin)

_BANNER = """
Mobile Robot — Keyboard Teleop
--------------------------------------------------
        w
   a    s    d
        x

  Linear axis (forward / backward):
    w / W  →  +v  increase linear velocity by linear_step  (forward)
    x / X  →  -v  decrease linear velocity by linear_step  (backward)

  Angular axis (left / right):
    a / A  →  +ω  increase angular velocity by angular_step  (turn left,  CCW)
    d / D  →  -ω  decrease angular velocity by angular_step  (turn right, CW)

  Emergency stop:
    s / S / SPACE  →  zero both v and ω immediately

  Quit:
    Ctrl-C  →  exit
--------------------------------------------------
"""


def _get_key() -> str:
    """Read one character from stdin, or '' if nothing arrives within the timeout."""
    tty.setraw(sys.stdin.fileno())
    try:
        ready, _, _ = select.select([sys.stdin], [], [], _KEY_READ_TIMEOUT_S)
        return sys.stdin.read(1) if ready else ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, _original_terminal_settings)


class IncrementalTeleop(BaseTeleopInput):
    """Incremental keyboard teleoperation.

    Allows controlling a mobile robot by adjusting linear and angular velocity
    in small increments defined by step sizes.
    """

    def __init__(
        self,
        max_linear_velocity: float,
        max_angular_velocity: float,
        linear_step: float,
        angular_step: float,
    ) -> None:
        """Initialise velocity limits and step sizes."""
        super().__init__()
        self._max_linear = max_linear_velocity
        self._max_angular = max_angular_velocity
        self._linear_step = linear_step
        self._angular_step = angular_step
        self._cmd = BodyVelocity(0.0, 0.0)

    def read_input(self) -> BodyVelocity:
        """Return the latest (v, ω) — updated by the input loop in start()."""
        return self._cmd

    def start(self) -> None:
        """Spawn the input loop in a daemon thread and return immediately."""
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self) -> None:
        """Read keys and update self._cmd until the input source is deactivated."""
        print(_BANNER)
        while self.is_active():
            key = _get_key()

            if key in ('w', 'W'):
                linear = min(self._cmd.linear + self._linear_step, self._max_linear)
                self._cmd = BodyVelocity(linear, self._cmd.angular)
            elif key in ('x', 'X'):
                linear = max(self._cmd.linear - self._linear_step, -self._max_linear)
                self._cmd = BodyVelocity(linear, self._cmd.angular)
            elif key in ('a', 'A'):
                angular = min(self._cmd.angular + self._angular_step, self._max_angular)
                self._cmd = BodyVelocity(self._cmd.linear, angular)
            elif key in ('d', 'D'):
                angular = max(self._cmd.angular - self._angular_step, -self._max_angular)
                self._cmd = BodyVelocity(self._cmd.linear, angular)
            elif key in (' ', 's', 'S'):
                self._cmd = BodyVelocity(0.0, 0.0)
            elif key == '\x03':
                self.cleanup()

            if key:
                print(
                    f'\r  linear: {self._cmd.linear:+.2f} m/s  |  '
                    f'angular: {self._cmd.angular:+.2f} rad/s    ',
                    end='', flush=True,
                )
