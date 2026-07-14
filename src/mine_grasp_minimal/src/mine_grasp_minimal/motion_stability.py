"""ROS-independent motion-settling primitives used by the mine grasp stack."""

from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional


@dataclass
class MotionExecutionResult:
    """Classified result of one controller-backed motion."""

    ok: bool
    code: str = "SUCCESS"
    detail: str = ""
    measurements: Dict[str, object] = field(default_factory=dict)

    def __bool__(self):
        return bool(self.ok)


class JointSpanWindow:
    """Track joint positions over a wall-clock stability interval."""

    def __init__(self, duration):
        self.duration = max(float(duration), 0.0)
        self.samples = deque()

    def clear(self):
        self.samples.clear()

    def add(self, stamp, positions):
        stamp = float(stamp)
        values = tuple(float(value) for value in positions)
        self.samples.append((stamp, values))
        cutoff = stamp - self.duration
        # Retain the newest sample immediately before the cutoff so coverage
        # and span are not biased by the polling phase.
        while len(self.samples) > 1 and self.samples[1][0] <= cutoff:
            self.samples.popleft()

    @property
    def coverage(self):
        if len(self.samples) < 2:
            return 0.0
        return max(0.0, self.samples[-1][0] - self.samples[0][0])

    @property
    def ready(self):
        return bool(self.samples) and self.coverage >= self.duration

    def maximum_span(self):
        if not self.samples:
            return float("inf")
        width = len(self.samples[0][1])
        if any(len(values) != width for _, values in self.samples):
            return float("inf")
        spans = []
        for index in range(width):
            values = [sample[index] for _, sample in self.samples]
            spans.append(max(values) - min(values))
        return max(spans) if spans else 0.0

    def maximum_endpoint_velocity(self):
        """Return the largest measured start-to-end joint speed.

        Gazebo contact constraints can produce a noisy instantaneous velocity
        field even while the measured joint position is stationary.  This is
        not a replacement for the raw velocity sample: callers use both to
        distinguish a solver impulse from a genuinely moving contact.
        """
        if len(self.samples) < 2 or self.coverage <= 0.0:
            return float("inf")
        first_stamp, first = self.samples[0]
        last_stamp, last = self.samples[-1]
        elapsed = last_stamp - first_stamp
        if elapsed <= 0.0 or len(first) != len(last):
            return float("inf")
        return max(
            (abs(end - start) / elapsed for start, end in zip(first, last)),
            default=0.0,
        )

    def maximum_sample_velocity(self):
        """Return the largest position-derived speed between fresh samples.

        Unlike the endpoint derivative this cannot hide a small, fast
        oscillation whose first and last samples happen to coincide.  Callers
        can require both metrics when deciding that a controller velocity field
        is inconsistent with the measured joint positions.
        """
        if len(self.samples) < 2:
            return float("inf")
        maximum = 0.0
        previous_stamp, previous = self.samples[0]
        for stamp, values in list(self.samples)[1:]:
            elapsed = stamp - previous_stamp
            if elapsed <= 0.0 or len(values) != len(previous):
                return float("inf")
            maximum = max(
                maximum,
                max(
                    (abs(value - old) / elapsed
                     for old, value in zip(previous, values)),
                    default=0.0,
                ),
            )
            previous_stamp, previous = stamp, values
        return maximum


def classify_stability(
    *,
    max_joint_error: Optional[float],
    max_joint_velocity: Optional[float],
    max_joint_span: Optional[float],
    window_ready: bool,
    tcp_position_error: Optional[float],
    tcp_orientation_error: Optional[float],
    joint_error_tolerance: float,
    joint_velocity_tolerance: float,
    joint_span_tolerance: float,
    tcp_position_tolerance: float,
    tcp_orientation_tolerance: float,
    require_tcp: bool,
):
    """Return ``(stable, category, detail)`` for measured terminal state.

    Position failures are kept distinct from residual velocity/oscillation and
    TCP failures. Missing measurements never count as a successful settle.
    """

    if max_joint_error is None:
        return False, "CONTROLLER_UNSETTLED", "joint position measurement unavailable"
    if max_joint_error > joint_error_tolerance:
        return (
            False,
            "TRUE_POSITION_ERROR",
            "max joint error {:.6f} rad exceeds {:.6f} rad".format(
                max_joint_error, joint_error_tolerance
            ),
        )
    if require_tcp:
        if tcp_position_error is None or tcp_orientation_error is None:
            return False, "TCP_NOT_REACHED", "TCP measurement unavailable"
        if tcp_position_error > tcp_position_tolerance:
            return (
                False,
                "TCP_NOT_REACHED",
                "TCP position error {:.6f} m exceeds {:.6f} m".format(
                    tcp_position_error, tcp_position_tolerance
                ),
            )
        if tcp_orientation_error > tcp_orientation_tolerance:
            return (
                False,
                "TCP_NOT_REACHED",
                "TCP orientation error {:.6f} rad exceeds {:.6f} rad".format(
                    tcp_orientation_error, tcp_orientation_tolerance
                ),
            )
    if max_joint_velocity is None:
        return False, "CONTROLLER_UNSETTLED", "joint velocity measurement unavailable"
    if max_joint_velocity > joint_velocity_tolerance:
        return (
            False,
            "CONTROLLER_UNSETTLED",
            "max joint velocity {:.6f} rad/s exceeds {:.6f} rad/s".format(
                max_joint_velocity, joint_velocity_tolerance
            ),
        )
    if not window_ready or max_joint_span is None:
        return False, "CONTROLLER_UNSETTLED", "joint stability window incomplete"
    if max_joint_span > joint_span_tolerance:
        return (
            False,
            "CONTROLLER_UNSETTLED",
            "joint span {:.6f} rad exceeds {:.6f} rad".format(
                max_joint_span, joint_span_tolerance
            ),
        )
    return True, "SUCCESS", "measured terminal state is stable"


def maximum_absolute(values: Iterable[float]):
    values = list(values)
    return max((abs(float(value)) for value in values), default=0.0)
