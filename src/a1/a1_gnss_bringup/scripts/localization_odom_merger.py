#!/usr/bin/env python3
"""Publish a 100 Hz map-frame Odometry for the controller: GNSS/INS motion,
corrected onto the map by the MOLA-LO pose.

Inputs:
  /a1_gnss/odom               nav_msgs/Odometry  odom -> base_link (gnss_odom_relay.py,
                                                 ~100 Hz), twist in base_link axes
  /a1_gnss/accel              geometry_msgs/AccelWithCovarianceStamped (gnss_odom_relay.py,
                                                 ~100 Hz), base_link axes, unfiltered
  /lidar_odometry/pose        nav_msgs/Odometry  map -> base_link (MOLA-LO, <= 10 Hz,
                                                 arrives 0.1-0.3 s after its scan stamp)
  /lidar_odometry/pose_quality std_msgs/Float32  ICP quality, published right after
                                                 each pose (no stamp of its own)
  /gsof_client/gsof/ins_solution_49              GNSS (RTK) and INS alignment status
  /control/status/wheel_info  a1_can_msgs/WheelInfo, optional (standstill only; the
                                                 merger runs without it)

Outputs (one message each per GNSS sample, same header.stamp):
  /localization/odom    nav_msgs/Odometry  map -> base_link
    pose              C * G(t), C = correction map <- odom, G = GNSS pose
                      (frozen at the last output during a standstill)
    pose.covariance   all 0 (publish_covariance:=true: the last accepted LO pose's)
    twist             linear.x / y and angular.z of this GNSS sample; the rest 0
                      (all 0 during a standstill)
    twist.angular.x   NOT a rate: heading of base_link in degrees, from the map
                      x axis, CCW positive, (-180, 180] (controller interface)
  /localization/accel   geometry_msgs/AccelWithCovarianceStamped, frame base_link
    accel.linear.x    /a1_gnss/accel x through a 2nd-order Butterworth low-pass
                      (accel_cutoff_hz, ~0.08 s delay at 3 Hz)
    accel.linear.y    mean(vx) * mean(wz) over yaw_accel_mean_sec + d(vy)/dt
                      (low-passed at lateral_slip_cutoff_hz), from the GNSS twist: kinematic, so free
                      of the body vibration that swamps the raw lateral accel

Correction. For each LO pose L stamped t_s, the GNSS pose at the same stamp is
interpolated from the buffer, G(t_s), and C_new = L * inv(G(t_s)). If the GNSS
buffer does not reach t_s yet, the LO pose waits; it is never matched against
an older GNSS sample (that is what biased MOLA's own map -> odom under load).
The difference between the pose published now and the pose C_new gives is taken
in the vehicle frame; only correction_gain of it is applied per LO pose, and the
step is clipped to max(correction_step_min_m, correction_step_dist_ratio x the
distance driven since the previous update) and correction_step_yaw_deg. A single
LO scan that slides along the track therefore moves the output by at most a few
cm, while a lasting map/GNSS offset is followed without a step.

With the GNSS not RTK fixed or the INS not aligned (or no status received
recently) the GNSS itself can drift fast, so the larger degraded_step_m /
degraded_step_yaw_deg limits apply and no jump is rejected. In good GNSS, a
difference over max_correction_jump_m / max_correction_yaw_deg is rejected as an
LO outlier, unless resync_after_rejections consistent rejections show it is
real; it is then followed with the degraded limits (never as a step).

GNSS jumps. Between two GNSS samples the pose should move as the previous
sample's twist predicts. If the GNSS moves more than gnss_jump_m or turns more
than gnss_jump_deg away from that prediction (RTK lost or regained, INS
re-alignment: 2026-10-06 bag, 0.84-0.89 m and 32-105 deg at once), the
correction absorbs the jump at once, so the output continues from where it was
with the predicted motion; LO then pulls it in with the limits above. LO poses
stamped before the jump are moved into the new odom frame with the same
transform before they are used.

LO-primary mode. While the INS is GPS_ONLY or COARSE_LEVELING, or GSOF #50
reports a heading RMS above lo_primary_heading_rms_deg, the GNSS heading is not
usable (GPS_ONLY: heading fixed at 90 deg ENU, yaw rate 0, 5 Hz). The output
then follows LO directly: each LO pose becomes an anchor, and between anchors
the position moves by the GNSS displacement rotated into the map with the
correction's rotation from before the mode started. Heading is the LO heading,
the yaw rate comes from consecutive LO headings, the velocity is the GNSS
velocity rotated into the LO heading, and the longitudinal acceleration is the
derivative of the GNSS speed. On leaving the mode the correction is re-anchored
to the last output.

Standstill. While the vehicle stands still the output pose is frozen and the
twist is zero, and LO corrections are not applied (LO scans jitter by 2-5 cm
at a standstill). Standstill = both wheel speeds 0 in a wheel_info message no
older than wheel_timeout_sec, and the GNSS speed below hold_max_gnss_speed. If
no recent wheel speed is available, a GNSS speed below hold_gnss_only_speed is
used, but only with RTK fixed and the INS aligned: without RTK the INS can
report ~0.7 m/s while standing still. On release the correction is re-anchored
to the frozen pose, so GNSS drift during the hold does not appear as a step.

Both clocks must agree: LO stamps and GNSS stamps have to come from the same
time base (see time_source in gsof_client.yaml).
"""

from collections import deque
import importlib
import math

from geometry_msgs.msg import AccelWithCovarianceStamped
from gsof_msgs.msg import NavigationPerformance50, NavigationSolution49
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

NS_PER_SEC = 1_000_000_000

# Indices of the 6x6 row-major covariance diagonal (x, y, z, rot x, rot y, rot z).
COV_DIAG = (0, 7, 14, 21, 28, 35)

IDENTITY_Q = (0.0, 0.0, 0.0, 1.0)

# gsof_msgs/Status values.
GNSS_FIXED_RTK = 4
IMU_ALIGNED = (3, 4)  # ALIGNED, FULL_NAV
IMU_NO_HEADING = (0, 1)  # GPS_ONLY, COARSE_LEVELING


def stamp_ns(stamp):
    return stamp.sec * NS_PER_SEC + stamp.nanosec


def diagonal_covariance(variances):
    covariance = [0.0] * 36
    for index, variance in zip(COV_DIAG, variances):
        covariance[index] = variance
    return covariance


# Poses are (position (x, y, z), quaternion (x, y, z, w)) tuples.
def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q, v):
    p = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), quat_conj(q))
    return (p[0], p[1], p[2])


def quat_slerp(q0, q1, s):
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1, dot = tuple(-c for c in q1), -dot
    if dot > 0.9995:
        q = tuple(a + s * (b - a) for a, b in zip(q0, q1))
    else:
        theta = math.acos(dot)
        w0 = math.sin((1.0 - s) * theta) / math.sin(theta)
        w1 = math.sin(s * theta) / math.sin(theta)
        q = tuple(w0 * a + w1 * b for a, b in zip(q0, q1))
    norm = math.sqrt(sum(c * c for c in q))
    return tuple(c / norm for c in q)


def quat_angle(q):
    return 2.0 * math.acos(min(1.0, abs(q[3])))


def heading_deg(q):
    """Yaw of q about z in degrees, from the parent x axis, CCW positive, (-180, 180]."""
    x, y, z, w = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def pose_compose(a, b):
    p = quat_rotate(a[1], b[0])
    return (tuple(x + y for x, y in zip(a[0], p)), quat_mul(a[1], b[1]))


def pose_inverse(a):
    q = quat_conj(a[1])
    p = quat_rotate(q, a[0])
    return ((-p[0], -p[1], -p[2]), q)


def pose_interpolate(a, b, s):
    return (tuple(x + s * (y - x) for x, y in zip(a[0], b[0])),
            quat_slerp(a[1], b[1], s))


def pose_from_msg(pose):
    p, o = pose.position, pose.orientation
    return ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))


def norm3(v):
    return math.sqrt(sum(c * c for c in v))


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class LowPass2:
    """2nd-order Butterworth low-pass (bilinear transform) for a fixed sample period.

    A gap longer than max_gap_ns (or a stamp going backwards) restarts the filter
    at the current input, since the coefficients assume the nominal period.
    """

    def __init__(self, cutoff_hz, period_s, max_gap_ns):
        k = math.tan(math.pi * cutoff_hz * period_s)
        norm = 1.0 / (1.0 + math.sqrt(2.0) * k + k * k)
        self.b0 = k * k * norm
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (k * k - 1.0) * norm
        self.a2 = (1.0 - math.sqrt(2.0) * k + k * k) * norm
        self.max_gap_ns = max_gap_ns
        self.t = None
        self.y = 0.0

    def update(self, t, x):
        if self.t is None or t <= self.t or t - self.t > self.max_gap_ns:
            self.x1 = self.x2 = self.y1 = self.y2 = x
        y = (self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
             - self.a1 * self.y1 - self.a2 * self.y2)
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        self.t = t
        self.y = y
        return y


class LocalizationOdomMerger(Node):

    def __init__(self):
        super().__init__("localization_odom_merger")

        pose_topic = self.declare_parameter(
            "pose_topic", "/lidar_odometry/pose").value
        quality_topic = self.declare_parameter(
            "pose_quality_topic", "/lidar_odometry/pose_quality").value
        gnss_topic = self.declare_parameter(
            "gnss_topic", "/a1_gnss/odom").value
        accel_topic = self.declare_parameter(
            "accel_topic", "/a1_gnss/accel").value
        status_topic = self.declare_parameter(
            "ins_status_topic", "/gsof_client/gsof/ins_solution_49").value
        wheel_topic = self.declare_parameter(
            "wheel_topic", "/control/status/wheel_info").value
        output_topic = self.declare_parameter(
            "output_topic", "/localization/odom").value
        output_accel_topic = self.declare_parameter(
            "output_accel_topic", "/localization/accel").value

        # LO correction: partial update, rate-limited (see module docstring).
        self.gain = self.declare_parameter("correction_gain", 0.3).value
        self.step_min_m = self.declare_parameter("correction_step_min_m", 0.05).value
        self.step_dist_ratio = self.declare_parameter(
            "correction_step_dist_ratio", 0.06).value
        self.step_yaw_rad = math.radians(
            self.declare_parameter("correction_step_yaw_deg", 0.3).value)
        self.degraded_step_m = self.declare_parameter("degraded_step_m", 0.3).value
        self.degraded_step_yaw_rad = math.radians(
            self.declare_parameter("degraded_step_yaw_deg", 2.0).value)
        self.max_jump_m = self.declare_parameter("max_correction_jump_m", 3.0).value
        self.max_jump_rad = math.radians(
            self.declare_parameter("max_correction_yaw_deg", 5.0).value)
        self.resync_count = self.declare_parameter("resync_after_rejections", 20).value
        self.min_quality = self.declare_parameter("min_pose_quality", 0.5).value
        self.status_timeout_ns = round(self.declare_parameter(
            "ins_status_timeout_sec", 1.0).value * NS_PER_SEC)

        # GNSS jump absorption and LO-primary mode (see module docstring).
        self.gnss_jump_m = self.declare_parameter("gnss_jump_m", 0.5).value
        self.gnss_jump_rad = math.radians(self.declare_parameter("gnss_jump_deg", 3.0).value)
        self.max_jump_predict_ns = round(self.declare_parameter(
            "gnss_jump_max_dt_sec", 1.0).value * NS_PER_SEC)
        self.lo_primary_enabled = self.declare_parameter("lo_primary_mode", True).value
        self.lo_primary_heading_rms = self.declare_parameter(
            "lo_primary_heading_rms_deg", 2.0).value
        # Each LO pose moves the LO-primary output this far toward LO (1.0 = jump to it),
        # by at most lo_primary_step_m per pose (heading is always taken in full).
        self.lo_primary_gain = self.declare_parameter("lo_primary_gain", 1.0).value
        self.lo_primary_step_m = self.declare_parameter("lo_primary_step_m", 0.3).value
        rms_topic = self.declare_parameter(
            "ins_rms_topic", "/gsof_client/gsof/ins_solution_rms_50").value
        # In GPS_ONLY the GNSS comes at 5 Hz; LO scans must still be matched.
        self.degraded_gnss_gap_ns = round(self.declare_parameter(
            "degraded_max_gnss_gap_sec", 0.3).value * NS_PER_SEC)

        # An LO pose whose scan stamp the GNSS buffer has not reached after this
        # long is dropped.
        self.max_lidar_wait_ns = round(self.declare_parameter(
            "max_lidar_wait_sec", 0.5).value * NS_PER_SEC)
        # Interpolating G(t_s) across a larger GNSS gap is refused.
        self.max_gnss_gap_ns = round(self.declare_parameter(
            "max_gnss_gap_sec", 0.1).value * NS_PER_SEC)
        # Pose quality follows its pose immediately; a pose still without one
        # after this long is used anyway.
        self.quality_wait_ns = round(self.declare_parameter(
            "quality_wait_sec", 0.1).value * NS_PER_SEC)
        self.buffer_ns = round(self.declare_parameter(
            "gnss_buffer_sec", 3.0).value * NS_PER_SEC)

        # Acceleration: raw Applanix acceleration swings +-10 m/s^2 from vibration.
        cutoff = self.declare_parameter("accel_cutoff_hz", 3.0).value
        period = self.declare_parameter("gnss_period_sec", 0.01).value
        max_gap_ns = round(3 * period * NS_PER_SEC)
        self.ax_filter = LowPass2(cutoff, period, max_gap_ns)
        # d(vy)/dt from 100 Hz differences is noisy (1.4 m/s^2 RMS raw); at 3 Hz it
        # still quadrupled the ay noise on the 2026-11-04 bag, at 1.5 Hz it does not.
        self.dvy_filter = LowPass2(
            self.declare_parameter("lateral_slip_cutoff_hz", 1.5).value, period, max_gap_ns)
        self.yaw_accel_mean_ns = round(self.declare_parameter(
            "yaw_accel_mean_sec", 0.1).value * NS_PER_SEC)
        self.accel_max_age_ns = round(self.declare_parameter(
            "accel_max_age_sec", 0.1).value * NS_PER_SEC)

        # Standstill.
        self.hold_enabled = self.declare_parameter("stationary_hold", True).value
        self.wheel_timeout_ns = round(self.declare_parameter(
            "wheel_timeout_sec", 0.2).value * NS_PER_SEC)
        self.hold_max_gnss_speed = self.declare_parameter("hold_max_gnss_speed", 0.1).value
        self.hold_gnss_only_speed = self.declare_parameter(
            "hold_gnss_only_speed", 0.05).value
        self.hold_enter_ns = round(self.declare_parameter(
            "hold_enter_sec", 0.2).value * NS_PER_SEC)

        # The controller does not use covariances: all zero unless asked for.
        self.publish_covariance = self.declare_parameter("publish_covariance", False).value
        # The Applanix velocity covariance is not usable (0 or up to 1e3, and
        # labelled in ENU axes), so fixed standard deviations are reported.
        linear_std = self.declare_parameter("twist_linear_std", 0.05).value
        yaw_rate_std = self.declare_parameter("twist_yaw_rate_std", 0.02).value
        accel_std = self.declare_parameter("accel_linear_std", 0.3).value
        unused_var = self.declare_parameter("twist_unused_variance", 1.0e6).value
        self.twist_covariance = diagonal_covariance(
            (linear_std ** 2, linear_std ** 2, unused_var,
             unused_var, unused_var, yaw_rate_std ** 2))
        self.accel_covariance = diagonal_covariance(
            (accel_std ** 2, accel_std ** 2, unused_var,
             unused_var, unused_var, unused_var))

        self.gnss = deque()        # (stamp_ns, pose, Odometry)
        self.yaw_terms = deque()   # (stamp_ns, vx, wz) for the lateral acceleration
        self.prev_vy = None        # (stamp_ns, vy)
        self.pending = deque()     # [scan_ns, arrival_gnss_ns, pose, covariance, quality]
        self.correction = None     # map <- odom
        self.rejected = []         # consecutive mutually consistent rejected corrections
        self.update_position = None  # GNSS position at the previous correction update
        self.pose_covariance = None
        self.last_output = None    # last published map -> base_link pose
        self.base_frame = None

        self.status = None         # (stamp_ns, gnss, imu_alignment)
        self.heading_rms = None    # (stamp_ns, deg) from GSOF #50
        self.gnss_ok = None        # last logged state of gnss_good()
        self.jumps = deque()       # (stamp_ns, J): odom frame change absorbed at a GNSS jump
        self.lo_primary = False
        self.lo_rot = None         # map <- odom rotation before LO-primary began
        self.lo_anchor = None      # (pose, GNSS pose in the current odom frame, stamp_ns)
        self.lo_prev = None        # (scan_ns, yaw) of the previous LO anchor
        self.lo_yaw_rate = 0.0
        self.speed_filter = LowPass2(cutoff, period, round(0.5 * NS_PER_SEC))
        self.prev_speed = None     # (stamp_ns, speed) for the LO-primary accel
        self.wheel = None          # (stamp_ns, right_kph, left_kph)
        self.still_since = None    # GNSS stamp since which standstill holds
        self.hold = False
        self.hold_pose = None

        # Best effort subscribers accept both reliable and best effort publishers.
        input_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self.accel_pub = self.create_publisher(
            AccelWithCovarianceStamped, output_accel_topic, 10)
        self.create_subscription(Odometry, gnss_topic, self.on_gnss, input_qos)
        self.create_subscription(
            AccelWithCovarianceStamped, accel_topic, self.on_accel, input_qos)
        self.create_subscription(Odometry, pose_topic, self.on_pose, input_qos)
        self.create_subscription(Float32, quality_topic, self.on_quality, input_qos)
        self.create_subscription(
            NavigationSolution49, status_topic, self.on_status, input_qos)
        self.create_subscription(
            NavigationPerformance50, rms_topic, self.on_rms, input_qos)

        # a1_can_msgs lives in the vehicle workspace (A1_Total), not here: the
        # wheel speed is optional.
        wheel_info = None
        if self.hold_enabled and wheel_topic:
            try:
                wheel_info = importlib.import_module("a1_can_msgs.msg").WheelInfo
            except (ImportError, AttributeError):
                self.get_logger().warn(
                    "a1_can_msgs not found: standstill is judged from the GNSS speed "
                    "(RTK fixed and INS aligned only)")
        if wheel_info is not None:
            self.create_subscription(wheel_info, wheel_topic, self.on_wheel, input_qos)

        self.get_logger().info(
            f"{gnss_topic} corrected by {pose_topic} -> {output_topic} (GNSS rate); "
            f"{accel_topic} -> {output_accel_topic} (ax low-pass {cutoff:.1f} Hz, "
            f"ay = vx*wz + dvy/dt); correction gain {self.gain:.2f}, step <= "
            f"max({self.step_min_m:.2f} m, {self.step_dist_ratio:.2f} x distance) / "
            f"{math.degrees(self.step_yaw_rad):.1f} deg "
            f"(degraded GNSS: {self.degraded_step_m:.2f} m / "
            f"{math.degrees(self.degraded_step_yaw_rad):.1f} deg); "
            f"wheel speed: {wheel_topic if wheel_info is not None else 'none'}")

    # ------------------------------------------------------------------ inputs
    def on_accel(self, msg):
        self.ax_filter.update(stamp_ns(msg.header.stamp), msg.accel.accel.linear.x)

    def received_ns(self):
        # GSOF #49/#50 stamps follow the driver's time_source (GPS epoch with
        # time_source gps), unlike the relayed /a1_gnss/odom: freshness is
        # judged by the GNSS stamp at reception instead.
        return self.gnss[-1][0] if self.gnss else None

    def on_status(self, msg):
        if self.gnss:
            self.status = (self.received_ns(), msg.status.gnss, msg.status.imu_alignment)

    def on_rms(self, msg):
        if self.gnss:
            self.heading_rms = (self.received_ns(), msg.attitude_rms_error_heading)

    def on_wheel(self, msg):
        self.wheel = (stamp_ns(msg.header.stamp), msg.right_speed_kph, msg.left_speed_kph)

    def on_pose(self, msg):
        self.base_frame = msg.child_frame_id
        if not self.gnss:
            # No GNSS sample can bracket this scan; queued without an arrival
            # stamp it would also block the queue if its quality never came.
            self.get_logger().warn(
                "LO pose before any GNSS sample; dropped", throttle_duration_sec=5.0)
            return
        if self.hold:
            # LO scans jitter by a few cm at a standstill; the pose is frozen.
            return
        self.pending.append([
            stamp_ns(msg.header.stamp), self.gnss[-1][0], pose_from_msg(msg.pose.pose),
            list(msg.pose.covariance), None])

    def on_quality(self, msg):
        for entry in reversed(self.pending):
            if entry[4] is None:
                entry[4] = msg.data
                break

    def on_gnss(self, msg):
        t = stamp_ns(msg.header.stamp)
        if self.gnss and t <= self.gnss[-1][0]:
            return
        self.gnss.append((t, pose_from_msg(msg.pose.pose), msg))
        while self.gnss and t - self.gnss[0][0] > self.buffer_ns:
            self.gnss.popleft()
        while self.jumps and t - self.jumps[0][0] > self.buffer_ns:
            self.jumps.popleft()
        self.check_gnss_jump()
        self.update_lo_primary(t)

        twist = msg.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        dspeed = 0.0
        if self.prev_speed is not None and 0 < t - self.prev_speed[0] <= self.speed_filter.max_gap_ns:
            dspeed = (speed - self.prev_speed[1]) * NS_PER_SEC / (t - self.prev_speed[0])
        self.prev_speed = (t, speed)
        self.speed_filter.update(t, dspeed)
        self.yaw_terms.append((t, twist.linear.x, twist.angular.z))
        while self.yaw_terms and t - self.yaw_terms[0][0] >= self.yaw_accel_mean_ns:
            self.yaw_terms.popleft()
        dvy = 0.0
        if self.prev_vy is not None and 0 < t - self.prev_vy[0] <= self.dvy_filter.max_gap_ns:
            dvy = (twist.linear.y - self.prev_vy[1]) * NS_PER_SEC / (t - self.prev_vy[0])
        self.prev_vy = (t, twist.linear.y)
        self.dvy_filter.update(t, dvy)

        self.update_hold(t, msg)
        if not self.hold:
            self.process_pending(t)
        self.publish(t, msg)

    # -------------------------------------------------------------- status
    def gnss_good(self, t):
        good = (self.status is not None
                and abs(t - self.status[0]) <= self.status_timeout_ns
                and self.status[1] == GNSS_FIXED_RTK
                and self.status[2] in IMU_ALIGNED)
        if good != self.gnss_ok:
            self.gnss_ok = good
            if good:
                self.get_logger().info("GNSS RTK fixed and INS aligned: fine correction limits")
            else:
                state = ("no recent status" if self.status is None
                         or abs(t - self.status[0]) > self.status_timeout_ns
                         else f"gnss={self.status[1]} imu_alignment={self.status[2]}")
                self.get_logger().warn(f"GNSS degraded ({state}): wide correction limits")
        return good

    def heading_unusable(self, t):
        """GNSS heading not usable: GPS_ONLY / COARSE_LEVELING, or a large heading RMS."""
        if self.status is not None and abs(t - self.status[0]) <= self.status_timeout_ns \
                and self.status[2] in IMU_NO_HEADING:
            return True
        return (self.heading_rms is not None
                and abs(t - self.heading_rms[0]) <= self.status_timeout_ns
                and self.heading_rms[1] > self.lo_primary_heading_rms)

    # -------------------------------------------------------------- GNSS jumps
    def check_gnss_jump(self):
        """Absorb a GNSS pose step that the previous sample's twist does not explain."""
        if self.correction is None or len(self.gnss) < 2:
            return
        (t0, p0, m0), (t1, p1, _) = self.gnss[-2], self.gnss[-1]
        dt_ns = t1 - t0
        if dt_ns > self.max_jump_predict_ns:
            return
        dt = dt_ns / NS_PER_SEC
        tw = m0.twist.twist
        actual = pose_compose(pose_inverse(p0), p1)
        predicted = ((tw.linear.x * dt, tw.linear.y * dt, actual[0][2]),
                     quat_from_yaw(tw.angular.z * dt))
        dpos = math.hypot(actual[0][0] - predicted[0][0], actual[0][1] - predicted[0][1])
        dyaw = abs(wrap_angle(yaw_of(actual[1]) - tw.angular.z * dt))
        if dpos <= self.gnss_jump_m and dyaw <= self.gnss_jump_rad:
            return
        # J maps the new odom frame onto the old one: C_new = C * J keeps C * G continuous.
        jump = pose_compose(pose_compose(p0, predicted), pose_inverse(p1))
        self.correction = pose_compose(self.correction, jump)
        self.jumps.append((t1, jump))
        if self.lo_anchor is not None:
            self.lo_anchor = (self.lo_anchor[0],
                              pose_compose(pose_inverse(jump), self.lo_anchor[1]),
                              self.lo_anchor[2])
        self.get_logger().warn(
            f"GNSS jumped {dpos:.2f} m / {math.degrees(dyaw):.1f} deg beyond its own "
            "motion; jump absorbed into the correction")

    def to_current_frame(self, pose, stamp):
        """Move a GNSS pose taken at `stamp` into the odom frame after later jumps."""
        for tj, jump in self.jumps:
            if tj > stamp:
                pose = pose_compose(pose_inverse(jump), pose)
        return pose

    # -------------------------------------------------------------- LO-primary
    def update_lo_primary(self, t):
        if not self.lo_primary_enabled or self.correction is None or self.last_output is None:
            return
        want = self.heading_unusable(t)
        if want and not self.lo_primary:
            self.lo_primary = True
            self.lo_rot = self.correction[1]
            self.lo_anchor = (self.last_output, self.gnss[-1][1], t)
            self.lo_prev = None
            self.lo_yaw_rate = 0.0
            self.get_logger().warn(
                "GNSS heading unusable (GPS_ONLY/COARSE or large heading RMS): "
                "output follows LO, GNSS displacement in between")
        elif not want and self.lo_primary:
            self.lo_primary = False
            self.correction = pose_compose(self.last_output, pose_inverse(self.gnss[-1][1]))
            self.update_position = self.gnss[-1][1][0]
            self.rejected = []
            self.get_logger().info("GNSS heading usable again; correction re-anchored")

    def lo_primary_pose(self, gnss_pose, stamp_ns):
        anchor, anchor_gnss, anchor_ns = self.lo_anchor
        moved = quat_rotate(self.lo_rot, tuple(
            a - b for a, b in zip(gnss_pose[0], anchor_gnss[0])))
        turn = quat_from_yaw(self.lo_yaw_rate * (stamp_ns - anchor_ns) / NS_PER_SEC)
        return (tuple(a + b for a, b in zip(anchor[0], moved)), quat_mul(turn, anchor[1]))

    def set_lo_anchor(self, lidar_pose, gnss_pose, scan_ns):
        # Position: move the propagated output part of the way toward LO.
        predicted = self.lo_primary_pose(gnss_pose, scan_ns)
        offset = tuple(self.lo_primary_gain * (a - b)
                       for a, b in zip(lidar_pose[0], predicted[0]))
        length = norm3(offset)
        if length > self.lo_primary_step_m:
            offset = tuple(c * self.lo_primary_step_m / length for c in offset)
        lidar_pose = (tuple(a + b for a, b in zip(predicted[0], offset)), lidar_pose[1])
        yaw = yaw_of(lidar_pose[1])
        if self.lo_prev is not None and 0 < scan_ns - self.lo_prev[0] <= NS_PER_SEC // 2:
            self.lo_yaw_rate = wrap_angle(yaw - self.lo_prev[1]) * NS_PER_SEC / (
                scan_ns - self.lo_prev[0])
        self.lo_prev = (scan_ns, yaw)
        self.lo_anchor = (lidar_pose, gnss_pose, scan_ns)

    # -------------------------------------------------------------- standstill
    def standstill(self, t, msg):
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if self.wheel is not None and abs(t - self.wheel[0]) <= self.wheel_timeout_ns:
            return (self.wheel[1] == 0.0 and self.wheel[2] == 0.0
                    and speed < self.hold_max_gnss_speed)
        return self.gnss_good(t) and speed < self.hold_gnss_only_speed

    def update_hold(self, t, msg):
        if not self.hold_enabled or self.correction is None or self.last_output is None:
            return
        if not self.standstill(t, msg):
            self.still_since = None
            if self.hold:
                self.exit_hold()
            return
        if self.still_since is None:
            self.still_since = t
        if not self.hold and t - self.still_since >= self.hold_enter_ns:
            self.hold = True
            self.hold_pose = self.last_output
            # LO poses queued for GNSS matching are superseded by the hold.
            self.pending.clear()
            self.get_logger().info("standstill: output pose frozen, LO corrections paused")

    def exit_hold(self):
        # Re-anchor so the GNSS drift accumulated during the hold is not a step.
        gnss_now = self.gnss[-1][1]
        self.correction = pose_compose(self.hold_pose, pose_inverse(gnss_now))
        self.update_position = gnss_now[0]
        if self.lo_primary:
            self.lo_anchor = (self.hold_pose, gnss_now, self.gnss[-1][0])
        self.last_output = self.hold_pose
        self.rejected = []
        self.pending.clear()
        self.hold = False
        self.get_logger().info("standstill ended; correction re-anchored")

    # -------------------------------------------------------------- correction
    def gnss_pose_at(self, t, max_gap_ns):
        """GNSS pose at stamp t in the current odom frame, or None if t is not bracketed."""
        if not self.gnss or t < self.gnss[0][0] or t > self.gnss[-1][0]:
            return None
        for (t0, p0, _), (t1, p1, _) in zip(self.gnss, list(self.gnss)[1:]):
            if t0 <= t <= t1:
                if t1 - t0 > max_gap_ns:
                    return None
                # A jump at t1: express p1 in the frame of p0 before interpolating.
                for tj, jump in self.jumps:
                    if tj == t1:
                        p1 = pose_compose(jump, p1)
                pose = pose_interpolate(p0, p1, (t - t0) / (t1 - t0) if t1 > t0 else 0.0)
                return self.to_current_frame(pose, t)
        return None

    def process_pending(self, now_ns):
        newest = self.gnss[-1][0]
        while self.pending:
            scan_ns, arrival_ns, lidar_pose, covariance, quality = self.pending[0]
            if quality is None and now_ns - arrival_ns < self.quality_wait_ns:
                return
            if scan_ns > newest:
                if now_ns - arrival_ns > self.max_lidar_wait_ns:
                    self.pending.popleft()
                    self.get_logger().warn(
                        "GNSS did not reach an LO scan stamp in time; LO pose dropped",
                        throttle_duration_sec=5.0)
                    continue
                return
            self.pending.popleft()

            max_gap = (self.max_gnss_gap_ns if self.gnss_good(now_ns)
                       else max(self.max_gnss_gap_ns, self.degraded_gnss_gap_ns))
            gnss_pose = self.gnss_pose_at(scan_ns, max_gap)
            if gnss_pose is None:
                self.get_logger().warn(
                    "no GNSS samples around an LO scan stamp; LO pose dropped",
                    throttle_duration_sec=5.0)
                continue
            if quality is not None and quality < self.min_quality:
                self.get_logger().warn(
                    f"LO pose quality {quality:.2f} < {self.min_quality:.2f}; "
                    "correction not updated", throttle_duration_sec=5.0)
                continue
            if self.lo_primary:
                self.set_lo_anchor(lidar_pose, gnss_pose, scan_ns)
                self.pose_covariance = covariance
                continue
            self.apply_correction(
                pose_compose(lidar_pose, pose_inverse(gnss_pose)), covariance, now_ns)

    def apply_correction(self, new_correction, covariance, now_ns):
        gnss_now = self.gnss[-1][1]

        if self.correction is None:
            self.correction = new_correction
            self.pose_covariance = covariance
            self.update_position = gnss_now[0]
            self.get_logger().info("first LO correction applied")
            return

        current = pose_compose(self.correction, gnss_now)
        diff = pose_compose(pose_inverse(current), pose_compose(new_correction, gnss_now))
        good = self.gnss_good(now_ns)
        travel = norm3(tuple(a - b for a, b in zip(gnss_now[0], self.update_position)))
        if good:
            step_m = max(self.step_min_m, self.step_dist_ratio * travel)
            step_rad = self.step_yaw_rad
        else:
            step_m, step_rad = self.degraded_step_m, self.degraded_step_yaw_rad

        if good and (norm3(diff[0]) > self.max_jump_m or quat_angle(diff[1]) > self.max_jump_rad):
            if self.rejected and not self.is_jump(self.rejected[-1], new_correction, gnss_now):
                self.rejected.append(new_correction)
            else:
                self.rejected = [new_correction]
            if len(self.rejected) < self.resync_count:
                self.get_logger().warn(
                    f"LO differs by {norm3(diff[0]):.2f} m / "
                    f"{math.degrees(quat_angle(diff[1])):.1f} deg; treated as an outlier",
                    throttle_duration_sec=2.0)
                return
            self.get_logger().warn(
                f"{len(self.rejected)} consecutive consistent LO poses differ by "
                f"{norm3(diff[0]):.2f} m; following them with the degraded limits")
            step_m, step_rad = self.degraded_step_m, self.degraded_step_yaw_rad
        self.rejected = []

        # Partial, clipped step in the vehicle frame: output moves to current * step.
        translation = tuple(self.gain * c for c in diff[0])
        length = norm3(translation)
        if length > step_m:
            translation = tuple(c * step_m / length for c in translation)
        angle = quat_angle(diff[1])
        fraction = self.gain
        if angle * fraction > step_rad:
            fraction = step_rad / angle
        rotation = quat_slerp(IDENTITY_Q, diff[1], fraction) if angle > 0.0 else IDENTITY_Q
        self.correction = pose_compose(
            pose_compose(current, (translation, rotation)), pose_inverse(gnss_now))
        self.update_position = gnss_now[0]
        self.pose_covariance = covariance

    def is_jump(self, correction_a, correction_b, gnss_pose):
        """Whether two corrections place the vehicle too far apart."""
        jump = pose_compose(
            pose_inverse(pose_compose(correction_a, gnss_pose)),
            pose_compose(correction_b, gnss_pose))
        return (norm3(jump[0]) > self.max_jump_m or quat_angle(jump[1]) > self.max_jump_rad)

    # ------------------------------------------------------------------ output
    def publish(self, t, gnss_msg):
        if self.correction is None:
            self.get_logger().warn(
                "waiting for the first LO pose; nothing published yet",
                throttle_duration_sec=5.0)
            return

        gnss_now = self.gnss[-1][1]
        lo_primary = self.lo_primary and self.lo_anchor is not None
        if self.hold:
            pose = self.hold_pose
        elif lo_primary:
            pose = self.lo_primary_pose(gnss_now, t)
        else:
            pose = pose_compose(self.correction, gnss_now)
        self.last_output = pose

        out = Odometry()
        out.header.stamp = gnss_msg.header.stamp
        out.header.frame_id = "map"
        out.child_frame_id = self.base_frame or gnss_msg.child_frame_id
        (out.pose.pose.position.x,
         out.pose.pose.position.y,
         out.pose.pose.position.z) = pose[0]
        (out.pose.pose.orientation.x,
         out.pose.pose.orientation.y,
         out.pose.pose.orientation.z,
         out.pose.pose.orientation.w) = pose[1]
        if self.publish_covariance:
            out.pose.covariance = self.pose_covariance
        twist = gnss_msg.twist.twist
        if self.hold:
            pass
        elif lo_primary:
            # GNSS body velocity -> odom -> map (pre-mode rotation) -> output body.
            v_map = quat_rotate(self.lo_rot, quat_rotate(
                gnss_now[1], (twist.linear.x, twist.linear.y, 0.0)))
            v_body = quat_rotate(quat_conj(pose[1]), v_map)
            out.twist.twist.linear.x = v_body[0]
            out.twist.twist.linear.y = v_body[1]
            out.twist.twist.angular.z = self.lo_yaw_rate
        else:
            out.twist.twist.linear.x = twist.linear.x
            out.twist.twist.linear.y = twist.linear.y
            out.twist.twist.angular.z = twist.angular.z
        # Not a rate: controller interface carries the heading here.
        out.twist.twist.angular.x = heading_deg(pose[1])
        if self.publish_covariance:
            out.twist.covariance = self.twist_covariance

        accel = AccelWithCovarianceStamped()
        accel.header.stamp = gnss_msg.header.stamp
        accel.header.frame_id = out.child_frame_id
        if lo_primary:
            # GPS_ONLY: the INS acceleration and yaw rate are 0; use GNSS speed and LO heading.
            speed = math.hypot(twist.linear.x, twist.linear.y)
            accel.accel.accel.linear.x = self.speed_filter.y
            accel.accel.accel.linear.y = speed * self.lo_yaw_rate
            if self.publish_covariance:
                accel.accel.covariance = self.accel_covariance
        elif self.ax_filter.t is not None and abs(t - self.ax_filter.t) <= self.accel_max_age_ns:
            n = len(self.yaw_terms)
            vx = sum(v for _, v, _ in self.yaw_terms) / n
            wz = sum(w for _, _, w in self.yaw_terms) / n
            accel.accel.accel.linear.x = self.ax_filter.y
            accel.accel.accel.linear.y = vx * wz + self.dvy_filter.y
            if self.publish_covariance:
                accel.accel.covariance = self.accel_covariance
        elif self.publish_covariance:
            accel.accel.covariance[0] = -1.0

        self.odom_pub.publish(out)
        self.accel_pub.publish(accel)


def main():
    rclpy.init()
    node = LocalizationOdomMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
