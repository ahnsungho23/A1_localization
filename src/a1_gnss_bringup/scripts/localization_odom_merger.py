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

Outputs (one message each per GNSS sample, same header.stamp):
  /localization/odom    nav_msgs/Odometry  map -> base_link
    pose              C * G(t), C = correction map <- odom, G = GNSS pose
    pose.covariance   from the last accepted LO pose
    twist             linear.x / y and angular.z of this GNSS sample; the rest 0
  /localization/accel   geometry_msgs/AccelWithCovarianceStamped, frame base_link
    accel.linear.x / y   mean of the GNSS samples in the last accel_window_sec

Correction. For each LO pose L stamped t_s, the GNSS pose at the same stamp is
interpolated from the buffer, G(t_s), and C_new = L * inv(G(t_s)). If the GNSS
buffer does not reach t_s yet, the LO pose waits; it is never matched against
an older GNSS sample (that is what biased MOLA's own map -> odom under load).
Updates with low ICP quality or a jump larger than max_correction_jump_m /
max_correction_yaw_deg are rejected with a warning. A jump that persists for
resync_after_rejections consistent updates is taken as real (e.g. the INS lost
its alignment) and applied as a step.

Blending. A new correction is not applied as a step. The offset between the
pose being published and the newly corrected pose is taken in the vehicle frame
and shrunk linearly to zero over correction_blend_sec, so the output moves
smoothly and a heading change does not swing the position about the odom origin.

Both clocks must agree: LO stamps and GNSS stamps have to come from the same
time base (see time_source in gsof_client.yaml).
"""

from collections import deque
import math

from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

NS_PER_SEC = 1_000_000_000

# Indices of the 6x6 row-major covariance diagonal (x, y, z, rot x, rot y, rot z).
COV_DIAG = (0, 7, 14, 21, 28, 35)

IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


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
        output_topic = self.declare_parameter(
            "output_topic", "/localization/odom").value
        output_accel_topic = self.declare_parameter(
            "output_accel_topic", "/localization/accel").value

        self.blend_ns = round(self.declare_parameter(
            "correction_blend_sec", 0.1).value * NS_PER_SEC)
        self.max_jump_m = self.declare_parameter("max_correction_jump_m", 0.5).value
        self.max_jump_rad = math.radians(
            self.declare_parameter("max_correction_yaw_deg", 5.0).value)
        self.min_quality = self.declare_parameter("min_pose_quality", 0.5).value
        # A large jump is an outlier only while it is isolated. When this many
        # consecutive rejected corrections agree with each other, the GNSS (or
        # the map match) really moved: the correction is reset to them.
        self.resync_count = self.declare_parameter("resync_after_rejections", 5).value
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
        # Trailing window: raw Applanix acceleration swings +-10 m/s^2 from
        # vibration; 0.4 s keeps the vehicle motion (and lags it by ~0.2 s).
        self.accel_window_ns = round(self.declare_parameter(
            "accel_window_sec", 0.4).value * NS_PER_SEC)

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
        self.accel = deque()       # (stamp_ns, ax, ay)
        self.pending = deque()     # [scan_ns, arrival_gnss_ns, pose, covariance, quality]
        self.correction = None     # map <- odom, target of the blend
        self.rejected = []         # consecutive mutually consistent rejected corrections
        self.blend_offset = IDENTITY  # vehicle-frame offset still being blended out
        self.blend_start_ns = None
        self.pose_covariance = None
        self.last_output = None    # last published map -> base_link pose
        self.base_frame = None

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

        self.get_logger().info(
            f"{gnss_topic} corrected by {pose_topic} -> {output_topic} (GNSS rate); "
            f"{accel_topic} -> {output_accel_topic}; "
            f"blend={self.blend_ns / NS_PER_SEC:.3f} s, "
            f"reject jumps > {self.max_jump_m:.2f} m / "
            f"{math.degrees(self.max_jump_rad):.1f} deg or quality < {self.min_quality:.2f}")

    # ------------------------------------------------------------------ inputs
    def on_accel(self, msg):
        a = msg.accel.accel.linear
        self.accel.append((stamp_ns(msg.header.stamp), a.x, a.y))

    def on_pose(self, msg):
        newest_gnss = self.gnss[-1][0] if self.gnss else None
        self.pending.append([
            stamp_ns(msg.header.stamp), newest_gnss, pose_from_msg(msg.pose.pose),
            list(msg.pose.covariance), None])
        self.base_frame = msg.child_frame_id

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
        while self.accel and t - self.accel[0][0] > self.buffer_ns:
            self.accel.popleft()

        self.process_pending(t)
        self.publish(t, msg)

    # -------------------------------------------------------------- correction
    def gnss_pose_at(self, t):
        """GNSS pose interpolated at stamp t, or None if t is not bracketed."""
        if not self.gnss or t < self.gnss[0][0] or t > self.gnss[-1][0]:
            return None
        for (t0, p0, _), (t1, p1, _) in zip(self.gnss, list(self.gnss)[1:]):
            if t0 <= t <= t1:
                if t1 - t0 > self.max_gnss_gap_ns:
                    return None
                return pose_interpolate(p0, p1, (t - t0) / (t1 - t0) if t1 > t0 else 0.0)
        return None

    def process_pending(self, now_ns):
        newest = self.gnss[-1][0]
        while self.pending:
            scan_ns, arrival_ns, lidar_pose, covariance, quality = self.pending[0]
            if quality is None and (arrival_ns is None
                                    or now_ns - arrival_ns < self.quality_wait_ns):
                return
            if scan_ns > newest:
                if arrival_ns is not None and now_ns - arrival_ns > self.max_lidar_wait_ns:
                    self.pending.popleft()
                    self.get_logger().warn(
                        "GNSS did not reach an LO scan stamp in time; LO pose dropped",
                        throttle_duration_sec=5.0)
                    continue
                return
            self.pending.popleft()

            gnss_pose = self.gnss_pose_at(scan_ns)
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
            self.apply_correction(
                pose_compose(lidar_pose, pose_inverse(gnss_pose)), covariance)

    def apply_correction(self, new_correction, covariance):
        gnss_now = self.gnss[-1][1]
        new_pose = pose_compose(new_correction, gnss_now)

        if self.correction is None:
            self.correction = new_correction
            self.pose_covariance = covariance
            self.get_logger().info("first LO correction applied")
            return

        # Jump against the correction currently being converged to.
        if self.is_jump(self.correction, new_correction, gnss_now):
            if self.rejected and not self.is_jump(
                    self.rejected[-1][0], new_correction, gnss_now):
                self.rejected.append((new_correction, covariance))
            else:
                self.rejected = [(new_correction, covariance)]
            if len(self.rejected) < self.resync_count:
                self.get_logger().warn(
                    "LO correction jump exceeds the limit; correction not updated",
                    throttle_duration_sec=2.0)
                return
            # Persistent shift: re-sync as a step (blending metres would fake speed).
            jump = pose_compose(pose_inverse(pose_compose(self.correction, gnss_now)), new_pose)
            self.get_logger().warn(
                f"{len(self.rejected)} consecutive consistent LO corrections disagree "
                f"by {math.sqrt(sum(c * c for c in jump[0])):.2f} m / "
                f"{math.degrees(quat_angle(jump[1])):.1f} deg; correction re-synced (step)")
            self.correction = new_correction
            self.pose_covariance = covariance
            self.blend_start_ns = None
            self.blend_offset = IDENTITY
            self.rejected = []
            return
        self.rejected = []

        # Keep publishing from where the output is now, and blend that offset out.
        current = self.last_output if self.last_output is not None else new_pose
        self.blend_offset = pose_compose(pose_inverse(new_pose), current)
        self.blend_start_ns = self.gnss[-1][0]
        self.correction = new_correction
        self.pose_covariance = covariance

    def is_jump(self, correction_a, correction_b, gnss_pose):
        """Whether two corrections place the vehicle too far apart."""
        jump = pose_compose(
            pose_inverse(pose_compose(correction_a, gnss_pose)),
            pose_compose(correction_b, gnss_pose))
        return (math.sqrt(sum(c * c for c in jump[0])) > self.max_jump_m
                or quat_angle(jump[1]) > self.max_jump_rad)

    # ------------------------------------------------------------------ output
    def publish(self, t, gnss_msg):
        if self.correction is None:
            self.get_logger().warn(
                "waiting for the first LO pose; nothing published yet",
                throttle_duration_sec=5.0)
            return

        pose = pose_compose(self.correction, self.gnss[-1][1])
        if self.blend_start_ns is not None:
            s = 1.0 if self.blend_ns <= 0 else min(
                1.0, (t - self.blend_start_ns) / self.blend_ns)
            remaining = pose_interpolate(self.blend_offset, IDENTITY, s)
            pose = pose_compose(pose, remaining)
            if s >= 1.0:
                self.blend_start_ns = None
                self.blend_offset = IDENTITY
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
        out.pose.covariance = self.pose_covariance
        twist = gnss_msg.twist.twist
        out.twist.twist.linear.x = twist.linear.x
        out.twist.twist.linear.y = twist.linear.y
        out.twist.twist.angular.z = twist.angular.z
        out.twist.covariance = self.twist_covariance

        accel = AccelWithCovarianceStamped()
        accel.header.stamp = gnss_msg.header.stamp
        accel.header.frame_id = out.child_frame_id
        window = [(ax, ay) for ts, ax, ay in self.accel
                  if t - self.accel_window_ns <= ts <= t]
        if window:
            accel.accel.accel.linear.x = sum(a[0] for a in window) / len(window)
            accel.accel.accel.linear.y = sum(a[1] for a in window) / len(window)
            accel.accel.covariance = self.accel_covariance
        else:
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
