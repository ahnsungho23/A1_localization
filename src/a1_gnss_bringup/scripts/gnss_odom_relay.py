#!/usr/bin/env python3
"""Relay Applanix (trimble_driver_ros) outputs into the A1 REP-105 frame tree.

Inputs (trimble_driver_ros, publish_rep103=true, publish_tf=false):
  /gsof_client/odom    nav_msgs/Odometry   pose of the INS output point ("ins")
                                           in the driver's local ENU plane
  /gsof_client/navsat  sensor_msgs/NavSatFix

Outputs:
  /tf                  odom -> base_link
  /a1_gnss/odom        nav_msgs/Odometry   odom -> base_link
  /a1_gnss/navsat      sensor_msgs/NavSatFix (same content, converted stamp)

With time_source=gps the driver stamps GPS-epoch time (week * 604800 s + ms,
counted from 1980-01-06), which is not comparable to ROS/LiDAR UTC Unix time.
Every output stamp is converted here:

  unix = gps + GPS_EPOCH_UNIX_SEC - gps_utc_offset_sec + extra_time_offset_sec

The pose is moved from the INS output point to base_link with the measured
static base_link -> ins extrinsic:

  T_odom_base = T_odom_ins * inv(T_base_ins)
"""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster

GPS_EPOCH_UNIX_SEC = 315964800  # 1980-01-06T00:00:00Z
NS_PER_SEC = 1_000_000_000


# Quaternions are (x, y, z, w) tuples.
def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


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


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


class GnssOdomRelay(Node):

    def __init__(self):
        super().__init__("gnss_odom_relay")

        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.ins_frame = self.declare_parameter("ins_frame", "ins").value
        self.publish_tf = self.declare_parameter("publish_tf", True).value

        self.time_conversion = self.declare_parameter(
            "time_conversion", "gps_to_utc").value
        if self.time_conversion not in ("gps_to_utc", "none"):
            raise ValueError(
                "time_conversion must be 'gps_to_utc' or 'none', "
                f"got {self.time_conversion!r}")
        # GPS - UTC (18 s since 2017-01-01). Kept as a parameter for future
        # leap seconds.
        gps_utc_offset_sec = self.declare_parameter(
            "gps_utc_offset_sec", 18.0).value
        # Additional constant correction, e.g. if the LiDAR stamps turn out to
        # be TAI instead of UTC. Unverified: leave 0 until measured.
        extra_time_offset_sec = self.declare_parameter(
            "extra_time_offset_sec", 0.0).value
        self.max_clock_offset_sec = self.declare_parameter(
            "max_clock_offset_sec", 1.0).value

        offset_sec = extra_time_offset_sec
        if self.time_conversion == "gps_to_utc":
            offset_sec += GPS_EPOCH_UNIX_SEC - gps_utc_offset_sec
        self.stamp_offset_ns = round(offset_sec * NS_PER_SEC)

        xyz = self._required_triplet("base_to_ins_xyz")
        rpy_deg = self._required_triplet("base_to_ins_rpy_deg")
        self.p_base_ins = tuple(xyz)
        self.q_base_ins = quat_from_rpy(*(math.radians(a) for a in rpy_deg))

        input_odom = self.declare_parameter(
            "input_odom_topic", "/gsof_client/odom").value
        input_navsat = self.declare_parameter(
            "input_navsat_topic", "/gsof_client/navsat").value
        output_odom = self.declare_parameter(
            "output_odom_topic", "/a1_gnss/odom").value
        output_navsat = self.declare_parameter(
            "output_navsat_topic", "/a1_gnss/navsat").value

        # trimble_driver_ros publishes with the default (reliable) QoS, depth 100.
        self.odom_pub = self.create_publisher(Odometry, output_odom, 100)
        self.navsat_pub = self.create_publisher(NavSatFix, output_navsat, 100)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Odometry, input_odom, self.on_odom, 100)
        self.create_subscription(NavSatFix, input_navsat, self.on_navsat, 100)

        self.warned_child_frame = False

        self.get_logger().info(
            f"{input_odom} -> {self.odom_frame}->{self.base_frame} "
            f"(tf={self.publish_tf}), {input_navsat} -> {output_navsat}; "
            f"time_conversion={self.time_conversion}, "
            f"stamp offset={self.stamp_offset_ns / NS_PER_SEC:.3f} s; "
            f"base_to_ins xyz={list(xyz)} rpy_deg={list(rpy_deg)}")

    def _required_triplet(self, name):
        value = self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY).value
        if value is None or len(value) != 3:
            raise ValueError(
                f"Parameter '{name}' must be a measured [a, b, c] array, "
                f"got {value!r}")
        return [float(v) for v in value]

    def convert_stamp(self, stamp):
        ns = stamp.sec * NS_PER_SEC + stamp.nanosec + self.stamp_offset_ns
        converted = Time(nanoseconds=ns).to_msg()

        skew = (ns - self.get_clock().now().nanoseconds) / NS_PER_SEC
        if abs(skew) > self.max_clock_offset_sec:
            self.get_logger().warn(
                f"Converted GNSS stamp differs from the host clock by {skew:.3f} s "
                "(check driver time_source, time_conversion, PTP lock)",
                throttle_duration_sec=5.0)
        return converted

    def on_navsat(self, msg):
        msg.header.stamp = self.convert_stamp(msg.header.stamp)
        self.navsat_pub.publish(msg)

    def on_odom(self, msg):
        if msg.child_frame_id != self.ins_frame and not self.warned_child_frame:
            self.get_logger().warn(
                f"Driver child_frame_id is '{msg.child_frame_id}', expected "
                f"'{self.ins_frame}'; base_to_ins must describe that point")
            self.warned_child_frame = True

        stamp = self.convert_stamp(msg.header.stamp)

        pose = msg.pose.pose
        p_odom_ins = (pose.position.x, pose.position.y, pose.position.z)
        o = pose.orientation
        q_odom_ins = (o.x, o.y, o.z, o.w)

        # T_odom_base = T_odom_ins * inv(T_base_ins)
        q_odom_base = quat_mul(q_odom_ins, quat_conj(self.q_base_ins))
        lever_in_odom = quat_rotate(q_odom_base, self.p_base_ins)
        p_odom_base = tuple(a - b for a, b in zip(p_odom_ins, lever_in_odom))

        # Body twist (REP-103 FLU of the INS) moved to base_link:
        #   w_b = R_bi w_i,  v_b = R_bi v_i - w_b x p_bi
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        w_base = quat_rotate(self.q_base_ins, (ang.x, ang.y, ang.z))
        v_rot = quat_rotate(self.q_base_ins, (lin.x, lin.y, lin.z))
        v_base = tuple(
            a - b for a, b in zip(v_rot, cross(w_base, self.p_base_ins)))

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        (out.pose.pose.position.x,
         out.pose.pose.position.y,
         out.pose.pose.position.z) = p_odom_base
        (out.pose.pose.orientation.x,
         out.pose.pose.orientation.y,
         out.pose.pose.orientation.z,
         out.pose.pose.orientation.w) = q_odom_base
        (out.twist.twist.linear.x,
         out.twist.twist.linear.y,
         out.twist.twist.linear.z) = v_base
        (out.twist.twist.angular.x,
         out.twist.twist.angular.y,
         out.twist.twist.angular.z) = w_base
        # Passed through unchanged (-1 in [0] means GSOF #50 was not received).
        # The lever arm adds attitude-induced position uncertainty that is not
        # propagated here.
        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance
        self.odom_pub.publish(out)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = out.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = p_odom_base[0]
            tf.transform.translation.y = p_odom_base[1]
            tf.transform.translation.z = p_odom_base[2]
            tf.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = GnssOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
