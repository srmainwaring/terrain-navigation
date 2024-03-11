#!/usr/bin/env python3

"""
ROS 2 node to convert from mavros to ardupilot DDS

  from: /setpoint_raw/global: mavros_msgs.msg.GlobalPositionTarget
  to:   /ap/cmd_gps_pose: ardupilot_msgs.msg.GlobalPosition
  note: convert mavros global position setpoint to ardupilot_dds

  from: /ap/mode: ardupilot_msgs.msg.Mode
  to:   /state: mavros_msgs.msg.State
  note: convert ardupilot_dds mode to mavros state.custom_mode

  from: /set_mode: mavros_msgs.srv.SetMode
  to:   /ap/mode_switch: ardupilot_msgs.srv.ModeSwitch
  note: forward requests for a mode change to ardupilot_dds

  topic: /tf_static: geometry_msgs.msg.TransformStamped
  note: provide transform from map -> map_ned as rviz expects at least one
  fixed frame will be available. Not required for the planner.
  The Euler angles (180 0 90) rotate vectors from ENU to NED.
"""

import math
import rclpy
import sys

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ardupilot_msgs.msg import GlobalPosition
from ardupilot_msgs.msg import Mode
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix

from ardupilot_msgs.srv import ModeSwitch
from mavros_msgs.srv import SetMode

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import pygeodesy
from pygeodesy.ellipsoidalKarney import LatLon


AP_PLANE_MODES = {
    0: "MANUAL",
    1: "CIRCLE",
    2: "STABILIZE",
    3: "TRAINING",
    4: "ACRO",
    5: "FBWA",
    6: "FBWB",
    7: "CRUISE",
    8: "AUTOTUNE",
    10: "AUTO",
    11: "RTL",
    12: "LOITER",
    15: "GUIDED",
}


def to_mode_string(mode_number):
    """
    Convert ArduPilot mode number to string - valid for ArduPlane only.
    """
    mode_str = AP_PLANE_MODES.get(mode_number)
    if mode_str is None:
        mode_str = ""
    return mode_str


def to_mode_number(mode_str):
    """
    Convert ArduPilot mode string to number - valid for ArduPlane only.
    """
    mode_number = [key for key, value in AP_PLANE_MODES.items() if value == mode_str]
    if not mode_number:
        mode_number = [0]
    return mode_number[0]


# https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = [0, 0, 0, 0]  # np.empty((4, ))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


# https://geographiclib.sourceforge.io/C++/doc/geoid.html#geoidinst
GEOID_FILE = "/usr/local/share/GeographicLib/geoids/egm96-5.pgm"
GEOID_INTERPOLATOR = pygeodesy.GeoidKarney(GEOID_FILE)


def geoid_height(lat, lon):
    """
    Get the geoid height at position: lat, lon.
    """
    position = LatLon(lat, lon)
    N = GEOID_INTERPOLATOR(position)
    return N


class MavrosDdsBridge(Node):
    """
    Bridge topics and services from mavros to ardupilot_dds
    """

    def __init__(self):
        super().__init__("mavros_dds_bridge")

        # TODO: make parameters
        self.enable_cmd_gps_pose = True
        self.enable_gps_global_origin = True
        self.enable_navsat = True
        self.enable_pose = False
        self.enable_twist = False
        self.enable_state = True
        self.enable_mode_service = True
        self.convert_orthometric_to_ellipsoid = True

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # setpoint global: mavros => ardupilot_dds
        if self.enable_cmd_gps_pose:
            self.cmd_gps_pose_pub = self.create_publisher(
                GlobalPosition,
                "/ap/cmd_gps_pose",
                10,
            )

            self.cmd_gps_pose_sub = self.create_subscription(
                GlobalPositionTarget,
                "/setpoint_raw/global",
                self.cmd_gps_pose_cb,
                10,
            )

        # gps_global_origin: ardupilot_dds => mavros
        if self.enable_gps_global_origin:
            self.gps_global_origin_pub = self.create_publisher(
                GeoPointStamped,
                "/global_position/gp_origin",
                10,
            )

            self.gps_global_origin_sub = self.create_subscription(
                GeoPointStamped,
                "/ap/gps_global_origin/filtered",
                self.gps_global_origin_cb,
                qos_profile=qos_profile,
            )

        # navsat: ardupilot_dds => mavros
        if self.enable_navsat:
            self.navsat_pub = self.create_publisher(
                NavSatFix,
                "/global_position/global",
                10,
            )

            self.navsat_sub = self.create_subscription(
                NavSatFix,
                "/ap/navsat/navsat0",
                self.navsat_cb,
                qos_profile=qos_profile,
            )

        # pose stamped: ardupilot_dds => mavros
        if self.enable_pose:
            self.pose_pub = self.create_publisher(
                PoseStamped,
                "/local_position/pose",
                10,
            )

            self.pose_sub = self.create_subscription(
                PoseStamped,
                "/ap/pose/filtered",
                self.pose_cb,
                qos_profile=qos_profile,
            )

        # twist stamped: ardupilot_dds => mavros
        if self.enable_twist:
            self.twist_pub = self.create_publisher(
                TwistStamped,
                "/local_position/velocity_local",
                10,
            )

            self.twist_sub = self.create_subscription(
                TwistStamped,
                "/ap/twist/filtered",
                self.twist_cb,
                qos_profile=qos_profile,
            )

        # state: ardupilot_dds => mavros
        if self.enable_state:
            self.state_pub = self.create_publisher(
                State,
                "/state",
                10,
            )

            self.mode_sub = self.create_subscription(
                Mode,
                "/ap/mode",
                self.mode_cb,
                10,
            )

        # forward mavros service request /setmode => /ap/mode_switch
        if self.enable_mode_service:
            self.create_service(SetMode, "/set_mode", self.set_mode_cb)
            self.mode_cli = self.create_client(ModeSwitch, "ap/mode_switch")
            while not self.mode_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    "AP_DDS ModeSwitch service not available, waiting..."
                )

        # publish static transforms once at startup
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        transformation = [
            "map",
            "map_ned",
            0,
            0,
            0,
            math.radians(180),
            0,
            math.radians(90),
        ]
        self.make_transforms(transformation)

    def cmd_gps_pose_cb(self, msg):
        out_msg = GlobalPosition()
        # header
        out_msg.header = msg.header
        out_msg.header.frame_id = "map"
        # coordinate frame and type mask
        out_msg.coordinate_frame = msg.coordinate_frame
        out_msg.type_mask = msg.type_mask
        # geodetic position (datum AMSL)
        out_msg.latitude = msg.latitude
        out_msg.longitude = msg.longitude
        out_msg.altitude = msg.altitude
        # velocity
        out_msg.velocity.linear.x = msg.velocity.x
        out_msg.velocity.linear.y = msg.velocity.y
        out_msg.velocity.linear.z = msg.velocity.z
        # acceleration or force
        out_msg.acceleration_or_force.linear.x = msg.acceleration_or_force.x
        out_msg.acceleration_or_force.linear.y = msg.acceleration_or_force.y
        out_msg.acceleration_or_force.linear.z = msg.acceleration_or_force.z
        # yaw rate
        out_msg.velocity.angular.z = msg.yaw_rate
        # TODO: yaw (no corresponding member in GlobalPosition)

        self.cmd_gps_pose_pub.publish(out_msg)

        # self.get_logger().info("recv: {}\n".format(msg))
        # self.get_logger().info("send: {}\n".format(pos_msg))

    def gps_global_origin_cb(self, msg):
        # convert altitude reference to WGS84 ellipsoid
        alt_ellipsoid = msg.position.altitude
        if self.convert_orthometric_to_ellipsoid:
            alt_ellipsoid += geoid_height(msg.position.latitude, msg.position.longitude)

        out_msg = GeoPointStamped()
        # header
        out_msg.header = msg.header
        out_msg.header.frame_id = msg.header.frame_id
        # position
        out_msg.position.latitude = msg.position.latitude
        out_msg.position.longitude = msg.position.longitude
        out_msg.position.altitude = alt_ellipsoid

        self.gps_global_origin_pub.publish(out_msg)

    def navsat_cb(self, msg):
        # convert altitude reference to WGS84 ellipsoid
        alt_ellipsoid = msg.altitude
        if self.convert_orthometric_to_ellipsoid:
            alt_ellipsoid += geoid_height(msg.latitude, msg.longitude)

        out_msg = NavSatFix()
        # header
        out_msg.header = msg.header
        out_msg.header.frame_id = msg.header.frame_id
        # status
        out_msg.status.status = msg.status.status
        out_msg.status.service = msg.status.service
        # lla
        out_msg.latitude = msg.latitude
        out_msg.longitude = msg.longitude
        out_msg.altitude = alt_ellipsoid
        # covariance
        out_msg.position_covariance = msg.position_covariance
        out_msg.position_covariance_type = msg.position_covariance_type

        self.navsat_pub.publish(out_msg)

    def pose_cb(self, msg):
        out_msg = PoseStamped()
        # header
        out_msg.header = msg.header
        out_msg.header.frame_id = msg.header.frame_id
        # pose.position
        out_msg.pose.position.x = msg.pose.position.x
        out_msg.pose.position.y = msg.pose.position.y
        out_msg.pose.position.z = msg.pose.position.z
        # pose.orientation
        out_msg.pose.orientation.x = msg.pose.orientation.x
        out_msg.pose.orientation.y = msg.pose.orientation.y
        out_msg.pose.orientation.z = msg.pose.orientation.z
        out_msg.pose.orientation.w = msg.pose.orientation.w

        self.pose_pub.publish(out_msg)

    def twist_cb(self, msg):
        out_msg = TwistStamped()
        # header
        out_msg.header = msg.header
        out_msg.header.frame_id = msg.header.frame_id
        # twist.linear
        out_msg.twist.linear.x = msg.twist.linear.x
        out_msg.twist.linear.y = msg.twist.linear.y
        out_msg.twist.linear.z = msg.twist.linear.z
        # twist.angular
        out_msg.twist.angular.x = msg.twist.angular.x
        out_msg.twist.angular.y = msg.twist.angular.y
        out_msg.twist.angular.z = msg.twist.angular.z

        self.twist_pub.publish(out_msg)

    def mode_cb(self, msg):
        mode_str = to_mode_string(msg.mode)

        out_msg = State()
        # header
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = ""
        # state
        out_msg.connected = True
        out_msg.armed = True
        out_msg.guided = mode_str == "GUIDED"
        out_msg.mode = mode_str
        out_msg.system_status = 4  # MAV_STATE_ACTIVE

        self.state_pub.publish(out_msg)

    def set_mode_cb(self, request, response):
        mode_number = to_mode_number(request.custom_mode)

        ap_request = ModeSwitch.Request()
        ap_request.mode = mode_number
        ap_future = self.mode_cli.call_async(ap_request)

        self.get_logger().info(
            "SetMode:   {} ({})".format(request.custom_mode, ap_request.mode)
        )

        def done_cb(ap_future):
            ap_response = ap_future.result()
            self.get_logger().info("status:    {}".format(ap_response.status))
            self.get_logger().info(
                "curr mode: {} ({}))".format(
                    to_mode_string(ap_response.curr_mode),
                    ap_response.curr_mode,
                )
            )

        ap_future.add_done_callback(done_cb)

        response.mode_sent = True
        return response

    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = transformation[0]
        t.child_frame_id = transformation[1]

        t.transform.translation.x = float(transformation[2])
        t.transform.translation.y = float(transformation[3])
        t.transform.translation.z = float(transformation[4])
        quat = quaternion_from_euler(
            float(transformation[5]), float(transformation[6]), float(transformation[7])
        )
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = MavrosDdsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
