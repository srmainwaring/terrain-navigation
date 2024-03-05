#!/usr/bin/env python3

"""
ROS 2 node to convert from mavros to ardupilot DDS

Conversions
mavros_msgs/GlobalPositionTarget => ardupilot_msgs/GlobalPosition


"""

import rclpy
import sys

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ardupilot_msgs.msg import GlobalPosition
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MavrosDdsBridge(Node):
    def __init__(self):
        super().__init__("mavros_dds_bridge")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # setpoint global: mavros => ap_dds
        self.cmd_gps_pose_pub = self.create_publisher(
            GlobalPosition,
            "/ap/cmd_gps_pose",
            10,
        )

        self.cmd_gps_pose_sub = self.create_subscription(
            GlobalPositionTarget,
            "/mav/setpoint_raw/global",
            self.cmd_gps_pose_cb,
            10,
        )

        # gps_global_origin: ap_dds => mavros
        self.gps_global_origin_pub = self.create_publisher(
            GeoPointStamped,
            "/mav/global_position/gp_origin",
            10,
        )

        self.gps_global_origin_sub = self.create_subscription(
            GeoPointStamped,
            "/ap/gps_global_origin/filtered",
            self.gps_global_origin_cb,
            qos_profile=qos_profile,
        )

        # navsat: ap_dds => mavros
        self.navsat_pub = self.create_publisher(
            NavSatFix,
            "/mav/global_position/global",
            10,
        )

        self.navsat_sub = self.create_subscription(
            NavSatFix,
            "/ap/navsat/navsat0",
            self.navsat_cb,
            qos_profile=qos_profile,
        )

        # pose stamped: ap_dds => mavros
        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/mav/local_position/pose",
            10,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/ap/pose/filtered",
            self.pose_cb,
            qos_profile=qos_profile,
        )

        # twist stamped: ap_dds => mavros
        self.twist_pub = self.create_publisher(
            TwistStamped,
            "/mav/local_position/velocity_local",
            10,
        )

        self.twist_sub = self.create_subscription(
            TwistStamped,
            "/ap/twist/filtered",
            self.twist_cb,
            qos_profile=qos_profile,
        )

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
        out_msg = GeoPointStamped()
        # header
        out_msg.header = msg.header
        out_msg.header.frame_id = msg.header.frame_id
        # position
        out_msg.position.latitude = msg.position.latitude
        out_msg.position.longitude = msg.position.longitude
        out_msg.position.altitude = msg.position.altitude

        self.gps_global_origin_pub.publish(out_msg)

    def navsat_cb(self, msg):
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
        out_msg.altitude = msg.altitude
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
