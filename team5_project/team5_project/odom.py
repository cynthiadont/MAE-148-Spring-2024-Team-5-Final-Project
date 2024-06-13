#!/usr/bin/env python
import rclpy
import math
import scipy.linalg as la
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Vector3
import time
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescStateStamped
from sensor_msgs.msg import Imu 
from scipy.spatial.transform import Rotation

class Odom(Node):
    def __init__(self):
        super().__init__("Team5_Odom")
        namespace1 = self.get_namespace()
        self.name = namespace1
        if namespace1=="/":
            namespace1 = "/team5"
        self.useOdom = True

        self.pubO = self.create_publisher(Odometry, namespace1+"/odom", 10)

        self.declare_parameter("speed_to_erpm_gain", -4112.82)
        self.declare_parameter("speed_to_erpm_offset", -0.0)

        self.speed_to_erpm_gain = self.get_parameter("speed_to_erpm_gain").get_parameter_value().double_value
        self.speed_to_erpm_offset = self.get_parameter("speed_to_erpm_offset").get_parameter_value().double_value

        self.create_subscription(VescStateStamped, "sensors/core", self.vescCallback, 
                                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Imu, namespace1+"/imu", self.imuCallback, 
                                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.inyaw = 0.0
        self.started = False
        self.current_speed = 0.0
        self.current_ang_speed = Vector3()
        self.orientation = Quaternion()
        self.time = time.time()

    def vescCallback(self, msg: VescStateStamped):
        self.current_speed = (-msg.state.speed-self.speed_to_erpm_offset)/self.speed_to_erpm_gain
        if abs(self.current_speed) < 0.05:
            self.current_speed = 0.0
        dt = time.time()-self.time
        self.time = time.time()
        self.x += self.current_speed*np.cos(self.yaw)*dt
        self.y += self.current_speed*np.sin(self.yaw)*dt
        Omsg = Odometry()
        Omsg.pose.pose.position.x = self.x
        Omsg.pose.pose.position.y = self.y
        Omsg.pose.pose.orientation = self.orientation
        Omsg.twist.twist.angular = self.current_ang_speed
        Omsg.twist.twist.linear.x = self.current_speed
        self.pubO.publish(Omsg)


    def imuCallback(self, msg: Imu):
        angles = self.toEulerAngles(msg.orientation)
        self.yaw = angles[0]-self.inyaw
        if not(self.started):
            self.inyaw = self.yaw
            self.started = True
        self.current_ang_speed = msg.angular_velocity
        rot = Rotation.from_euler('zyx', [self.yaw, 0.0, 0.0], degrees=False)
        rot_quat = rot.as_quat()
        self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w = rot_quat[0],rot_quat[1],rot_quat[2],rot_quat[3]

    def toEulerAngles(self, q: Quaternion):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = np.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
        cosp = np.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
        pitch = 2 * np.arctan2(sinp, cosp) - np.pi / 2
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)   # [-pi, pi]
        return [roll, pitch, yaw]

def main(args=None):
    rclpy.init(args=args)
    node = Odom();
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
