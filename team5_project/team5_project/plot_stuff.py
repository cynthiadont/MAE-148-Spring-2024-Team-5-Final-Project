#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
import matplotlib.pyplot as plt
import utm
import pickle
import time
import math

class plot_stuff(Node):
    def __init__(self):
        super().__init__("Plot_Car_And_Ball")
        self.robot = self.get_namespace()
        if self.robot=="/":
            self.robot = "/team5"

        self.create_subscription(Float64MultiArray, self.robot+"/ball_loc", self.ballCallback, 
                                                    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.create_subscription(PoseStamped, self.robot+"/pose", self.locCallback, 
        #                                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, self.robot+"/odom", self.odomCallback, 
                                                    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.create_subscription(Imu, "/team5/imu", self.imuCallback, 
        #                                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.create_subscription(NavSatFix, "/fix", self.gpsCallback, 
        #                                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.create_subscription(Quaternion, "/heading", self.headingCallback, 
        #                                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.get_logger().info("Waiting for data from "+self.robot+"...")
        self.pose = np.array([0.0,0.0])
        self.inpose = np.array([0.0,0.0])
        self.yaw = 0.0
        self.ball = np.array([0.0,0.0])
        self.preX = []
        self. preY = []
        self.file = open("recordedPlotData.pcl", "wb")

    def ballCallback(self, msg: Float64MultiArray):
        data = msg.data
        self.ball[0] = data[0]-self.inpose[0]
        self.ball[1] = data[1]-self.inpose[1]
        self.plot_things()
    
    def locCallback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        if self.pose[0]==0.0:
            self.inpose = np.array([x,y])
            self.plot_things()
            plt.xlim(self.inpose[0]-30,self.inpose[0]+30)
            plt.ylim(self.inpose[1]-30,self.inpose[1]+30)
            plt.pause(0.01)
        self.pose[0] = x
        self.pose[1] = y
        angles = self.toEulerAngles(msg.pose.orientation)
        self.yaw = math.remainder(-angles[2]-np.pi, math.tau)
        self.plot_things()
        print(f"Yaw angle: {np.rad2deg(self.yaw)}")

    def odomCallback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.pose[0]==0.0:
            self.inpose = np.array([x,y])
            self.plot_things()
            plt.xlim(self.inpose[0]-10,self.inpose[0]+10)
            plt.ylim(self.inpose[1]-10,self.inpose[1]+10)
            plt.pause(0.01)
        self.pose[0] = x
        self.pose[1] = y
        angles = self.toEulerAngles(msg.pose.pose.orientation)
        self.yaw = angles[2]
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        print(self.yaw)
        self.plot_things()

    def imuCallback(self, msg: Imu):
        angles = self.toEulerAngles(msg.orientation)
        self.yaw = angles[0]
        self.plot_things()
        print(f"Yaw angle: {np.rad2deg(self.yaw)}")

    def gpsCallback(self, msg: NavSatFix):
        lat = msg.latitude
        lng = msg.longitude
        pos = utm.from_latlon(lat, lng)
        x = float(pos[0])-self.inpose[0]
        y = float(pos[1])-self.inpose[1]
        if self.pose[0]==0.0:
            self.inpose = np.array([x,y])
            self.plot_things()
            plt.xlim((-60,60))
            plt.ylim((-60,60))
            plt.pause(0.01)
        self.pose[0] = x
        self.pose[1] = y
        self.plot_things()
    def headingCallback(self, msg: Quaternion):
        angles = self.toEulerAngles(msg)
        self.yaw = angles[2]

    def plot_things(self):
        pickle.dumps([time.time(), self.pose, self.yaw])
        plt.cla()
        plt.scatter(self.pose[0],self.pose[1], marker = "s", color ='blue', s=30, label = 'Car')
        plt.scatter(self.ball[0],self.ball[1], marker = "o", color = 'red', s=30, label = 'Tennis-ball')
        plt.text(self.pose[0]-0.5,self.pose[1]+0.5,f"({np.round(self.pose[0],3)},{np.round(self.pose[1],3)})")
        self.preX.append(self.pose[0])
        self.preY.append(self.pose[1])
        n = len(self.preX)-2
        plt.scatter(self.preX[0:n-21],self.preY[0:n-21])
        plt.scatter(self.preX[n-21:n-15],self.preY[n-21:n-15])
        plt.scatter(self.preX[n-15:n-9],self.preY[n-15:n-9])
        plt.scatter(self.preX[n-9:n-3],self.preY[n-9:n-3])
        plt.scatter(self.preX[n-3:-1],self.preY[n-3:-1])
        plt.arrow(self.pose[0],self.pose[1],2.1*np.cos(self.yaw),2.1*np.sin(self.yaw), width=0.15, label = 'orientation')
        plt.legend()
        # plt.xlim(self.inpose[0]-60,self.inpose[0]+60)
        # plt.ylim(self.inpose[1]-60,self.inpose[1]+60)
        plt.xlim(self.inpose[0]-10,self.inpose[0]+10)
        plt.ylim(self.inpose[1]-10,self.inpose[1]+10)
        plt.pause(0.01)

    def toEulerAngles(self, q:Quaternion):
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
    node = plot_stuff();
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()