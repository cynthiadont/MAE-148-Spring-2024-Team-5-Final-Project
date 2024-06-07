#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import pickle
import time
import math

class simulation(Node):
    def __init__(self):
        super().__init__("Simulation")
        self.robot = self.get_namespace()
        if self.robot=="/":
            self.robot = "/team5"

        self.pubB = self.create_publisher(Float64MultiArray, self.robot+"/ball_loc", 10)
        self.pubL = self.create_publisher(PoseStamped, self.robot+"/pose", 10)

        self.create_subscription(Twist, self.robot+"/cmd_vel", self.update, 
                                                    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.get_logger().info("Waiting for data to send to "+self.robot+"...")
        self.pose = np.array([0.0,0.0])
        self.inpose = np.array([0.0,0.0])
        self.yaw = 0.0
        ballx, bally = float(input("Input ball x coord: ")), float(input("Input ball y coord: "))
        self.ball = np.array([ballx,bally,0.0,1.0])
        self.preX = []
        self. preY = []
        self.t = time.time()
        self.dt = 0.1
        self.L = 0.3302  # Wheel base of the vehicle [m]
        self.v = 0.0
        self.create_timer(0.1, self.sendLoc)
        self.sendBall(self.ball)

    def sendBall(self, coords):
        msg = Float64MultiArray()
        msg.data = [a for a in coords]
        self.pubB.publish(msg)
    
    def sendLoc(self):
        msg = PoseStamped()
        msg.pose.position.x = self.pose[0]
        msg.pose.position.y = self.pose[1]
        myaw = -(self.yaw+np.pi/2)
        rot = Rotation.from_euler('zyx', [myaw, 0.0, 0.0], degrees=False)
        rot_quat = rot.as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = rot_quat[0],rot_quat[1],rot_quat[2],rot_quat[3]
        self.pubL.publish(msg)

    def update(self, msg: Twist):
        self.dt = time.time()-self.t
        self.t = time.time()
        self.pose[0] = self.pose[0] + self.v * math.cos(self.yaw) * self.dt
        self.pose[1] = self.pose[1] + self.v * math.sin(self.yaw) * self.dt
        self.yaw = self.yaw + self.v / self.L * math.tan(-1*msg.angular.z*self.dt/3) * self.dt
        self.v = msg.linear.x*3
        self.plot_things()

    def plot_things(self):
        pickle.dumps([time.time(), self.pose, self.yaw])
        plt.cla()
        plt.scatter(self.pose[0],self.pose[1], marker = "s", color ='blue', s=30, label = 'Car')
        plt.scatter(self.ball[0],self.ball[1], marker = "o", color = 'red', s=30, label = 'Tennis-ball')
        plt.text(self.pose[0]-0.5,self.pose[1]+0.5,f"({np.round(self.pose[0],3)},{np.round(self.pose[1],3)})")
        self.preX.append(self.pose[0])
        self.preY.append(self.pose[1])
        n = len(self.preX)-2
        # plt.scatter(self.preX[0:n-21],self.preY[0:n-21])
        # plt.scatter(self.preX[n-21:n-15],self.preY[n-21:n-15])
        # plt.scatter(self.preX[n-15:n-9],self.preY[n-15:n-9])
        plt.scatter(self.preX[n-9:n-3],self.preY[n-9:n-3])
        plt.scatter(self.preX[n-3:-1],self.preY[n-3:-1])
        plt.arrow(self.pose[0],self.pose[1],5*np.cos(self.yaw),5*np.sin(self.yaw), width=0.3, label = 'orientation')
        plt.legend()
        # plt.xlim(self.inpose[0]-60,self.inpose[0]+60)
        # plt.ylim(self.inpose[1]-60,self.inpose[1]+60)
        plt.xlim(self.inpose[0]-30,self.inpose[0]+30)
        plt.ylim(self.inpose[1]-30,self.inpose[1]+30)
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
    node = simulation();
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()