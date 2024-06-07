#!/usr/bin/env python
import rclpy
import math
import scipy.linalg as la
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from .PathPlanning.CubicSpline import cubic_spline_planner
import time
from .utils.angle import angle_mod
from .find_path import pathFinder
from sensor_msgs.msg import NavSatFix
import utm
from .find_tennis_ball_2 import ballDection
from ackermann_msgs.msg import AckermannDrive

class followTarget(Node):
    def __init__(self):
        super().__init__("FollowTennisBall")
        namespace1 = self.get_namespace()
        self.name = namespace1
        if namespace1=="/":
            namespace1 = "/team5"

        self.pubT = self.create_publisher(Twist, namespace1+"/cmd_vel", 10)

        # self.create_subscription(Float64MultiArray, namespace1+"/ball_loc", self.ballCallback, 
        #                                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PoseStamped, namespace1+"/pose", self.locCallback, 
                                                    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.create_subscription(NavSatFix, "/fix", self.gpsCallback, 
        #                                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.create_subscription(Quaternion, "/heading", self.headingCallback, 
        #                                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.ball = np.array([0.0,0.0])
        self.inpose = np.array([0.0,0.0])
        self.steerMid = -0.0   # -0.12
        self.maxSpeed = 1.2
        self.minSpeed = 0.6
        self.x, self.y, self.yaw, self.v = 0.0, 0.0, 0.0, 0.0
        # LQR parameter
        self.lqr_Q = np.eye(5)
        self.lqr_R = np.eye(2)
        self.lqr_R[0][0] = 1.5
        self.t = time.time()  # time tick[s]
        self.L = 0.3302  # Wheel base of the vehicle [m]
        self.max_steer = np.deg2rad(30.0)  # maximum steering angle[rad]
        self.goal_dis = 0.3
        self.stop_speed = 0.05
        self.dt = 0.0
        self.initCtrlSf()
        self.gotBall = False
        self.go = False
        self.n = 6
        self.px = 0.0
        self.py = 0.0
        self.gotPose = False
        self.detector = ballDection()            # comment this out for simulation
        tem = input("Press enter when ready ")
        self.create_timer(0.1, self.timerCallback)

    def ballCallback(self, msg: Float64MultiArray):
        self.lostCnt = 0
        data = msg.data
        self.ball[0] = data[0]
        self.ball[1] = data[1]
        if not(self.go) and not(self.gotBall):
            self.get_logger().info(f"Got ball location data, following {self.ball}...")
            self.move()
    
    def locCallback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.x = x
        self.y = y
        angles = self.toEulerAngles(msg.pose.orientation)
        self.yaw = math.remainder(-angles[2]-np.pi, math.tau)
        if not(self.gotPose):
            self.inpose[0],self.inpose[1] = x,y
            self.gotPose = True
            self.get_logger().info("Got Position")

    def gpsCallback(self, msg: NavSatFix):
        lat = msg.latitude
        lng = msg.longitude
        pos = utm.from_latlon(lat, lng)
        if la.norm([self.x-self.px, self.y-self.py])>=0.05:
            self.px = self.x
            self.py = self.y
            self.x = float(pos[0])
            self.y = float(pos[1])
            self.yaw = math.atan2(self.y-self.py, self.x-self.px)
        else:
            self.x = float(pos[0])
            self.y = float(pos[1])

        if not(self.gotPose):
            self.gotPose = True
            self.get_logger().info("Got Position")

    def headingCallback(self, msg: Quaternion):
        angles = self.toEulerAngles(msg)
        self.yaw = angles[2]

    def move(self):
        self.initCtrlSf()
        if self.gotBall:
            self.ctrlSf['goal'] = self.inpose
        else:
            self.ctrlSf['goal'] = self.ball
        tem = pathFinder([self.x, self.y], self.ctrlSf['goal'], self.n)
        ax, ay = tem.getWayPts()
        self.ctrlSf['cx'], self.ctrlSf['cy'], self.ctrlSf['cyaw'], self.ctrlSf['ck'], s = cubic_spline_planner.calc_spline_course(
                ax, ay, ds=0.1)
        target_speed = 10.0 / 3.6
        self.ctrlSf['sp'] = self.calc_speed_profile(self.ctrlSf['cyaw'], target_speed)
        self.go = True

    def stop(self):
        cmd = Twist()
        self.pubT.publish(cmd)

    def update(self, accel, delta):
        if delta >= self.max_steer:
            delta = self.max_steer
        if delta <= - self.max_steer:
            delta = - self.max_steer
        self.v = self.v+accel*self.dt
        if self.v >= self.maxSpeed:
            self.v = self.maxSpeed
        if self.v <= self.minSpeed:
            self.v = self.minSpeed

        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = -1*delta/self.dt/5+self.steerMid
        # print(f"Want to turn: {delta}")
        self.pubT.publish(cmd)

    def timerCallback(self):
        if self.gotPose and not(self.go) and not(self.gotBall):            # commment this out for simulation
            self.ball = self.detector.get_ball_loc(self.x, self.y, self.yaw)
            self.get_logger().info(f"Got ball location data, following {self.ball}...")
            self.move()
        # if self.gotPose and not(self.go) and self.gotBall:
        #     self.get_logger().info(f"Now going back to where it started...")
        #     self.move()
        if self.go:
            self.dt = time.time()-self.t
            dl, target_ind, self.ctrlSf['e'], self.ctrlSf['e_th'], ai = self.lqr_speed_steering_control(
                                                                        self.ctrlSf['cx'], self.ctrlSf['cy'], self.ctrlSf['cyaw'], 
                                                                        self.ctrlSf['ck'], self.ctrlSf['e'], self.ctrlSf['e_th'], 
                                                                        self.ctrlSf['sp'], self.lqr_Q, self.lqr_R)
            self.update(ai, dl)

            if abs(self.v) <= self.stop_speed:
                target_ind += 1

            self.t = time.time()

            # check goal
            dx = self.x - self.ctrlSf['goal'][0]
            dy = self.y - self.ctrlSf['goal'][1]
            # print(dx,dy)
            if math.hypot(dx, dy) <= self.goal_dis:
                print("Goal")
                self.gotBall = not(self.gotBall)
                self.go = False
                self.stop()

    def initCtrlSf(self):
        self.ctrlSf = {'cx': [], 'cy': [], 'cyaw': [], 'ck': [], 'e': 0.0, 'e_th': 0.0, 
                        'sp': [], 'goal': [0.0, 0.0]}

    def lqr_speed_steering_control(self, cx, cy, cyaw, ck, pe, pth_e, sp, Q, R):
        ind, e = self.calc_nearest_index(cx, cy, cyaw)

        tv = sp[ind]

        k = ck[ind]
        v = self.v
        th_e = self.pi_2_pi(self.yaw - cyaw[ind])

        # A = [1.0, dt, 0.0, 0.0, 0.0
        #      0.0, 0.0, v, 0.0, 0.0]
        #      0.0, 0.0, 1.0, dt, 0.0]
        #      0.0, 0.0, 0.0, 0.0, 0.0]
        #      0.0, 0.0, 0.0, 0.0, 1.0]
        A = np.zeros((5, 5))
        A[0, 0] = 1.0
        A[0, 1] = self.dt
        A[1, 2] = v
        A[2, 2] = 1.0
        A[2, 3] = self.dt
        A[4, 4] = 1.0

        # B = [0.0, 0.0
        #     0.0, 0.0
        #     0.0, 0.0
        #     v/L, 0.0
        #     0.0, dt]
        B = np.zeros((5, 2))
        B[3, 0] = v / self.L
        B[4, 1] = self.dt

        K, _, _ = self.dlqr(A, B, Q, R)

        # state vector
        # x = [e, dot_e, th_e, dot_th_e, delta_v]
        # e: lateral distance to the path
        # dot_e: derivative of e
        # th_e: angle difference to the path
        # dot_th_e: derivative of th_e
        # delta_v: difference between current speed and target speed
        x = np.zeros((5, 1))
        x[0, 0] = e
        x[1, 0] = (e - pe) / self.dt
        x[2, 0] = th_e
        x[3, 0] = (th_e - pth_e) / self.dt
        x[4, 0] = v - tv

        # input vector
        # u = [delta, accel]
        # delta: steering angle
        # accel: acceleration
        ustar = -K @ x

        # calc steering input
        ff = math.atan2(self.L * k, 1)  # feedforward steering angle
        fb = self.pi_2_pi(ustar[0, 0])  # feedback steering angle
        delta = ff + fb

        # calc accel input
        accel = ustar[1, 0]

        return delta, ind, e, th_e, accel

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
    
    def pi_2_pi(self, angle):
        return angle_mod(angle)
    
    def solve_dare(self, A, B, Q, R):
        """
        solve a discrete time_Algebraic Riccati equation (DARE)
        """
        x = Q
        x_next = Q
        max_iter = 150
        eps = 0.01

        for i in range(max_iter):
            x_next = A.T @ x @ A - A.T @ x @ B @ \
                    la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
            if (abs(x_next - x)).max() < eps:
                break
            x = x_next

        return x_next
    
    def dlqr(self, A, B, Q, R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """

        # first, try to solve the ricatti equation
        X = self.solve_dare(A, B, Q, R)

        # compute the LQR gain
        K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

        eig_result = la.eig(A - B @ K)

        return K, X, eig_result[0]
    
    def calc_nearest_index(self, cx, cy, cyaw):
        dx = [self.x - icx for icx in cx]
        dy = [self.y - icy for icy in cy]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind)

        mind = math.sqrt(mind)

        dxl = cx[ind] - self.x
        dyl = cy[ind] - self.y

        angle = self.pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind
    
    def calc_speed_profile(self, cyaw, target_speed):
        speed_profile = [target_speed] * len(cyaw)

        direction = 1.0

        # Set stop point
        for i in range(len(cyaw) - 1):
            dyaw = abs(cyaw[i + 1] - cyaw[i])
            switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

            if switch:
                direction *= -1

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

            if switch:
                speed_profile[i] = 0.0

        # speed down
        for i in range(len(speed_profile)):
            if i==50: continue
            speed_profile[-i] = target_speed / (50 - i)
            if speed_profile[-i] <= 1.0 / 3.6:
                speed_profile[-i] = 1.0 / 3.6

        return speed_profile


def main(args=None):
    rclpy.init(args=args)
    node = followTarget();
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
