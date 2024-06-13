
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from pathlib import Path
import depthai as dai
import time
import json
import utm
from sensor_msgs.msg import NavSatFix

class detectTran(Node):
    def __init__(self):
        super().__init__("Find_Tennis_Ball")
        self.timer = self.create_timer(0.3, self.on_timer)

        self.T = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.inR = np.array([[0,0,-1,0],[-1,0,0,0],[0,1,0,0],[0,0,0,1]])
        namespace1 = self.get_namespace()
        self.name = namespace1
        if namespace1=="/":
            namespace1 = "/team5"

        self.pose = np.array([0.0,0.0,1.0]).T
        self.startPose = np.array([0.0,0.0,1.0]).T
        self.ball = np.array([0.0,0.0,0.0,1.0]).T
        self.gotPose = False
        self.yaw = 0.0

        self.publ_pts = self.create_publisher(Float64MultiArray, namespace1+"/ball_loc", 10)

        # self.create_subscription(PoseStamped, namespace1+"/pose", self.callbackFn2, 
        #                                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, namespace1+"/odom", self.odomCallback, 
                                                    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.create_subscription(NavSatFix, "/fix", self.gpsCallback, 
        #                                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.create_subscription(Quaternion, "/heading", self.headingCallback, 
        #                                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        configPath = Path('src/team5_project/models/team5-final-project_2.json')
        if not configPath.exists():
           raise ValueError("Path {} does not exist!".format(configPath))

        with configPath.open() as f:
           config = json.load(f)
        nnConfig = config.get("nn_config", {})

        #parse input shape
        if "input_size" in nnConfig:
           W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

        #extract metadata
        metadata = nnConfig.get("NN_specific_metadata", {})
        classes = metadata.get("classes", {})
        coordinates = metadata.get("coordinates", {})
        anchors = metadata.get("anchors", {})
        anchorMasks = metadata.get("anchor_masks", {})
        iouThreshold = metadata.get("iou_threshold", {})
        confidenceThreshold = metadata.get("confidence_threshold", {})


        # parse labels
        self.nnMappings = config.get("mappings", {})
        self.labels = self.nnMappings.get("labels", {})
        self.labelMap = self.labels
        # get model path
        nnPath = "src/team5_project/models/best_openvino_2022.1_6shave.blob"
        """
        if not Path(nnPath).exists():
            print("No blob found at {}. Looking into DepthAI model zoo.".format(nnPath))
            nnPath = str(blobconverter.from_zoo(args.model, shaves = 6, zoo_type = "depthai", use_cache=True)) """
        # sync outputs
        syncNN = True


        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        camRgb = pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        nnNetworkOut = pipeline.create(dai.node.XLinkOut)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutNN = pipeline.create(dai.node.XLinkOut)
        xoutDepth = pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")
        xoutNN.setStreamName("detections")
        xoutDepth.setStreamName("depth")
        nnNetworkOut.setStreamName("nnNetwork")

        # Properties
        camRgb.setPreviewSize(352, 352)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")

        # setting node configs
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Align depth map to the perspective of RGB camera, on which inference is done
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
        stereo.setSubpixel(True)

        # Network specific settings
        spatialDetectionNetwork.setConfidenceThreshold(confidenceThreshold)
        spatialDetectionNetwork.setNumClasses(classes)
        spatialDetectionNetwork.setCoordinateSize(coordinates)
        spatialDetectionNetwork.setAnchors(anchors)
        spatialDetectionNetwork.setAnchorMasks(anchorMasks)
        spatialDetectionNetwork.setIouThreshold(iouThreshold)
        spatialDetectionNetwork.setBlobPath(nnPath)
        spatialDetectionNetwork.setNumInferenceThreads(2)
        spatialDetectionNetwork.input.setBlocking(False)

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb.preview.link(spatialDetectionNetwork.input)
        if syncNN:
            spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        else:
            camRgb.preview.link(xoutRgb.input)

        spatialDetectionNetwork.out.link(xoutNN.input)

        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
        spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)

        

        self.device = dai.Device(pipeline)
        self.previewQueue = self.device.getOutputQueue(name="rgb", maxSize=6, blocking=False)
        self.detectionNNQueue = self.device.getOutputQueue(name="detections", maxSize=6, blocking=False)
        self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=6, blocking=False)
        self.networkQueue = self.device.getOutputQueue(name="nnNetwork", maxSize=6, blocking=False)

        self.startTime = time.monotonic()
        self.counter = 0
        self.fps = 0
        color = (255, 255, 255)

    def on_timer(self):
        inDet = self.detectionNNQueue.get()

        self.counter+=1
        current_time = time.monotonic()
        if (current_time - self.startTime) > 1 :
            self.fps = self.counter / (current_time - self.startTime)
            self.counter = 0
            self.startTime = current_time

        detections = inDet.detections

        # If the frame is available, draw bounding boxes on it and show the frame
        for detection in detections:
            if self.labelMap[detection.label] == 'tennis-ball':
                tem = np.array([detection.spatialCoordinates.x/1000,detection.spatialCoordinates.y/1000,-1*(detection.spatialCoordinates.z)/1000,1.0]).T
                tem = np.matmul(self.inR,tem)
                tem = np.matmul(self.T,tem)
                tem2 = np.array([tem[0],tem[1], 1]).T
                R = np.array([[np.cos(self.yaw),-1*np.sin(self.yaw),0],[np.sin(self.yaw),np.cos(self.yaw),0],[0,0,1]])
                T = np.array([[1,0,self.pose[0]],[0,1,self.pose[1]],[0,0,1]])
                Tr = np.matmul(T,R)
                tem2 = np.matmul(Tr, tem2)
                self.ball[0] = tem2[0]
                self.ball[1] = tem2[1]
                self.ball[2] = tem[2]
                msg = Float64MultiArray()
                msg.data = [a for a in self.ball]
                msg.layout.dim = [MultiArrayDimension()]
                msg.layout.dim[0].label = self.name
                self.publ_pts.publish(msg)
                self.get_logger().info(f'Tennis ball at: {np.round(self.ball,3)} m')
    
    def callbackFn2(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.pose[0] = x
        self.pose[1] = y
        angles = self.toEulerAngles(msg.pose.orientation)
        self.yaw = -angles[2]-np.pi/2
        if not(self.gotPose) and self.yaw != 0.0:
            self.gotPose = True
            self.get_logger().info("Got Position")
            self.startPose[0] = x
            self.startPose[1] = y

    def odomCallback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pose[0] = x
        self.pose[1] = y
        angles = self.toEulerAngles(msg.pose.pose.orientation)
        self.yaw = angles[2]
        if not(self.gotPose):
            self.startPose[0],self.startPose[1] = x,y
            self.gotPose = True
            self.get_logger().info("Got Position")

    def gpsCallback(self, msg: NavSatFix):
        lat = msg.latitude
        lng = msg.longitude
        pos = utm.from_latlon(lat, lng)
        x = float(pos[0])
        y = float(pos[1])
        self.pose[0] = x
        self.pose[1] = y
        if not(self.gotPose):
            self.gotPose = True
            self.get_logger().info("Got Position")
            self.startPose[0] = x
            self.startPose[1] = y
        
    def headingCallback(self, msg: Quaternion):
        angles = self.toEulerAngles(msg)
        self.yaw = angles[2]

    def rotate(self,Quat: Quaternion):
        w = Quat.w
        x = Quat.x
        y = Quat.y
        z = Quat.z
        R = np.array([[2*(w**2+x**2)-1,2*(x*y-w*z),2*(x*z+w*y),0],[2*(x*y+w*z),2*(w**2+y**2)-1,2*(y*z-w*x),0],[2*(x*z-w*y),2*(y*z+w*x),2*(w**2+z**2)-1,0],[0,0,0,1]])
        return R
    def trans(self,xyz: Vector3):
        x = xyz.x
        y = xyz.y
        z = xyz.z
        T = np.array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
        return T
    def TR(self,xyz: Vector3,Quat: Quaternion):
        return np.matmul(self.trans(xyz),self.rotate(Quat))
    def RT(self,xyz: Vector3,Quat: Quaternion):
        return np.linalg.inv(self.TR(xyz,Quat))
    def toEulerAngles(self, q:Quaternion):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = np.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
        cosp = np.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
        pitch = 2 * np.arctan2(sinp, cosp) - np.pi / 2
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp);
        return [roll, pitch, yaw]


def main(args=None):
    rclpy.init(args=args)
    node = detectTran();
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
