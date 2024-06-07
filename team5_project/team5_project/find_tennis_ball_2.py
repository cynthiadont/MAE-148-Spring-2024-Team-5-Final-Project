from pathlib import Path
import depthai as dai
import numpy as np
import time
import json

class ballDection():
    def __init__(self):
        self.T = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.inR = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]])
        self.pose = np.array([0.0,0.0,1.0]).T
        self.startPose = np.array([0.0,0.0,1.0]).T
        self.ball = np.array([0.0,0.0,0.0,1.0]).T
        self.gotPose = False
        self.yaw = 0.0

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
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = self.pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        nnNetworkOut = self.pipeline.create(dai.node.XLinkOut)

        xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        xoutNN = self.pipeline.create(dai.node.XLinkOut)
        xoutDepth = self.pipeline.create(dai.node.XLinkOut)

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

    def get_ball_loc(self, x, y, yaw):
        self.yaw = yaw
        self.pose = np.array([x,y,1.0]).T
        with dai.Device(self.pipeline) as device:
            previewQueue = device.getOutputQueue(name="rgb", maxSize=6, blocking=False)
            detectionNNQueue = device.getOutputQueue(name="detections", maxSize=6, blocking=False)
            depthQueue = device.getOutputQueue(name="depth", maxSize=6, blocking=False)
            networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=6, blocking=False)

            startTime = time.monotonic()
            counter = 0
            fps = 0
            color = (255, 255, 255)

            while True:
                inDet = detectionNNQueue.get()

                counter+=1
                current_time = time.monotonic()
                if (current_time - startTime) > 1 :
                    fps = counter / (current_time - startTime)
                    counter = 0
                    startTime = current_time

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
                        print(f'Tennis ball at: {np.round(self.ball,3)} m')
                        return np.array([self.ball[0], self.ball[1]])