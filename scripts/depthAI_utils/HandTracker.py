import numpy as np
import depthAI_utils.mediapipe_utils as mpu
import depthai as dai
import cv2
from pathlib import Path
from depthAI_utils.FPS import FPS, now



SCRIPT_DIR = Path(__file__).resolve().parent
PALM_DETECTION_MODEL = str(SCRIPT_DIR / "palm_detection_sh4.blob")
LANDMARK_MODEL_LITE = str(SCRIPT_DIR / "hand_landmark_lite_sh4.blob")


def to_planar(arr: np.ndarray, shape: tuple) -> np.ndarray:
    return cv2.resize(arr, shape).transpose(2,0,1)

class HandTracker:
    """
    This is a modified version of the original depthai mediapipe repo:
    https://github.com/geaxgx/depthai_hand_tracker/tree/main

    """
    def __init__(self):

        self.pd_model = PALM_DETECTION_MODEL
        print(f"Palm detection blob : {self.pd_model}")
        self.lm_model = LANDMARK_MODEL_LITE
        print(f"Landmark blob       : {self.lm_model}")

        self.pd_score_thresh = 0.5
        self.pd_nms_thresh = 0.3
        self.use_lm = True
        self.lm_score_thresh = 0.5

        self.solo =False
        self.lm_nb_threads = 2

        # in our case, we only need one hand
        self.max_hands = 2

        self.xyz = False
        self.crop = False
        self.use_world_landmarks = True
        self.internal_fps = 23
        self.stats = False
        self.trace = 0
        self.use_gesture = False
        self.use_handedness_average = True
        self.single_hand_tolerance_thresh = 10

        self.device = dai.Device()

        ###setup camera
        self.input_type = "rgb" # OAK* internal color camera
        print(f"Internal camera FPS set to: {self.internal_fps}")
        self.resolution = (1920, 1080)
        print("Sensor resolution:", self.resolution)

        # Check if the device supports stereo
        cameras = self.device.getConnectedCameras()
        if dai.CameraBoardSocket.LEFT in cameras and dai.CameraBoardSocket.RIGHT in cameras:
            self.xyz = True
        else:
            print("Warning: depth unavailable on this device, 'xyz' argument is ignored")

        ####image size
        width, self.scale_nd = mpu.find_isp_scale_params(640 * self.resolution[0] / self.resolution[1], self.resolution, is_height=False)
        self.img_h = int(round(self.resolution[1] * self.scale_nd[0] / self.scale_nd[1]))
        self.img_w = int(round(self.resolution[0] * self.scale_nd[0] / self.scale_nd[1]))
        self.pad_h = (self.img_w - self.img_h) // 2
        self.pad_w = 0
        self.frame_size = self.img_w
        self.crop_w = 0

        print(f"Internal camera image size: {self.img_w} x {self.img_h} - crop_w:{self.crop_w} pad_h: {self.pad_h}")

        #### Create SSD anchors
        self.pd_input_length = 128 # Palm detection
        # self.pd_input_length = 192 # Palm detection
        self.anchors = mpu.generate_handtracker_anchors(self.pd_input_length, self.pd_input_length)
        self.nb_anchors = self.anchors.shape[0]
        print(f"{self.nb_anchors} anchors have been created")

        #### Define and start pipeline
        usb_speed = self.device.getUsbSpeed()
        self.device.startPipeline(self.create_pipeline())
        print(f"Pipeline started - USB speed: {str(usb_speed).split('.')[-1]}")
       

        #### Define data queues
        if self.input_type == "rgb":
            self.q_video = self.device.getOutputQueue(name="cam_out", maxSize=1, blocking=False)
            self.q_pd_out = self.device.getOutputQueue(name="pd_out", maxSize=1, blocking=False)
            self.q_manip_cfg = self.device.getInputQueue(name="manip_cfg")
            if self.use_lm:
                self.q_lm_out = self.device.getOutputQueue(name="lm_out", maxSize=2, blocking=False)
                self.q_lm_in = self.device.getInputQueue(name="lm_in")
            if self.xyz:
                self.q_spatial_data = self.device.getOutputQueue(name="spatial_data_out", maxSize=4, blocking=False)
                self.q_spatial_config = self.device.getInputQueue("spatial_calc_config_in")

        self.fps = FPS()

        self.nb_frames_pd_inference = 0
        self.nb_frames_lm_inference = 0
        self.nb_lm_inferences = 0
        self.nb_failed_lm_inferences = 0
        self.nb_frames_lm_inference_after_landmarks_ROI = 0
        self.nb_frames_no_hand = 0
        self.nb_spatial_requests = 0
        self.glob_pd_rtrip_time = 0
        self.glob_lm_rtrip_time = 0
        self.glob_spatial_rtrip_time = 0

        self.use_previous_landmarks = False
        self.nb_hands_in_previous_frame = 0
        self.single_hand_count = 0

        if self.use_handedness_average:
            # handedness_avg: for more robustness, instead of using the last inferred handedness, we prefer to use the average of the inferred handedness since use_previous_landmarks is True.
            self.handedness_avg = [mpu.HandednessAverage() for i in range(self.max_hands)]


    def create_pipeline(self):
        print("Creating pipeline...")
        # Start defining a pipeline
        pipeline = dai.Pipeline()
        pipeline.setOpenVINOVersion(version = dai.OpenVINO.Version.VERSION_2021_4)

        if self.input_type == "rgb":
            # ColorCamera
            print("Creating Color Camera...")
            cam = pipeline.createColorCamera()
            if self.resolution[0] == 1920:
                cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            else:
                cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
            cam.setBoardSocket(dai.CameraBoardSocket.RGB)
            cam.setInterleaved(False)
            cam.setIspScale(self.scale_nd[0], self.scale_nd[1])
            cam.setFps(self.internal_fps)

            manip = pipeline.createImageManip()
            manip.setMaxOutputFrameSize(self.pd_input_length*self.pd_input_length*3)
            manip.setWaitForConfigInput(True)
            manip.inputImage.setQueueSize(1)
            manip.inputImage.setBlocking(False)
            cam.preview.link(manip.inputImage)
            if self.crop:
                cam.setVideoSize(self.frame_size, self.frame_size)
                cam.setPreviewSize(self.pd_input_length, self.pd_input_length)

            else:
                cam.setVideoSize(self.img_w, self.img_h)
                cam.setPreviewSize(self.img_w, self.img_h)

            manip_cfg_in = pipeline.createXLinkIn()
            manip_cfg_in.setStreamName("manip_cfg")
            manip_cfg_in.out.link(manip.inputConfig)

            cam_out = pipeline.createXLinkOut()
            cam_out.setStreamName("cam_out")
            cam_out.input.setQueueSize(1)
            cam_out.input.setBlocking(False)
            cam.video.link(cam_out.input)

            if self.xyz:
                print("Creating MonoCameras, Stereo and SpatialLocationCalculator nodes...")
                # For now, RGB needs fixed focus to properly align with depth.
                # The value used during calibration should be used here
                calib_data = self.device.readCalibration()
                calib_lens_pos = calib_data.getLensPosition(dai.CameraBoardSocket.RGB)
                print(f"RGB calibration lens position: {calib_lens_pos}")
                cam.initialControl.setManualFocus(calib_lens_pos)

                mono_resolution = dai.MonoCameraProperties.SensorResolution.THE_400_P
                left = pipeline.createMonoCamera()
                left.setBoardSocket(dai.CameraBoardSocket.LEFT)
                left.setResolution(mono_resolution)
                left.setFps(self.internal_fps)

                right = pipeline.createMonoCamera()
                right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
                right.setResolution(mono_resolution)
                right.setFps(self.internal_fps)

                stereo = pipeline.createStereoDepth()
                stereo.setConfidenceThreshold(230)
                # LR-check is required for depth alignment
                stereo.setLeftRightCheck(True)
                stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
                stereo.setSubpixel(False)  # subpixel True -> latency

                spatial_location_calculator = pipeline.createSpatialLocationCalculator()
                spatial_location_calculator.setWaitForConfigInput(True)
                spatial_location_calculator.inputDepth.setBlocking(False)
                spatial_location_calculator.inputDepth.setQueueSize(1)

                spatial_data_out = pipeline.createXLinkOut()
                spatial_data_out.setStreamName("spatial_data_out")
                spatial_data_out.input.setQueueSize(1)
                spatial_data_out.input.setBlocking(False)

                spatial_calc_config_in = pipeline.createXLinkIn()
                spatial_calc_config_in.setStreamName("spatial_calc_config_in")

                left.out.link(stereo.left)
                right.out.link(stereo.right)

                stereo.depth.link(spatial_location_calculator.inputDepth)

                spatial_location_calculator.out.link(spatial_data_out.input)
                spatial_calc_config_in.out.link(spatial_location_calculator.inputConfig)

        # Define palm detection model
        print("Creating Palm Detection Neural Network...")
        pd_nn = pipeline.createNeuralNetwork()
        pd_nn.setBlobPath(self.pd_model)
        # Palm detection input
        if self.input_type == "rgb":
            # Specify that network takes latest arriving frame in non-blocking manner
            pd_nn.input.setQueueSize(1)
            pd_nn.input.setBlocking(False)
            if self.crop:
                cam.preview.link(pd_nn.input)
            else:
                manip.out.link(pd_nn.input)
        else:
            pd_in = pipeline.createXLinkIn()
            pd_in.setStreamName("pd_in")
            pd_in.out.link(pd_nn.input)

        # Palm detection output
        pd_out = pipeline.createXLinkOut()
        pd_out.setStreamName("pd_out")
        pd_nn.out.link(pd_out.input)

         # Define hand landmark model
        if self.use_lm:
            print(f"Creating Hand Landmark Neural Network ({'1 thread' if self.lm_nb_threads == 1 else '2 threads'})...")
            lm_nn = pipeline.createNeuralNetwork()
            lm_nn.setBlobPath(self.lm_model)
            lm_nn.setNumInferenceThreads(self.lm_nb_threads)
            # Hand landmark input
            self.lm_input_length = 224
            lm_in = pipeline.createXLinkIn()
            lm_in.setStreamName("lm_in")
            lm_in.out.link(lm_nn.input)
            # Hand landmark output
            lm_out = pipeline.createXLinkOut()
            lm_out.setStreamName("lm_out")
            lm_nn.out.link(lm_out.input)

        print("Pipeline created.")
        return pipeline


    def pd_postprocess(self, inference):
        # print(inference.getAllLayerNames())
        scores = np.array(inference.getLayerFp16("classificators"), dtype=np.float16) # 896
        # scores = np.array(inference.getLayerFp16("Identity_1"), dtype=np.float16)
        bboxes = np.array(inference.getLayerFp16("regressors"), dtype=np.float16).reshape((self.nb_anchors,18)) # 896x18
        # bboxes = np.array(inference.getLayerFp16("Identity"), dtype=np.float16).reshape((self.nb_anchors,18))
        # Decode bboxes
        hands = mpu.decode_bboxes(self.pd_score_thresh, scores, bboxes, self.anchors, scale=self.pd_input_length, best_only=self.solo)
        # Non maximum suppression (not needed if solo)
        if not self.solo:
            hands = mpu.non_max_suppression(hands, self.pd_nms_thresh)[:self.max_hands]
        if self.use_lm:
            mpu.detections_to_rect(hands)
            mpu.rect_transformation(hands, self.frame_size, self.frame_size)
        return hands


    def lm_postprocess(self, hand, inference):
        # print(inference.getAllLayerNames())
        # The output names of the landmarks model are :
        # Identity_1 (1x1) : score
        # Identity_2 (1x1) : handedness
        # Identity_3 (1x63) : world 3D landmarks (in meters)
        # Identity (1x63) : screen 3D landmarks (in pixels)
        hand.lm_score = inference.getLayerFp16("Identity_1")[0]
        if hand.lm_score > self.lm_score_thresh:
            hand.handedness = inference.getLayerFp16("Identity_2")[0]
            lm_raw = np.array(inference.getLayerFp16("Identity_dense/BiasAdd/Add")).reshape(-1,3)
            # hand.norm_landmarks contains the normalized ([0:1]) 3D coordinates of landmarks in the square rotated body bounding box
            hand.norm_landmarks = lm_raw / self.lm_input_length
            # hand.norm_landmarks[:,2] /= 0.4

            # Now calculate hand.landmarks = the landmarks in the image coordinate system (in pixel)
            src = np.array([(0, 0), (1, 0), (1, 1)], dtype=np.float32)
            dst = np.array([ (x, y) for x,y in hand.rect_points[1:]], dtype=np.float32) # hand.rect_points[0] is left bottom point and points going clockwise!
            mat = cv2.getAffineTransform(src, dst)
            lm_xy = np.expand_dims(hand.norm_landmarks[:,:2], axis=0)
            # lm_z = hand.norm_landmarks[:,2:3] * hand.rect_w_a  / 0.4
            hand.landmarks = np.squeeze(cv2.transform(lm_xy, mat)).astype(np.int32)

            # World landmarks
            if self.use_world_landmarks:
                hand.world_landmarks = np.array(inference.getLayerFp16("Identity_3_dense/BiasAdd/Add")).reshape(-1,3)


    def spatial_loc_roi_from_palm_center(self, hand):
        half_size = int(hand.pd_box[2] * self.frame_size / 2)
        zone_size = max(half_size//2, 8)
        rect_center = dai.Point2f(int(hand.pd_box[0]*self.frame_size) + half_size - zone_size//2 + self.crop_w, int(hand.pd_box[1]*self.frame_size) + half_size - zone_size//2 - self.pad_h)
        rect_size = dai.Size2f(zone_size, zone_size)
        return dai.Rect(rect_center, rect_size)

    def spatial_loc_roi_from_wrist_landmark(self, hand):
        zone_size = max(int(hand.rect_w_a / 10), 8)
        rect_center = dai.Point2f(*(hand.landmarks[0]-np.array((zone_size//2 - self.crop_w, zone_size//2 + self.pad_h))))
        rect_size = dai.Size2f(zone_size, zone_size)
        return dai.Rect(rect_center, rect_size)

    def query_xyz(self, spatial_loc_roi_func):
        conf_datas = []
        for h in self.hands:
            conf_data = dai.SpatialLocationCalculatorConfigData()
            conf_data.depthThresholds.lowerThreshold = 100
            conf_data.depthThresholds.upperThreshold = 10000
            conf_data.roi = spatial_loc_roi_func(h)
            conf_datas.append(conf_data)
        if len(conf_datas) > 0:
            cfg = dai.SpatialLocationCalculatorConfig()
            cfg.setROIs(conf_datas)

            spatial_rtrip_time = now()
            self.q_spatial_config.send(cfg)

            # Receives spatial locations
            spatial_data = self.q_spatial_data.get().getSpatialLocations()
            self.glob_spatial_rtrip_time += now() - spatial_rtrip_time
            self.nb_spatial_requests += 1
            for i,sd in enumerate(spatial_data):
                self.hands[i].xyz_zone =  [
                    int(sd.config.roi.topLeft().x) - self.crop_w,
                    int(sd.config.roi.topLeft().y),
                    int(sd.config.roi.bottomRight().x) - self.crop_w,
                    int(sd.config.roi.bottomRight().y)
                    ]
                self.hands[i].xyz = np.array([
                    sd.spatialCoordinates.x,
                    sd.spatialCoordinates.y,
                    sd.spatialCoordinates.z
                    ])

    def next_frame(self):
        hand_label = None
        bag = {}
        self.fps.update()
        if self.input_type == "rgb":
            if not self.use_previous_landmarks:
                # Send image manip config to the device
                cfg = dai.ImageManipConfig()
                # We prepare the input to the Palm detector
                cfg.setResizeThumbnail(self.pd_input_length, self.pd_input_length)
                self.q_manip_cfg.send(cfg)

            in_video = self.q_video.get()
            video_frame = in_video.getCvFrame()
            #in our case, we need to flip the camera
            video_frame = cv2.flip(video_frame, -1)
            if self.pad_h:
                square_frame = cv2.copyMakeBorder(video_frame, self.pad_h, self.pad_h, self.pad_w, self.pad_w, cv2.BORDER_CONSTANT)
            else:
                square_frame = video_frame

        # Get palm detection
        if self.use_previous_landmarks:
            self.hands = self.hands_from_landmarks
        else:
            inference = self.q_pd_out.get()

            hands = self.pd_postprocess(inference)
            if self.trace & 1:
                print(f"Palm detection - nb hands detected: {len(hands)}")
            self.nb_frames_pd_inference += 1
            bag["pd_inference"] = 1
            if not self.solo and self.nb_hands_in_previous_frame == 1 and len(hands) <= 1:
                self.hands = self.hands_from_landmarks
            else:
                self.hands = hands

        if len(self.hands) == 0: self.nb_frames_no_hand += 1

        if self.use_lm:
            nb_lm_inferences = len(self.hands)
            # Hand landmarks, send requests
            for i,h in enumerate(self.hands):
                img_hand = mpu.warp_rect_img(h.rect_points, square_frame, self.lm_input_length, self.lm_input_length)
                nn_data = dai.NNData()
                nn_data.setLayer("input_1", to_planar(img_hand, (self.lm_input_length, self.lm_input_length)))
                self.q_lm_in.send(nn_data)
                if i == 0: lm_rtrip_time = now() # We measure only for the first hand
            # Get inference results
            for i,h in enumerate(self.hands):
                inference = self.q_lm_out.get()
                if i == 0: self.glob_lm_rtrip_time += now() - lm_rtrip_time
                self.lm_postprocess(h, inference)
            bag["lm_inference"] = len(self.hands)
            self.hands = [ h for h in self.hands if h.lm_score > self.lm_score_thresh]

            if self.trace & 1:
                print(f"Landmarks - nb hands detected : {len(self.hands)}")

            # Check that 2 detected hands do not correspond to the same hand in the image
            # That may happen when one hand in the image cross another one
            # A simple method is to assure that the center of the rotated rectangles are not too close
            # if len(self.hands) == 2:
            #     dist_rect_centers = mpu.distance(np.array((self.hands[0].rect_x_center_a, self.hands[0].rect_y_center_a)), np.array((self.hands[1].rect_x_center_a, self.hands[1].rect_y_center_a)))
            #     if dist_rect_centers < 5:
            #         # Keep the hand with higher landmark score
            #         if self.hands[0].lm_score > self.hands[1].lm_score:
            #             self.hands = [self.hands[0]]
            #         else:
            #             self.hands = [self.hands[1]]
            #         if self.trace & 1: print("!!! Removing one hand because too close to the other one")

            if self.xyz:
                self.query_xyz(self.spatial_loc_roi_from_wrist_landmark)

            self.hands_from_landmarks = [mpu.hand_landmarks_to_rect(h) for h in self.hands]

            nb_hands = len(self.hands)

            if self.use_handedness_average:
                if not self.use_previous_landmarks or self.nb_hands_in_previous_frame != nb_hands:
                    for i in range(self.max_hands):
                        self.handedness_avg[i].reset()
                for i in range(nb_hands):
                    self.hands[i].handedness = self.handedness_avg[i].update(self.hands[i].handedness)

            # In duo mode , make sure only one left hand and one right hand max is returned everytime
            if not self.solo and nb_hands == 2 and (self.hands[0].handedness - 0.5) * (self.hands[1].handedness - 0.5) > 0:
                self.hands = [self.hands[0]] # We keep the hand with best score
                nb_hands = 1
                if self.trace & 1: print("!!! Removing one hand because same handedness")

            if not self.solo:
                if nb_hands == 1:
                    self.single_hand_count += 1
                else:
                    self.single_hand_count = 0

            # Stats
            if nb_lm_inferences: self.nb_frames_lm_inference += 1
            self.nb_lm_inferences += nb_lm_inferences
            self.nb_failed_lm_inferences += nb_lm_inferences - nb_hands
            if self.use_previous_landmarks: self.nb_frames_lm_inference_after_landmarks_ROI += 1

            self.use_previous_landmarks = True
            if nb_hands == 0:
                self.use_previous_landmarks = False
            elif not self.solo and nb_hands == 1:
                    if self.single_hand_count >= self.single_hand_tolerance_thresh:
                        self.use_previous_landmarks = False
                        self.single_hand_count = 0

            self.nb_hands_in_previous_frame = nb_hands

            for hand in self.hands:
                # If we added padding to make the image square, we need to remove this padding from landmark coordinates and from rect_points
                if self.pad_h > 0:
                    hand.landmarks[:,1] -= self.pad_h
                    for i in range(len(hand.rect_points)):
                        hand.rect_points[i][1] -= self.pad_h
                if self.pad_w > 0:
                    hand.landmarks[:,0] -= self.pad_w
                    for i in range(len(hand.rect_points)):
                        hand.rect_points[i][0] -= self.pad_w

                # Set the hand label
                hand.label = "right" if hand.handedness > 0.5 else "left"

        else: # not use_lm
            if self.xyz:
                self.query_xyz(self.spatial_loc_roi_from_palm_center)

        return video_frame, self.hands, bag


    def exit(self):
        self.device.close()
        # Print some stats
        if self.stats:
            nb_frames = self.fps.nb_frames()
            print(f"FPS : {self.fps.get_global():.1f} f/s (# frames = {nb_frames})")
            print(f"# frames w/ no hand           : {self.nb_frames_no_hand} ({100*self.nb_frames_no_hand/nb_frames:.1f}%)")
            print(f"# frames w/ palm detection    : {self.nb_frames_pd_inference} ({100*self.nb_frames_pd_inference/nb_frames:.1f}%)")
            if self.use_lm:
                print(f"# frames w/ landmark inference : {self.nb_frames_lm_inference} ({100*self.nb_frames_lm_inference/nb_frames:.1f}%)- # after palm detection: {self.nb_frames_lm_inference - self.nb_frames_lm_inference_after_landmarks_ROI} - # after landmarks ROI prediction: {self.nb_frames_lm_inference_after_landmarks_ROI}")
                if not self.solo:
                    print(f"On frames with at least one landmark inference, average number of landmarks inferences/frame: {self.nb_lm_inferences/self.nb_frames_lm_inference:.2f}")
                print(f"# lm inferences: {self.nb_lm_inferences} - # failed lm inferences: {self.nb_failed_lm_inferences} ({100*self.nb_failed_lm_inferences/self.nb_lm_inferences:.1f}%)")
            if self.input_type != "rgb":
                print(f"Palm detection round trip            : {self.glob_pd_rtrip_time/self.nb_frames_pd_inference*1000:.1f} ms")
                if self.use_lm and self.nb_lm_inferences:
                    print(f"Hand landmark round trip             : {self.glob_lm_rtrip_time/self.nb_lm_inferences*1000:.1f} ms")
            if self.xyz:
                print(f"Spatial location requests round trip : {self.glob_spatial_rtrip_time/self.nb_spatial_requests*1000:.1f} ms")
