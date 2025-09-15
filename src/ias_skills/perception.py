from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
import skiros2_common.tools.logger as log
import moveit_commander
import sys
import threading
import math

import rospy
from cv_bridge import CvBridge, CvBridgeError
import tf
import cv2 as cv
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from wsg_50_common.srv import Move
import numpy as np


class HSVDetector:
    def __init__(self, hsv_min, hsv_max):
        self.hsv_min = hsv_min
        self.hsv_max = hsv_max

    def detect(self, hsv_image):
        """
        TASK:
            Fill in this function

            This function should make use of the min and max hsv values
            in self.hsv_min and self.hsv_max.

            It should perform HSV detection using these values,
            and return a rectangle bounding box for its detection.

        HINTS:
            Your detector may pick up some pixels that are not part of the toy blocks.
            The OpenCV functions findContours() and contourArea() may help you deal with this.

        Parameters:
            hsv_image: An OpenCV image in HSV color space.

        Returns:
            A rectangle bounding box as a tuple (x, y, width, height)
            representing the detection made by this detector.
            If detection fails, return None.
        """
        [m, n, d] = hsv_image.shape
        # cv.imshow('image',hsv_image)
        # cv.waitKey(0)
        # print("blue ",hsv_image[214,441,:])
        # print("green ",hsv_image[159,584,:])
        # print("yellow ",hsv_image[83,473,:])
        # print("orange ",hsv_image[129,346,:])
        # print("yellow edge ",hsv_image[104,470,:])
        # print("green edge ",hsv_image[160,559,:])
        mask = np.zeros((m, n))
        for i in range(0, m):
            for j in range(0, n):
                if hsv_image[i, j, 0] <= self.hsv_max[0] and hsv_image[i, j, 0] >= self.hsv_min[0] and hsv_image[i, j, 1] <= self.hsv_max[1] and hsv_image[i, j, 1] >= self.hsv_min[1] and hsv_image[i, j, 2] <= self.hsv_max[2] and hsv_image[i, j, 2] >= self.hsv_min[2]:
                    # print('hello')
                    mask[i, j] = 1

        kernel = np.ones((5, 5))
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # cv.imshow('hello',mask)
        # cv.waitKey(0)
        # cv.imwrite('~/mask.jpg',mask)
        mask_copy = mask.copy()
        mask = np.array(mask, dtype=np.uint8)
        contours, hierarchy = cv.findContours(
            mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        valid_contours = []
        for i in contours:
            area = cv.contourArea(i)
            if area > 100:
                valid_contours.append(i)
        # cv.drawContours(hsv_image,valid_contours,-1, (0,255,0),3)
        # cv.imshow("contour", hsv_image)
        # cv.waitKey(0)
        if len(valid_contours) == 0:
            return None
        else:
            return cv.boundingRect(valid_contours[0])


"""
TASK:
    Fill in appropriate min and max values for HSV detection.

HINTS:
    Make a window with sliders like the one you saw in the ConstructSim course.
"""
HSV_DETECTORS = {
    'blue':   HSVDetector(hsv_min=(84, 77, 185),
                          hsv_max=(125, 255, 255)),

    'orange': HSVDetector(hsv_min=(14, 162, 218),
                          hsv_max=(20, 255, 255)),

    'yellow': HSVDetector(hsv_min=(25, 93, 196),
                          hsv_max=(32, 255, 255)),

    'green':  HSVDetector(hsv_min=(32, 79, 177),
                          hsv_max=(82, 255, 255)),
}


def detect_free_space_impl(depth_image, blocks):
    """
    TASK:
        Fill in this function.

        This function takes a depth image and detections from one or more HSVDetector.

        It should return x,y coordinates of a point where a new block can be placed.

        The returned point must be on the table and away from the detected blocks.

        You may assume that the table is clear (except for the detected blocks),
        and that the camera is looking straight down on the table.

    HINTS:
        The floor has depth-value 0.
        Look at the OpenCV function distanceTransform().

    Parameters:
        depth_image: An OpenCV depth image.
        blocks: Detected blocks in the form of a dictionary {color: bounding_box, ...}.

    Returns:
        A tuple (x, y) representing a free-space position, where a new block could be placed.
    """
    [m, n] = depth_image.shape
    # print(blocks)
    # cv.imshow("depth", depth_image)
    # cv.waitKey(0)
    mask = np.zeros((m, n))
    for i in range(0, m):
        for j in range(0, n):
            if depth_image[i, j] != 0:
                isInside = False
                for k in blocks.values():
                    if i >= k[1] and i <= k[1]+k[3] and j >= k[0] and j <= k[0] + k[2]:
                        isInside = True
                        break
                if not isInside:
                    mask[i, j] = 1

    kernel = np.ones((20, 20))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    # cv.imshow("mask",mask)
    # cv.waitKey(0)
    mask_padded = np.zeros((m+2, n+2))
    mask_padded[1:-1, 1:-1] = mask.copy()

    # cv.imshow("padded", mask_padded)
    # cv.waitKey(0)

    mask_padded = np.uint8(mask_padded)
    distances = cv.distanceTransform(mask_padded, cv.DIST_L2, 5)
    # normalized = cv.normalize(distances,None, 0,255,cv.NORM_MINMAX, cv.CV_8U)
    # cv.imshow("distance",normalized)
    # cv.waitKey(0)
    # print(distances[0,:])
    mm, nn = distances.shape
    maxx = 0
    ind_i = 0
    ind_j = 0
    for i in range(0, mm):
        for j in range(0, nn):
            if distances[i, j] > maxx:
                maxx = distances[i, j]
                ind_i = i
                ind_j = j
    # print(ind_i,ind_j)
    x = ind_j
    y = ind_i
    return (x, y)


class DepthListener:
    def __init__(self, topic='/realsense/aligned_depth_to_color/image_raw'):
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image = 0.0

        rospy.Subscriber(topic, Image, callback=self.callback)

    def callback(self, data):
        if self.image is not None:
            return

        """
        TASK:
            The variable 'data' is a ROS message containing a depth image.
            Transform it into an OpenCV image using self.bridge.

            The result should be stored in self.image.

        HINTS:
            The resulting image should be a single-channel image of type uint16.
        """
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            # self.image = np.zeros((1, 1), dtype=np.uint16)
        except CvBridgeError as e:
            print(e)

    def get(self):
        self.image = None
        while self.image is None:
            self.rate.sleep()
        return self.image


class RGBListener:
    def __init__(self, topic='/realsense/rgb/image_raw'):
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image = 0.0

        rospy.Subscriber(topic, Image, callback=self.image_callback)

    def image_callback(self, data):
        if self.image is not None:
            return

        """
        TASK:
            The variable 'data' is a ROS message containing a RGB image.
            Transform it into an OpenCV image using self.bridge.

            The result should be stored in self.image.

        HINTS:
            The resulting image should be an 8-bit image in BGR color space.
        """
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # self.image = np.zeros((1, 1, 3), dtype=np.uint8)

    def get(self):
        self.image = None
        while self.image is None:
            self.rate.sleep()
        return self.image

# This is just a helper function to perform HSV
# detection using several detectors on the same image.


def detect_blocks(rgb_image, detectors):
    hsv = cv.cvtColor(rgb_image, cv.COLOR_BGR2HSV)

    out = {}
    for key, detector in detectors.items():
        det = detector.detect(hsv)
        if det is not None:
            out[key] = det

    return out


class CameraPoseEstimator:

    # This is a helper method to get fx/fy, cx/cy from the camera_info topic.
    @staticmethod
    def receive_camera_info(info_topic):
        class Dummy:
            def __init__(self):
                self.data = None

            def callback(self, x):
                self.data = x

        data = Dummy()
        sub = rospy.Subscriber(info_topic, CameraInfo, callback=data.callback)

        r = rospy.Rate(10)
        while data.data is None:
            r.sleep()
        sub.unregister()

        k = np.array(data.data.K).reshape((3, 3))

        fx, fy = k[0, 0], k[1, 1]
        cx, cy = k[0, 2], k[1, 2]
        return (fx, fy), (cx, cy)

    def __init__(self, info_topic):
        (self.fx, self.fy), (self.cx, self.cy) = self.receive_camera_info(info_topic)

    def pixel_to_3d(self, pix_x, pix_y, depth_m):
        """
        TASK:
            Implement this.
            This function should take pixel x/y-coordinates and convert
            them into a 3D location.

            At your disposal, you have:
                self.fx: the x-coordinate focal length.
                self.fy: the y-coordinate focal length.
                self.cx: the x-coordinate frame offset.
                self.cy: the y-coordinate frame offset.

        HINTS:
            Make sure you understand how a camera transforms from 3D into 2D pixel coordinates.

        Parameters:
            pix_x: pixel x-coordinate.
            pix_y: pixel y-coordinate.
            depth_m: the z-coordinate (i.e. depth away from the camera) in meters.
        Returns:
            A tuple (x, y) representing the x/y-coordinates converted into 3D space.
        """
        x = (pix_x - self.cx)*depth_m/self.fx
        y = (pix_y - self.cy)*depth_m/self.fy
        return (x, y)


class Transformer:
    def __init__(self):
        self.tflistener = tf.TransformListener()

    def transform_coordinates(self, x, y, z, from_frame, to_frame):
        """
        TASKS:
            Implement this.
            This function should transform a set of x/y/z-coordinates from one
            transformation frame into another.

        HINTS:
            You may be tempted to disregard orientation completely,
            and simply add the vector offset between the transformation frames:
                pos, _ = self.tflistener.lookupTransform(to_frame, from_frame, ...)
                x += pos[0]
                y += pos[1]
                z += pos[2]

            BUT, that doesn't account for the fact that the transformation frames may be
            rotated with respect to each other.
            For example the z-coordinate in to_frame may point in the same direction as
            the y-coordinate in the from_frame.

            The function TransformListener.transformPose(...) may be helpful here.

        Parameters:
            x: the x-coordinate to be transformed.
            y: the y-coordinate to be transformed.
            z: the z-coordinate to be transformed.
            from_frame: the transformation frame the the x/y/z-coordinates are currently in.
            to_frame: the transoformation frame the x/y/z-coordinates should be transformed into.
        Returns:
            A tuple (x, y, z) representing the transformed coordinates.
        """
        mpose = PoseStamped()
        mpose.pose.position.x = x
        mpose.pose.position.y = y
        mpose.pose.position.z = z
        mpose.header.frame_id = from_frame

        mpose_transf = self.tflistener.transformPose(to_frame, mpose)
        return (mpose_transf.pose.position.x, mpose_transf.pose.position.y, mpose_transf.pose.position.z)

#################################################################################
# Block Detection
#################################################################################


class DetectBlock(SkillDescription):

    def createDescription(self):
        self.addParam("Object", Element(
            "skiros:TransformationPose"), ParamTypes.Required)


class detect_block(PrimitiveBase):

    def createDescription(self):
        self.setDescription(DetectBlock(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def run(self):

        obj = self.params['Object'].value
        object_frame = obj.getProperty('skiros:BaseFrameId').value

        #hsv_detector = HSV_DETECTORS['blue']
        hsv_detector = HSV_DETECTORS['orange']
        #hsv_detector = HSV_DETECTORS['yellow']
        #hsv_detector = HSV_DETECTORS['green']

        transformer = Transformer()
        rgb_listener = RGBListener()
        depth_listener = DepthListener()
        pose_estimator = CameraPoseEstimator('/realsense/rgb/camera_info')

        rgb = rgb_listener.get()
        depth = depth_listener.get()

        hsv = cv.cvtColor(rgb, cv.COLOR_BGR2HSV)

        (x, y, w, h) = hsv_detector.detect(hsv)

        # Just making sure everything is a float, because python2...
        x, y, w, h = float(x), float(y), float(w), float(h)

        center_x = x + w/2.0
        center_y = y + h/2.0

        depth_meters = depth[int(center_y), int(center_x)] / 1000.0

        dx, dy = pose_estimator.pixel_to_3d(center_x, center_y, depth_meters)

        # The rgb camera position is slightly wrong, so we fix that...
        #dx -= 0.04

        object_x, object_y, object_z = transformer.transform_coordinates(dx, dy, depth_meters,
                                                                         'realsense_rgb_optical_frame', object_frame)

        # This is the part of the code that updates the world model.
        obj.setData(':Position', [object_x, object_y, object_z+0.08])
        self._wmi.update_element_properties(obj)

        self.done = True

    def onStart(self):
        self.done = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        return True

    def execute(self):
        if not self.done:
            return self.step('Running...')

        self.thread.join()

        return self.success('Done')

#################################################################################
# Free Space Detection
#################################################################################


class DetectFreeSpace(SkillDescription):

    def createDescription(self):
        self.addParam("PlaneLocation", Element("skiros:TransformationPose"), ParamTypes.Required,
                      description='Table location to update with correct coordinates.')


class detect_free_space(PrimitiveBase):

    def createDescription(self):
        self.setDescription(DetectFreeSpace(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def run(self):
        tabletop = self.params['PlaneLocation'].value

        transformer = Transformer()
        rgb_listener = RGBListener()
        depth_listener = DepthListener()

        rgb_image = rgb_listener.get()
        depth_image = depth_listener.get()
        pose_estimator = CameraPoseEstimator('/realsense/rgb/camera_info')

        blocks = detect_blocks(rgb_image, HSV_DETECTORS)

        x, y = detect_free_space_impl(depth_image, blocks)

        depth_meters = depth_image[y, x] / 1000.0

        x, y = float(x), float(y)

        dx, dy = pose_estimator.pixel_to_3d(x, y, depth_meters)

        # The rgb camera position is slightly wrong, so we fix that...
        #dx -= 0.04

        free_space_frame = tabletop.getProperty('skiros:BaseFrameId').value
        free_space_x, free_space_y, free_space_z = transformer.transform_coordinates(dx, dy, depth_meters,
                                                                                     'realsense_rgb_optical_frame', free_space_frame)

        pos = np.array([
            free_space_x,
            free_space_y,
            free_space_z+.08
        ])

        tabletop.setData(':Position', pos)
        self._wmi.update_element_properties(tabletop)

        self.done = True

    def onStart(self):
        self.done = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        return True

    def execute(self):
        if not self.done:
            return self.step('Running...')

        self.thread.join()

        return self.success('Done')
