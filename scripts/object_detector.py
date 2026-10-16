#!/usr/bin/env python3
"""AI object detection node.

Subscribes to the depth camera's color image + point cloud, detects the target
object, and publishes its 3D pose (in the robot base frame) on
/detected_object_pose.

The default implementation is a robust HSV color-blob detector. The detection
logic is isolated in `ColorBlobDetector.detect()` so you can drop in a learned
model (e.g. YOLO) without touching the ROS plumbing -- just implement a class
with the same `detect(bgr_image) -> (u, v) or None` contract.
"""
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform)


class ColorBlobDetector:
    """Detects a colored blob and returns its pixel centroid.

    SWAP POINT FOR YOUR OWN AI:
    Replace this class with one exposing the same interface:
        detect(bgr_image: np.ndarray) -> tuple[int, int] | None
    e.g. run a trained detector, take the highest-confidence box, and return
    its center pixel (u, v). Everything downstream (depth lookup, TF, publish)
    stays the same.
    """

    def __init__(self, p):
        self.lower1 = np.array(p['hsv_lower_1'])
        self.upper1 = np.array(p['hsv_upper_1'])
        self.lower2 = np.array(p['hsv_lower_2'])
        self.upper2 = np.array(p['hsv_upper_2'])
        self.min_area = p['min_blob_area']

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower1, self.upper1) | \
            cv2.inRange(hsv, self.lower2, self.upper2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < self.min_area:
            return None
        m = cv2.moments(c)
        if m['m00'] == 0:
            return None
        return int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])


class ObjectDetectorNode:
    def __init__(self):
        rospy.init_node('object_detector')
        p = rospy.get_param('color_detector')
        self.camera_frame = p['camera_frame']
        self.base_frame = p['base_frame']

        self.bridge = CvBridge()
        self.detector = ColorBlobDetector(p)
        self.last_cloud = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher(p['publish_topic'], PoseStamped,
                                   queue_size=1)
        rospy.Subscriber(p['depth_points_topic'], PointCloud2,
                         self.cloud_cb, queue_size=1)
        rospy.Subscriber(p['color_image_topic'], Image,
                         self.image_cb, queue_size=1)
        rospy.loginfo('object_detector ready.')

    def cloud_cb(self, msg):
        self.last_cloud = msg

    def image_cb(self, msg):
        if self.last_cloud is None:
            return
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        uv = self.detector.detect(bgr)
        if uv is None:
            return
        u, v = uv

        # Look up the 3D point at pixel (u, v) from the organized point cloud.
        pts = list(pc2.read_points(self.last_cloud, field_names=('x', 'y', 'z'),
                                   skip_nans=False, uvs=[(u, v)]))
        if not pts:
            return
        x, y, z = pts[0]
        if any(np.isnan([x, y, z])):
            return

        pose_cam = PoseStamped()
        pose_cam.header.frame_id = self.camera_frame
        pose_cam.header.stamp = rospy.Time(0)
        pose_cam.pose.position.x = float(x)
        pose_cam.pose.position.y = float(y)
        pose_cam.pose.position.z = float(z)
        pose_cam.pose.orientation.w = 1.0

        try:
            pose_base = self.tf_buffer.transform(
                pose_cam, self.base_frame, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(2.0, 'TF transform failed: %s' % e)
            return

        pose_base.header.stamp = rospy.Time.now()
        self.pub.publish(pose_base)
        rospy.loginfo_throttle(
            1.0, 'Detected object at base frame: (%.3f, %.3f, %.3f)' %
            (pose_base.pose.position.x, pose_base.pose.position.y,
             pose_base.pose.position.z))


if __name__ == '__main__':
    try:
        ObjectDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
