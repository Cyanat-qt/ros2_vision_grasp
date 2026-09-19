"""Synchronized RGB-D color detection and TF2 localization."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from .core import color_regions


class Perception(Node):
    def __init__(self):
        super().__init__('fr3_color_detector')
        for key, default in [('base_frame', 'fr3_link0'), ('table_height', 0.05),
                             ('object_size', 0.03), ('workspace', [0.25, 0.72, -0.28, 0.28])]:
            self.declare_parameter(key, default)
        self.base = self.get_parameter('base_frame').value
        self.size = float(self.get_parameter('object_size').value)
        self.table = float(self.get_parameter('table_height').value)
        self.workspace = self.get_parameter('workspace').value
        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.listener = TransformListener(self.tf, self)
        self.cv = CvBridge()
        self.annotated = self.create_publisher(Image, '/perception/annotated_image', qos_profile_sensor_data)
        self.detections = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.markers = self.create_publisher(MarkerArray, '/perception/markers', 10)
        self.targets = {c: self.create_publisher(PoseStamped, f'/perception/{c}/pose', 10)
                        for c in ('red', 'green', 'blue')}
        self.subs = [message_filters.Subscriber(self, kind, topic, qos_profile=qos_profile_sensor_data)
                     for kind, topic in [(Image, '/camera/color/image_raw'),
                                         (Image, '/camera/depth/image_raw'),
                                         (CameraInfo, '/camera/color/camera_info')]]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subs, 5, 0.025)
        self.sync.registerCallback(self.process)
        self.last_warning = 0
        self.get_logger().info('Color detector ready; output /perception/annotated_image and /perception/<color>/pose')

    def process(self, rgb_msg, depth_msg, info):
        if rgb_msg.header.frame_id != depth_msg.header.frame_id or rgb_msg.header.frame_id != info.header.frame_id:
            self.warn('RGB, aligned depth and CameraInfo must share their optical frame')
            return
        if depth_msg.encoding not in ('32FC1', '16UC1'):
            self.warn('Unsupported depth encoding: ' + depth_msg.encoding)
            return
        rgb = self.cv.imgmsg_to_cv2(rgb_msg, 'rgb8')
        depth = self.cv.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)
        if depth_msg.encoding == '16UC1':
            depth *= 0.001
        if depth.shape != rgb.shape[:2] or info.width != rgb.shape[1] or info.height != rgb.shape[0]:
            self.warn('RGB-D dimensions do not agree with CameraInfo')
            return
        k = np.array(info.k).reshape(3, 3)
        if k[0, 0] <= 0 or k[1, 1] <= 0:
            self.warn('Invalid camera intrinsics')
            return
        display = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        array = Detection2DArray()
        array.header = rgb_msg.header
        markers = MarkerArray()
        accepted = {}
        try:
            transform = self.tf.lookup_transform(self.base, rgb_msg.header.frame_id,
                                                  Time.from_msg(rgb_msg.header.stamp))
        except TransformException as exc:
            self.warn('Waiting for camera TF: ' + str(exc))
            transform = None
        for region in color_regions(rgb, depth, k):
            if transform is None:
                continue
            point = PointStamped()
            point.header = rgb_msg.header
            point.point.x, point.point.y, point.point.z = map(float, region['point'])
            base = do_transform_point(point, transform).point
            xmin, xmax, ymin, ymax = self.workspace
            # Initial demo assumes known cube height on a horizontal table. Reject colored bins.
            if not (xmin < base.x < xmax and ymin < base.y < ymax and
                    abs(base.z - (self.table + self.size)) < 0.012):
                continue
            color = region['color']
            if color not in accepted or region['area'] > accepted[color][0]['area']:
                accepted[color] = region, base
        for index, (color, (region, point)) in enumerate(accepted.items()):
            pose = PoseStamped()
            pose.header.stamp, pose.header.frame_id = rgb_msg.header.stamp, self.base
            pose.pose.position.x, pose.pose.position.y = point.x, point.y
            pose.pose.position.z = point.z - self.size / 2  # visible top -> cube centre
            pose.pose.orientation.w = 1.0
            self.targets[color].publish(pose)
            x, y, w, h = region['box']
            detection = Detection2D()
            detection.header = rgb_msg.header
            detection.id = color
            detection.bbox.center.position.x = float(x + w / 2)
            detection.bbox.center.position.y = float(y + h / 2)
            detection.bbox.size_x, detection.bbox.size_y = float(w), float(h)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = color, 1.0
            # Detection2D remains in image frame; metric poses use separate stamped topics.
            detection.results = [hypothesis]
            array.detections.append(detection)
            bgr = {'red': (0, 0, 255), 'green': (0, 200, 0), 'blue': (255, 120, 0)}[color]
            cv2.rectangle(display, (x, y), (x+w, y+h), bgr, 2)
            label = f'{color}  ({point.x:.3f}, {point.y:.3f}, {pose.pose.position.z:.3f}) m'
            cv2.putText(display, label, (max(0, x-35), max(18, y-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1)
            marker = Marker()
            marker.header, marker.ns, marker.id = pose.header, 'objects', index
            marker.type, marker.action, marker.pose = Marker.CUBE, Marker.ADD, pose.pose
            marker.scale.x = marker.scale.y = marker.scale.z = self.size
            marker.color.r, marker.color.g, marker.color.b = [c/255 for c in reversed(bgr)]
            marker.color.a = 0.75
            marker.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers.append(marker)
        cv2.putText(display, 'RGB-D / TF2 | target coordinates in fr3_link0', (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1)
        output = self.cv.cv2_to_imgmsg(display, 'bgr8')
        output.header = rgb_msg.header
        self.annotated.publish(output)
        self.detections.publish(array)
        self.markers.publish(markers)

    def warn(self, message):
        now = self.get_clock().now().nanoseconds
        if now - self.last_warning > 2e9:
            self.get_logger().warning(message)
            self.last_warning = now


def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
