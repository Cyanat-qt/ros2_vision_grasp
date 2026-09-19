"""Live ROS RGB and detection display; close the window or press Esc to exit."""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main(args=None):
    rclpy.init(args=args)
    node = Node('fr3_camera_view')
    cv = CvBridge()
    frames = {}
    def receive(msg, key):
        frames[key] = cv.imgmsg_to_cv2(msg, 'bgr8')
    subscriptions = [node.create_subscription(Image, topic, lambda msg, key=key: receive(msg, key),
                                               qos_profile_sensor_data)
                     for key, topic in [('Raw RGB', '/camera/color/image_raw'),
                                        ('Detection / base XYZ', '/perception/annotated_image')]]
    window = 'FR3 | ROS2 camera + detection'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if len(frames) == 2:
                cv2.imshow(window, cv2.hconcat([frames['Raw RGB'], frames['Detection / base XYZ']]))
            if cv2.waitKey(1) == 27 or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
