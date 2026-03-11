import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class RealsenseViewNode(Node):
    def __init__(self) -> None:
        super().__init__('realsense_view_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Realsense view node started. Subscribed to /camera/color/image_raw')

    def image_callback(self, msg: Image) -> None:
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the frame
            cv2.imshow('RealSense RGB Feed', frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealsenseViewNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()