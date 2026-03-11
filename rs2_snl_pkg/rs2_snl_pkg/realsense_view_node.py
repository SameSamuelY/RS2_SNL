import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RealsenseViewNode(Node):

    def __init__(self):
        super().__init__('realsense_view_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info(
            'Realsense view node started. Subscribed to /camera/camera/color/image_raw'
        )

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Make a copy for drawing contours
        contour_frame = frame.copy()

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Threshold the image
        _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)

        # Find contours
        contours, _ = cv2.findContours(
            thresh,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Filter small contours and draw the valid ones
        min_area = 500
        valid_count = 0

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < min_area:
                continue

            valid_count += 1
            cv2.drawContours(contour_frame, [contour], -1, (0, 255, 0), 2)

        # Add some debug text
        cv2.putText(
            contour_frame,
            f'Contours: {valid_count}',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

        # Show images
        cv2.imshow("RealSense RGB Feed", frame)
        cv2.imshow("Threshold", thresh)
        cv2.imshow("Contours", contour_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = RealsenseViewNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()