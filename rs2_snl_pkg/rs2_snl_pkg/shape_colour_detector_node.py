import json
import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException


class ShapeColourDetectorNode(Node):

    def __init__(self):
        super().__init__('shape_colour_detector_node')

        self.bridge = CvBridge()

        # Latest depth frame
        self.latest_depth_frame = None

        # Camera intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # TF buffer + listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Frame names
        self.camera_frame = 'camera_color_optical_frame'
        self.tag_frame = 'tag_0'

        # RGB subscription
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Aligned depth subscription
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        # Camera info subscription
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Detection result publisher
        self.publisher_ = self.create_publisher(
            String,
            '/detection_result',
            10
        )

        self.get_logger().info(
            'Shape colour detector node started. '
            'Publishing valid RGB objects only (red/green/blue + circle/square/rectangle/triangle). '
            'Transforming object points into tag_0 when available.'
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')

    def detect_shape(self, contour):
        """
        Returns one of:
        Triangle, Square, Rectangle, Circle, Irregular Shape
        """
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        vertices = len(approx)

        if vertices == 3:
            return 'Triangle'

        if vertices == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = w / float(h) if h != 0 else 0.0
            if 0.9 <= aspect_ratio <= 1.1:
                return 'Square'
            return 'Rectangle'

        if vertices > 4:
            area = cv2.contourArea(contour)
            circularity = 4 * np.pi * area / (peri * peri + 1e-6)
            if circularity > 0.75:
                return 'Circle'
            return 'Irregular Shape'

        return 'Irregular Shape'

    def detect_colour(self, hsv_frame, contour):
        """
        Returns Red, Green, Blue, or Unknown.

        Unknown is returned if the contour region does not contain enough
        red/green/blue pixels to be considered a valid coloured object.
        This helps reject black/white AprilTags and other background items.
        """
        contour_mask = np.zeros(hsv_frame.shape[:2], dtype='uint8')
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)

        contour_pixels = cv2.countNonZero(contour_mask)
        if contour_pixels == 0:
            return 'Unknown'

        # Red wraps in HSV
        lower_red1 = np.array([0, 100, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 80])
        upper_red2 = np.array([179, 255, 255])

        lower_green = np.array([40, 60, 60])
        upper_green = np.array([85, 255, 255])

        lower_blue = np.array([90, 60, 60])
        upper_blue = np.array([140, 255, 255])

        red_mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

        red_pixels = cv2.countNonZero(cv2.bitwise_and(red_mask, red_mask, mask=contour_mask))
        green_pixels = cv2.countNonZero(cv2.bitwise_and(green_mask, green_mask, mask=contour_mask))
        blue_pixels = cv2.countNonZero(cv2.bitwise_and(blue_mask, blue_mask, mask=contour_mask))

        colour_counts = {
            'Red': red_pixels,
            'Green': green_pixels,
            'Blue': blue_pixels
        }

        best_colour = max(colour_counts, key=colour_counts.get)
        best_count = colour_counts[best_colour]

        # Require a minimum amount of the contour area to belong to one of the target colours
        coverage_ratio = best_count / float(contour_pixels)

        # Tune this if needed
        if coverage_ratio < 0.20:
            return 'Unknown'

        return best_colour

    def get_depth_at_pixel(self, u, v):
        if self.latest_depth_frame is None:
            return None

        h, w = self.latest_depth_frame.shape[:2]

        if not (0 <= u < w and 0 <= v < h):
            return None

        # Small window around centroid for stability
        half_window = 2
        u_min = max(0, u - half_window)
        u_max = min(w, u + half_window + 1)
        v_min = max(0, v - half_window)
        v_max = min(h, v + half_window + 1)

        depth_patch = self.latest_depth_frame[v_min:v_max, u_min:u_max]
        valid_depths = depth_patch[depth_patch > 0]

        if valid_depths.size == 0:
            return None

        depth_value = np.median(valid_depths)

        # RealSense depth commonly arrives as uint16 in millimetres
        depth_m = float(depth_value) / 1000.0
        return depth_m

    def pixel_to_camera_coordinates(self, u, v, depth_m):
        if None in (self.fx, self.fy, self.cx, self.cy):
            return None

        x = (u - self.cx) * depth_m / self.fx
        y = (v - self.cy) * depth_m / self.fy
        z = depth_m

        return x, y, z

    def transform_to_frame(self, x, y, z, target_frame, source_frame=None):
        if source_frame is None:
            source_frame = self.camera_frame

        point_in = PointStamped()
        point_in.header.stamp = self.get_clock().now().to_msg()
        point_in.header.frame_id = source_frame
        point_in.point.x = float(x)
        point_in.point.y = float(y)
        point_in.point.z = float(z)

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )

            point_out = do_transform_point(point_in, transform)

            return (
                point_out.point.x,
                point_out.point.y,
                point_out.point.z
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform from {source_frame} to {target_frame}: {ex}'
            )
            return None

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        display_frame = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Current contour extraction approach
        _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(
            thresh,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        min_area = 500
        valid_count = 0
        detection_list = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue

            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])

            shape = self.detect_shape(contour)
            colour = self.detect_colour(hsv, contour)

            # Reject anything not matching required shapes/colours
            if colour not in ['Red', 'Green', 'Blue']:
                continue

            if shape not in ['Circle', 'Square', 'Rectangle', 'Triangle']:
                continue

            depth_m = self.get_depth_at_pixel(cX, cY)
            camera_xyz = None
            tag_xyz = None

            if depth_m is not None:
                camera_xyz = self.pixel_to_camera_coordinates(cX, cY, depth_m)

            if camera_xyz is not None:
                tag_xyz = self.transform_to_frame(
                    camera_xyz[0],
                    camera_xyz[1],
                    camera_xyz[2],
                    self.tag_frame,
                    self.camera_frame
                )

            detection = {
                'shape': shape,
                'colour': colour,
                'centroid_x': cX,
                'centroid_y': cY,
                'depth_m': depth_m
            }

            if camera_xyz is not None:
                detection['camera_x_m'] = camera_xyz[0]
                detection['camera_y_m'] = camera_xyz[1]
                detection['camera_z_m'] = camera_xyz[2]

            if tag_xyz is not None:
                detection['tag_x_m'] = tag_xyz[0]
                detection['tag_y_m'] = tag_xyz[1]
                detection['tag_z_m'] = tag_xyz[2]

            detection_list.append(detection)
            valid_count += 1

            # Draw only valid objects
            cv2.drawContours(display_frame, [contour], -1, (0, 255, 0), 2)
            cv2.circle(display_frame, (cX, cY), 5, (0, 0, 255), -1)

            label = f'{colour} {shape}'
            cv2.putText(
                display_frame,
                label,
                (cX - 70, cY - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            pixel_text = f'uv: ({cX}, {cY})'
            cv2.putText(
                display_frame,
                pixel_text,
                (cX - 70, cY + 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 0),
                2
            )

            if camera_xyz is not None:
                xyz_text = (
                    f'C: {camera_xyz[0]:.3f}, '
                    f'{camera_xyz[1]:.3f}, '
                    f'{camera_xyz[2]:.3f}'
                )
                cv2.putText(
                    display_frame,
                    xyz_text,
                    (cX - 100, cY + 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 255, 255),
                    2
                )

            if tag_xyz is not None:
                tag_text = (
                    f'T: {tag_xyz[0]:.3f}, '
                    f'{tag_xyz[1]:.3f}, '
                    f'{tag_xyz[2]:.3f}'
                )
                cv2.putText(
                    display_frame,
                    tag_text,
                    (cX - 100, cY + 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 165, 255),
                    2
                )

        cv2.putText(
            display_frame,
            f'Valid Objects: {valid_count}',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

        # Publish only valid objects
        msg_out = String()
        msg_out.data = json.dumps(detection_list)
        self.publisher_.publish(msg_out)

        cv2.imshow('RGB Feed', frame)
        cv2.imshow('Threshold', thresh)
        cv2.imshow('Detected Valid Objects and Coordinates', display_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = ShapeColourDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()