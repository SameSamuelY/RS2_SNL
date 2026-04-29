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
        self.latest_depth_frame = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_frame = 'camera_color_optical_frame'

        # Use the tag frame that your WORKING manual AprilTag node publishes.
        # If /tf says child_frame_id: tag_0, use tag_0.
        # If /tf says child_frame_id: tag36h11:0, use tag36h11:0.
        self.tag_frame = 'tag_0'

        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            String,
            '/detection_result',
            10
        )

        self.get_logger().info(
            'Shape colour detector started. Detecting strict red/green/blue geometric objects only.'
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
        peri = cv2.arcLength(contour, True)

        if peri == 0:
            return 'Irregular Shape'

        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        vertices = len(approx)

        if vertices == 3:
            return 'Triangle'

        if vertices == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = w / float(h) if h != 0 else 0.0

            if 0.85 <= aspect_ratio <= 1.15:
                return 'Square'

            return 'Rectangle'

        if vertices > 4:
            area = cv2.contourArea(contour)
            circularity = 4 * np.pi * area / (peri * peri + 1e-6)

            if circularity > 0.65:
                return 'Circle'

        return 'Irregular Shape'

    def get_depth_at_pixel(self, u, v):
        if self.latest_depth_frame is None:
            return None

        h, w = self.latest_depth_frame.shape[:2]

        if not (0 <= u < w and 0 <= v < h):
            return None

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

        return float(depth_value) / 1000.0

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

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0,   150, 120])
        upper_red1 = np.array([10,  255, 255])
        lower_red2 = np.array([170, 150, 120])
        upper_red2 = np.array([179, 255, 255])

        lower_green = np.array([40, 100, 80])   # narrowed hue, raised sat
        upper_green = np.array([95, 255, 255])

        lower_blue = np.array([100, 160, 100])  # raised saturation floor
        upper_blue = np.array([125, 255, 255])

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2)
        )
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        colour_masks = [
            ('Red', red_mask),
            ('Green', green_mask),
            ('Blue', blue_mask),
        ]

        kernel = np.ones((5, 5), np.uint8)

        valid_count = 0
        detection_list = []

        img_h, img_w = frame.shape[:2]

        for colour, mask in colour_masks:
            cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                cleaned_mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)

                # Tune these if needed:
                # lower min area if small objects are missed
                # lower max area if large intended objects are missed
                if area < 200 or area > 1500:
                    continue

                x, y, w, h = cv2.boundingRect(contour)

                aspect_ratio = w / float(h) if h != 0 else 0.0
                if aspect_ratio < 0.35 or aspect_ratio > 2.8:
                    continue

                # Reject partial objects / border noise
                if x <= 5 or y <= 5 or (x + w) >= img_w - 5 or (y + h) >= img_h - 5:
                    continue

                M = cv2.moments(contour)
                if M['m00'] == 0:
                    continue

                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])

                shape = self.detect_shape(contour)

                if shape not in ['Circle', 'Square', 'Rectangle', 'Triangle']:
                    continue

                contour_mask = np.zeros(mask.shape, dtype='uint8')
                cv2.drawContours(contour_mask, [contour], -1, 255, -1)

                colour_pixels = cv2.countNonZero(cv2.bitwise_and(mask, mask, mask=contour_mask))
                contour_pixels = cv2.countNonZero(contour_mask)

                if contour_pixels == 0:
                    continue

                colour_ratio = colour_pixels / float(contour_pixels)

                if colour_ratio < 0.70:
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

                cv2.putText(
                    display_frame,
                    f'uv: ({cX}, {cY})',
                    (cX - 70, cY + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 0),
                    2
                )

                if camera_xyz is not None:
                    cv2.putText(
                        display_frame,
                        f'C: {camera_xyz[0]:.3f}, {camera_xyz[1]:.3f}, {camera_xyz[2]:.3f}',
                        (cX - 100, cY + 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (0, 255, 255),
                        2
                    )

                if tag_xyz is not None:
                    cv2.putText(
                        display_frame,
                        f'T: {tag_xyz[0]:.3f}, {tag_xyz[1]:.3f}, {tag_xyz[2]:.3f}',
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

        msg_out = String()
        msg_out.data = json.dumps(detection_list)
        self.publisher_.publish(msg_out)

        cv2.imshow('RGB Feed', frame)
        cv2.imshow('Mask Red', red_mask)
        cv2.imshow('Mask Green', green_mask)
        cv2.imshow('Mask Blue', blue_mask)
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