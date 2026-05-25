"""
cube_detector.py – ROS2-node for fargedeteksjon av kuber.

Subscriberer på kamera-topic, kjører HSV-fargesegmentering,
og publiserer detekterte kubeposisjoner (pikselkoordinater).

Publiserer:
  /cube_positions  (geometry_msgs/PoseArray)
    - Hver Pose: position.x/y = pikselkoordinater, position.z = farge-ID
      (0 = rød, 1 = gul, 2 = blå)

Service:
  /detect_cubes  (std_srvs/Trigger)
    - Trigger én deteksjon på neste mottatte bilde.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        # --- Deklarer parametre (leses fra project_params.yaml) ---
        self.declare_parameter('camera.image_topic', '/camera/color/image_raw')

        # Røde HSV-terskler (to intervaller fordi rød wrapper rundt 0/180)
        self.declare_parameter('color_thresholds.red.lower1', [0, 120, 70])
        self.declare_parameter('color_thresholds.red.upper1', [10, 255, 255])
        self.declare_parameter('color_thresholds.red.lower2', [170, 120, 70])
        self.declare_parameter('color_thresholds.red.upper2', [180, 255, 255])

        # Gul
        self.declare_parameter('color_thresholds.yellow.lower', [20, 100, 100])
        self.declare_parameter('color_thresholds.yellow.upper', [35, 255, 255])

        # Blå
        self.declare_parameter('color_thresholds.blue.lower', [100, 150, 50])
        self.declare_parameter('color_thresholds.blue.upper', [130, 255, 255])

        # Deteksjon
        self.declare_parameter('detection.min_contour_area', 500)

        # --- Les parametre ---
        image_topic = self.get_parameter('camera.image_topic').value
        self._load_color_thresholds()

        self.min_area = self.get_parameter('detection.min_contour_area').value

        # --- Intern tilstand ---
        self.bridge = CvBridge()
        self.latest_image = None
        self.detection_requested = False
        self.last_results = PoseArray()

        # --- Subscriber ---
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # --- Publisher ---
        self.cube_pub = self.create_publisher(PoseArray, '/cube_positions', 10)

        # --- Service ---
        self.detect_srv = self.create_service(
            Trigger,
            '/detect_cubes',
            self.detect_service_callback
        )

        self.get_logger().info(
            f'CubeDetector startet – lytter på "{image_topic}"'
        )

    # ------------------------------------------------------------------
    # Parameterinnlasting
    # ------------------------------------------------------------------
    def _load_color_thresholds(self):
        """Les HSV-terskler fra parametre og lagre som numpy-arrays."""
        self.color_ranges = {}

        # Rød har to intervaller
        self.color_ranges['red'] = [
            (
                np.array(self.get_parameter('color_thresholds.red.lower1').value, dtype=np.uint8),
                np.array(self.get_parameter('color_thresholds.red.upper1').value, dtype=np.uint8),
            ),
            (
                np.array(self.get_parameter('color_thresholds.red.lower2').value, dtype=np.uint8),
                np.array(self.get_parameter('color_thresholds.red.upper2').value, dtype=np.uint8),
            ),
        ]

        # Gul – ett intervall
        self.color_ranges['yellow'] = [
            (
                np.array(self.get_parameter('color_thresholds.yellow.lower').value, dtype=np.uint8),
                np.array(self.get_parameter('color_thresholds.yellow.upper').value, dtype=np.uint8),
            ),
        ]

        # Blå – ett intervall
        self.color_ranges['blue'] = [
            (
                np.array(self.get_parameter('color_thresholds.blue.lower').value, dtype=np.uint8),
                np.array(self.get_parameter('color_thresholds.blue.upper').value, dtype=np.uint8),
            ),
        ]

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        """Lagre siste bilde. Kjør deteksjon hvis forespurt."""
        self.latest_image = msg

        if self.detection_requested:
            self.detection_requested = False
            self._run_detection()

    def detect_service_callback(self, request, response):
        """
        Service-kall: trigger deteksjon på neste bilde.
        Hvis vi allerede har et bilde, kjør med en gang.
        """
        if self.latest_image is not None:
            self._run_detection()
            found_colors = self._summarize_results()
            response.success = len(found_colors) > 0
            response.message = f'Detekterte: {", ".join(found_colors)}' if found_colors else 'Ingen kuber funnet'
        else:
            # Vent på neste bilde
            self.detection_requested = True
            response.success = True
            response.message = 'Venter på neste kamerabilde...'

        return response

    # ------------------------------------------------------------------
    # Deteksjonslogikk
    # ------------------------------------------------------------------
    def _run_detection(self):
        """Kjør fargesegmentering på siste mottatte bilde."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Kunne ikke konvertere bilde: {e}')
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Gaussisk blur for å redusere støy
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'camera_frame'

        color_ids = {'red': 0.0, 'yellow': 1.0, 'blue': 2.0}

        for color_name, ranges in self.color_ranges.items():
            center = self._detect_color(hsv, ranges)
            if center is not None:
                pose = Pose()
                pose.position.x = float(center[0])   # piksel-x
                pose.position.y = float(center[1])   # piksel-y
                pose.position.z = color_ids[color_name]  # farge-ID
                pose_array.poses.append(pose)
                self.get_logger().info(
                    f'  {color_name}: piksel ({center[0]}, {center[1]})'
                )

        self.last_results = pose_array
        self.cube_pub.publish(pose_array)

        self.get_logger().info(
            f'Deteksjon ferdig – fant {len(pose_array.poses)} kube(r)'
        )

    def _detect_color(self, hsv_image, ranges):
        """
        Finn senterpunktet til den største konturen som matcher
        de gitte HSV-intervallene.

        Returnerer (cx, cy) eller None.
        """
        # Kombiner masker fra alle intervaller for denne fargen
        combined_mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.inRange(hsv_image, lower, upper)
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        # Morfologisk opprydding: fjern støy, fyll hull
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        # Finn konturer
        contours, _ = cv2.findContours(
            combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None

        # Velg den største konturen over minimumsareal
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            return None

        # Beregn senterpunkt via moments
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return (cx, cy)

    def _summarize_results(self):
        """Returner liste med fargenavn som ble detektert."""
        id_to_name = {0.0: 'rød', 1.0: 'gul', 2.0: 'blå'}
        return [
            id_to_name.get(p.position.z, '?')
            for p in self.last_results.poses
        ]


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
