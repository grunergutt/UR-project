"""
coordinate_transformer.py – ROS2-node for piksel→robot-koordinattransformasjon.

Subscriberer på /cube_positions (pikselkoordinater fra cube_detector)
og publiserer /cube_positions_robot (koordinater i robotens base_frame).

Støtter multi-homografi: én homografi per kameraposisjon (foto + søkeposisjoner).
Aktiv homografi velges via topic /active_camera_position (std_msgs/Int32):
  0 = fotoposisjon
  1 = søkeposisjon 1
  2 = søkeposisjon 2
  3 = søkeposisjon 3

Publiserer:
  /cube_positions_robot  (geometry_msgs/PoseArray)

Service:
  /calibrate_transform  (std_srvs/Trigger)
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32
from std_srvs.srv import Trigger


class CoordinateTransformer(Node):
    def __init__(self):
        super().__init__('coordinate_transformer')

        # --- Parametre ---
        self.declare_parameter('calibration.calibration_points', [0.0])
        self.declare_parameter('calibration.table_z', 0.01)
        self.declare_parameter('positions.safe_z_offset', 0.10)

        # Kalibreringspunkter per søkeposisjon (flat liste, samme format)
        self.declare_parameter('calibration.search_calibration_points_1', [0.0])
        self.declare_parameter('calibration.search_calibration_points_2', [0.0])
        self.declare_parameter('calibration.search_calibration_points_3', [0.0])

        # --- Aktiv posisjon (0=foto, 1-3=søk) ---
        self.active_position = 0

        # --- Beregn alle homografier ---
        self.homographies = {}  # {posisjon_id: H-matrise}
        self._compute_all_homographies()

        # --- Subscriber: pikselkoordinater ---
        self.pixel_sub = self.create_subscription(
            PoseArray,
            '/cube_positions',
            self.pixel_callback,
            10
        )

        # --- Subscriber: hvilken posisjon kameraet er i ---
        self.position_sub = self.create_subscription(
            Int32,
            '/active_camera_position',
            self.position_callback,
            10
        )

        # --- Publisher: robotkoordinater ---
        self.robot_pub = self.create_publisher(
            PoseArray, '/cube_positions_robot', 10
        )

        # --- Service: re-kalibrer ---
        self.cal_srv = self.create_service(
            Trigger,
            '/calibrate_transform',
            self.calibrate_callback
        )

        self.get_logger().info('CoordinateTransformer startet')

    # ------------------------------------------------------------------
    # Homografi
    # ------------------------------------------------------------------
    def _compute_homography_from_points(self, raw, label=''):
        """Beregn homografi fra flat liste [px,py,rx,ry, ...]."""
        if not raw or len(raw) < 16 or (len(raw) == 1 and raw[0] == 0.0):
            self.get_logger().warn(
                f'Kalibreringspunkter mangler for {label} – homografi deaktivert.'
            )
            return None

        try:
            points = np.array(raw, dtype=np.float64).reshape(-1, 4)
        except ValueError:
            self.get_logger().error(f'Feil format på kalibreringspunkter for {label}.')
            return None

        if len(points) < 4:
            self.get_logger().warn(f'For få punktpar for {label}: {len(points)}, trenger minst 4.')
            return None

        pixel_pts = points[:, 0:2].astype(np.float32)
        robot_pts = points[:, 2:4].astype(np.float32)

        H, _ = cv2.findHomography(pixel_pts, robot_pts)

        if H is None:
            self.get_logger().error(f'Kunne ikke beregne homografi for {label}.')
            return None

        self.get_logger().info(f'Homografi beregnet for {label} ({len(points)} punktpar) ✓')
        return H

    def _compute_all_homographies(self):
        """Beregn homografi for fotoposisjon og alle søkeposisjoner."""
        # Posisjon 0: fotoposisjon
        raw = self.get_parameter('calibration.calibration_points').value
        self.homographies[0] = self._compute_homography_from_points(raw, 'fotoposisjon')

        # Posisjon 1–3: søkeposisjoner
        for i in range(1, 4):
            raw = self.get_parameter(f'calibration.search_calibration_points_{i}').value
            self.homographies[i] = self._compute_homography_from_points(raw, f'søkeposisjon {i}')

    def _pixel_to_robot(self, px, py):
        """Transformer pikselkoordinat til robotkoordinat med aktiv homografi."""
        H = self.homographies.get(self.active_position)
        if H is None:
            self.get_logger().warn(
                f'Ingen homografi for posisjon {self.active_position} – dropper.',
                throttle_duration_sec=5.0
            )
            return None

        pt = np.array([[[px, py]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, H)
        return float(transformed[0][0][0]), float(transformed[0][0][1])

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def position_callback(self, msg: Int32):
        """Oppdater hvilken kameraposisjon som er aktiv."""
        self.active_position = msg.data
        self.get_logger().info(
            f'Aktiv kameraposisjon satt til: '
            f'{"foto" if msg.data == 0 else f"søk {msg.data}"}'
        )

    def pixel_callback(self, msg: PoseArray):
        """Motta pikselkoordinater, transformer, publiser robotkoordinater."""
        H = self.homographies.get(self.active_position)
        if H is None:
            self.get_logger().warn(
                f'Homografi for posisjon {self.active_position} ikke kalibrert – dropper.',
                throttle_duration_sec=5.0
            )
            return

        table_z = self.get_parameter('calibration.table_z').value
        z_offset = self.get_parameter('positions.safe_z_offset').value

        robot_poses = PoseArray()
        robot_poses.header.stamp = msg.header.stamp
        robot_poses.header.frame_id = 'base_link'

        color_names = {0.0: 'rød', 1.0: 'gul', 2.0: 'blå'}

        for pose_in in msg.poses:
            result = self._pixel_to_robot(pose_in.position.x, pose_in.position.y)
            if result is None:
                continue

            robot_x, robot_y = result

            pose_out = Pose()
            pose_out.position.x = robot_x
            pose_out.position.y = robot_y
            pose_out.position.z = pose_in.position.z  # farge-ID

            pose_out.orientation.x = -0.3787
            pose_out.orientation.y = -0.9254
            pose_out.orientation.z = 0.0118
            pose_out.orientation.w = 0.0090

            robot_poses.poses.append(pose_out)

            color = color_names.get(pose_in.position.z, '?')
            self.get_logger().info(
                f'  {color} [pos {self.active_position}]: '
                f'piksel ({pose_in.position.x:.0f}, {pose_in.position.y:.0f}) '
                f'→ robot ({robot_x:.4f}, {robot_y:.4f})'
            )

        self.robot_pub.publish(robot_poses)

    def calibrate_callback(self, request, response):
        """Re-les kalibreringspunkter og beregn alle homografier på nytt."""
        self._compute_all_homographies()
        ok = any(H is not None for H in self.homographies.values())
        response.success = ok
        response.message = 'Homografier oppdatert ✓' if ok else 'Kalibrering feilet'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()