"""
coordinate_transformer.py – ROS2-node for piksel→robot-koordinattransformasjon.

Subscriberer på /cube_positions (pikselkoordinater fra cube_detector)
og publiserer /cube_positions_robot (koordinater i robotens base_frame).

Bruker en homografi-matrise beregnet fra kalibreringspunkter.

Publiserer:
  /cube_positions_robot  (geometry_msgs/PoseArray)
    - Hver Pose: position.x/y = robot-koordinater (meter),
      position.z = farge-ID (0=rød, 1=gul, 2=blå)

Service:
  /calibrate_transform  (std_srvs/Trigger)
    - Beregner homografi-matrisen på nytt fra parametrene.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger


class CoordinateTransformer(Node):
    def __init__(self):
        super().__init__('coordinate_transformer')

        # --- Parametre ---
        # Kalibreringspunkter: liste av [piksel_x, piksel_y, robot_x, robot_y]
        self.declare_parameter('calibration.calibration_points', [])

        # Fast z-høyde for robotmål (bordflate + offset)
        self.declare_parameter('calibration.table_z', 0.01)
        self.declare_parameter('positions.safe_z_offset', 0.10)

        # --- Beregn homografi fra parametre ---
        self.homography = None
        self._compute_homography()

        # --- Subscriber: pikselkoordinater fra cube_detector ---
        self.pixel_sub = self.create_subscription(
            PoseArray,
            '/cube_positions',
            self.pixel_callback,
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
    def _compute_homography(self):
        """
        Beregn homografi-matrisen fra kalibreringspunktene.
        Trenger minst 4 punktpar [piksel_x, piksel_y, robot_x, robot_y].
        """
        raw = self.get_parameter('calibration.calibration_points').value

        if not raw or len(raw) < 4:
            self.get_logger().warn(
                'For få kalibreringspunkter – trenger minst 4 stk. '
                'Transformer er deaktivert til kalibrering er gjort.'
            )
            self.homography = None
            return

        # raw er en flat liste: [px1,py1,rx1,ry1, px2,py2,rx2,ry2, ...]
        # Reshape til Nx4
        try:
            points = np.array(raw, dtype=np.float64).reshape(-1, 4)
        except ValueError:
            self.get_logger().error(
                'Kalibreringspunkter har feil format. '
                'Forventet flat liste delelig på 4.'
            )
            self.homography = None
            return

        if len(points) < 4:
            self.get_logger().warn(
                f'Bare {len(points)} punktpar – trenger minst 4.'
            )
            self.homography = None
            return

        pixel_pts = points[:, 0:2].astype(np.float32)
        robot_pts = points[:, 2:4].astype(np.float32)

        H, status = cv2.findHomography(pixel_pts, robot_pts)

        if H is None:
            self.get_logger().error('Kunne ikke beregne homografi.')
            self.homography = None
            return

        self.homography = H
        self.get_logger().info(
            f'Homografi beregnet fra {len(points)} punktpar ✓'
        )

    def _pixel_to_robot(self, px, py):
        """
        Transformer ett pikselkoordinat til robotkoordinat via homografi.
        Returnerer (robot_x, robot_y) eller None.
        """
        if self.homography is None:
            return None

        # Homogen koordinat
        pt = np.array([[[px, py]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.homography)

        robot_x = float(transformed[0][0][0])
        robot_y = float(transformed[0][0][1])
        return (robot_x, robot_y)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def pixel_callback(self, msg: PoseArray):
        """Motta pikselkoordinater, transformer, publiser robotkoordinater."""
        if self.homography is None:
            self.get_logger().warn(
                'Homografi ikke kalibrert – dropper transformasjon.',
                throttle_duration_sec=5.0
            )
            return

        table_z = self.get_parameter('calibration.table_z').value
        z_offset = self.get_parameter('positions.safe_z_offset').value
        target_z = table_z + z_offset

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
            pose_out.position.z = pose_in.position.z  # behold farge-ID

            # Sett orientasjon til "pekende rett ned" (quaternion)
            pose_out.orientation.x = 1.0
            pose_out.orientation.y = 0.0
            pose_out.orientation.z = 0.0
            pose_out.orientation.w = 0.0

            robot_poses.poses.append(pose_out)

            color = color_names.get(pose_in.position.z, '?')
            self.get_logger().info(
                f'  {color}: piksel ({pose_in.position.x:.0f}, '
                f'{pose_in.position.y:.0f}) → robot ({robot_x:.4f}, '
                f'{robot_y:.4f})'
            )

        self.robot_pub.publish(robot_poses)

    def calibrate_callback(self, request, response):
        """Re-les kalibreringspunkter og beregn homografi på nytt."""
        self._compute_homography()
        if self.homography is not None:
            response.success = True
            response.message = 'Homografi oppdatert'
        else:
            response.success = False
            response.message = 'Kalibrering feilet – sjekk parametrene'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()