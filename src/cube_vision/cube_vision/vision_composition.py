"""
vision_composition.py – Kjører cube_detector og coordinate_transformer
i én prosess med felles MultiThreadedExecutor.

Dette er ROS2 composition for Python-noder: flere noder deler
prosess og kan kommunisere via intra-process (lavere latens,
ingen serialisering for delte meldinger).

Bruk:
  ros2 run cube_vision vision_composed
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from cube_vision.cube_detector import CubeDetector
from cube_vision.coordinate_transformer import CoordinateTransformer


def main(args=None):
    rclpy.init(args=args)

    detector = CubeDetector()
    transformer = CoordinateTransformer()

    executor = MultiThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(transformer)

    try:
        detector.get_logger().info(
            'Vision composition startet – '
            'cube_detector + coordinate_transformer i én prosess'
        )
        executor.spin()
    finally:
        detector.destroy_node()
        transformer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

