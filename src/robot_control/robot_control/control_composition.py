"""
control_composition.py – Kjører robot_mover og coordinator
i én prosess med felles MultiThreadedExecutor.

MultiThreadedExecutor er kritisk her fordi coordinator kaller
services på robot_mover – med SingleThreadedExecutor ville
dette deadlocke.

Bruk:
  ros2 run robot_control control_composed
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_control.robot_mover import RobotMover
from robot_control.coordinator import Coordinator


def main(args=None):
    rclpy.init(args=args)

    mover = RobotMover()
    coordinator = Coordinator()

    executor = MultiThreadedExecutor()
    executor.add_node(mover)
    executor.add_node(coordinator)

    try:
        mover.get_logger().info(
            'Control composition startet – '
            'robot_mover + coordinator i én prosess'
        )
        executor.spin()
    finally:
        mover.destroy_node()
        coordinator.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

