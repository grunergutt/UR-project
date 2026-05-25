"""
robot_mover.py – ROS2-node for robotbevegelse.

Tilbyr services for å flytte roboten til:
  - Hjemmeposisjon    (/move_to_home)
  - Fotoposisjon      (/move_to_photo)
  - Vilkårlige joints (/move_to_joints)  – via FollowJointTrajectory
  - Kartesisk pose    (/move_to_pose)    – via MoveIt MoveGroup

Services:
  /move_to_home   (std_srvs/Trigger)
  /move_to_photo  (std_srvs/Trigger)
  /move_to_joints (robot_control custom – bruker Trigger med JSON i message)
  /move_to_pose   (robot_control custom – bruker Trigger med JSON i message)
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration

# MoveIt er valgfritt – kartesisk bevegelse deaktiveres hvis ikke installert
try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (
        MotionPlanRequest,
        Constraints,
        PositionConstraint,
        OrientationConstraint,
        BoundingVolume,
        WorkspaceParameters,
    )
    from shape_msgs.msg import SolidPrimitive
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False

# UR joint-navn (standard for alle UR-roboter)
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')

        # Callback-gruppe som tillater parallelle kall
        self.cb_group = ReentrantCallbackGroup()

        # --- Parametre ---
        self.declare_parameter('positions.home',
                               [0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0])
        self.declare_parameter('positions.photo',
                               [0.0, -1.2, 1.4, -1.77, -1.5708, 0.0])
        self.declare_parameter('positions.safe_z_offset', 0.10)
        self.declare_parameter('robot.controller',
                               'scaled_joint_trajectory_controller')
        self.declare_parameter('calibration.table_z', 0.01)

        controller = self.get_parameter('robot.controller').value

        # --- Action-klienter ---
        # 1) Joint trajectory (direkte bevegelse)
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{controller}/follow_joint_trajectory',
            callback_group=self.cb_group,
        )

        # 2) MoveIt MoveGroup (kartesisk bevegelse med IK)
        self.moveit_client = None
        if MOVEIT_AVAILABLE:
            self.moveit_client = ActionClient(
                self,
                MoveGroup,
                '/move_action',
                callback_group=self.cb_group,
            )

        # --- Services ---
        self.create_service(
            Trigger, '/move_to_home',
            self.home_callback,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger, '/move_to_photo',
            self.photo_callback,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger, '/move_to_joints',
            self.joints_callback,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger, '/move_to_pose',
            self.pose_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('RobotMover startet – venter på action-servere...')
        self._wait_for_servers()
        self.get_logger().info('RobotMover klar ✓')

    # ------------------------------------------------------------------
    # Oppstart
    # ------------------------------------------------------------------
    def _wait_for_servers(self):
        """Vent til action-serverne er tilgjengelige."""
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn(
                '  FollowJointTrajectory-server ikke funnet – '
                'er ur_robot_driver startet?'
            )
        else:
            self.get_logger().info('  FollowJointTrajectory-server funnet')

        if not MOVEIT_AVAILABLE:
            self.get_logger().warn(
                '  moveit_msgs ikke installert – kartesisk bevegelse deaktivert. '
                'Installer med: sudo apt install ros-jazzy-moveit'
            )
        elif self.moveit_client and not self.moveit_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                '  MoveGroup-server ikke funnet – kartesisk bevegelse deaktivert. '
                'Start MoveIt med: ros2 launch ur_moveit_config ur_moveit.launch.py'
            )

    # ------------------------------------------------------------------
    # Hjelpemetode for å vente på futures uten å spinne executoren
    # ------------------------------------------------------------------
    def _wait_for_future(self, future, timeout_sec=10.0):
        """Vent på at en future fullføres uten å kalle spin (unngår re-entrant spin)."""
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                self.get_logger().warn('Timeout ved venting på future')
                return False
            time.sleep(0.05)
        return True

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------
    def home_callback(self, request, response):
        joints = self.get_parameter('positions.home').value
        self.get_logger().info('Beveger til hjemmeposisjon...')
        success = self._send_joint_goal(joints)
        response.success = success
        response.message = 'Hjemme ✓' if success else 'Feil ved bevegelse'
        return response

    def photo_callback(self, request, response):
        joints = self.get_parameter('positions.photo').value
        self.get_logger().info('Beveger til fotoposisjon...')
        success = self._send_joint_goal(joints)
        response.success = success
        response.message = 'Fotoposisjon ✓' if success else 'Feil ved bevegelse'
        return response

    def joints_callback(self, request, response):
        """
        Flytt til vilkårlige joint-verdier.
        Kall med: ros2 service call /move_to_joints std_srvs/srv/Trigger
                  "{}" -- men target settes via parameter.
        For programmatisk bruk: sett parameter 'target_joints' før kall.
        """
        self.declare_parameter('target_joints', [0.0] * 6)
        joints = self.get_parameter('target_joints').value
        self.get_logger().info(f'Beveger til joints: {joints}')
        success = self._send_joint_goal(joints)
        response.success = success
        response.message = 'Joints nådd ✓' if success else 'Feil ved bevegelse'
        return response

    def pose_callback(self, request, response):
        """
        Flytt til en kartesisk pose via MoveIt.
        Sett parameter 'target_pose' = [x, y, z, qx, qy, qz, qw] før kall.
        """
        self.declare_parameter('target_pose', [0.0] * 7)
        pose_vals = self.get_parameter('target_pose').value

        if len(pose_vals) != 7:
            response.success = False
            response.message = 'target_pose må ha 7 verdier: [x,y,z,qx,qy,qz,qw]'
            return response

        self.get_logger().info(
            f'Beveger til pose: xyz=({pose_vals[0]:.3f}, {pose_vals[1]:.3f}, '
            f'{pose_vals[2]:.3f})'
        )
        success = self._send_cartesian_goal(pose_vals)
        response.success = success
        response.message = 'Pose nådd ✓' if success else 'MoveIt feil'
        return response

    # ------------------------------------------------------------------
    # Offentlig metode for coordinator
    # ------------------------------------------------------------------
    def move_to_joints(self, joint_values, duration_sec=4.0):
        """Direkte kall for bruk fra coordinator-noden."""
        return self._send_joint_goal(joint_values, duration_sec)

    def move_to_cartesian(self, x, y, z, qx=1.0, qy=0.0, qz=0.0, qw=0.0):
        """Direkte kall for kartesisk bevegelse."""
        return self._send_cartesian_goal([x, y, z, qx, qy, qz, qw])

    # ------------------------------------------------------------------
    # Joint trajectory
    # ------------------------------------------------------------------
    def _send_joint_goal(self, joint_values, duration_sec=4.0):
        """Send en joint-trajectory med ett punkt til kontrolleren."""
        if len(joint_values) != 6:
            self.get_logger().error(f'Forventet 6 joints, fikk {len(joint_values)}')
            return False

        goal = FollowJointTrajectory.Goal()

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(j) for j in joint_values]
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        trajectory.points.append(point)

        goal.trajectory = trajectory

        # Send og vent på resultat
        send_future = self.trajectory_client.send_goal_async(goal)
        self._wait_for_future(send_future, timeout_sec=5.0)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Joint-goal ble avvist')
            return False

        self.get_logger().info('Joint-goal akseptert – venter...')
        result_future = goal_handle.get_result_async()
        self._wait_for_future(result_future, timeout_sec=duration_sec + 10.0)

        result = result_future.result()
        if result and result.result.error_code == 0:
            self.get_logger().info('Bevegelse fullført ✓')
            return True
        else:
            error = result.result.error_code if result else 'timeout'
            self.get_logger().error(f'Bevegelse feilet: {error}')
            return False

    # ------------------------------------------------------------------
    # MoveIt kartesisk bevegelse
    # ------------------------------------------------------------------
    def _send_cartesian_goal(self, pose_vals):
        """Send en kartesisk pose til MoveIt MoveGroup."""
        if not MOVEIT_AVAILABLE or self.moveit_client is None:
            self.get_logger().error(
                'moveit_msgs ikke installert. '
                'Installer med: sudo apt install ros-jazzy-moveit'
            )
            return False

        if not self.moveit_client.server_is_ready():
            self.get_logger().error(
                'MoveGroup-server ikke tilgjengelig. Start MoveIt først.'
            )
            return False

        x, y, z, qx, qy, qz, qw = pose_vals

        # Bygg MoveGroup-goal
        goal = MoveGroup.Goal()

        # Motion plan request
        req = MotionPlanRequest()
        req.group_name = 'ur_manipulator'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0

        # Målpose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

        # Posisjonsconstraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = 'tool0'
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        bounding = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1 cm toleranse
        bounding.primitives.append(primitive)
        bounding.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = bounding
        pos_constraint.weight = 1.0

        # Orienteringsconstraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header = target_pose.header
        orient_constraint.link_name = 'tool0'
        orient_constraint.orientation = target_pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        req.goal_constraints.append(constraints)

        # Workspace
        ws = WorkspaceParameters()
        ws.header.frame_id = 'base_link'
        ws.min_corner.x = -1.0
        ws.min_corner.y = -1.0
        ws.min_corner.z = -0.5
        ws.max_corner.x = 1.0
        ws.max_corner.y = 1.0
        ws.max_corner.z = 1.5
        req.workspace_parameters = ws

        goal.request = req
        goal.planning_options.plan_only = False  # planlegg OG utfør

        # Send og vent
        send_future = self.moveit_client.send_goal_async(goal)
        self._wait_for_future(send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('MoveIt-goal avvist')
            return False

        self.get_logger().info('MoveIt planlegger og kjører...')
        result_future = goal_handle.get_result_async()
        self._wait_for_future(result_future, timeout_sec=30.0)

        result = result_future.result()
        if result and result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Kartesisk bevegelse fullført ✓')
            return True
        else:
            code = result.result.error_code.val if result else 'timeout'
            self.get_logger().error(f'MoveIt feilet med kode: {code}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()