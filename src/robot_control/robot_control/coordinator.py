"""
coordinator.py – Overordnet koordineringsnode (state machine).

Orkestrerer hele flyten:
  1. Flytt til hjemmeposisjon
  2. Flytt til fotoposisjon
  3. Ta bilde og detekter kuber
  4. Flytt til rød → gul → blå kube
  5. Hvis en farge mangler: søk fra alternative posisjoner
  6. Hvis fortsatt ikke funnet: stopp og varsle

Publiserer /active_camera_position (std_msgs/Int32) for å fortelle
coordinate_transformer hvilken homografi som skal brukes:
  0 = fotoposisjon, 1–3 = søkeposisjon 1–3

States:
  IDLE → HOME → PHOTO → DETECT → MOVE_TO_CUBE → SEARCH → DONE / ERROR
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from enum import Enum, auto


class State(Enum):
    IDLE = auto()
    HOME = auto()
    PHOTO = auto()
    DETECT = auto()
    MOVE_TO_CUBE = auto()
    SEARCH = auto()
    DONE = auto()
    ERROR = auto()


# Fargerekkefølge som oppgaven krever
COLOR_ORDER = [
    (0.0, 'rød'),
    (1.0, 'gul'),
    (2.0, 'blå'),
]


class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')

        self.cb_group = ReentrantCallbackGroup()

        # Parametre
        self.declare_parameter('detection.max_search_attempts', 3)
        self.declare_parameter('positions.safe_z_offset', 0.10)
        self.declare_parameter('calibration.table_z', 0.01)
        self.declare_parameter('positions.search_positions', [
            0.3, -1.2, 1.4, -1.77, -1.5708, 0.0,
            -0.3, -1.2, 1.4, -1.77, -1.5708, 0.0,
        ])

        self.max_search = self.get_parameter('detection.max_search_attempts').value

        # Intern tilstand
        self.state = State.IDLE
        self.cube_positions = {}
        self.current_color_idx = 0
        self.search_attempt = 0
        self.missing_colors = []

        # Publisher: aktiv kameraposisjon for coordinate_transformer
        self.camera_pos_pub = self.create_publisher(Int32, '/active_camera_position', 10)

        # Service-klienter
        self.home_client = self.create_client(
            Trigger, '/move_to_home', callback_group=self.cb_group
        )
        self.photo_client = self.create_client(
            Trigger, '/move_to_photo', callback_group=self.cb_group
        )
        self.detect_client = self.create_client(
            Trigger, '/detect_cubes', callback_group=self.cb_group
        )
        self.joints_client = self.create_client(
            Trigger, '/move_to_joints', callback_group=self.cb_group
        )
        self.pose_client = self.create_client(
            Trigger, '/move_to_pose', callback_group=self.cb_group
        )

        # Subscriber for kubeposisjoner (robotkoordinater)
        self.positions_sub = self.create_subscription(
            PoseArray,
            '/cube_positions_robot',
            self.positions_callback,
            10,
            callback_group=self.cb_group,
        )

        # Service for å starte sekvensen
        self.start_srv = self.create_service(
            Trigger, '/start_sequence',
            self.start_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            'Coordinator klar. Start med: '
            'ros2 service call /start_sequence std_srvs/srv/Trigger "{}"'
        )


    # Publiser aktiv kameraposisjon
    def _publish_camera_position(self, position_id: int):
        """Fortell coordinate_transformer hvilken homografi som skal brukes."""
        msg = Int32()
        msg.data = position_id
        self.camera_pos_pub.publish(msg)
        self._spin_wait(0.1)  # gi subscriberen tid til å motta


    # Motta kubeposisjoner
    def positions_callback(self, msg: PoseArray):
        """Lagre siste sett med detekterte kubeposisjoner."""
        for pose in msg.poses:
            color_id = pose.position.z
            self.cube_positions[color_id] = (
                pose.position.x,
                pose.position.y,
                pose.orientation,
            )

        color_names = {0.0: 'rød', 1.0: 'gul', 2.0: 'blå'}
        found = [color_names.get(cid, '?') for cid in self.cube_positions]
        self.get_logger().info(f'Oppdatert kubeposisjoner: {found}')


    # Start sekvensen
    def start_callback(self, request, response):
        """Start hele pek-på-kubene-sekvensen."""
        if self.state != State.IDLE and self.state != State.DONE \
                and self.state != State.ERROR:
            response.success = False
            response.message = f'Allerede aktiv i tilstand {self.state.name}'
            return response

        self.get_logger().info('========== STARTER SEKVENS ==========')
        self.cube_positions.clear()
        self.current_color_idx = 0
        self.search_attempt = 0
        self.missing_colors.clear()

        success = self._run_sequence()

        response.success = success
        response.message = (
            'Sekvens fullført ✓' if success
            else f'Sekvens stoppet i tilstand {self.state.name}'
        )
        return response


    # Hovedsekvens
    def _run_sequence(self):
        """Kjør hele state-machine-flyten synkront."""

        # 1. Hjemmeposisjon
        self.state = State.HOME
        self.get_logger().info('[1/4] Flytter til hjemmeposisjon...')
        if not self._call_trigger('/move_to_home', self.home_client):
            return self._error('Kunne ikke nå hjemmeposisjon')

        # 2. Fotoposisjon – sett homografi 0
        self.state = State.PHOTO
        self.get_logger().info('[2/4] Flytter til fotoposisjon...')
        self._publish_camera_position(0)
        if not self._call_trigger('/move_to_photo', self.photo_client):
            return self._error('Kunne ikke nå fotoposisjon')

        # 3. Deteksjon
        self.state = State.DETECT
        self.get_logger().info('[3/4] Tar bilde og detekterer kuber...')
        self._call_trigger('/detect_cubes', self.detect_client)

        self._spin_wait(1.0)

        # Sjekk hvilke farger som mangler
        self.missing_colors = self._find_missing_colors()
        if self.missing_colors:
            self.get_logger().warn(
                f'Mangler: {[c[1] for c in self.missing_colors]}. Starter søk...'
            )
            if not self._search_for_missing():
                still_missing = self._find_missing_colors()
                if still_missing:
                    for _, name in still_missing:
                        self.get_logger().error(
                            f'⚠ VARSEL: Fant ikke {name} kube etter '
                            f'{self.max_search} forsøk!'
                        )
                        
        # Etter søk, før pekefasen — tilbake til hjemme for ren IK-løsning
        if self.missing_colors is not None:
            self.get_logger().info('Tilbake til hjemme før pekefase...')
            self._call_trigger('/move_to_home', self.home_client)

        # 4. Beveg til kubene i rekkefølge
        self.state = State.MOVE_TO_CUBE
        self.get_logger().info('[4/4] Beveger til kubene i rekkefølge...')

        for color_id, color_name in COLOR_ORDER:
            if color_id not in self.cube_positions:
                self.get_logger().warn(f'Hopper over {color_name} – ble ikke funnet')
                continue

            x, y, orientation = self.cube_positions[color_id]
            table_z = self.get_parameter('calibration.table_z').value
            z_offset = self.get_parameter('positions.safe_z_offset').value
            target_z = table_z + z_offset

            self.get_logger().info(
                f'  → Peker på {color_name} kube ved ({x:.3f}, {y:.3f}, {target_z:.3f})'
            )

            if not self._move_to_cartesian(x, y, target_z, orientation):
                self.get_logger().error(f'Kunne ikke nå {color_name} kube')
                continue

            self.get_logger().info(f'  ✓ Peker på {color_name}!')
            self._spin_wait(1.5)

        # Ferdig – tilbake til hjemme
        self.get_logger().info('Tilbake til hjemmeposisjon...')
        self._call_trigger('/move_to_home', self.home_client)

        self.state = State.DONE
        self.get_logger().info('========== SEKVENS FULLFØRT ==========')
        return True


    # Søkelogikk
    def _search_for_missing(self):
        """Prøv søkeposisjoner for å finne manglende kuber."""
        self.state = State.SEARCH
        search_positions = self._get_search_positions()

        for attempt in range(self.max_search):
            self.search_attempt = attempt + 1
            search_idx = attempt + 1  # 1-basert: søkeposisjon 1, 2, 3

            if attempt >= len(search_positions):
                self.get_logger().warn('Ingen flere søkeposisjoner å prøve')
                break

            joints = search_positions[attempt]
            self.get_logger().info(
                f'  Søkeforsøk {attempt + 1}/{self.max_search}: '
                f'prøver søkeposisjon {search_idx}...'
            )

            # Publiser aktiv kameraposisjon FØR bevegelse
            self._publish_camera_position(search_idx)

            # Flytt til søkeposisjon
            self._set_parameter_and_move(joints)

            # Detekter på nytt
            self._call_trigger('/detect_cubes', self.detect_client)
            self._spin_wait(1.0)

            # Sjekk om alle er funnet
            self.missing_colors = self._find_missing_colors()
            if not self.missing_colors:
                self.get_logger().info('  Alle kuber funnet ✓')
                return True

            self.get_logger().info(
                f'  Mangler fortsatt: {[c[1] for c in self.missing_colors]}'
            )

        return len(self._find_missing_colors()) == 0

    def _find_missing_colors(self):
        return [
            (cid, name) for cid, name in COLOR_ORDER
            if cid not in self.cube_positions
        ]

    def _get_search_positions(self):
        raw = self.get_parameter('positions.search_positions').value
        positions = []
        for i in range(0, len(raw) - 5, 6):
            positions.append(raw[i:i + 6])
        return positions


    # Hjelpemetoder
    def _wait_for_future(self, future, timeout_sec=10.0):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                self.get_logger().warn('Timeout ved venting på future')
                return False
            time.sleep(0.05)
        return True

    def _call_trigger(self, name, client, timeout=30.0):
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Service {name} ikke tilgjengelig')
            return False

        future = client.call_async(Trigger.Request())
        self._wait_for_future(future, timeout_sec=timeout)

        if future.result() is None:
            self.get_logger().error(f'{name} ga ingen respons')
            return False

        result = future.result()
        if not result.success:
            self.get_logger().warn(f'{name}: {result.message}')
        return result.success

    def _move_to_cartesian(self, x, y, z, orientation):
        pose_vals = [
            x, y, z,
            orientation.x, orientation.y,
            orientation.z, orientation.w,
        ]

        from rcl_interfaces.msg import Parameter as ParameterMsg
        from rcl_interfaces.msg import ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters

        set_client = self.create_client(
            SetParameters,
            '/robot_mover/set_parameters',
            callback_group=self.cb_group,
        )

        if not set_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Kan ikke nå robot_mover parametertjeneste')
            return False

        param = ParameterMsg()
        param.name = 'target_pose'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        param.value.double_array_value = pose_vals

        req = SetParameters.Request()
        req.parameters = [param]

        future = set_client.call_async(req)
        self._wait_for_future(future, timeout_sec=5.0)

        return self._call_trigger('/move_to_pose', self.pose_client, timeout=30.0)

    def _set_parameter_and_move(self, joint_values):
        from rcl_interfaces.msg import Parameter as ParameterMsg
        from rcl_interfaces.msg import ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters

        set_client = self.create_client(
            SetParameters,
            '/robot_mover/set_parameters',
            callback_group=self.cb_group,
        )

        if not set_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Kan ikke nå robot_mover parametertjeneste')
            return False

        param = ParameterMsg()
        param.name = 'target_joints'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        param.value.double_array_value = [float(j) for j in joint_values]

        req = SetParameters.Request()
        req.parameters = [param]

        future = set_client.call_async(req)
        self._wait_for_future(future, timeout_sec=5.0)

        return self._call_trigger('/move_to_joints', self.joints_client)

    def _spin_wait(self, seconds):
        time.sleep(seconds)

    def _error(self, msg):
        self.state = State.ERROR
        self.get_logger().error(f'FEIL: {msg}')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
