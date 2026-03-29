"""
Motorcortex → ROS2 Joint State Bridge

Motorcortex actuatorControlLoop 파라미터를 구독하고
sensor_msgs/JointState 메시지로 퍼블리시합니다.

조인트 매핑:
  HL_joint2_thigh_r  ← actuatorControlLoop01
  HL_joint3_thigh_p  ← actuatorControlLoop02
  HL_joint4_knee_p   ← actuatorControlLoop03
  HL_joint5_ankle_p  ← actuatorControlLoop04
  HL_joint6_toe_p    ← actuatorControlLoop05
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

import motorcortex
import threading
import time
import math

# 엔코더 → 라디안 변환 상수 (4체배 × 1024 CPT = 4096 counts/rev)
COUNTS_PER_REV = 4096
ENCODER_TO_RAD = (2.0 * math.pi) / COUNTS_PER_REV


# 조인트명 ↔ Motorcortex actuatorControlLoop 번호 매핑
JOINT_LOOP_MAP = [
    ('HL_joint2_thigh_r', '01'),
    ('HL_joint3_thigh_p', '02'),
    ('HL_joint4_knee_p',  '03'),
    ('HL_joint5_ankle_p', '04'),
    ('HL_joint6_toe_p',   '05'),
]

BASE_PATH = 'root/AxesControl/actuatorControlLoops/actuatorControlLoop'


def make_param_path(loop_num: str, param_name: str) -> str:
    return f'{BASE_PATH}{loop_num}/{param_name}'


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('motorcortex_joint_state_bridge')

        # ROS2 파라미터 선언
        self.declare_parameter('mcx_url', 'wss://192.168.2.100')
        self.declare_parameter('mcx_cert', '')
        self.declare_parameter('mcx_login', 'admin')
        self.declare_parameter('mcx_password', 'vectioneer')
        self.declare_parameter('publish_rate', 50.0)       # Hz
        self.declare_parameter('use_actual', True)         # True=actual 위치, False=target 위치

        self.mcx_url = self.get_parameter('mcx_url').value
        self.mcx_cert = self.get_parameter('mcx_cert').value
        self.mcx_login = self.get_parameter('mcx_login').value
        self.mcx_password = self.get_parameter('mcx_password').value
        publish_rate = self.get_parameter('publish_rate').value
        self.use_actual = self.get_parameter('use_actual').value

        self._publisher = self.create_publisher(JointState, '/joint_states', 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_joint_states)

        # 조인트 위치 저장 (thread-safe)
        self._positions = {name: 0.0 for name, _ in JOINT_LOOP_MAP}
        self._lock = threading.Lock()

        # Motorcortex 연결 (별도 스레드)
        self._req = None
        self._sub = None
        self._mcx_subscriptions = []
        self._connect_thread = threading.Thread(target=self._connect_motorcortex, daemon=True)
        self._connect_thread.start()

        param_type = 'motorPositionActual' if self.use_actual else 'motorPositionTarget'
        self.get_logger().info(
            f'Joint State Bridge 시작: {self.mcx_url} | 파라미터: {param_type}'
        )

    def _connect_motorcortex(self):
        """Motorcortex 서버에 연결하고 조인트 파라미터를 구독합니다."""
        parameter_tree = motorcortex.ParameterTree()
        motorcortex_types = motorcortex.MessageTypes()

        cert = self.mcx_cert if self.mcx_cert else '/home/jsh/mcx-client-app-template/mcx.cert.crt'

        while rclpy.ok():
            try:
                self.get_logger().info(f'Motorcortex 연결 시도: {self.mcx_url}')
                self._req, self._sub = motorcortex.connect(
                    self.mcx_url,
                    motorcortex_types,
                    parameter_tree,
                    certificate=cert,
                    timeout_ms=5000,
                    login=self.mcx_login,
                    password=self.mcx_password,
                    reconnect=True
                )
                self.get_logger().info('Motorcortex 연결 성공')
                self._subscribe_joints()
                break
            except Exception as e:
                self.get_logger().error(f'Motorcortex 연결 실패: {e} — 5초 후 재시도')
                time.sleep(5)

    def _subscribe_joints(self):
        """조인트 위치 파라미터를 한 번에 구독합니다 (datalogger 방식)."""
        param_name = 'motorPositionActual' if self.use_actual else 'motorPositionTarget'
        joint_names = [name for name, _ in JOINT_LOOP_MAP]
        paths = [make_param_path(loop_num, param_name) for _, loop_num in JOINT_LOOP_MAP]

        try:
            subscription = self._sub.subscribe(paths, 'joint_state_group', frq_divider=1)

            def callback(msg):
                with self._lock:
                    for i, param in enumerate(msg):
                        if param.value:
                            self._positions[joint_names[i]] = float(param.value[0]) * ENCODER_TO_RAD

            subscription.notify(callback)
            self._mcx_subscriptions.append(subscription)

            for name, path in zip(joint_names, paths):
                self.get_logger().info(f'구독 성공: {name} ← {path}')
        except Exception as e:
            self.get_logger().error(f'구독 오류: {e}')

    def _publish_joint_states(self):
        """현재 조인트 위치를 /joint_states 토픽으로 퍼블리시합니다."""
        msg = JointState()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = ''

        with self._lock:
            for joint_name, _ in JOINT_LOOP_MAP:
                msg.name.append(joint_name)
                msg.position.append(self._positions[joint_name])
                msg.velocity.append(0.0)
                msg.effort.append(0.0)

        self._publisher.publish(msg)

    def destroy_node(self):
        for sub in self._mcx_subscriptions:
            try:
                sub.unsubscribe()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
