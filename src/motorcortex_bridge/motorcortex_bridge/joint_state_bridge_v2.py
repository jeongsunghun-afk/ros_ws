"""
joint_state_bridge_v2.py
Motorcortex → ROS2 Joint State Bridge + CSP 1축 궤적 제어

[v1 기능 유지]
  - actuatorControlLoop01~05 의 motorPositionActual 구독
  - /joint_states 퍼블리시 → Rviz 시각화

[v2 추가]
  - 1축 CSP 위치 제어: root/MachineControl/jointPositionsTarget (배열, rad)
  - 궤적: 초기 위치 중심 ± 10° 사인파, 주기 4 s
  - /target_joint_states 퍼블리시 → Rviz 목표 vs 실제 비교
  - 0.5 s 주기 Target/Actual 모니터 로그

조인트 매핑:
  index 0 = HL_joint2_thigh_r  ← actuatorControlLoop01  ← 궤적 제어 축
  index 1 = HL_joint3_thigh_p  ← actuatorControlLoop02
  index 2 = HL_joint4_knee_p   ← actuatorControlLoop03
  index 3 = HL_joint5_ankle_p  ← actuatorControlLoop04
  index 4 = HL_joint6_toe_p    ← actuatorControlLoop05
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import motorcortex
import threading
import time
import math

# ── 엔코더 상수 (motorPositionActual 읽기 전용) ───────────────────────────────
COUNTS_PER_REV = 4096
ENCODER_TO_RAD = (2.0 * math.pi) / COUNTS_PER_REV

# ── 궤적 파라미터 ─────────────────────────────────────────────────────────────
TRAJ_AMPLITUDE_DEG = 10.0
TRAJ_AMPLITUDE_RAD = math.radians(TRAJ_AMPLITUDE_DEG)
TRAJ_PERIOD_SEC    = 4.0
TRAJ_RATE_HZ       = 100.0

# ── Motorcortex 파라미터 경로 ─────────────────────────────────────────────────
JOINT_LOOP_MAP = [
    ('HL_joint2_thigh_r', '01'),
    ('HL_joint3_thigh_p', '02'),
    ('HL_joint4_knee_p',  '03'),
    ('HL_joint5_ankle_p', '04'),
    ('HL_joint6_toe_p',   '05'),
]
BASE_PATH          = 'root/AxesControl/actuatorControlLoops/actuatorControlLoop'
JOINT_TARGET_PATH  = 'root/MachineControl/jointPositionsTarget'   # 배열, rad

def _param_path(loop_num: str, param_name: str) -> str:
    return f'{BASE_PATH}{loop_num}/{param_name}'


class JointStateBridgeV2(Node):
    def __init__(self):
        super().__init__('motorcortex_joint_state_bridge_v2')

        # ── ROS2 파라미터 ─────────────────────────────────────────────────────
        self.declare_parameter('mcx_url',      'wss://192.168.2.100')
        self.declare_parameter('mcx_cert',     '')
        self.declare_parameter('mcx_login',    'admin')
        self.declare_parameter('mcx_password', 'vectioneer')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('enable_csp',   True)

        self.mcx_url      = self.get_parameter('mcx_url').value
        self.mcx_cert     = self.get_parameter('mcx_cert').value
        self.mcx_login    = self.get_parameter('mcx_login').value
        self.mcx_password = self.get_parameter('mcx_password').value
        publish_rate      = self.get_parameter('publish_rate').value
        self.enable_csp   = self.get_parameter('enable_csp').value

        # ── ROS2 퍼블리셔 & 타이머 ───────────────────────────────────────────
        self._pub_actual  = self.create_publisher(JointState, '/joint_states',        10)
        self._pub_target  = self.create_publisher(JointState, '/target_joint_states', 10)
        self._pub_timer    = self.create_timer(1.0 / publish_rate, self._publish_joint_states)
        self._traj_timer   = self.create_timer(1.0 / TRAJ_RATE_HZ, self._send_csp_setpoint)
        self._verify_timer = self.create_timer(0.5,                 self._log_target_verify)

        # ── 상태 변수 ─────────────────────────────────────────────────────────
        self._positions_actual   = {name: 0.0 for name, _ in JOINT_LOOP_MAP}
        self._positions_target   = {name: 0.0 for name, _ in JOINT_LOOP_MAP}
        self._lock               = threading.Lock()

        self._joint_target_array = None   # jointPositionsTarget 배열 (rad)
        self._axis1_center_rad   = 0.0
        self._traj_start_time    = None
        self._csp_ready          = False

        # ── Motorcortex 연결 ──────────────────────────────────────────────────
        self._req  = None
        self._sub  = None
        self._mcx_subscriptions = []
        self._connect_thread = threading.Thread(
            target=self._connect_motorcortex, daemon=True
        )
        self._connect_thread.start()

        self.get_logger().info(
            f'JointStateBridgeV2 시작 | URL: {self.mcx_url} '
            f'| CSP: {"활성" if self.enable_csp else "비활성"} '
            f'| 궤적: ±{TRAJ_AMPLITUDE_DEG}° / {TRAJ_PERIOD_SEC}s'
        )

    # ── Motorcortex 연결 ──────────────────────────────────────────────────────
    def _connect_motorcortex(self):
        parameter_tree    = motorcortex.ParameterTree()
        motorcortex_types = motorcortex.MessageTypes()
        cert = self.mcx_cert or '/home/jsh/mcx-client-app-template/mcx.cert.crt'

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

    # ── 실제 관절 위치 구독 ───────────────────────────────────────────────────
    def _subscribe_joints(self):
        joint_names = [name for name, _ in JOINT_LOOP_MAP]
        paths = [
            _param_path(loop_num, 'motorPositionActual')
            for _, loop_num in JOINT_LOOP_MAP
        ]

        try:
            subscription = self._sub.subscribe(paths, 'joint_state_group', frq_divider=1)

            def callback(msg):
                with self._lock:
                    for i, param in enumerate(msg):
                        if param.value:
                            self._positions_actual[joint_names[i]] = (
                                float(param.value[0]) * ENCODER_TO_RAD
                            )

                    # 초기화: jointPositionsTarget 배열에서 중심 위치 캡처
                    if not self._csp_ready and self._req is not None:
                        try:
                            reply = self._req.getParameter(JOINT_TARGET_PATH).get()
                            if reply and reply.value and len(reply.value) >= 1:
                                self._joint_target_array = list(reply.value)
                                self._axis1_center_rad   = self._joint_target_array[0]
                                self._traj_start_time    = time.monotonic()
                                self._csp_ready          = True
                                self.get_logger().info(
                                    f'1축 초기 위치 캡처 (jointPositionsTarget[0]): '
                                    f'{math.degrees(self._axis1_center_rad):.3f} rad '
                                    f'({math.degrees(self._axis1_center_rad):.2f}°) '
                                    f'| 배열 크기: {len(self._joint_target_array)}'
                                )
                        except Exception as e:
                            self.get_logger().warn(f'초기화 실패: {e}')

            subscription.notify(callback)
            self._mcx_subscriptions.append(subscription)

            for name, path in zip(joint_names, paths):
                self.get_logger().info(f'구독: {name} ← {path}')

        except Exception as e:
            self.get_logger().error(f'구독 오류: {e}')

    # ── CSP 위치 지령 전송 (100 Hz) ───────────────────────────────────────────
    def _send_csp_setpoint(self):
        if not self.enable_csp or self._req is None or not self._csp_ready:
            return
        if self._joint_target_array is None:
            return

        elapsed    = time.monotonic() - self._traj_start_time
        target_rad = (
            self._axis1_center_rad
            + TRAJ_AMPLITUDE_RAD * math.sin(2.0 * math.pi * elapsed / TRAJ_PERIOD_SEC)
        )

        # Rviz 목표 위치 저장
        with self._lock:
            self._positions_target['HL_joint2_thigh_r'] = target_rad

        # 배열 전체 복사 후 index 0 (1축)만 갱신하여 전송
        targets    = list(self._joint_target_array)
        targets[0] = target_rad

        try:
            self._req.setParameter(JOINT_TARGET_PATH, targets)
        except Exception as e:
            self.get_logger().warn(f'CSP 지령 오류: {e}', throttle_duration_sec=5.0)

    # ── Target / Actual 모니터 로그 (0.5 s) ──────────────────────────────────
    def _log_target_verify(self):
        if self._req is None or not self._csp_ready:
            return
        try:
            tgt   = self._req.getParameter(JOINT_TARGET_PATH).get()
            act   = self._req.getParameter(_param_path('01', 'motorPositionActual')).get()

            tgt_rad = float(tgt.value[0]) if tgt and tgt.value else float('nan')
            act_cnt = float(act.value[0]) if act and act.value else float('nan')
            act_rad = act_cnt * ENCODER_TO_RAD

            self.get_logger().info(
                f'[모니터] '
                f'Target={math.degrees(tgt_rad):+.3f}°  '
                f'Actual={math.degrees(act_rad):+.3f}°  '
                f'Δ={math.degrees(tgt_rad - act_rad):+.3f}°'
            )
        except Exception as e:
            self.get_logger().warn(f'모니터 오류: {e}', throttle_duration_sec=5.0)

    # ── /joint_states 퍼블리시 ────────────────────────────────────────────────
    def _publish_joint_states(self):
        now = self.get_clock().now().to_msg()

        actual_msg = JointState()
        actual_msg.header.stamp    = now
        actual_msg.header.frame_id = ''

        target_msg = JointState()
        target_msg.header.stamp    = now
        target_msg.header.frame_id = ''

        with self._lock:
            for joint_name, _ in JOINT_LOOP_MAP:
                actual_msg.name.append(joint_name)
                actual_msg.position.append(self._positions_actual[joint_name])
                actual_msg.velocity.append(0.0)
                actual_msg.effort.append(0.0)

                target_msg.name.append(joint_name)
                if joint_name == 'HL_joint2_thigh_r':
                    target_msg.position.append(self._positions_target[joint_name])
                else:
                    target_msg.position.append(self._positions_actual[joint_name])
                target_msg.velocity.append(0.0)
                target_msg.effort.append(0.0)

        self._pub_actual.publish(actual_msg)
        self._pub_target.publish(target_msg)

    # ── 종료 ─────────────────────────────────────────────────────────────────
    def destroy_node(self):
        for sub in self._mcx_subscriptions:
            try:
                sub.unsubscribe()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridgeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
