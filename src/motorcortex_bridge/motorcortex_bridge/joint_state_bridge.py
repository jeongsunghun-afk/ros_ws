"""
joint_state_bridge.py
ROS2 브릿지 노드

역할:
  - MotorcortexInterface (MCX 통신) 초기화
  - MotionController     (운동 제어) 초기화
  - ROS2 퍼블리셔 / 구독자 관리

토픽 구성:
  [publish]
    /joint_states      sensor_msgs/JointState       — RViz 시각화 (50Hz)
    /low_state         sensor_msgs/JointState       — Unitree LowState 호환 관측값

  [subscribe]
    /low_cmd           sensor_msgs/JointState       — 외부 명령 (tracking 모드)
                         position → target q   [rad]
                         velocity → target dq  [rad/s]
                         effort   → tau_ff     [Nm]
    /rl_gains          std_msgs/Float64MultiArray   — Kp/Kd 게인
                         data = [kp0,kp1,kp2,kp3, kd0,kd1,kd2,kd3]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time as TimeMsg

import threading
import time
import math

from motorcortex_bridge.motorcortex_interface import (
    MotorcortexInterface,
    N_AXES,
    JOINT_LOOP_MAP,
)
from motorcortex_bridge.motion_controller import (
    MotionController,
    TRAJ_FILE_DEFAULT,
    TRAJ_DT_DEFAULT,
)

PUBLISH_HZ = 50


class JointStateBridge(Node):

    def __init__(self):
        super().__init__('motorcortex_joint_state_bridge')

        # ── ROS2 파라미터 ──────────────────────────────────────────────────
        self.declare_parameter('mcx_url',      'wss://192.168.2.100')
        self.declare_parameter('mcx_cert',     '')
        self.declare_parameter('mcx_login',    'admin')
        self.declare_parameter('mcx_password', 'vectioneer')
        self.declare_parameter('traj_file',    TRAJ_FILE_DEFAULT)
        self.declare_parameter('traj_dt',      TRAJ_DT_DEFAULT)

        mcx_url      = self.get_parameter('mcx_url').value
        mcx_cert     = self.get_parameter('mcx_cert').value
        mcx_login    = self.get_parameter('mcx_login').value
        mcx_password = self.get_parameter('mcx_password').value
        traj_file    = self.get_parameter('traj_file').value
        traj_dt      = self.get_parameter('traj_dt').value

        # ── 모듈 초기화 ───────────────────────────────────────────────────
        self._mcx = MotorcortexInterface(mcx_url, mcx_cert, mcx_login, mcx_password)
        self._ctrl = MotionController(self._mcx, traj_file, traj_dt)

        # ── ROS2 퍼블리셔 ─────────────────────────────────────────────────
        self._pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self._pub_state = self.create_publisher(JointState, '/low_state',    10)

        # ── ROS2 구독자 ───────────────────────────────────────────────────
        self._sub_cmd  = self.create_subscription(
            JointState, '/low_cmd', self._on_low_cmd, 10
        )
        self._sub_gain = self.create_subscription(
            Float64MultiArray, '/rl_gains', self._on_rl_gains, 10
        )
        self._rl_kp = [20.0] * N_AXES
        self._rl_kd = [0.5]  * N_AXES

        # ── 타이머 ────────────────────────────────────────────────────────
        self._pub_timer     = self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self._monitor_timer = self.create_timer(0.5, self._log_monitor)

        # ── 공유 상태 ─────────────────────────────────────────────────────
        self._lock = threading.Lock()

        # ── publish rate 측정 ─────────────────────────────────────────────
        self._pub_count     = 0
        self._pub_rate_last = time.monotonic()

        # ── 연결 스레드 ────────────────────────────────────────────────────
        self._connect_thread = threading.Thread(
            target=self._connect_and_run, daemon=True
        )
        self._connect_thread.start()

        self.get_logger().info(f'JointStateBridge 시작 | URL: {mcx_url} | ROS2: {PUBLISH_HZ}Hz')

    # ── MCX 연결 ────────────────────────────────────────────────────────────
    def _connect_and_run(self):
        while rclpy.ok():
            try:
                self.get_logger().info('Motorcortex 연결 시도...')
                self._mcx.connect()
                self.get_logger().info('연결 성공')

                if not self._mcx.engage():
                    self.get_logger().error('Engage 실패 — 재시도')
                    time.sleep(5)
                    continue
                self.get_logger().info('Engaged')

                # self._mcx.set_jog_mode()  # MCX 시작 시 자동 적용 (services_config.json)
                # self.get_logger().info('JogMode 활성')

                self._mcx.subscribe_positions()
                self._mcx.subscribe_control_mode(self._on_control_mode)

                actual = self._mcx.get_actual_positions_snapshot()
                self._ctrl.set_initial_positions(actual)
                self.get_logger().info(
                    '초기 위치 로드: '
                    + ', '.join(f'j{i+1}={math.degrees(actual[i]):.1f}°'
                                for i in range(N_AXES))
                )

                self._ctrl._ctrl_ready = True

                n = self._ctrl.start(log_cb=lambda s: self.get_logger().info(s))
                self.get_logger().info(
                    f'궤적 로드: {n} waypoints — jumpmode/homemode 대기 중'
                )
                break

            except Exception as e:
                self.get_logger().error(f'연결 실패: {e} — 5초 후 재시도')
                time.sleep(5)

    # ── controlMode 변경 콜백 ─────────────────────────────────────────────────
    def _on_control_mode(self, mode: int):
        mode_names = {0: 'standby', 1: 'mpc', 2: 'tracking'}
        self.get_logger().info(f'controlMode → {mode} ({mode_names.get(mode, "standby")})')
        self._ctrl.set_control_mode(mode)

    # ── /low_cmd 구독 콜백 ────────────────────────────────────────────────────
    def _on_low_cmd(self, msg: JointState):
        if len(msg.position) < N_AXES:
            self.get_logger().warn(
                f'/low_cmd: position 길이 부족 ({len(msg.position)} < {N_AXES})'
            )
            return

        q   = list(msg.position[:N_AXES])
        dq  = list(msg.velocity[:N_AXES]) if len(msg.velocity) >= N_AXES else None
        tau = list(msg.effort[:N_AXES])   if len(msg.effort)   >= N_AXES else None

        with self._lock:
            kp = list(self._rl_kp)
            kd = list(self._rl_kd)

        self._ctrl.set_command(q=q, dq=dq, kp=kp, kd=kd, tau=tau)

    # ── /rl_gains 구독 콜백 ──────────────────────────────────────────────────
    def _on_rl_gains(self, msg: Float64MultiArray):
        if len(msg.data) < N_AXES * 2:
            return
        with self._lock:
            self._rl_kp = list(msg.data[:N_AXES])
            self._rl_kd = list(msg.data[N_AXES:N_AXES*2])

    # ── 50Hz publish ─────────────────────────────────────────────────────────
    def _publish(self):
        ns  = time.time_ns()
        now = TimeMsg(sec=ns // 10**9, nanosec=ns % 10**9)

        actual_pos = self._mcx.actual_positions

        # ── /joint_states (RViz) ─────────────────────────────────────────
        js = JointState()
        js.header.stamp    = now
        js.header.frame_id = ''
        for i, (name, _) in enumerate(JOINT_LOOP_MAP):
            js.name.append(name)
            js.position.append(actual_pos[i])
            js.velocity.append(0.0)
            js.effort.append(0.0)
        self._pub_joint.publish(js)
        self._pub_count += 1

        # ── /low_state (Unitree LowState 호환 RL 관측값) ─────────────────
        ls = JointState()
        ls.header.stamp    = now
        ls.header.frame_id = ''
        for i in range(N_AXES):
            ls.name.append(JOINT_LOOP_MAP[i][0])
            ls.position.append(actual_pos[i])
            ls.velocity.append(0.0)
            ls.effort.append(0.0)
        self._pub_state.publish(ls)

    # ── 모니터 로그 (0.5s) ───────────────────────────────────────────────────
    def _log_monitor(self):
        if not self._mcx.is_connected:
            return

        now = time.monotonic()
        elapsed = now - self._pub_rate_last
        rate = self._pub_count / elapsed if elapsed > 0 else 0.0
        self._pub_count = 0
        self._pub_rate_last = now

        try:
            rows = self._ctrl.get_monitor_snapshot()
        except Exception:
            return

        ch_info = '  |  '.join(
            f'ch{i} tgt={math.degrees(r[0]):+7.2f}° act={math.degrees(r[1]):+7.2f}°'
            for i, r in enumerate(rows)
        )
        self.get_logger().info(f'[{rate:.1f}Hz]  {ch_info}')

    # ── 종료 ─────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._mcx.disconnect()
        super().destroy_node()


def main(args=None):
    import os, signal
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        os.kill(os.getpid(), signal.SIGKILL)


if __name__ == '__main__':
    main()
