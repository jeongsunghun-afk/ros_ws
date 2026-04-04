"""
joint_state_bridge.py
ROS2 브릿지 전담 노드

역할:
  - MotorcortexInterface (MCX 통신) 초기화
  - MotionController     (운동 제어) 초기화
  - ROS2 퍼블리셔 / 구독자 관리
  - GRID jumpmode 인터럽트 → moveL 트리거
  - GRID controlMode 구독 → MotionController 모드 전환

토픽 구성:
  [publish]
    /joint_states      sensor_msgs/JointState       — RViz 시각화 (500 Hz)
    /low_state         sensor_msgs/JointState       — Unitree LowState 호환 관측값

  [subscribe]
    /low_cmd           sensor_msgs/JointState       — 외부 명령 (tracking 모드)
                         position → target q   [rad]
                         velocity → target dq  [rad/s]
                         effort   → tau_ff     [Nm]
    /rl_gains          std_msgs/Float64MultiArray   — Kp/Kd 게인
                         data = [kp0,kp1,kp2,kp3, kd0,kd1,kd2,kd3]

제어 모드 (GRID root/UserParameters/controlMode):
  0 = standby  — 위치 유지 (기본)
  1 = mpc      — 내부 MPC 계산 (추후 구현)
  2 = tracking — 외부 /low_cmd 추종
  그 외        → standby

Unitree 모터 인덱스 매핑 (이 로봇 기준):
  index 0 → HL_joint2_thigh_r  (ch0)
  index 1 → HL_joint3_thigh_p  (ch1)
  index 2 → HL_joint4_knee_p   (ch2)
  index 3 → HL_joint5_ankle_p  (ch3)
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
    Q_HOME_RAD,
    TRAJ_FILE_DEFAULT,
    TRAJ_DT_DEFAULT,
)

PUBLISH_HZ = 50    # ROS2 publish 주기


class JointStateBridge(Node):

    def __init__(self):
        super().__init__('motorcortex_joint_state_bridge')

        # ── ROS2 파라미터 ──────────────────────────────────────────────────
        self.declare_parameter('mcx_url',      'wss://192.168.2.100')
        self.declare_parameter('mcx_cert',     '')
        self.declare_parameter('mcx_login',    'admin')
        self.declare_parameter('mcx_password', 'vectioneer')
        self.declare_parameter('mcx_mode',     'sim')   # 'sim' | 'prod'
        self.declare_parameter('traj_file',    TRAJ_FILE_DEFAULT)
        self.declare_parameter('traj_dt',      TRAJ_DT_DEFAULT)

        mcx_url      = self.get_parameter('mcx_url').value
        mcx_cert     = self.get_parameter('mcx_cert').value
        mcx_login    = self.get_parameter('mcx_login').value
        mcx_password = self.get_parameter('mcx_password').value
        mcx_mode     = self.get_parameter('mcx_mode').value
        traj_file    = self.get_parameter('traj_file').value
        traj_dt      = self.get_parameter('traj_dt').value

        # ── 모듈 초기화 ───────────────────────────────────────────────────
        self._mcx  = MotorcortexInterface(mcx_url, mcx_cert, mcx_login, mcx_password, mode=mcx_mode)
        self._ctrl = MotionController(self._mcx, traj_file, traj_dt)

        # ── ROS2 퍼블리셔 ─────────────────────────────────────────────────
        self._pub_joint  = self.create_publisher(JointState, '/joint_states', 10)
        self._pub_state  = self.create_publisher(JointState, '/low_state',    10)

        # ── ROS2 구독자 (Unitree RL 인터페이스) ───────────────────────────
        self._sub_cmd  = self.create_subscription(
            JointState, '/low_cmd', self._on_low_cmd, 10
        )
        self._sub_gain = self.create_subscription(
            Float64MultiArray, '/rl_gains', self._on_rl_gains, 10
        )
        self._rl_kp = [20.0] * N_AXES
        self._rl_kd = [0.5]  * N_AXES

        # ── 타이머 ────────────────────────────────────────────────────────
        self._pub_timer     = self.create_timer(1.0 / PUBLISH_HZ, self._publish)   # 50 Hz
        self._monitor_timer = self.create_timer(0.5, self._log_monitor)            # 2 Hz

        # ── 공유 상태 (lock 보호) ─────────────────────────────────────────
        self._lock = threading.Lock()
        # RViz 초기값: Q_HOME 표시
        self._positions_for_rviz = {
            JOINT_LOOP_MAP[i][0]: (Q_HOME_RAD[i] if i < N_AXES else 0.0)
            for i in range(len(JOINT_LOOP_MAP))
        }

        # ── jump / home 이벤트 ────────────────────────────────────────────
        self._jump_event = threading.Event()
        self._home_event = threading.Event()

        # ── publish rate 측정 ─────────────────────────────────────────────
        self._pub_count     = 0
        self._pub_rate_last = time.monotonic()

        # ── 연결 스레드 ────────────────────────────────────────────────────
        self._connect_thread = threading.Thread(
            target=self._connect_and_run, daemon=True
        )
        self._connect_thread.start()

        self.get_logger().info(
            f'JointStateBridge 시작 | URL: {mcx_url} '
            f'| 궤적: {traj_file} | DT: {traj_dt*1000:.1f}ms | ROS2: {PUBLISH_HZ}Hz'
        )

    # ── MCX 연결 및 메인 루프 ────────────────────────────────────────────────
    def _connect_and_run(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f'Motorcortex 연결 시도...')
                self._mcx.connect()
                self.get_logger().info('연결 성공')

                self._mcx.disable_drives()
                self.get_logger().info('disableDrive 완료')

                if not self._mcx.engage():
                    self.get_logger().error('Engage 실패 — 재시도')
                    time.sleep(5)
                    continue
                self.get_logger().info('Engaged')

                self._mcx.set_jog_mode()
                self.get_logger().info('JogMode 활성')

                self._mcx.subscribe_positions()
                self._mcx.subscribe_jumpmode(self._on_jump)
                self._mcx.subscribe_homemode(self._on_home)
                self._mcx.subscribe_control_mode(self._on_control_mode)
                self._mcx.subscribe_position_mode(self._on_position_mode)

                # if not self._ctrl.home():
                #     self.get_logger().error('Homing 실패 — 재시도')
                #     time.sleep(5)
                #     continue
                # self.get_logger().info(
                #     'Homing 완료: '
                #     + ', '.join(f'j{i+1}={math.degrees(Q_HOME_RAD[i]):.1f}°'
                #                 for i in range(N_AXES))
                # )

                # 현재 실제 위치 읽어 초기 위치로 설정
                actual = self._mcx.get_actual_positions_snapshot()
                self._ctrl.set_initial_positions(actual)
                self.get_logger().info(
                    '초기 실제 위치 로드: '
                    + ', '.join(f'j{i+1}={math.degrees(actual[i]):.1f}°'
                                for i in range(N_AXES))
                )

                # 제어 루프 활성화
                self._ctrl._ctrl_ready = True

                waypoints = self._ctrl.load_trajectory()
                self.get_logger().info(
                    f'궤적 로드: {len(waypoints)} waypoints '
                    f'({len(waypoints)*self._ctrl.traj_dt:.2f}s) — GRID jumpmode=1 대기'
                )

                # ── jump / home 대기 루프 ────────────────────────────────
                while rclpy.ok():
                    jump_triggered = self._jump_event.wait(timeout=0.1)
                    if not rclpy.ok():
                        break

                    if jump_triggered:
                        self._jump_event.clear()
                        self._mcx.reset_jumpmode()
                        self.get_logger().info('점프 궤적 실행')
                        self._ctrl.move_l(
                            waypoints,
                            log_cb=lambda s: self.get_logger().info(s)
                        )
                        self.get_logger().info('moveL 완료. 다음 점프 대기 중...')

                    if self._home_event.is_set():
                        self._home_event.clear()
                        self.get_logger().info('홈 복귀 실행')
                        self._ctrl.move_to_home(
                            log_cb=lambda s: self.get_logger().info(s)
                        )
                        self._mcx.reset_homemode()
                        self.get_logger().info('홈 복귀 완료. homemode → 0')
                break

            except Exception as e:
                self.get_logger().error(f'연결 실패: {e} — 5초 후 재시도')
                time.sleep(5)

    # ── jumpmode 인터럽트 콜백 ────────────────────────────────────────────────
    def _on_jump(self):
        self.get_logger().info('jumpmode=1 감지')
        self._jump_event.set()

    # ── homemode 인터럽트 콜백 ────────────────────────────────────────────────
    def _on_home(self):
        self.get_logger().info('homemode=1 감지')
        self._home_event.set()

    # ── controlMode 변경 콜백 ─────────────────────────────────────────────────
    def _on_control_mode(self, mode: int):
        mode_names = {0: 'standby', 1: 'mpc', 2: 'tracking'}
        self.get_logger().info(f'controlMode → {mode} ({mode_names.get(mode, "standby")})')
        self._ctrl.set_control_mode(mode)

    # ── positionMode 변경 콜백 ────────────────────────────────────────────────
    def _on_position_mode(self, mode: int):
        mode_names = {0: 'position', 1: 'force'}
        self.get_logger().info(f'positionMode → {mode} ({mode_names.get(mode, "position")})')
        self._ctrl.set_position_mode(mode)

    # ── /low_cmd 구독 콜백 (Unitree LowCmd 호환) ─────────────────────────────
    def _on_low_cmd(self, msg: JointState):
        """
        Unitree LowCmd 호환 RL 명령 수신.
          msg.position → target joint positions [rad]   (필수, 4개)
          msg.velocity → target joint velocities [rad/s] (선택)
          msg.effort   → feedforward torques [Nm]        (선택)
        """
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
        """
        data = [kp0, kp1, kp2, kp3, kd0, kd1, kd2, kd3]
        """
        if len(msg.data) < N_AXES * 2:
            return
        with self._lock:
            self._rl_kp = list(msg.data[:N_AXES])
            self._rl_kd = list(msg.data[N_AXES:N_AXES*2])

    # ── 500 Hz publish ────────────────────────────────────────────────────────
    def _publish(self):
        ns  = time.time_ns()
        now = TimeMsg(sec=ns // 10**9, nanosec=ns % 10**9)

        actual_pos  = self._mcx.actual_positions    # len = 5 (전체 조인트)
        actual_vel  = self._mcx.actual_velocities
        target_pos  = self._ctrl.last_cmd_positions  # len = 4 (N_AXES)
        in_movel    = self._ctrl.in_movel

        # ── /joint_states (RViz) ─────────────────────────────────────────
        js = JointState()
        js.header.stamp    = now
        js.header.frame_id = ''
        for i, (name, _) in enumerate(JOINT_LOOP_MAP):
            # moveL 중: target / 대기 중: actual
            pos = (target_pos[i] if (in_movel and i < N_AXES)
                   else actual_pos[i])
            js.name.append(name)
            js.position.append(pos)
            js.velocity.append(actual_vel[i])
            js.effort.append(0.0)
        self._pub_joint.publish(js)
        self._pub_count += 1

        # ── /low_state (Unitree LowState 호환 RL 관측값) ─────────────────
        # position → q     [rad]    (실제 encoder 피드백)
        # velocity → dq    [rad/s]  (미지원 → 0)
        # effort   → tau   [Nm]     (미지원 → 0)
        ls = JointState()
        ls.header.stamp    = now
        ls.header.frame_id = ''
        for i in range(N_AXES):
            ls.name.append(JOINT_LOOP_MAP[i][0])
            ls.position.append(actual_pos[i])
            ls.velocity.append(actual_vel[i])
            ls.effort.append(0.0)
        self._pub_state.publish(ls)

    # ── 모니터 로그 (0.5 s) ──────────────────────────────────────────────────
    def _log_monitor(self):
        if not self._mcx.is_connected:
            return

        # publish rate 계산
        now = time.monotonic()
        elapsed = now - self._pub_rate_last
        rate = self._pub_count / elapsed if elapsed > 0 else 0.0
        self._pub_count = 0
        self._pub_rate_last = now

        try:
            rows = self._mcx.get_monitor_snapshot()
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
