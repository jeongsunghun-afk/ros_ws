"""
joint_state_bridge_v2.py
Motorcortex → ROS2 Joint State Bridge + CSP 1축 궤적 제어

[v1 기능 유지]
  - actuatorControlLoop01~05 의 motorPositionActual 구독
  - /joint_states 퍼블리시 → Rviz 시각화

[v2 추가]
  - Engage 시퀀스 (mcx-client-app_v3.py 참조): 연결 후 상태머신 ENGAGED 진입
  - Homing 시퀀스 (mcx-client-app_v3.py 참조): 1축을 0°로 램프 이동 후 궤적 시작
  - 1축 CSP 위치 제어: root/MachineControl/jointPositionsTarget (배열, rad)
  - 궤적: 0° 중심 ± 10° 사인파, 주기 4 s
  - /target_joint_states 퍼블리시 → Rviz 목표 vs 실제 비교
  - 0.5 s 주기 Target/Actual 모니터 로그

시작 시퀀스 (mcx-client-app_v3.py 참조):
  1. Motorcortex 연결
  2. Engage: stateCommand=2 → state==4(ENGAGED) 대기
  3. Homing: motorPositionActual(enc→rad) 읽기 → jointPositionsTarget[0]을 0°로 램프
  4. CSP 궤적 시작: 0° 중심 ±10° 사인파 → jointPositionsTarget[0]

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
from builtin_interfaces.msg import Time as TimeMsg

import motorcortex
import threading
import time
import math

# ── 엔코더 상수 (motorPositionActual 읽기 전용) ───────────────────────────────
COUNTS_PER_REV = 4096
ENCODER_TO_RAD = (2.0 * math.pi) / COUNTS_PER_REV

# ── 상태머신 상수 (mcx-client-app_v3.py 참조) ────────────────────────────────
STATE_CMD_PATH  = 'root/Logic/stateCommand'
STATE_PATH      = 'root/Logic/state'
ENGAGE_CMD      = 2       # GOTO_ENGAGED_E
ENGAGED_STATE   = 4       # ENGAGED_S
ENGAGE_TIMEOUT  = 10.0

# ── 홈 복귀 파라미터 (mcx-client-app_v3.py 참조) ─────────────────────────────
HOMING_THRESHOLD_RAD = math.radians(0.1)   # 0.1° 이내면 홈으로 간주
HOMING_VEL_DEG_S     = 30.0               # 홈 복귀 속도 (°/s)
HOMING_VEL_RAD_S     = math.radians(HOMING_VEL_DEG_S)
HOMING_DT            = 0.001              # 1 ms step

# ── 궤적 파라미터 ─────────────────────────────────────────────────────────────
TRAJ_AMPLITUDE_DEG = 10.0
TRAJ_AMPLITUDE_RAD = math.radians(TRAJ_AMPLITUDE_DEG)
TRAJ_PERIOD_SEC    = 4.0
CYCLE_TIME         = 0.001    # 1 ms → 1 kHz (v3 CYCLE_TIME 동일)

# CSP 루프 안에서 퍼블리시 주기 (매 N회 → 1kHz/5 = 200 Hz)
PUB_DIVIDER        = 5

# ── Motorcortex 파라미터 경로 ─────────────────────────────────────────────────
JOINT_LOOP_MAP = [
    ('HL_joint2_thigh_r', '01'),
    ('HL_joint3_thigh_p', '02'),
    ('HL_joint4_knee_p',  '03'),
    ('HL_joint5_ankle_p', '04'),
    ('HL_joint6_toe_p',   '05'),
]
BASE_PATH          = 'root/AxesControl/actuatorControlLoops/actuatorControlLoop'
# [읽기] EtherCAT(0x6064) → motorPositionActual(raw enc counts) — v3 ACTUAL_PATH 동일
ACTUAL_PATH        = f'{BASE_PATH}01/motorPositionActual'
# [쓰기] mcx-client-app_v3.py 동일 경로: additive offset(rad) → jointPositionsTarget → EtherCAT(0x607A)
JOINT_TARGET_PATH  = 'root/MachineControl/hostInJointAdditivePosition1'

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
        # _publish_joint_states는 CSP 루프에서 직접 호출 (타이머 제거)
        self._verify_timer = self.create_timer(0.5,                self._log_target_verify)

        # ── 상태 변수 ─────────────────────────────────────────────────────────
        self._positions_actual   = {name: 0.0 for name, _ in JOINT_LOOP_MAP}
        self._positions_target   = {name: 0.0 for name, _ in JOINT_LOOP_MAP}
        self._lock               = threading.Lock()

        self._home_offset      = 0.0    # homing 후 0점 기준 additive offset (v3 참조)
        self._traj_start_time  = None
        self._csp_ready        = False
        self._loop_count       = 0      # 전체 루프 카운터
        self._last_iter        = 0.0    # 직전 루프 시각 (v3 참조)
        self._hz_last_log_time = 0.0    # 1초 Hz 로그 기준 시각
        self._hz_period_count  = 0      # 1초 내 루프 횟수
        self._pub_ctr          = 0      # CSP 루프 내 퍼블리시 카운터

        # ── Motorcortex 연결 ──────────────────────────────────────────────────
        self._req  = None
        self._sub  = None
        # CSP 전송 전용 스레드 (v3 iterate() 방식: time.sleep(CYCLE_TIME) → 1 kHz 목표)
        self._csp_thread = threading.Thread(target=self._csp_loop, daemon=True)
        self._csp_thread.start()
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

                # ── 1단계: Engage (mcx-client-app_v3.py _engage() 참조) ──────
                if not self._engage():
                    self.get_logger().error('Engage 실패 — 5초 후 재시도')
                    time.sleep(5)
                    continue

                # ── 1.5단계: JogMode/PauseMode 설정 (mcx-client-app_v3.py startOp() 참조)
                try:
                    self._req.setParameter('root/MachineControl/gotoJogMode', True).get()
                    self._req.setParameter('root/MachineControl/gotoPauseMode', False).get()
                    self.get_logger().info('JogMode 활성 / PauseMode 해제 완료.')
                except Exception as e:
                    self.get_logger().warn(f'모드 설정 실패: {e}')

                # ── 2단계: Homing (mcx-client-app_v3.py _homing() 참조) ─────
                if self.enable_csp and not self._homing_axis1():
                    self.get_logger().error('Homing 실패 — 5초 후 재시도')
                    time.sleep(5)
                    continue

                # ── 3단계: CSP 궤적 준비 ─────────────────────────────────────
                # home_offset은 _homing_axis1()에서 설정됨
                # (homing이 motorPositionActual=0°를 달성한 최종 additive 값)
                self._traj_start_time = time.monotonic()
                self._csp_ready       = True
                self.get_logger().info(
                    f'CSP 궤적 시작 — 0° 중심 ±{TRAJ_AMPLITUDE_DEG}° / {TRAJ_PERIOD_SEC}s'
                )

                self._subscribe_joints()
                break
            except Exception as e:
                self.get_logger().error(f'Motorcortex 연결 실패: {e} — 5초 후 재시도')
                time.sleep(5)

    # ── Engage 시퀀스 (mcx-client-app_v3.py _engage() 참조) ──────────────────
    def _engage(self) -> bool:
        try:
            state = self._req.getParameter(STATE_PATH).get()
            if state and state.value and state.value[0] == ENGAGED_STATE:
                self.get_logger().info('이미 Engaged 상태.')
                return True
        except Exception:
            pass

        self.get_logger().info('Engage 명령 전송...')
        self._req.setParameter(STATE_CMD_PATH, [ENGAGE_CMD]).get()

        deadline = time.time() + ENGAGE_TIMEOUT
        while time.time() < deadline:
            try:
                state = self._req.getParameter(STATE_PATH).get()
                if state and state.value and state.value[0] == ENGAGED_STATE:
                    self.get_logger().info('Engage 완료.')
                    return True
            except Exception:
                pass
            time.sleep(0.1)

        self.get_logger().error('Engage 타임아웃!')
        return False

    # ── Homing 시퀀스 (클로즈드루프: motorPositionActual → 0) ────────────────
    def _homing_axis1(self) -> bool:
        """ACTUAL_PATH를 매 스텝마다 읽어 motorPositionActual이 0°가 될 때까지 additive 조정."""
        def _read_actual():
            reply = self._req.getParameter(ACTUAL_PATH).get()
            if not reply or not reply.value:
                return None
            return float(reply.value[0]) * ENCODER_TO_RAD

        current_rad = _read_actual()
        if current_rad is None:
            self.get_logger().warn('1축 위치 읽기 실패, Homing 생략.')
            return True

        self.get_logger().info(f'1축 현재 위치: {math.degrees(current_rad):.2f}°')

        if abs(current_rad) <= HOMING_THRESHOLD_RAD:
            self.get_logger().info('홈 위치 이내, Homing 생략.')
            self._req.setParameter(JOINT_TARGET_PATH, [0.0]).get()
            self._home_offset = 0.0
            return True

        self.get_logger().info(
            f'Homing 시작: {math.degrees(current_rad):.2f}° → 0° ({HOMING_VEL_DEG_S}°/s)'
        )

        # [클로즈드루프] 매 스텝 ACTUAL_PATH를 읽어 motorPositionActual → 0°
        offset = 0.0
        while abs(current_rad) > HOMING_THRESHOLD_RAD:
            step   = math.copysign(HOMING_VEL_RAD_S * HOMING_DT, -current_rad)
            offset += step
            try:
                self._req.setParameter(JOINT_TARGET_PATH, [offset]).get()
            except Exception as e:
                self.get_logger().warn(f'Homing 쓰기 오류: {e}')
                return False
            time.sleep(HOMING_DT)
            val = _read_actual()
            if val is not None:
                current_rad = val   # 실제 위치 갱신 → 클로즈드루프

        self._req.setParameter(JOINT_TARGET_PATH, [offset]).get()
        # homing이 motorPositionActual=0°를 달성한 additive 값을 기준으로 저장
        # CSP: home_offset + AMPLITUDE*sin(t) → 0° 기준 ±10° 유지
        self._home_offset = offset
        self.get_logger().info('Homing 완료 → 0°')
        return True

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

            subscription.notify(callback)
            self._mcx_subscriptions.append(subscription)

            for name, path in zip(joint_names, paths):
                self.get_logger().info(f'구독: {name} ← {path}')

        except Exception as e:
            self.get_logger().error(f'구독 오류: {e}')

    # ── CSP 전송 루프 (전용 스레드, v3 iterate() 동일 방식) ──────────────────
    def _csp_loop(self):
        while rclpy.ok():
            if not self.enable_csp or self._req is None or not self._csp_ready:
                time.sleep(0.01)
                continue

            now = time.perf_counter()
            self._loop_count      += 1
            self._hz_period_count += 1

            # 1초마다 실제 루프 주파수 출력 (v3 iterate() 참조, 시간 기반)
            if self._last_iter > 0.0:
                dt = now - self._last_iter
                if self._hz_last_log_time > 0.0:
                    elapsed_log = now - self._hz_last_log_time
                    if elapsed_log >= 1.0:
                        avg_hz = self._hz_period_count / elapsed_log
                        self.get_logger().info(
                            f'[루프] 실제 주기: {1000.0/avg_hz:.2f} ms  ({avg_hz:.0f} Hz)'
                        )
                        self._hz_last_log_time = now
                        self._hz_period_count  = 0
                else:
                    self._hz_last_log_time = now
            self._last_iter = now

            elapsed = time.monotonic() - self._traj_start_time
            # [쓰기] v3 iterate()와 동일: home_offset 기준 ±AMPLITUDE 사인파 additive offset
            offset_rad = (
                self._home_offset
                + TRAJ_AMPLITUDE_RAD * math.sin(2.0 * math.pi * elapsed / TRAJ_PERIOD_SEC)
            )

            with self._lock:
                self._positions_target['HL_joint2_thigh_r'] = offset_rad

            # [쓰기] hostInJointAdditivePosition1 에 scalar offset 전송 (v3 동일)
            try:
                self._req.setParameter(JOINT_TARGET_PATH, [offset_rad])
            except Exception as e:
                self.get_logger().warn(f'CSP 지령 오류: {e}', throttle_duration_sec=5.0)

            # CSP 루프에서 직접 퍼블리시 (타이머 대비 지연 없음, PUB_DIVIDER회마다 1번)
            self._pub_ctr += 1
            if self._pub_ctr >= PUB_DIVIDER:
                self._pub_ctr = 0
                self._publish_joint_states()

            time.sleep(CYCLE_TIME)   # 1 ms → 1 kHz 목표 (v3 CYCLE_TIME 동일)

    # ── Target / Actual 모니터 로그 (0.5 s) ──────────────────────────────────
    def _log_target_verify(self):
        if self._req is None or not self._csp_ready:
            return
        try:
            tgt = self._req.getParameter(JOINT_TARGET_PATH).get()
            act = self._req.getParameter(ACTUAL_PATH).get()

            # additive offset 값
            tgt_offset = float(tgt.value[0]) if tgt and tgt.value else float('nan')
            act_cnt    = float(act.value[0]) if act and act.value else float('nan')
            act_rad    = act_cnt * ENCODER_TO_RAD

            self.get_logger().info(
                f'[모니터] '
                f'AdditiveOffset={math.degrees(tgt_offset):+.3f}°  '
                f'Actual={math.degrees(act_rad):+.3f}°'
            )
        except Exception as e:
            self.get_logger().warn(f'모니터 오류: {e}', throttle_duration_sec=5.0)

    # ── /joint_states 퍼블리시 ────────────────────────────────────────────────
    def _publish_joint_states(self):
        # 백그라운드 스레드에서 get_clock()은 미세한 비단조성 발생 가능
        # → time.time_ns() 로 직접 구성하면 항상 단조 증가 보장
        ns = time.time_ns()
        now = TimeMsg(sec=ns // 10**9, nanosec=ns % 10**9)

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
