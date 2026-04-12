"""
motorcortex_interface.py
MCX-OS 와의 저수준 통신 전담 클래스
ROS2 의존성 없음 — motion_controller 또는 단독으로 사용 가능

담당:
  - WebSocket(WSS) 연결 / 재연결
  - Engage, JogMode 시퀀스
  - hostInJointPosition2 쓰기
  - axesPositionsActual 구독
  - jumpmode / homemode / controlMode 구독 (인터럽트 콜백)

  절대 위치 모드 - hostInJointPosition2 4축 동시 제어
  [Machinecontrol] -> [Axescontrol]
  [Joint(ch)] -> [Axes(ch,rad) -> Actuator(rad) -> motor(ticks)]
"""

import motorcortex
import threading
import time
import os

_DEFAULT_CERT = os.path.join(os.path.dirname(__file__), '..', 'config', 'mcx.cert.crt')

# ── MCX 파라미터 경로 ──────────────────────────────────────────────────────────
STATE_CMD_PATH     = 'root/Logic/stateCommand'
STATE_PATH         = 'root/Logic/state'
ENGAGE_CMD         = 2      # GOTO_ENGAGED_E
ENGAGED_STATE      = 4      # ENGAGED_S
ENGAGE_TIMEOUT     = 10.0

JUMP_MODE_PATH     = 'root/UserParameters/jumpmode'
HOME_MODE_PATH     = 'root/UserParameters/homemode'
CONTROL_MODE_PATH  = 'root/UserParameters/controlMode'
POSITION_MODE_PATH = 'root/UserParameters/positionMode'

# ── 제어 모드 값 (controlMode) ─────────────────────────────────────────────────
CTRL_MODE_STANDBY  = 0 # swing leg
CTRL_MODE_MPC      = 1 # stance leg
CTRL_MODE_TRACKING = 2 # RL control

# ── 테스트 모드 값 (positionMode) ────────────────────────────────────────────
TEST_MODE_POSITION   = 0 # moveJ, moveL
TEST_MODE_FORCE      = 1 # Impedance control
TEST_MODE_Trajectory = 2 # Jump, gait

# ── 조인트 매핑: (ROS joint name, ch index) ───────────────────────────────────
#   ch0 = HL_joint2_thigh_r
#   ch1 = HL_joint3_thigh_p
#   ch2 = HL_joint4_knee_p
#   ch3 = HL_joint5_ankle_p
#   ch4 = HL_joint6_toe_p    (read-only)

N_AXES = 4
NUM_CH = 6   # hostInJointPosition2 전체 채널 수

JOINT_LOOP_MAP = [
    ('HL_joint2_thigh_r', '00'),
    ('HL_joint3_thigh_p', '01'),
    ('HL_joint4_knee_p',  '02'),
    ('HL_joint5_ankle_p', '03'),
    ('HL_joint6_toe_p',   '04'),
]
ACTUAL_PATH  = 'root/AxesControl/axesPositionsActual'   # 부모 경로, value[0~4] 인덱싱
POS_CMD_PATH = 'root/MachineControl/hostInJointPosition2'

class MotorcortexInterface:
    """
    MCX-OS 통신 래퍼.
    connect() 후 engage() → set_jog_mode() → subscribe_positions() 순서로 초기화.
    """

    def __init__(self, url: str, cert: str, login: str, password: str):
        self._url      = url
        self._cert     = cert or _DEFAULT_CERT
        self._login    = login
        self._password = password

        self._req  = None
        self._sub  = None
        self._subs = []

        self._lock            = threading.Lock()
        self._actual_pos_rad  = [0.0] * len(JOINT_LOOP_MAP)
        self._last_target_rad = [0.0] * N_AXES

    # ── 연결 ─────────────────────────────────────────────────────────────────
    def connect(self, timeout_ms: int = 5000) -> bool:
        parameter_tree    = motorcortex.ParameterTree()
        motorcortex_types = motorcortex.MessageTypes()
        self._req, self._sub = motorcortex.connect(
            self._url,
            motorcortex_types,
            parameter_tree,
            certificate=self._cert,
            timeout_ms=timeout_ms,
            login=self._login,
            password=self._password,
            reconnect=False,
        )
        return True

    def disconnect(self):
        for sub in self._subs:
            try:
                sub.unsubscribe()
            except Exception:
                pass
        self._subs.clear()

    @property
    def is_connected(self) -> bool:
        return self._req is not None

    # ── Engage / JogMode ──────────────────────────────────────────────────────
    def engage(self, timeout: float = ENGAGE_TIMEOUT) -> bool:
        """Engage 상태 진입. 이미 Engaged면 즉시 True 반환."""
        result = self._req.getParameter(STATE_PATH).get()
        if result and result.value and result.value[0] == ENGAGED_STATE:
            return True

        self._req.setParameter(STATE_CMD_PATH, [ENGAGE_CMD]).get()
        deadline = time.time() + timeout
        while time.time() < deadline:
            result = self._req.getParameter(STATE_PATH).get()
            if result and result.value and result.value[0] == ENGAGED_STATE:
                return True
            time.sleep(0.1)
        return False

    def set_jog_mode(self):
        """JogMode 활성, PauseMode 해제."""
        self._req.setParameter('root/MachineControl/gotoJogMode',  True).get()
        self._req.setParameter('root/MachineControl/gotoPauseMode', False).get()

    # ── 위치 명령 ─────────────────────────────────────────────────────────────
    def set_target_positions(self, positions_rad: list, blocking: bool = False):
        """
        hostInJointPosition2 에 6ch 배열로 목표 위치 명령 전송.
        positions_rad: N_AXES 길이의 절대 위치 리스트 [rad]
        """
        cmd = list(positions_rad[:N_AXES]) + [0.0] * (NUM_CH - N_AXES)
        future = self._req.setParameter(POS_CMD_PATH, cmd)
        with self._lock:
            self._last_target_rad = list(positions_rad[:N_AXES])
        if blocking:
            future.get()

    def get_actual_positions_snapshot(self) -> list:
        """현재 실제 위치 읽기 [rad] — 초기값 설정용 1회성 폴링."""
        try:
            result = self._req.getParameter(ACTUAL_PATH).get()
            if result and result.value:
                return [float(result.value[i]) for i in range(N_AXES)]
        except Exception:
            pass
        return [0.0] * N_AXES

    def get_target_positions(self) -> list:
        """현재 목표 위치 [rad] 읽기 — 캐시 반환."""
        with self._lock:
            return list(self._last_target_rad)

    # ── 구독 ─────────────────────────────────────────────────────────────────
    def subscribe_positions(self):
        """axesPositionsActual 부모 경로 구독, value[0~4] 인덱싱."""
        sub_pos = self._sub.subscribe([ACTUAL_PATH], 'pos_group', frq_divider=1)

        def _cb_pos(msg):
            if msg and msg[0].value:
                with self._lock:
                    for i in range(len(JOINT_LOOP_MAP)):
                        self._actual_pos_rad[i] = float(msg[0].value[i])

        sub_pos.notify(_cb_pos)
        self._subs.append(sub_pos)

    def subscribe_jumpmode(self, on_jump: callable):
        sub = self._sub.subscribe([JUMP_MODE_PATH], 'jump_group', frq_divider=1)

        def _cb(msg):
            if msg and msg[0].value and int(msg[0].value[0]) == 1:
                on_jump()

        sub.notify(_cb)
        self._subs.append(sub)

    def reset_jumpmode(self):
        self._req.setParameter(JUMP_MODE_PATH, [0]).get()

    def subscribe_homemode(self, on_home: callable):
        sub = self._sub.subscribe([HOME_MODE_PATH], 'home_group', frq_divider=1)

        def _cb(msg):
            if msg and msg[0].value and int(msg[0].value[0]) == 1:
                on_home()

        sub.notify(_cb)
        self._subs.append(sub)

    def reset_homemode(self):
        self._req.setParameter(HOME_MODE_PATH, [0]).get()

    def subscribe_control_mode(self, on_mode_change: callable):
        sub = self._sub.subscribe([CONTROL_MODE_PATH], 'ctrl_mode_group', frq_divider=1)

        def _cb(msg):
            if msg and msg[0].value:
                raw = int(msg[0].value[0])
                mode = raw if raw in (CTRL_MODE_MPC, CTRL_MODE_TRACKING) else CTRL_MODE_STANDBY
                on_mode_change(mode)

        sub.notify(_cb)
        self._subs.append(sub)

    def subscribe_test_mode(self, on_mode_change: callable):
        sub = self._sub.subscribe([POSITION_MODE_PATH], 'pos_mode_group', frq_divider=1)

        def _cb(msg):
            if msg and msg[0].value:
                raw = int(msg[0].value[0])
                mode = raw if raw in (TEST_MODE_POSITION, TEST_MODE_FORCE, TEST_MODE_Trajectory) else TEST_MODE_POSITION
                on_mode_change(mode)

        sub.notify(_cb)
        self._subs.append(sub)

    # ── 상태 읽기 ─────────────────────────────────────────────────────────────
    @property
    def actual_positions(self) -> list:
        with self._lock:
            return list(self._actual_pos_rad)

