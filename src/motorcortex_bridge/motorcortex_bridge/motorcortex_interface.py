"""
motorcortex_interface.py
MCX-OS 와의 저수준 통신 전담 클래스
ROS2 의존성 없음 — motion_controller 또는 단독으로 사용 가능

담당:
  - WebSocket(WSS) 연결 / 재연결
  - Engage, JogMode 시퀀스
  - targetPosition 쓰기
  - motorPositionActual 구독  (ticks → rad)
  - jumpmode / homemode / controlMode 구독 (인터럽트 콜백)
"""

import motorcortex
import threading
import time
import math

# ── MCX 파라미터 경로 ──────────────────────────────────────────────────────────
STATE_CMD_PATH     = 'root/Logic/stateCommand'
STATE_PATH         = 'root/Logic/state'
IS_ENGAGED_PATH    = 'root/Logic/:ctrlToState/isAtEngaged'
ENGAGE_CMD         = 2      # GOTO_ENGAGED_E
ENGAGED_STATE      = 4      # ENGAGED_S
ENGAGE_TIMEOUT     = 10.0

JUMP_MODE_PATH     = 'root/UserParameters/jumpmode'
HOME_MODE_PATH     = 'root/UserParameters/homemode'
CONTROL_MODE_PATH  = 'root/UserParameters/controlMode'
POSITION_MODE_PATH = 'root/UserParameters/positionMode'

# ── 제어 모드 값 (controlMode) ─────────────────────────────────────────────────
CTRL_MODE_STANDBY  = 0 # swing leg - 정해진 궤적을 따라 발을 원하는 위치로 정확하게 이동시킴 
CTRL_MODE_MPC      = 1 # stance leg - 지면에 닿아 로봇의 몸체를 지탱하고 CoM의 안정적인 동역학 구현 목표 속도 추종
CTRL_MODE_TRACKING = 2 # RL control

# ── 테스트 모드 값 (TestMode) ────────────────────────────────────────────────
TEST_MODE_POSITION = 0 #moveJ, moveL // waypoint 입력
TEST_MODE_FORCE    = 1 #Impedance control // 힘/토크 입력
TEST_MODE_Trajectory = 2 #Jump, gait // .txt 궤적실행 or 궤적생성값 입력

BASE_PATH = 'root/AxesControl/actuatorControlLoops/actuatorControlLoop'

# ── 조인트 매핑: (ROS joint name, loop suffix) ────────────────────────────────
#   ch0 = HL_joint2_thigh_r  ← actuatorControlLoop01
#   ch1 = HL_joint3_thigh_p  ← actuatorControlLoop02
#   ch2 = HL_joint4_knee_p   ← actuatorControlLoop03
#   ch3 = HL_joint5_ankle_p  ← actuatorControlLoop04
#   ch4 = HL_joint6_toe_p    ← actuatorControlLoop05 (read-only)
N_AXES = 4
JOINT_LOOP_MAP = [
    ('HL_joint2_thigh_r', '01'),
    ('HL_joint3_thigh_p', '02'),
    ('HL_joint4_knee_p',  '03'),
    ('HL_joint5_ankle_p', '04'),
    ('HL_joint6_toe_p',   '05'),
]
ACTUAL_PATHS = [
    f'{BASE_PATH}{loop}/motorPositionActual'
    for _, loop in JOINT_LOOP_MAP
]
TORQUE_ACTUAL_PATHS = [
    f'{BASE_PATH}{loop}/motorTorqueActual'
    for _, loop in JOINT_LOOP_MAP[:N_AXES]
]
TARGET_PATHS = [
    f'{BASE_PATH}{loop}/motorPositionTarget'
    for _, loop in JOINT_LOOP_MAP[:N_AXES]
]
TORQUE_TARGET_PATHS = [
    f'{BASE_PATH}{loop}/motorTorqueTarget'
    for _, loop in JOINT_LOOP_MAP[:N_AXES]
]
VELOCITY_ACTUAL_PATHS = [
    f'{BASE_PATH}{loop}/motorVelocityActual'
    for _, loop in JOINT_LOOP_MAP[:N_AXES]
]
VELOCITY_TARGET_PATHS = [
    f'{BASE_PATH}{loop}/motorVelocityTarget'
    for _, loop in JOINT_LOOP_MAP[:N_AXES]
]
TORQUE_OFFSET_TARGET_PATHS = [
    f'{BASE_PATH}{loop}/motorTorqueOffsetTarget'
    for _, loop in JOINT_LOOP_MAP[:N_AXES]
]
OFFSET_PATHS = [
    f'{BASE_PATH}{loop}/positionTransformation/transducer/offset'
    for _, loop in JOINT_LOOP_MAP[:N_AXES]
]
TICKS_PER_REV_PATHS = [
    f'{BASE_PATH}{loop}/positionTransformation/transducer/ticksPerRevolution'
    for _, loop in JOINT_LOOP_MAP[:N_AXES]
]


class MotorcortexInterface:
    """
    MCX-OS 통신 래퍼.
    connect() 후 engage() → set_jog_mode() → read_encoder_resolution() → subscribe_positions() 순서로 초기화.
    인코더 분해능은 connect 후 MCX에서 직접 읽어옴 (ticksPerRevolution).
    """

    def __init__(self, url: str, cert: str, login: str, password: str):
        self._url      = url
        self._cert     = cert or '/home/jsh/mcx-client-app-template/mcx.cert.crt'
        self._login    = login
        self._password = password

        self._enc_to_rad = None   # connect 후 read_encoder_resolution()으로 설정

        self._req  = None
        self._sub  = None
        self._subs = []

        self._lock             = threading.Lock()
        self._actual_pos_rad   = [0.0] * len(JOINT_LOOP_MAP)
        self._actual_vel_rad   = [0.0] * len(JOINT_LOOP_MAP)
        self._actual_torque_nm = [0.0] * N_AXES
        self._last_target_rad  = [0.0] * N_AXES

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
            reconnect=True,
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

    # ── 인코더 분해능 읽기 ────────────────────────────────────────────────────
    def read_encoder_resolution(self) -> float:
        """
        ch0 의 ticksPerRevolution 을 읽어 _enc_to_rad 설정.
        모든 축이 동일한 인코더를 사용한다고 가정.
        반환: ticks_per_rev (float)
        """
        result = self._req.getParameter(TICKS_PER_REV_PATHS[0]).get()
        if result and result.value:
            ticks_per_rev = float(result.value[0])
        else:
            raise RuntimeError('ticksPerRevolution 읽기 실패')
        self._enc_to_rad = (2.0 * math.pi) / ticks_per_rev
        return ticks_per_rev

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
        ch0~ch3 에 목표 위치 명령 전송.
        positions_rad: N_AXES 길이의 절대 위치 리스트 [rad]
        """
        futures = []
        for i in range(min(N_AXES, len(positions_rad))):
            ticks = int(round(positions_rad[i] / self._enc_to_rad))
            futures.append(self._req.setParameter(TARGET_PATHS[i], [ticks]))
        with self._lock:
            self._last_target_rad = list(positions_rad[:N_AXES])
        if blocking:
            for f in futures:
                f.get()

    def get_joint_offsets(self) -> list:
        """각 축의 positionTransformation/transducer/offset 값 읽기 [rad]."""
        offsets = []
        for path in OFFSET_PATHS:
            try:
                result = self._req.getParameter(path).get()
                if result and result.value:
                    offsets.append(float(result.value[0]))
                else:
                    offsets.append(0.0)
            except Exception:
                offsets.append(0.0)
        return offsets

    def get_actual_positions_snapshot(self) -> list:
        """현재 실제 위치 읽기 [rad] — 초기값 설정용 1회성 폴링."""
        positions = []
        for path in ACTUAL_PATHS[:N_AXES]:
            try:
                result = self._req.getParameter(path).get()
                if result and result.value:
                    positions.append(float(result.value[0]) * self._enc_to_rad)
                else:
                    positions.append(0.0)
            except Exception:
                positions.append(0.0)
        return positions

    def get_target_positions(self) -> list:
        """현재 목표 위치 [rad] 읽기."""
        positions = []
        for path in TARGET_PATHS:
            try:
                result = self._req.getParameter(path).get()
                if result and result.value:
                    positions.append(float(result.value[0]) * self._enc_to_rad)
                else:
                    positions.append(0.0)
            except Exception:
                positions.append(0.0)
        return positions

    # ── 구독 ─────────────────────────────────────────────────────────────────
    def subscribe_positions(self):
        """모든 조인트의 motorPositionActual 및 ch0~ch3 motorTorqueActual 구독."""
        sub_pos = self._sub.subscribe(ACTUAL_PATHS, 'pos_group', frq_divider=1)

        def _cb_pos(msg):
            with self._lock:
                for i, param in enumerate(msg):
                    if i < len(JOINT_LOOP_MAP) and param.value:
                        self._actual_pos_rad[i] = float(param.value[0]) * self._enc_to_rad

        sub_pos.notify(_cb_pos)
        self._subs.append(sub_pos)

        sub_torque = self._sub.subscribe(TORQUE_ACTUAL_PATHS, 'torque_group', frq_divider=1)

        def _cb_torque(msg):
            with self._lock:
                for i, param in enumerate(msg):
                    if i < N_AXES and param.value:
                        self._actual_torque_nm[i] = float(param.value[0])

        sub_torque.notify(_cb_torque)
        self._subs.append(sub_torque)

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

    @property
    def actual_velocities(self) -> list:
        with self._lock:
            return list(self._actual_vel_rad)

    @property
    def actual_torques(self) -> list:
        with self._lock:
            return list(self._actual_torque_nm)

    def set_torque_targets(self, torques_nm: list, blocking: bool = False):
        futures = []
        for i in range(min(N_AXES, len(torques_nm))):
            futures.append(self._req.setParameter(TORQUE_TARGET_PATHS[i], [torques_nm[i]]))
        if blocking:
            for f in futures:
                f.get()

    def get_monitor_snapshot(self) -> list:
        """모니터 로그용: [(tgt_rad, act_rad), ...] for ch0~ch3. 캐시만 사용."""
        with self._lock:
            return [(self._last_target_rad[i], self._actual_pos_rad[i])
                    for i in range(N_AXES)]
