"""
motorcortex_interface.py
MCX-OS 와의 저수준 통신 전담 클래스
ROS2 의존성 없음 — motion_controller 또는 단독으로 사용 가능

모드:
  'sim'  — Simulator/targetPosition (절대 ticks), 인코더 1,048,576 (2^20)
  'prod' — hostInJointAdditivePosition1 (additive rad), 인코더 4,096

담당:
  - WebSocket(WSS) 연결 / 재연결
  - disableDrive, Engage, JogMode 시퀀스
  - targetPosition / additive 쓰기
  - motorPositionActual 구독  (ticks → rad)
  - jumpmode 구독 (인터럽트 콜백)
"""

import motorcortex
import threading
import time
import math

# ── MCX 파라미터 경로 ──────────────────────────────────────────────────────────
STATE_CMD_PATH     = 'root/Logic/stateCommand'
STATE_PATH         = 'root/Logic/state'
ENGAGE_CMD         = 2      # GOTO_ENGAGED_E
ENGAGED_STATE      = 4      # ENGAGED_S
ENGAGE_TIMEOUT     = 10.0

PROD_TARGET_PATH   = 'root/MachineControl/hostInJointAdditivePosition1'  # prod 모드 쓰기
DISABLE_DRIVE_PATH = 'root/DriveLogic/disableDrive'
DISABLE_CH_IDX     = [4, 5]
JUMP_MODE_PATH     = 'root/UserParameters/jumpmode'
HOME_MODE_PATH     = 'root/UserParameters/homemode'
CONTROL_MODE_PATH  = 'root/UserParameters/controlMode'
POSITION_MODE_PATH = 'root/UserParameters/positionMode'

# ── 제어 모드 값 (controlMode) ─────────────────────────────────────────────────
#   1 = mpc      (내부 MPC 계산)
#   2 = tracking (외부 RL 명령 추종)
#   그 외 → 0 = standby (위치 유지)
CTRL_MODE_STANDBY  = 0
CTRL_MODE_MPC      = 1
CTRL_MODE_TRACKING = 2

# ── 위치 모드 값 (positionMode) ────────────────────────────────────────────────
#   0 = position (moveJ / moveL 궤적 제어)
#   1 = force    (force control)
#   그 외 → 0 = position
POS_MODE_POSITION = 0
POS_MODE_FORCE    = 1

BASE_PATH = 'root/AxesControl/actuatorControlLoops/actuatorControlLoop'

# ── 조인트 매핑: (ROS joint name, loop suffix, channel index) ──────────────────
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
    f'root/AxesControl/actuatorControlLoops/actuatorControlLoop{i:02d}/motorPositionActual'
    for i in range(1, len(JOINT_LOOP_MAP) + 1)
]
TORQUE_ACTUAL_PATHS = [
    f'root/AxesControl/actuatorControlLoops/actuatorControlLoop{i:02d}/motorTorqueActual'
    for i in range(1, N_AXES + 1)
]
TARGET_PATHS = [
    f'root/AxesControl/actuatorControlLoops/actuatorControlLoop{i:02d}/motorPositionTarget'
    for i in range(1, N_AXES + 1)
]
TORQUE_TARGET_PATHS = [
    f'root/AxesControl/actuatorControlLoops/actuatorControlLoop{i:02d}/motorTorqueTarget'
    for i in range(1, N_AXES + 1)
]
OFFSET_PATHS = [
    f'{BASE_PATH}{loop}/positionTransformation/transducer/offset'
    for _, loop in JOINT_LOOP_MAP[:N_AXES]
]
ACTUAL_POS_PATH = 'root/AxesControl/actuatorControlLoops/actuatorPositionsActual'


# ── 모드별 인코더 상수 ──────────────────────────────────────────────────────────
_COUNTS_PER_REV = {
    'sim':  1048576,   # 2^20 (20-bit 시뮬레이터)
    'prod': 4096,      # 실제 EtherCAT 모터 인코더
}

# ── Production 홈 복귀 파라미터 (비활성화) ────────────────────────────────────
# HOMING_THRESHOLD_RAD = math.radians(0.1)
# HOMING_VEL_RAD_S     = math.radians(30.0)
# HOMING_CYCLE_TIME    = 0.001


class MotorcortexInterface:
    """
    MCX-OS 통신 래퍼.

    mode='sim'  : Simulator/targetPosition에 절대 ticks 전송 (기본)
    mode='prod' : hostInJointAdditivePosition1에 additive rad 전송.
                  prod 모드에서는 home_production() 호출 후 set_target_positions() 사용.

    connect() 호출 후 setup_drives() → subscribe_positions() → subscribe_jumpmode() 순서로 초기화.
    """

    def __init__(self, url: str, cert: str, login: str, password: str, mode: str = 'sim'):
        assert mode in ('sim', 'prod'), f"mode는 'sim' 또는 'prod'여야 합니다: {mode}"
        self._mode     = mode
        self._url      = url
        self._cert     = cert or '/home/jsh/mcx-client-app-template/mcx.cert.crt'
        self._login    = login
        self._password = password

        self._counts_per_rev = _COUNTS_PER_REV[mode]
        self._enc_to_rad     = (2.0 * math.pi) / self._counts_per_rev

        self._req  = None
        self._sub  = None
        self._subs = []          # 등록된 MCX subscription 목록 (cleanup용)

        self._lock             = threading.Lock()
        self._actual_pos_rad   = [0.0] * len(JOINT_LOOP_MAP)   # 전체 조인트 (5개)
        self._actual_vel_rad   = [0.0] * len(JOINT_LOOP_MAP)   # rad/s (미지원 시 0)
        self._actual_torque_nm = [0.0] * N_AXES                 # Nm (ch0~ch3)

        # prod 모드 전용: homing 후 각 축의 additive 기준값 (비활성화)
        # self._home_offsets: list[float] = [0.0] * N_AXES

    @property
    def mode(self) -> str:
        return self._mode

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

    # ── 드라이브 초기화 시퀀스 ─────────────────────────────────────────────
    def disable_drives(self, channels: list = None):
        """지정 채널 disableDrive=True (기본: ch4, ch5)."""
        channels = channels if channels is not None else DISABLE_CH_IDX
        arr = [False] * (max(channels) + 1 if channels else 6)
        for ch in channels:
            arr[ch] = True
        self._req.setParameter(DISABLE_DRIVE_PATH, arr).get()

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

    # ── Production 홈 복귀 (비활성화) ────────────────────────────────────────
    # def home_production(self, axis_is_real: list = None) -> bool: ...
    # def _read_actual_prod(self, axis: int) -> float | None: ...

    # ── 위치 명령 ─────────────────────────────────────────────────────────────
    def set_target_positions(self, positions_rad: list, blocking: bool = False):
        """
        ch0~ch3 에 목표 위치 명령 전송.
        positions_rad: N_AXES 길이의 절대 위치 리스트 [rad]
        각 축을 TARGET_PATHS[i] 에 개별 전송 (ticks 변환).
        """
        futures = []
        for i in range(min(N_AXES, len(positions_rad))):
            ticks = int(round(positions_rad[i] / self._enc_to_rad))
            futures.append(self._req.setParameter(TARGET_PATHS[i], [ticks]))

        if blocking:
            for f in futures:
                f.get()

    def get_joint_offsets(self) -> list:
        """
        각 축의 positionTransformation/transducer/offset 값 읽기 [rad].
        MCX에 저장된 joint offset을 초기 위치 기준값으로 사용.
        실패한 축은 0.0 으로 반환.
        """
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
        """
        actuatorPositionsActual 경로에서 현재 실제 위치 읽기 [rad].
        초기 last_cmd_pos 설정용 (구독 콜백 대신 1회성 폴링).
        실패 시 [0.0] * N_AXES 반환.
        """
        try:
            result = self._req.getParameter(ACTUAL_POS_PATH).get()
            if result and result.value and len(result.value) >= N_AXES:
                return [float(result.value[i]) * self._enc_to_rad for i in range(N_AXES)]
        except Exception:
            pass
        return [0.0] * N_AXES

    def get_target_positions(self) -> list:
        """현재 목표 위치 [rad] 읽기 (각 축 개별 폴링)."""
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
        """
        모든 조인트(5개)의 motorPositionActual 구독.
        ch0~ch3 의 motorTorqueActual 구독.
        콜백에서 _actual_pos_rad, _actual_torque_nm 갱신.
        """
        sub_pos = self._sub.subscribe(ACTUAL_PATHS, 'pos_group', frq_divider=2)

        def _cb_pos(msg):
            with self._lock:
                for i, param in enumerate(msg):
                    if i < len(JOINT_LOOP_MAP) and param.value:
                        self._actual_pos_rad[i] = float(param.value[0]) * self._enc_to_rad

        sub_pos.notify(_cb_pos)
        self._subs.append(sub_pos)

        sub_torque = self._sub.subscribe(TORQUE_ACTUAL_PATHS, 'torque_group', frq_divider=2)

        def _cb_torque(msg):
            with self._lock:
                for i, param in enumerate(msg):
                    if i < N_AXES and param.value:
                        self._actual_torque_nm[i] = float(param.value[0])

        sub_torque.notify(_cb_torque)
        self._subs.append(sub_torque)

    def subscribe_jumpmode(self, on_jump: callable):
        """
        jumpmode 파라미터 구독.
        값이 1 이 되면 on_jump() 콜백 호출.
        """
        sub = self._sub.subscribe([JUMP_MODE_PATH], 'jump_group', frq_divider=1)

        def _cb(msg):
            if msg and msg[0].value and int(msg[0].value[0]) == 1:
                on_jump()

        sub.notify(_cb)
        self._subs.append(sub)

    def reset_jumpmode(self):
        self._req.setParameter(JUMP_MODE_PATH, [0]).get()

    def subscribe_homemode(self, on_home: callable):
        """
        homemode 파라미터 구독.
        값이 1 이 되면 on_home() 콜백 호출.
        """
        sub = self._sub.subscribe([HOME_MODE_PATH], 'home_group', frq_divider=1)

        def _cb(msg):
            if msg and msg[0].value and int(msg[0].value[0]) == 1:
                on_home()

        sub.notify(_cb)
        self._subs.append(sub)

    def reset_homemode(self):
        self._req.setParameter(HOME_MODE_PATH, [0]).get()

    def subscribe_control_mode(self, on_mode_change: callable):
        """
        controlMode 파라미터 구독.
        값이 변경되면 on_mode_change(mode: int) 콜백 호출.
        1=tracking, 2=mpc, 그 외=0(standby)로 정규화하여 전달.
        """
        sub = self._sub.subscribe([CONTROL_MODE_PATH], 'ctrl_mode_group', frq_divider=1)

        def _cb(msg):
            if msg and msg[0].value:
                raw = int(msg[0].value[0])
                mode = raw if raw in (CTRL_MODE_MPC, CTRL_MODE_TRACKING) else CTRL_MODE_STANDBY
                on_mode_change(mode)

        sub.notify(_cb)
        self._subs.append(sub)

    def subscribe_position_mode(self, on_mode_change: callable):
        """
        positionMode 파라미터 구독.
        값이 변경되면 on_mode_change(mode: int) 콜백 호출.
        0=position(moveJ/moveL), 1=force, 그 외=0(position)으로 정규화하여 전달.
        """
        sub = self._sub.subscribe([POSITION_MODE_PATH], 'pos_mode_group', frq_divider=1)

        def _cb(msg):
            if msg and msg[0].value:
                raw = int(msg[0].value[0])
                mode = raw if raw in (POS_MODE_POSITION, POS_MODE_FORCE) else POS_MODE_POSITION
                on_mode_change(mode)

        sub.notify(_cb)
        self._subs.append(sub)

    # ── 상태 읽기 ─────────────────────────────────────────────────────────────
    @property
    def actual_positions(self) -> list:
        """모든 조인트 실제 위치 [rad] (len = len(JOINT_LOOP_MAP))."""
        with self._lock:
            return list(self._actual_pos_rad)

    @property
    def actual_velocities(self) -> list:
        """모든 조인트 실제 속도 [rad/s] (현재 미지원 → 0)."""
        with self._lock:
            return list(self._actual_vel_rad)

    @property
    def actual_torques(self) -> list:
        """ch0~ch3 실제 토크 [Nm]."""
        with self._lock:
            return list(self._actual_torque_nm)

    def set_torque_targets(self, torques_nm: list, blocking: bool = False):
        """
        ch0~ch3 에 목표 토크 명령 전송 [Nm].
        torques_nm: N_AXES 길이의 토크 리스트
        """
        futures = []
        for i in range(min(N_AXES, len(torques_nm))):
            futures.append(self._req.setParameter(TORQUE_TARGET_PATHS[i], [torques_nm[i]]))
        if blocking:
            for f in futures:
                f.get()

    def get_monitor_snapshot(self) -> list:
        """
        모니터 로그용: [(tgt_rad, act_rad), ...] for ch0~ch3
        """
        tgt_positions = self.get_target_positions()

        rows = []
        for i in range(N_AXES):
            tgt_rad = tgt_positions[i] if i < len(tgt_positions) else 0.0
            with self._lock:
                act_rad = self._actual_pos_rad[i]
            rows.append((tgt_rad, act_rad))
        return rows
