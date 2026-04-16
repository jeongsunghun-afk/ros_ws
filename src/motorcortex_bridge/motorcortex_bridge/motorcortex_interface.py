"""
motorcortex_interface.py
MCX-OS 와의 저수준 통신 전담 클래스
ROS2 의존성 없음 — motion_controller 또는 단독으로 사용 가능

담당:
  - WebSocket(WSS) 연결 / 재연결
  - Engage, JogMode 시퀀스
  - hostInJointPosition2 쓰기
  - axesPositionsActual 구독
  - JumpEvent / HomeEvent / moveLEvent / ForceSEvent / ForceTEvent / GaitEvent / controlMode 구독

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

JOG_MODE_PATH       = 'root/MachineControl/gotoJogMode'
PAUSE_MODE_PATH     = 'root/MachineControl/gotoPauseMode'
# ── 위치제어경로 ──────────────────────────────────────────────────────────
ADDITIVE_CMD_PATH   = 'root/MachineControl/hostInJointAdditivePosition2'  # additive [rad]
POS_CMD_PATH       = 'root/MachineControl/hostInJointPosition2'              # 절대 위치 [rad] (배열, 6ch) — ch0~3 = 제어축, ch4~5 = 토우/예비 (0 고정)
ACTUAL_PATH        = 'root/AxesControl/axesPositionsActual'   # 부모 경로, value[0~4] 인덱싱
# ── 토크 제어 경로 ────────────────────────────────────────────────────────────
TORQUE_INPUT_PATH       = 'root/AxesControl/axesTorquesInput'          # 토크 입력 읽기 (배열, Nm)
TORQUE_CMD_PATH_FMT     = 'root/AxesControl/axesTorquesInput/ch{:d}'   # 채널별 쓰기 (스칼라, Nm) — ch0~5
TORQUE_ACTUAL_PATH_FMT  = (                                             # 실제 토크 (스칼라, Nm) — {:02d} = 축 번호 (1-based)
    'root/AxesControl/actuatorControlLoops'
    '/actuatorControlLoop{:02d}/actuatorTorqueActual'
)
# ── 이벤트 경로 (GRID UserParameters) ──────────────────────────────────────────
# [leg_test 이벤트]
CONTROL_MODE_PATH   = 'root/UserParameters/controlMode'
JUMP_EVENT_PATH    = 'root/UserParameters/jump'     # 점프 궤적 실행
HOME_EVENT_PATH    = 'root/UserParameters/home'     # 홈 복귀
MOVE_L_EVENT_PATH  = 'root/UserParameters/moveL'     # moveL
FORCE_S_EVENT_PATH = 'root/UserParameters/forceS'     # forceS
FORCE_T_EVENT_PATH = 'root/UserParameters/forceT'     # forceT
GAIT_EVENT_PATH    = 'root/UserParameters/gait'     # gait
# [connect 하위 모드 이벤트] — 경로는 추후 변경 예정
CONNECT_MODE_PATH   = 'root/UserParameters/connectMode'
STANDBY_EVENT_PATH = 'root/UserParameters/standby'     # standby 복귀
RL_EVENT_PATH      = 'root/UserParameters/RL'     # RL 모드 시작
MPC_EVENT_PATH     = 'root/UserParameters/MPC'     # MPC 모드 시작 (추후 구현)

# ── 제어 모드 값 (controlMode) ─────────────────────────────────────────────────
#   0 = action   : Standby (디폴트) / RL / MPC 이벤트 대기
#   1 = leg_test : jump / home / moveL / force / gait 이벤트 처리
CTRL_MODE_CONNECT   = 0
CTRL_MODE_LEG_TEST = 1

# ── connect 하위 모드 ───────────────────────────────────────────────────────────
CONNECT_STANDBY = 0   # 현재 위치 유지 (디폴트)
CONNECT_RL      = 1   # 외부 low_cmd 추종 (50Hz → 200Hz 보간)
CONNECT_MPC     = 2   # 추후 구현

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
        self._actual_torque   = [0.0] * N_AXES   # actuatorTorqueActual (Nm)
        self._base_pos        = [0.0] * N_AXES   # JogMode 진입 시점 절대 위치 (additive 기준)

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

    def set_base_pos(self, positions_rad: list):
        """
        JogMode 진입 시점 절대 위치를 base_pos로 저장.
        set_additive_positions()의 기준점이 됨.
        초기화 시 get_actual_positions_snapshot() 결과를 전달.
        """
        with self._lock:
            self._base_pos = list(positions_rad[:N_AXES])

    def set_additive_positions(self, target_abs: list, blocking: bool = False):
        """
        hostInJointAdditivePosition2에 additive 명령 전송.
        additive[i] = target_abs[i] - base_pos[i]

        target_abs : 목표 절대 위치 [rad]  (N_AXES,)
        """
        with self._lock:
            base = list(self._base_pos)
        additive = [target_abs[i] - base[i] for i in range(N_AXES)]
        cmd      = additive + [0.0] * (NUM_CH - N_AXES)
        future   = self._req.setParameter(ADDITIVE_CMD_PATH, cmd)
        with self._lock:
            self._last_target_rad = list(target_abs[:N_AXES])
        if blocking:
            future.get()

    def reset_additive(self, blocking: bool = False):
        """hostInJointAdditivePosition2 에 0 배열 전송 — additive 초기화."""
        cmd    = [0.0] * NUM_CH
        future = self._req.setParameter(ADDITIVE_CMD_PATH, cmd)
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

    # ── 토크 명령 ─────────────────────────────────────────────────────────────
    def set_target_torques(self, torques_nm: list, blocking: bool = False):
        """
        axesTorquesInput/ch{i} 에 채널별 토크 오프셋 전송 (Nm).
        torques_nm : N_AXES 길이의 토크 리스트 [Nm]
        """
        futures = []
        for i, tau in enumerate(torques_nm[:N_AXES]):
            future = self._req.setParameter(TORQUE_CMD_PATH_FMT.format(i), [float(tau)])
            futures.append(future)
        if blocking:
            for f in futures:
                f.get()

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

    # ── 이벤트 구독 헬퍼 ──────────────────────────────────────────────────────────
    def _subscribe_event(self, path: str, group: str, callback: callable):
        """value[0] == 1 일 때만 callback 호출하는 범용 이벤트 구독."""
        sub = self._sub.subscribe([path], group, frq_divider=1)

        def _cb(msg):
            if msg and msg[0].value and int(msg[0].value[0]) == 1:
                callback()

        sub.notify(_cb)
        self._subs.append(sub)

    def _reset_event(self, path: str):
        """이벤트 파라미터를 0으로 리셋."""
        self._req.setParameter(path, [0]).get()

    # ── 이벤트 구독 / 리셋 ────────────────────────────────────────────────────────
    def subscribe_jump_event(self, cb: callable):
        self._subscribe_event(JUMP_EVENT_PATH, 'jump_group', cb)

    def reset_jump_event(self):
        self._reset_event(JUMP_EVENT_PATH)

    def subscribe_home_event(self, cb: callable):
        self._subscribe_event(HOME_EVENT_PATH, 'home_group', cb)

    def reset_home_event(self):
        self._reset_event(HOME_EVENT_PATH)

    def subscribe_movel_event(self, cb: callable):
        self._subscribe_event(MOVE_L_EVENT_PATH, 'movel_group', cb)

    def reset_movel_event(self):
        self._reset_event(MOVE_L_EVENT_PATH)

    def subscribe_force_s_event(self, cb: callable):
        self._subscribe_event(FORCE_S_EVENT_PATH, 'forces_group', cb)

    def reset_force_s_event(self):
        self._reset_event(FORCE_S_EVENT_PATH)

    def subscribe_force_t_event(self, cb: callable):
        self._subscribe_event(FORCE_T_EVENT_PATH, 'forcet_group', cb)

    def reset_force_t_event(self):
        self._reset_event(FORCE_T_EVENT_PATH)

    def subscribe_gait_event(self, cb: callable):
        self._subscribe_event(GAIT_EVENT_PATH, 'gait_group', cb)

    def reset_gait_event(self):
        self._reset_event(GAIT_EVENT_PATH)

    # ── connect 하위 모드 이벤트 구독 / 리셋 ──────────────────────────────────
    def subscribe_standby_event(self, cb: callable):
        self._subscribe_event(STANDBY_EVENT_PATH, 'standby_group', cb)

    def reset_standby_event(self):
        self._reset_event(STANDBY_EVENT_PATH)

    def subscribe_rl_event(self, cb: callable):
        self._subscribe_event(RL_EVENT_PATH, 'rl_group', cb)

    def reset_rl_event(self):
        self._reset_event(RL_EVENT_PATH)

    def subscribe_mpc_event(self, cb: callable):
        self._subscribe_event(MPC_EVENT_PATH, 'mpc_group', cb)

    def reset_mpc_event(self):
        self._reset_event(MPC_EVENT_PATH)

    # ── controlMode 구독 ──────────────────────────────────────────────────────
    def subscribe_control_mode(self, on_mode_change: callable):
        sub = self._sub.subscribe([CONTROL_MODE_PATH], 'ctrl_mode_group', frq_divider=1)

        def _cb(msg):
            if msg and msg[0].value:
                raw = int(msg[0].value[0])
                mode = raw if raw in (CTRL_MODE_CONNECT, CTRL_MODE_LEG_TEST) else CTRL_MODE_CONNECT
                on_mode_change(mode)

        sub.notify(_cb)
        self._subs.append(sub)

    # ── JogMode / PauseMode 동시 0 구독 ──────────────────────────────────────
    def subscribe_idle_mode(self, on_idle: callable, on_busy: callable = None):
        """
        JogMode/PauseMode 상태 전환 에지 콜백.
          on_idle : non-idle → idle (둘 다 0) 전환 시 1회 호출
          on_busy : idle → non-idle (어느 하나라도 non-0) 전환 시 1회 호출
        """
        sub = self._sub.subscribe(
            [JOG_MODE_PATH, PAUSE_MODE_PATH], 'idle_mode_group', frq_divider=1
        )
        self._jog_val   = None
        self._pause_val = None
        self._was_idle  = False

        def _cb(msg):
            if not msg or len(msg) < 2:
                return
            if msg[0].value:
                self._jog_val   = int(msg[0].value[0])
            if msg[1].value:
                self._pause_val = int(msg[1].value[0])
            if self._jog_val is None or self._pause_val is None:
                return
            is_idle = (self._jog_val == 0 and self._pause_val == 0)
            if is_idle and not self._was_idle:
                on_idle()
            elif not is_idle and self._was_idle and on_busy:
                on_busy()
            self._was_idle = is_idle

        sub.notify(_cb)
        self._subs.append(sub)

    # ── 토크 센싱 구독 ───────────────────────────────────────────────────────
    def subscribe_torque_actual(self):
        """
        actuatorTorqueActual 채널별 구독 (ch0~N_AXES-1).
        경로: actuatorControlLoop{01~04}/actuatorTorqueActual
        """
        paths = [TORQUE_ACTUAL_PATH_FMT.format(i + 1) for i in range(N_AXES)]
        sub   = self._sub.subscribe(paths, 'torque_actual_group', frq_divider=1)

        def _cb(msg):
            if msg:
                with self._lock:
                    for i, m in enumerate(msg[:N_AXES]):
                        if m and m.value:
                            self._actual_torque[i] = float(m.value[0])

        sub.notify(_cb)
        self._subs.append(sub)

    # ── 상태 읽기 ─────────────────────────────────────────────────────────────
    @property
    def actual_positions(self) -> list:
        with self._lock:
            return list(self._actual_pos_rad)

    @property
    def actual_torque(self) -> list:
        with self._lock:
            return list(self._actual_torque)

