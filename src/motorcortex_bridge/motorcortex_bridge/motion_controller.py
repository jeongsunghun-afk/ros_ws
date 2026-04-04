"""
motion_controller.py
궤적 실행 / moveL / 제어 루프 / 제어 모드 처리 전담 클래스
ROS2 의존성 없음 — joint_state_bridge 에서 생성하여 사용

제어 모드 (GRID root/UserParameters/controlMode):
  0 = standby  — 마지막 위치 유지 (기본)
  1 = mpc      — 내부 MPC 계산 (추후 구현)
  2 = tracking — 외부 명령(RL 등) 추종, 50Hz → 200Hz 선형 보간
  그 외 값     → standby로 처리

보간 (tracking 모드):
  interp_q[i] = prev[i] + t * (target[i] - prev[i])   t ∈ [0, 1]
  t = elapsed / CMD_PERIOD   (CMD_PERIOD = 1/50Hz = 0.02s)

Unitree LowCmd 매핑:
  motorCmd[i].q   → target joint position  [rad]
  motorCmd[i].dq  → target joint velocity  [rad/s]  (시뮬레이터 미지원 → 무시)
  motorCmd[i].Kp  → position gain          (시뮬레이터 미지원 → 무시)
  motorCmd[i].Kd  → velocity gain          (시뮬레이터 미지원 → 무시)
  motorCmd[i].tau → feedforward torque     [Nm]      (시뮬레이터 미지원 → 무시)
"""

import threading
import time
import math

from motorcortex_bridge.motorcortex_interface import (
    MotorcortexInterface,
    N_AXES,
    CTRL_MODE_STANDBY,
    CTRL_MODE_TRACKING,
    CTRL_MODE_MPC,
    POS_MODE_POSITION,
    POS_MODE_FORCE,
)

# ── 홈 자세 ────────────────────────────────────────────────────────────────────
Q_HOME_DEG = [0.0, -150.0, -90.0, 90.0]
#Q_HOME_DEG = [0.0, 0.0, 0.0, 0.0]
Q_HOME_RAD = [math.radians(d) for d in Q_HOME_DEG]

# ── 기본값 ─────────────────────────────────────────────────────────────────────
TRAJ_FILE_DEFAULT  = '/home/jsh/leg_sim/trajectory_jump.txt'
TRAJ_DT_DEFAULT    = 0.005    # 200 Hz
CMD_PERIOD         = 0.02    # 외부 명령 수신 주기 [s] (tracking 모드 기준, 50 Hz)
HOLD_CYCLE         = 0.005   # 200 Hz (보간 루프 주기)
HOME_DURATION      = 3.0     # 홈 복귀 소요 시간 [s]
HOME_THRESHOLD_RAD = math.radians(0.1)  # 홈 판정 임계값 (0.1°)


def load_trajectory(filepath: str) -> list:
    """
    trajectory_jump.txt 로드.
    형식: frame\\tth1_deg\\tth2_deg\\tth3_deg\\tth4_deg
    반환: list of tuple(th1, th2, th3, th4) [rad]
    각 waypoint에서 Q_HOME_RAD를 빼서 반환 (홈 기준 상대 궤적).
    """
    waypoints = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split('\t')
            if not parts[0].lstrip('-').replace('.', '', 1).isdigit():
                continue
            if len(parts) < 5:
                continue
            waypoints.append(
                tuple(math.radians(float(parts[i])) - Q_HOME_RAD[i - 1] for i in range(1, 5))
            )
    return waypoints


class MotionController:
    """
    운동 제어 클래스.
    MCX 통신은 MotorcortexInterface 에 위임.

    제어 모드는 GRID controlMode 값으로 결정:
      set_control_mode(mode) 를 통해 외부에서 갱신.
    """

    def __init__(self, mcx: MotorcortexInterface,
                 traj_file: str = TRAJ_FILE_DEFAULT,
                 traj_dt: float = TRAJ_DT_DEFAULT):
        self._mcx      = mcx
        self.traj_file = traj_file
        self.traj_dt   = traj_dt

        self._lock       = threading.Lock()
        self._ctrl_ready = False
        self._in_movel   = False

        # 제어 모드 (GRID controlMode 동기화)
        # 1=mpc, 2=tracking, 그 외=0(standby)
        self._ctrl_mode = CTRL_MODE_STANDBY

        # 위치 모드 (GRID positionMode 동기화)
        # 0=position(moveJ/moveL), 1=force
        self._pos_mode = POS_MODE_POSITION

        # 마지막 명령 위치 (standby / publish 참조용)
        self._last_cmd_pos = list(Q_HOME_RAD)

        # ── tracking 모드: 선형 보간 상태 ─────────────────────────────────
        self._interp_prev   = list(Q_HOME_RAD)
        self._interp_target = list(Q_HOME_RAD)
        self._interp_time   = time.monotonic()

        # ── tracking 모드: 외부 명령 부가 정보 ────────────────────────────
        self._cmd_dq  = [0.0]  * N_AXES
        self._cmd_kp  = [20.0] * N_AXES
        self._cmd_kd  = [0.5]  * N_AXES
        self._cmd_tau = [0.0]  * N_AXES

        # 보간 루프 스레드 시작
        self._interp_thread = threading.Thread(target=self._interp_loop, daemon=True)
        self._interp_thread.start()

    # ── 프로퍼티 ─────────────────────────────────────────────────────────────
    @property
    def in_movel(self) -> bool:
        return self._in_movel

    @property
    def last_cmd_positions(self) -> list:
        with self._lock:
            return list(self._last_cmd_pos)

    @property
    def ctrl_mode(self) -> int:
        return self._ctrl_mode

    # ── 제어 모드 전환 (GRID 구독 콜백에서 호출) ─────────────────────────────
    def set_control_mode(self, mode: int):
        """
        GRID controlMode 값으로 제어 모드 전환.
        1=mpc, 2=tracking, 그 외 → 0(standby).
        """
        if mode not in (CTRL_MODE_MPC, CTRL_MODE_TRACKING):
            mode = CTRL_MODE_STANDBY
        with self._lock:
            self._ctrl_mode = mode

    def set_position_mode(self, mode: int):
        """
        GRID positionMode 값으로 위치 모드 전환.
        0=position(moveJ/moveL), 1=force, 그 외 → 0(position).
        """
        if mode not in (POS_MODE_POSITION, POS_MODE_FORCE):
            mode = POS_MODE_POSITION
        with self._lock:
            self._pos_mode = mode

    @property
    def pos_mode(self) -> int:
        return self._pos_mode

    # ── 초기 위치 설정 (MCX joint offset 기준) ───────────────────────────────
    def set_initial_positions(self, positions: list):
        """
        MCX joint offset에서 읽은 값을 제어 루프 초기 위치로 설정.
        _ctrl_ready=True 전에 호출해야 함.
        """
        with self._lock:
            self._last_cmd_pos  = list(positions[:N_AXES])
            self._interp_prev   = list(positions[:N_AXES])
            self._interp_target = list(positions[:N_AXES])

    # ── 시뮬레이터 홈 초기화 (비활성화) ─────────────────────────────────────
    # def home(self) -> bool:
    #     """ch0~ch3 을 Q_HOME 으로 이동 (blocking)."""
    #     try:
    #         self._mcx.set_target_positions(Q_HOME_RAD, blocking=True)
    #         with self._lock:
    #             self._last_cmd_pos  = list(Q_HOME_RAD)
    #             self._interp_prev   = list(Q_HOME_RAD)
    #             self._interp_target = list(Q_HOME_RAD)
    #         return True
    #     except Exception:
    #         return False

    # ── 궤적 실행 ─────────────────────────────────────────────────────────────
    def load_trajectory(self) -> list:
        return load_trajectory(self.traj_file)

    def move_l(self, waypoints: list, dt: float = None, log_cb=None):
        """
        4축 동시 궤적 실행.
        waypoints : list of tuple(th1, th2, th3, th4) [rad]
        dt        : waypoint 간격 [s]
        log_cb    : 진행 로그용 콜백 fn(str) — 선택
        """
        dt = dt if dt is not None else self.traj_dt

        self._ctrl_ready = False
        self._in_movel   = True

        # 제어 루프 종료 대기 + 네트워크 큐 소진
        time.sleep(0.05)
        try:
            self._mcx.get_target_positions()
        except Exception:
            pass

        if log_cb:
            log_cb(f'moveL 시작: {len(waypoints)} waypoints / dt={dt*1000:.1f}ms')

        t0 = time.monotonic()
        for idx, wp in enumerate(waypoints):
            self._mcx.set_target_positions(list(wp))
            with self._lock:
                self._last_cmd_pos = list(wp)

            if log_cb and idx % max(1, int(1.0 / dt)) == 0:
                log_cb(
                    f'moveL [{idx}/{len(waypoints)}] '
                    + '  '.join(f'th{i+1}={math.degrees(wp[i]):+.2f}°'
                                for i in range(N_AXES))
                )

            sleep_t = t0 + (idx + 1) * dt - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)

        self._in_movel = False
        # interp 상태를 최종 위치로 동기화 (복귀 방지)
        with self._lock:
            self._interp_prev   = list(self._last_cmd_pos)
            self._interp_target = list(self._last_cmd_pos)
        self._ctrl_ready = True
        if log_cb:
            log_cb('moveL 완료 — interp loop 재개.')

    # ── 홈 복귀 궤적 ─────────────────────────────────────────────────────────────
    def move_to_home(self, log_cb=None):
        """
        현재 actual 위치 → [0, 0, 0, 0] 선형 궤적 생성 후 실행 (blocking).
        모든 축이 이미 HOME_THRESHOLD_RAD 이내이면 즉시 반환.
        """
        current = self._mcx.actual_positions[:N_AXES]

        if all(abs(p) < HOME_THRESHOLD_RAD for p in current):
            if log_cb:
                log_cb('홈 복귀: 이미 홈 위치 — 스킵.')
            return

        n_steps = max(1, int(HOME_DURATION / self.traj_dt))
        waypoints = [
            tuple(current[i] * (1.0 - step / n_steps) for i in range(N_AXES))
            for step in range(1, n_steps + 1)
        ]

        if log_cb:
            log_cb(
                f'홈 복귀 시작: {n_steps} steps / {HOME_DURATION:.1f}s  '
                + ', '.join(f'j{i+1}={math.degrees(current[i]):+.1f}°→0°'
                            for i in range(N_AXES))
            )

        self.move_l(waypoints, log_cb=log_cb)

    # ── tracking 모드: 외부 명령 수신 ─────────────────────────────────────────
    def set_command(self,
                    q:   list,
                    dq:  list = None,
                    kp:  list = None,
                    kd:  list = None,
                    tau: list = None):
        """
        외부 제어기(RL 등) 명령 수신 — tracking 모드에서만 유효.
        보간 시작점을 현재 진행률 기준으로 갱신하여 불연속 방지.

        Parameters
        ----------
        q   : target joint positions  [rad]   (4축, 필수)
        dq  : target joint velocities [rad/s] (선택, 현재 MCX 미지원)
        kp  : position gains                  (선택, 현재 MCX 미지원)
        kd  : velocity gains                  (선택, 현재 MCX 미지원)
        tau : feedforward torques     [Nm]    (선택, 현재 MCX 미지원)
        """
        now = time.monotonic()
        with self._lock:
            # 현재 보간 진행률로 시작점 갱신
            elapsed = now - self._interp_time
            t = min(elapsed / CMD_PERIOD, 1.0)
            self._interp_prev = [
                self._interp_prev[i] + t * (self._interp_target[i] - self._interp_prev[i])
                for i in range(N_AXES)
            ]
            self._interp_target = list(q)
            self._interp_time   = now

            self._cmd_dq  = list(dq)  if dq  is not None else [0.0]  * N_AXES
            self._cmd_kp  = list(kp)  if kp  is not None else [20.0] * N_AXES
            self._cmd_kd  = list(kd)  if kd  is not None else [0.5]  * N_AXES
            self._cmd_tau = list(tau) if tau is not None else [0.0]  * N_AXES
            self._last_cmd_pos = list(q)

    # ── 보간 루프 (200 Hz) ────────────────────────────────────────────
    def _interp_loop(self):
        """
        제어 모드에 따라 200 Hz 로 MCX 에 위치 전송.

          standby  : 마지막 위치 유지
          tracking : prev → target 선형 보간
          mpc      : 추후 구현 (현재는 standby와 동일하게 동작)
        """
        while True:
            if not self._ctrl_ready or not self._mcx.is_connected:
                time.sleep(0.01)
                continue

            with self._lock:
                mode = self._ctrl_mode

                if mode == CTRL_MODE_TRACKING:
                    elapsed = time.monotonic() - self._interp_time
                    t = min(elapsed / CMD_PERIOD, 1.0)
                    positions = [
                        self._interp_prev[i] + t * (self._interp_target[i] - self._interp_prev[i])
                        for i in range(N_AXES)
                    ]
                elif mode == CTRL_MODE_MPC:
                    # TODO: MPC 계산 구현
                    positions = list(self._last_cmd_pos)
                else:
                    # standby (mode == 0 또는 미정의 값)
                    positions = list(self._last_cmd_pos)

            try:
                self._mcx.set_target_positions(positions)
            except Exception:
                pass

            time.sleep(HOLD_CYCLE)
