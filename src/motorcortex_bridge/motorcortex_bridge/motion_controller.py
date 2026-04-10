"""
motion_controller.py
궤적 실행 / moveJ / moveL / 제어 루프 / 제어 모드 처리 전담 클래스
ROS2 의존성 없음 — joint_state_bridge 에서 생성하여 사용

제어 모드 (GRID root/UserParameters/controlMode):
  0 = standby  — 마지막 위치 유지 (기본)
  1 = mpc      — 내부 MPC 계산 (추후 구현)
  2 = tracking — 외부 명령(RL 등) 추종, 50Hz → 200Hz 선형 보간
  그 외 값     → standby로 처리

보간 (tracking 모드):
  interp_q[i] = prev[i] + t * (target[i] - prev[i])   t ∈ [0, 1]
  t = elapsed / CMD_PERIOD   (CMD_PERIOD = 1/50Hz = 0.02s)

moveJ: joint 공간 waypoint → 사다리꼴 속도 프로파일 → MCX 전송
moveL: Cartesian waypoint → 선형 보간 → IK → MCX 전송

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
import numpy as np

from motorcortex_bridge.motorcortex_interface import (
    MotorcortexInterface,
    N_AXES,
    CTRL_MODE_STANDBY,
    CTRL_MODE_TRACKING,
    CTRL_MODE_MPC,
    TEST_MODE_POSITION,
    TEST_MODE_FORCE,
    TEST_MODE_Trajectory,
)

# ── 홈 자세 ────────────────────────────────────────────────────────────────────
Q_HOME_DEG = [0.0, -150.0, -90.0, 90.0]
Q_HOME_RAD = [math.radians(d) for d in Q_HOME_DEG]

# ── 기본값 ─────────────────────────────────────────────────────────────────────
TRAJ_FILE_DEFAULT  = '/home/jsh/leg_sim/trajectory_jump.txt'
TRAJ_DT_DEFAULT    = 0.005    # 200 Hz
CMD_PERIOD         = 0.02    # 외부 명령 수신 주기 [s] (tracking 모드 기준, 50 Hz)
HOLD_CYCLE         = 0.005   # 200 Hz (보간 루프 주기)
HOME_THRESHOLD_RAD = math.radians(0.1)   # 홈 판정 임계값 (0.1°)
HOME_MAX_VEL       = math.radians(60.0)  # 홈 복귀 최대 각속도 [rad/s] (60°/s)
HOME_MAX_ACC       = math.radians(30.0)  # 홈 복귀 최대 각가속도 [rad/s²] (30°/s²)

# ── DH 파라미터 (leg_sim_v1.py 동기화) ─────────────────────────────────────────
# [alpha, a(m), d(m)]
_DH = [
    (-math.pi/2, 0.0,   0.0   ),   # Joint 1: Hip Abduction
    (0.0,        0.21,  0.0075),   # Joint 2: Hip Pitch
    (0.0,        0.21,  0.0   ),   # Joint 3: Knee
    (0.0,        0.148, 0.0   ),   # Joint 4: Ankle
]
_A2 = _DH[1][1]    # 0.21 m
_A3 = _DH[2][1]    # 0.21 m
_A4 = _DH[3][1]    # 0.148 m
_D2 = _DH[1][2]    # 0.0075 m


# ── FK ────────────────────────────────────────────────────────────────────────
def _dh_matrix(alpha, a, d, theta):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1],
    ], dtype=float)


def forward_kinematics(thetas: list) -> list:
    """
    4축 FK. 각 관절 원점 좌표 반환 (힙=원점 기준).
    반환: [p0, p1, p2, p3, p4]  (p0=힙, p4=발끝)
    """
    T = np.eye(4)
    pts = [np.zeros(3)]
    for i, (alpha, a, d) in enumerate(_DH):
        T = T @ _dh_matrix(alpha, a, d, thetas[i])
        pts.append(T[:3, 3].copy())
    return pts


def analytical_ik(Px: float, Py: float, Pz: float,
                  phi: float, elbow_up: bool = True):
    """
    해석적 IK (leg_sim_v1.py 참조, 4축).
    Px, Py, Pz : 발끝 목표 좌표 [m]  (힙 원점 기준)
    phi        : θ2+θ3+θ4 유지 각도 [rad]
    반환       : [θ1, θ2, θ3, θ4] [rad]  또는 None (해 없음)
    """
    D2 = Px**2 + Py**2 - _D2**2
    if D2 < 0:
        return None
    R = math.sqrt(D2)

    theta1 = math.atan2(-Px, Py) - math.atan2(R, _D2)

    c1, s1 = math.cos(theta1), math.sin(theta1)
    x_s = c1 * Px + s1 * Py
    Z   = -Pz

    x3 = x_s - _A4 * math.cos(phi)
    z3 = Z   - _A4 * math.sin(phi)

    cos_th3 = (x3**2 + z3**2 - _A2**2 - _A3**2) / (2.0 * _A2 * _A3)
    cos_th3 = max(-1.0, min(1.0, cos_th3))
    theta3  = -math.acos(cos_th3) if elbow_up else math.acos(cos_th3)

    theta2 = (math.atan2(z3, x3)
              - math.atan2(_A3 * math.sin(theta3),
                           _A2 + _A3 * math.cos(theta3)))

    theta4 = phi - theta2 - theta3

    def wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    return [wrap(theta1), wrap(theta2), wrap(theta3), wrap(theta4)]


# ── Trajectory 생성 ────────────────────────────────────────────────────────────
def _quintic_coeffs(t0, tf, p0, v0, a0, pf, vf, af):
    """5차 다항식 계수 계산 (경계조건: 위치·속도·가속도)"""
    T_mat = np.array([
        [1, t0, t0**2,   t0**3,   t0**4,   t0**5],
        [0,  1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [0,  0,    2,   6*t0, 12*t0**2, 20*t0**3],
        [1, tf, tf**2,   tf**3,   tf**4,   tf**5],
        [0,  1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0,  0,    2,   6*tf, 12*tf**2, 20*tf**3],
    ])
    return np.linalg.solve(T_mat, np.array([p0, v0, a0, pf, vf, af]))


def _eval_quintic(c, t):
    return (c[0] + c[1]*t + c[2]*t**2 + c[3]*t**3
            + c[4]*t**4 + c[5]*t**5)


def _quintic_segment(p0, pf, duration, dt):
    """
    단일 구간 5차 다항식 보간 (시작/끝 속도·가속도=0).
    반환: list of float (위치)
    """
    c = _quintic_coeffs(0, duration, p0, 0, 0, pf, 0, 0)
    n = max(1, int(duration / dt))
    return [_eval_quintic(c, k * duration / n) for k in range(1, n + 1)]


def trapezoid_profile(dist: float, max_vel: float, max_acc: float, dt: float) -> list:
    """
    단일 축 사다리꼴 속도 프로파일 생성.
    dist     : 이동 거리 (양수)
    max_vel  : 최대 속도 [단위/s]
    max_acc  : 최대 가속도 [단위/s²]
    dt       : 샘플 주기 [s]
    반환     : 각 스텝의 누적 이동량 리스트 (0 → dist)
    """
    d_acc = max_vel ** 2 / (2.0 * max_acc)

    if 2.0 * d_acc >= dist:
        # 삼각형 프로파일 (최대 속도 도달 전에 감속 시작)
        v_peak  = math.sqrt(max_acc * dist)
        t_acc   = v_peak / max_acc
        t_total = 2.0 * t_acc
    else:
        # 사다리꼴 프로파일
        t_acc   = max_vel / max_acc
        t_const = (dist - 2.0 * d_acc) / max_vel
        t_total = 2.0 * t_acc + t_const
        v_peak  = max_vel

    n_steps = max(1, int(t_total / dt))
    positions = []
    for k in range(1, n_steps + 1):
        t = t_total * k / n_steps
        if 2.0 * d_acc >= dist:
            t_acc_loc = v_peak / max_acc
            if t <= t_acc_loc:
                p = 0.5 * max_acc * t ** 2
            else:
                t2 = t - t_acc_loc
                p = 0.5 * max_acc * t_acc_loc ** 2 + v_peak * t2 - 0.5 * max_acc * t2 ** 2
        else:
            if t <= t_acc:
                p = 0.5 * max_acc * t ** 2
            elif t <= t_acc + t_const:
                p = d_acc + max_vel * (t - t_acc)
            else:
                t3 = t - t_acc - t_const
                p = d_acc + max_vel * t_const + max_vel * t3 - 0.5 * max_acc * t3 ** 2
        positions.append(min(p, dist))
    return positions


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
        self._pos_mode = TEST_MODE_POSITION

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

        # jump / home 이벤트
        self._jump_event = threading.Event()
        self._home_event = threading.Event()

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

    def set_test_mode(self, mode: int):
        """
        GRID positionMode 값으로 테스트 모드 전환.
        0=position(moveJ/moveL), 1=force, 2=trajectory, 그 외 → 0(position).
        """
        if mode not in (TEST_MODE_POSITION, TEST_MODE_FORCE, TEST_MODE_Trajectory):
            mode = TEST_MODE_POSITION
        with self._lock:
            self._pos_mode = mode

    @property
    def test_mode(self) -> int:
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

    # ── jump / home 이벤트 루프 시작 ──────────────────────────────────────────
    def start(self, log_cb=None) -> int:
        """
        궤적 로드 + jumpmode/homemode 구독 + 이벤트 루프 스레드 시작.
        연결 완료 후 1회 호출.
        반환: waypoint 수
        """
        waypoints = load_trajectory(self.traj_file)
        self._waypoints = waypoints

        # 이전 세션에서 남은 값 초기화 (구독 전 리셋 — 초기값 즉시 발화 방지)
        self._mcx.reset_jumpmode()
        self._mcx.reset_homemode()

        self._mcx.subscribe_jumpmode(self._on_jump)
        self._mcx.subscribe_homemode(self._on_home)

        self._event_thread = threading.Thread(
            target=self._event_loop, args=(log_cb,), daemon=True
        )
        self._event_thread.start()

        return len(waypoints)

    def _on_jump(self):
        # move_j 실행 중에는 추가 발화 무시 (이중 실행 방지)
        if not self._in_movel:
            self._jump_event.set()

    def _on_home(self):
        self._home_event.set()

    def _event_loop(self, log_cb=None):
        while True:
            jump_triggered = self._jump_event.wait(timeout=0.1)

            if jump_triggered:
                self._in_movel = True        # 즉시 잠금: reset 완료 전 콜백 재발화 차단
                self._jump_event.clear()
                self._mcx.reset_jumpmode()   # blocking — MCX jumpmode=0 확정 후 복귀
                self._jump_event.clear()     # reset 완료 후 재발화된 이벤트 제거
                if log_cb:
                    log_cb('점프 궤적 실행')
                self.move_j(self._waypoints, log_cb=log_cb)
                if log_cb:
                    log_cb('moveL 완료. 다음 점프 대기 중...')

            if self._home_event.is_set():
                self._home_event.clear()
                if log_cb:
                    log_cb('홈 복귀 실행')
                self.move_to_home(log_cb=log_cb)
                self._mcx.reset_homemode()
                if log_cb:
                    log_cb('홈 복귀 완료')

    # ── 궤적 실행 공통 내부 함수 ──────────────────────────────────────────────
    def load_trajectory(self) -> list:
        return load_trajectory(self.traj_file)

    def _send_waypoints(self, waypoints: list, dt: float, label: str, log_cb=None):
        """
        미리 계산된 joint 공간 waypoints를 MCX에 전송 (blocking).
        waypoints : list of list[float]  [rad]
        dt        : waypoint 간격 [s]
        """
        self._ctrl_ready = False
        self._in_movel   = True

        time.sleep(0.05)
        try:
            self._mcx.get_target_positions()
        except Exception:
            pass

        if log_cb:
            log_cb(f'{label} 시작: {len(waypoints)} waypoints / dt={dt*1000:.1f}ms')

        t0 = time.monotonic()
        for idx, wp in enumerate(waypoints):
            self._mcx.set_target_positions(list(wp))
            with self._lock:
                self._last_cmd_pos = list(wp)

            if log_cb and idx % max(1, int(1.0 / dt)) == 0:
                log_cb(
                    f'{label} [{idx}/{len(waypoints)}] '
                    + '  '.join(f'th{i+1}={math.degrees(wp[i]):+.2f}°'
                                for i in range(N_AXES))
                )

            sleep_t = t0 + (idx + 1) * dt - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)

        self._in_movel = False
        with self._lock:
            self._interp_prev   = list(self._last_cmd_pos)
            self._interp_target = list(self._last_cmd_pos)
        self._ctrl_ready = True
        if log_cb:
            log_cb(f'{label} 완료 — interp loop 재개.')

    # ── moveJ ─────────────────────────────────────────────────────────────────
    def move_j(self, joint_waypoints: list, dt: float = None,
               max_vel: float = HOME_MAX_VEL, max_acc: float = HOME_MAX_ACC,
               log_cb=None):
        """
        Joint 공간 moveJ.
        joint_waypoints : list of list[float] [rad]
                          waypoint가 1개이면 현재 위치 → 목표 사다리꼴 프로파일 생성.
                          여러 개이면 순서대로 각 구간을 5차 다항식으로 보간.
        dt              : 전송 주기 [s] (기본 200Hz)
        max_vel / max_acc : 단일 목표점 이동 시 사다리꼴 프로파일 파라미터
        log_cb          : 로그 콜백 fn(str)
        """
        dt = dt if dt is not None else self.traj_dt

        with self._lock:
            current = list(self._last_cmd_pos)

        all_waypoints = [current] + [list(wp) for wp in joint_waypoints]
        dense = []

        for seg_i in range(len(all_waypoints) - 1):
            p0 = all_waypoints[seg_i]
            pf = all_waypoints[seg_i + 1]
            dists = [abs(pf[i] - p0[i]) for i in range(N_AXES)]
            max_dist = max(dists) if max(dists) > 0 else 1e-6

            # 사다리꼴 프로파일로 구간 시간 결정
            profile = trapezoid_profile(max_dist, max_vel, max_acc, dt)
            n = len(profile)
            duration = n * dt

            # 각 축을 5차 다항식으로 보간
            segs = [_quintic_segment(p0[i], pf[i], duration, dt)
                    for i in range(N_AXES)]
            n_pts = len(segs[0])
            for k in range(n_pts):
                dense.append([segs[i][k] for i in range(N_AXES)])

        self._send_waypoints(dense, dt, 'moveJ', log_cb)

    # ── moveL ─────────────────────────────────────────────────────────────────
    def move_l(self, cartesian_waypoints: list, phi: float = None,
               dt: float = None, log_cb=None):
        """
        Cartesian 공간 moveL.
        cartesian_waypoints : list of (Px, Py, Pz) [m]  힙 원점 기준
                              각 구간을 선형 보간 → IK → joint 전송.
        phi     : θ2+θ3+θ4 유지 각도 [rad].
                  None이면 현재 joint 값에서 자동 계산.
        dt      : 전송 주기 [s] (기본 200Hz)
        log_cb  : 로그 콜백 fn(str)
        """
        dt = dt if dt is not None else self.traj_dt

        with self._lock:
            current_j = list(self._last_cmd_pos)

        # phi 자동 계산 (현재 joint 기준)
        if phi is None:
            phi = current_j[1] + current_j[2] + current_j[3]

        # 현재 발끝 위치를 시작점으로
        fk_pts = forward_kinematics(current_j)
        p_start = fk_pts[-1].tolist()

        all_cart = [p_start] + [list(wp) for wp in cartesian_waypoints]
        dense_j  = []

        for seg_i in range(len(all_cart) - 1):
            p0 = np.array(all_cart[seg_i])
            pf = np.array(all_cart[seg_i + 1])
            dist = float(np.linalg.norm(pf - p0))
            n = max(1, int(dist / (max(HOME_MAX_VEL, 0.01) * dt)))

            prev_j = list(current_j) if seg_i == 0 else dense_j[-1]
            for k in range(1, n + 1):
                t = k / n
                pt = p0 + t * (pf - p0)
                result = analytical_ik(pt[0], pt[1], pt[2], phi)
                if result is None:
                    if log_cb:
                        log_cb(f'moveL IK 실패: seg={seg_i} k={k} pt={pt}')
                    result = prev_j
                prev_j = result
                dense_j.append(result)

        self._send_waypoints(dense_j, dt, 'moveL', log_cb)

    # ── 홈 복귀 궤적 ─────────────────────────────────────────────────────────────
    def move_to_home(self, max_vel: float = HOME_MAX_VEL,
                     max_acc: float = HOME_MAX_ACC, log_cb=None):
        """
        현재 actual 위치 → [0, 0, 0, 0] 사다리꼴 속도 프로파일 궤적 실행 (blocking).
        가장 많이 움직이는 축 기준으로 시간이 결정되고, 나머지 축은 비례 스케일.
        모든 축이 HOME_THRESHOLD_RAD 이내이면 즉시 반환.

        Parameters
        ----------
        max_vel : 최대 각속도 [rad/s]  (기본 HOME_MAX_VEL = 60°/s)
        max_acc : 최대 각가속도 [rad/s²] (기본 HOME_MAX_ACC = 30°/s²)
        """
        current = self._mcx.get_actual_positions_snapshot()

        if all(abs(p) < HOME_THRESHOLD_RAD for p in current):
            if log_cb:
                log_cb('홈 복귀: 이미 홈 위치 — 스킵.')
            return

        # 가장 큰 이동 거리 축 기준 사다리꼴 프로파일 생성
        max_dist = max(abs(p) for p in current)
        profile  = trapezoid_profile(max_dist, max_vel, max_acc, self.traj_dt)
        n_steps  = len(profile)

        # 각 축을 거리에 비례하여 스케일
        waypoints = [
            tuple(
                current[i] * (1.0 - profile[k] / max_dist)
                for i in range(N_AXES)
            )
            for k in range(n_steps)
        ]

        t_total = n_steps * self.traj_dt
        if log_cb:
            log_cb(
                f'홈 복귀 시작: {n_steps} steps / {t_total:.2f}s '
                f'(max_vel={math.degrees(max_vel):.0f}°/s, max_acc={math.degrees(max_acc):.0f}°/s²)  '
                + ', '.join(f'j{i+1}={math.degrees(current[i]):+.1f}°→0°'
                            for i in range(N_AXES))
            )

        self.move_j(waypoints, log_cb=log_cb)

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

    def get_monitor_snapshot(self) -> list:
        """모니터 로그용: [(tgt_rad, act_rad), ...] for ch0~ch3."""
        actual = self._mcx.actual_positions
        with self._lock:
            return [(self._last_cmd_pos[i], actual[i]) for i in range(N_AXES)]
