"""
motion_controller.py
궤적 실행 / moveJ / moveL / forceS / forceT / 제어 루프 / 제어 모드 처리 전담 클래스
ROS2 의존성 없음 — joint_state_bridge 에서 생성하여 사용

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
제어 모드 (GRID root/UserParameters/controlMode)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  0 = standby  — 마지막 위치 유지 (기본)
  1 = MPC      — 내부 MPC 계산 (추후 구현)
  2 = RL       — 외부 RL 명령 추종, 50Hz → 200Hz 선형 보간
  3 = leg_test — 다리 테스트 모드 (이벤트로 동작 트리거)
  그 외 값     → standby로 처리

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
이벤트 트리거 (GRID root/UserParameters/*)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  JumpEvent   — 점프 궤적 실행 (.txt)
  HomeEvent   — 홈 복귀
  moveLEvent  — moveL (PD 위치 제어, Cartesian)  [leg_test]
  ForceSEvent — forceS (정적 임피던스 I.C.)       [leg_test]
  ForceTEvent — forceT (GRF 궤적 임피던스)        [leg_test]
  GaitEvent   — gait (보행 궤적)                 [leg_test, 추후 구현]

  * moveJ 는 이벤트 없이 직접 호출 메서드로 제공

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
RL 보간 (RL 모드):
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
import numpy as np

from motorcortex_bridge.motorcortex_interface import (
    MotorcortexInterface,
    N_AXES,
    CTRL_MODE_ACTION,
    CTRL_MODE_LEG_TEST,
    ACTION_STANDBY,
    ACTION_RL,
    ACTION_MPC,
)

# ── 홈 자세 ────────────────────────────────────────────────────────────────────
Q_HOME_DEG = [0.0, -150.0, -90.0, 90.0]
Q_HOME_RAD = [math.radians(d) for d in Q_HOME_DEG]

# ── 기본값 ─────────────────────────────────────────────────────────────────────
TRAJ_FILE_DEFAULT  = '/home/jsh/leg_sim/trajectory_jump.txt'
TRAJ_DT_DEFAULT    = 0.005    # 200 Hz
CMD_PERIOD         = 0.02    # 외부 명령 수신 주기 [s] (tracking 모드 기준, 50 Hz)
HOLD_CYCLE         = 0.005   # 200 Hz (보간 루프 주기)
HOME_THRESHOLD_RAD = math.radians(0.01)  # 홈 판정 임계값 (0.01°)
HOME_SPEED_DEG     = 10.0                # ★ 홈 복귀 속도 조정 [°/s] ← 이 값만 변경
HOME_MAX_VEL       = math.radians(HOME_SPEED_DEG)        # [rad/s]
HOME_MAX_ACC       = math.radians(HOME_SPEED_DEG / 2.0)  # [rad/s²]
MOVEL_VEL          = 0.05               # moveL Cartesian 이동 속도 [m/s]

# ── DH 파라미터 (leg_sim_v4.py 동기화) ─────────────────────────────────────────
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

# ── 물리 파라미터 (leg_sim_v4.py 동기화) ─────────────────────────────────────
# 링크 질량 [kg]  (Hip Abduction / Thigh / Shin / Foot)
LINK_MASS = np.array([3.34, 0.8, 0.2, 0.2])
G         = 9.81
G_VEC     = np.array([-G, 0.0, 0.0])   # 중력 방향 (-X = 아래, 월드 기준)


# ── FK / Jacobian 공통 유틸 ───────────────────────────────────────────────────
def _dh_matrix(alpha, a, d, theta):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1],
    ], dtype=float)


def _get_origins_zaxes(thetas: list):
    """
    DH 순기구학으로 관절 원점·z축 배열 반환.
    FK와 Jacobian이 공유하는 공통 유틸 — DH 행렬을 한 번만 계산.
    반환: (origins, z_axes)
      origins : [p0, p1, ..., p4]  각 관절 원점 (p0=힙)
      z_axes  : [z0, z1, ..., z4]  각 관절 z축
    """
    T       = np.eye(4)
    origins = [np.zeros(3)]
    z_axes  = [np.array([0.0, 0.0, 1.0])]
    for (alpha, a, d), theta in zip(_DH, thetas):
        T = T @ _dh_matrix(alpha, a, d, theta)
        origins.append(T[:3, 3].copy())
        z_axes.append(T[:3, 2].copy())
    return origins, z_axes


def forward_kinematics(thetas: list) -> list:
    """
    4축 FK. 각 관절 원점 좌표 반환 (힙=원점 기준).
    반환: [p0, p1, p2, p3, p4]  (p0=힙, p4=발끝)
    """
    origins, _ = _get_origins_zaxes(thetas)
    return origins


def compute_jacobian(thetas: list) -> np.ndarray:
    """
    3D 위치 자코비안 (3×N_AXES).
    J[:,i] = z_i × (p_e - p_i)
    반환: (3, N_AXES) ndarray
    """
    origins, z_axes = _get_origins_zaxes(thetas)
    pe = origins[-1]
    J  = np.zeros((3, len(thetas)))
    for i in range(len(thetas)):
        J[:, i] = np.cross(z_axes[i], pe - origins[i])
    return J


def compute_gravity_torque(thetas: list) -> np.ndarray:
    """
    링크 무게에 의한 중력 보상 토크 (N_AXES×1) [N·m].
    τ_g[j] = Σ_{k≥j} (z_j × (p_com_k − p_j)) · (m_k · G_VEC)

    p_com_k : 링크 k의 무게중심 = 관절 k와 k+1 원점의 중간점
    _get_origins_zaxes 재사용으로 DH 행렬 중복 계산 없음.
    """
    origins, z_axes = _get_origins_zaxes(thetas)
    n      = len(thetas)
    tau_g  = np.zeros(n)
    for k in range(n):                        # 링크 k (관절 k → k+1 사이)
        p_com  = (origins[k] + origins[k + 1]) / 2.0
        f_grav = LINK_MASS[k] * G_VEC        # 중력 하중 [N]
        for j in range(k + 1):               # 관절 j (j ≤ k 인 관절이 링크 k에 영향)
            tau_g[j] += np.dot(np.cross(z_axes[j], p_com - origins[j]), f_grav)
    return tau_g


def _compute_kinematics(thetas: list):
    """
    단일 DH 패스로 FK / Jacobian / 중력 토크를 동시 계산.
    forceS / forceT 200Hz 루프에서 DH 행렬 중복 계산 방지용.

    반환: (x_foot, J, tau_g)
      x_foot : 발끝 위치 [m]       (3,) ndarray
      J      : 위치 자코비안 (3, N_AXES)
      tau_g  : 중력 보상 토크 [Nm] (N_AXES,)
    """
    origins, z_axes = _get_origins_zaxes(thetas)
    pe = origins[-1]
    n  = len(thetas)

    J = np.zeros((3, n))
    for i in range(n):
        J[:, i] = np.cross(z_axes[i], pe - origins[i])

    tau_g = np.zeros(n)
    for k in range(n):
        p_com  = (origins[k] + origins[k + 1]) / 2.0
        f_grav = LINK_MASS[k] * G_VEC
        for j in range(k + 1):
            tau_g[j] += np.dot(np.cross(z_axes[j], p_com - origins[j]), f_grav)

    return pe.copy(), J, tau_g


# ── 임피던스 제어 게인 (leg_sim_v4.py 동기화) ─────────────────────────────────
KP_IMP  = np.array([800.0, 800.0, 800.0])   # Cartesian 강성 [N/m]
KD_IMP  = np.array([ 40.0,  40.0,  40.0])   # Cartesian 감쇠 [N·s/m]
MU_DAMP = 1e-3                               # Jacobian 댐핑 계수 (특이점 방지)


def compute_grf(tau_actual, tau_gravity, J: np.ndarray) -> np.ndarray:
    """
    실제 관절 토크로부터 지면반력(GRF) 추정 (leg_sim_v4.py 동기화).
    λ = (J·Jᵀ + μI)⁻¹ · J · (τ_gravity - τ_actual)  [N]

    tau_actual  : 실측 관절 토크 [Nm]  (N_AXES,) — actuatorTorqueActual
    tau_gravity : 중력 보상 토크  [Nm]  (N_AXES,) — compute_gravity_torque()
    J           : 자코비안 (3, N_AXES) — compute_jacobian()
    반환        : GRF 추정값 [N]  (3,)  — [Fx, Fy, Fz]
    """
    tau_a = np.asarray(tau_actual,  dtype=float)
    tau_g = np.asarray(tau_gravity, dtype=float)
    JJT   = J @ J.T + MU_DAMP * np.eye(3)
    return np.linalg.solve(JJT, J @ (tau_g - tau_a))


def compute_impedance_torque(x_r, x_a, J: np.ndarray,
                              dx_r=None, dx_a=None) -> np.ndarray:
    """
    Cartesian 공간 임피던스 제어 토크 (N_AXES×1) [N·m].

    f_imp   = Kp*(x_r - x_a) + Kd*(dx_r - dx_a)
    tau_imp = J^T · f_imp

    x_r, x_a  : 발끝 목표/실제 위치 [m]  (3,)
    J          : 자코비안 (3, N_AXES)   — compute_jacobian(q_actual)
    dx_r, dx_a : 발끝 목표/실제 속도 [m/s]  (3,) — None 이면 0으로 처리
    """
    x_r  = np.asarray(x_r,  dtype=float)
    x_a  = np.asarray(x_a,  dtype=float)
    dx_r = np.zeros(3) if dx_r is None else np.asarray(dx_r, dtype=float)
    dx_a = np.zeros(3) if dx_a is None else np.asarray(dx_a, dtype=float)

    f_imp = KP_IMP * (x_r - x_a) + KD_IMP * (dx_r - dx_a)
    return J.T @ f_imp


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
        # 0=action, 1=leg_test
        self._ctrl_mode      = CTRL_MODE_ACTION
        # action 하위 모드: 0=standby, 1=RL, 2=MPC
        self._action_sub_mode = ACTION_STANDBY

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

        # 이벤트 트리거 (GRID → Python Event)
        self._idle_ev    = threading.Event()   # JogMode=0 & PauseMode=0 전환
        # [action 하위 모드 이벤트]
        self._standby_ev = threading.Event()   # standby 복귀
        self._rl_ev      = threading.Event()   # RL 모드 시작
        self._mpc_ev     = threading.Event()   # MPC 모드 시작
        # [leg_test 이벤트]
        self._jump_ev    = threading.Event()   # JumpEvent
        self._home_ev    = threading.Event()   # HomeEvent
        self._movel_ev   = threading.Event()   # moveLEvent
        self._force_s_ev = threading.Event()   # ForceSEvent
        self._force_t_ev = threading.Event()   # ForceTEvent
        self._gait_ev    = threading.Event()   # GaitEvent

        # moveL 목표 좌표 (힙 원점 기준, [m])  — 외부에서 set_movel_target()으로 설정
        self._movel_target: list = None   # (Px, Py, Pz)  or  [(Px,Py,Pz), ...]

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

    # ── 제어 모드 전환 (GRID controlMode 구독 콜백에서 호출) ─────────────────────
    def set_control_mode(self, mode: int):
        """
        GRID controlMode 값으로 상위 모드 전환.
          0 = action   : Standby / RL / MPC 하위 모드 (기본: Standby)
          1 = leg_test : jump / home / moveL / force / gait 이벤트 처리
          그 외        → action(0)
        """
        if mode not in (CTRL_MODE_ACTION, CTRL_MODE_LEG_TEST):
            mode = CTRL_MODE_ACTION
        with self._lock:
            self._ctrl_mode = mode

    def set_action_sub_mode(self, sub_mode: int):
        """
        action 모드 하위 모드 전환.
          0 = standby : 현재 위치 유지
          1 = RL      : 외부 low_cmd 추종
          2 = MPC     : 추후 구현
        """
        with self._lock:
            self._action_sub_mode = sub_mode

    # ── moveL 목표 좌표 설정 ──────────────────────────────────────────────────
    def set_movel_target(self, target):
        """
        moveLEvent 트리거 전에 호출하여 목표 좌표를 설정.
        target : (Px, Py, Pz) [m]  or  [(Px,Py,Pz), ...]  (힙 원점 기준)
        """
        if target is not None and not isinstance(target[0], (list, tuple)):
            target = [target]   # 단일 점 → 리스트로 통일
        with self._lock:
            self._movel_target = target

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
        궤적 로드 + 전체 이벤트 구독 + 이벤트 루프 스레드 시작.
        연결 완료 후 1회 호출.
        반환: waypoint 수
        """
        waypoints = load_trajectory(self.traj_file)
        self._waypoints = waypoints
        time.sleep(0.01)
        # 이전 세션에서 남은 값 초기화 (구독 전 리셋 — 초기값 즉시 발화 방지)
        self._mcx.reset_jump_event()
        self._mcx.reset_home_event()
        self._mcx.reset_movel_event()
        self._mcx.reset_force_s_event()
        self._mcx.reset_force_t_event()
        self._mcx.reset_gait_event()

        # 이벤트 구독
        self._mcx.subscribe_idle_mode(self._on_idle_mode, self._on_busy_mode)
        # [action 하위 모드 이벤트]
        self._mcx.subscribe_standby_event(self._on_standby)
        self._mcx.subscribe_rl_event(self._on_rl)
        self._mcx.subscribe_mpc_event(self._on_mpc)
        # [leg_test 이벤트]
        self._mcx.subscribe_jump_event(self._on_jump)
        self._mcx.subscribe_home_event(self._on_home)
        self._mcx.subscribe_movel_event(self._on_movel)
        self._mcx.subscribe_force_s_event(self._on_force_s)
        self._mcx.subscribe_force_t_event(self._on_force_t)
        self._mcx.subscribe_gait_event(self._on_gait)

        self._event_thread = threading.Thread(
            target=self._event_loop, args=(log_cb,), daemon=True
        )
        self._event_thread.start()

        return len(waypoints)

    # ── 이벤트 콜백 (MCX GRID → Python Event) ────────────────────────────────
    def _on_idle_mode(self):
        self._mcx.reset_additive()
        with self._lock:
            self._last_cmd_pos  = [0.0] * N_AXES
            self._interp_prev   = [0.0] * N_AXES
            self._interp_target = [0.0] * N_AXES
        self._idle_ev.set()

    def _on_busy_mode(self):
        self._idle_ev.clear()
        self._ctrl_ready = False

    def _on_standby(self):
        self._standby_ev.set()

    def _on_rl(self):
        self._rl_ev.set()

    def _on_mpc(self):
        self._mpc_ev.set()

    def _on_jump(self):
        if not self._in_movel:
            self._jump_ev.set()

    def _on_home(self):
        self._home_ev.set()

    def _on_movel(self):
        if not self._in_movel:
            self._movel_ev.set()

    def _on_force_s(self):
        if not self._in_movel:
            self._force_s_ev.set()

    def _on_force_t(self):
        if not self._in_movel:
            self._force_t_ev.set()

    def _on_gait(self):
        if not self._in_movel:
            self._gait_ev.set()

    def _event_loop(self, log_cb=None):
        while True:
            # ── IdleMode 대기 (JogMode=0 & PauseMode=0) ───────────────────────
            if not self._idle_ev.is_set():
                if log_cb:
                    log_cb('JogMode/PauseMode 대기 중...')
                self._idle_ev.wait()
                self._ctrl_ready = True
                if log_cb:
                    log_cb('제어권 획득 — 이벤트 수신 대기 중')

            with self._lock:
                mode     = self._ctrl_mode
                sub_mode = self._action_sub_mode

            # ── LEG_TEST 모드: 이벤트 처리 ───────────────────────────────────
            if mode == CTRL_MODE_LEG_TEST:
                # JumpEvent
                jump_triggered = self._jump_ev.wait(timeout=0.05)
                if jump_triggered:
                    self._in_movel = True
                    self._jump_ev.clear()
                    self._mcx.reset_jump_event()
                    self._jump_ev.clear()
                    if log_cb:
                        log_cb('JumpEvent: 점프 궤적 실행')
                    self._send_cst(self._waypoints, self.traj_dt, 'jump', log_cb)
                    if log_cb:
                        log_cb('jump 완료. 다음 이벤트 대기 중...')

                # HomeEvent
                if self._home_ev.is_set():
                    self._home_ev.clear()
                    time.sleep(0.01)
                    self._mcx.reset_home_event()
                    self._home_ev.clear()
                    if log_cb:
                        log_cb('HomeEvent: 홈 복귀 실행')
                    self.move_to_home(log_cb=log_cb)
                    if log_cb:
                        log_cb('홈 복귀 완료')

                # moveLEvent
                if self._movel_ev.is_set():
                    self._movel_ev.clear()
                    self._mcx.reset_movel_event()
                    actual_j = self._mcx.actual_positions
                    fk_pts   = forward_kinematics(actual_j)
                    p_cur    = fk_pts[-1].tolist()
                    target   = [(p_cur[0] - 0.01, p_cur[1], p_cur[2])]
                    if log_cb:
                        log_cb(
                            f'moveLEvent: moveL 실행 '
                            f'({p_cur[0]*1000:.1f}, {p_cur[1]*1000:.1f}, {p_cur[2]*1000:.1f}) mm'
                            f' → ({target[0][0]*1000:.1f}, {target[0][1]*1000:.1f}, {target[0][2]*1000:.1f}) mm'
                        )
                    self.move_l(target, log_cb=log_cb)

                # ForceSEvent
                if self._force_s_ev.is_set():
                    self._force_s_ev.clear()
                    self._mcx.reset_force_s_event()
                    if log_cb:
                        log_cb('ForceSEvent: 정적 임피던스 제어 시작')
                    self._run_force_s(log_cb=log_cb)

                # ForceTEvent
                if self._force_t_ev.is_set():
                    self._force_t_ev.clear()
                    self._mcx.reset_force_t_event()
                    if log_cb:
                        log_cb('ForceTEvent: GRF 궤적 임피던스 제어 시작')
                    self._run_force_t(log_cb=log_cb)
                    if log_cb:
                        log_cb('forceT 완료. 다음 이벤트 대기 중...')

                # GaitEvent
                if self._gait_ev.is_set():
                    self._gait_ev.clear()
                    self._mcx.reset_gait_event()
                    if log_cb:
                        log_cb('GaitEvent: 보행 궤적 실행 (미구현)')
                    # TODO: GaitController 연동

            # ── ACTION 모드: 이벤트 기반 하위 모드 전환 ─────────────────────
            elif mode == CTRL_MODE_ACTION:

                # RL 이벤트
                if self._rl_ev.is_set():
                    self._rl_ev.clear()
                    self._mcx.reset_rl_event()
                    with self._lock:
                        self._action_sub_mode = ACTION_RL
                    if log_cb:
                        log_cb('ACTION: RL 모드 시작 — standby 이벤트 대기')
                    # standby 이벤트 수신까지 RL 모드 유지
                    self._standby_ev.wait()
                    self._standby_ev.clear()
                    self._mcx.reset_standby_event()
                    with self._lock:
                        self._action_sub_mode = ACTION_STANDBY
                    if log_cb:
                        log_cb('ACTION: Standby 복귀')

                # MPC 이벤트
                elif self._mpc_ev.is_set():
                    self._mpc_ev.clear()
                    self._mcx.reset_mpc_event()
                    with self._lock:
                        self._action_sub_mode = ACTION_MPC
                    if log_cb:
                        log_cb('ACTION: MPC 모드 시작 (미구현) — standby 이벤트 대기')
                    # TODO: MPC 구현
                    self._standby_ev.wait()
                    self._standby_ev.clear()
                    self._mcx.reset_standby_event()
                    with self._lock:
                        self._action_sub_mode = ACTION_STANDBY
                    if log_cb:
                        log_cb('ACTION: Standby 복귀')

                # STANDBY (디폴트): 위치 유지 → _interp_loop 에서 처리
                else:
                    if self._standby_ev.is_set():
                        self._standby_ev.clear()
                        self._mcx.reset_standby_event()
                    time.sleep(0.01)

    # ── forceS: 정적 임피던스 제어 루프 ──────────────────────────────────────
    def _run_force_s(self, log_cb=None):
        """
        정적 임피던스 제어 루프 (blocking).
        목표: 현재 발끝 위치에서 X축 +10mm 지점을 q_r로 유지하며
              Cartesian 임피던스 토크(tau_imp) + 중력 보상(tau_g)을 동시 전송.

        종료 조건: HomeEvent 또는 JumpEvent 발생
        """
        # ── 목표 발끝 위치 계산 ───────────────────────────────────────────────
        q_a   = self._mcx.actual_positions
        x_a   = np.array(forward_kinematics(q_a)[-1])
        x_r   = x_a + np.array([0.01, 0.0, 0.0])    # X +10mm
        phi   = q_a[1] + q_a[2] + q_a[3]

        q_r = analytical_ik(x_r[0], x_r[1], x_r[2], phi)
        if q_r is None:
            if log_cb:
                log_cb('forceS: IK 실패 — 중단')
            return

        if log_cb:
            log_cb(
                f'forceS 시작: x_r=({x_r[0]*1000:.1f}, {x_r[1]*1000:.1f},'
                f' {x_r[2]*1000:.1f}) mm'
            )

        self._in_movel   = True
        self._ctrl_ready = False
        dt = self.traj_dt   # 200 Hz

        try:
            # 속도 추정용 이전 발끝 위치 초기화
            x_a_prev, _, _ = _compute_kinematics(self._mcx.actual_positions)

            t0 = time.monotonic()
            k  = 0
            while not (self._home_ev.is_set() or self._jump_ev.is_set()):
                q_now          = self._mcx.actual_positions
                x_a, J, tau_g  = _compute_kinematics(q_now)   # DH 1회만 계산

                dx_a     = (x_a - x_a_prev) / dt              # 발끝 속도 추정
                x_a_prev = x_a.copy()

                tau_imp = compute_impedance_torque(x_r, x_a, J, dx_a=dx_a)
                tau_off = (tau_g + tau_imp).tolist()

                self._mcx.set_target_positions(q_r)
                self._mcx.set_target_torques(tau_off)
                with self._lock:
                    self._last_cmd_pos = list(q_r)

                k += 1
                sleep_t = t0 + k * dt - time.monotonic()
                if sleep_t > 0:
                    time.sleep(sleep_t)

        finally:
            self._mcx.set_target_torques([0.0] * N_AXES)
            self._in_movel   = False
            self._ctrl_ready = True
            if log_cb:
                log_cb('forceS 종료 — 토크 오프셋 초기화')

    # ── forceT: GRF 궤적 임피던스 제어 루프 ──────────────────────────────────
    def _run_force_t(self, log_cb=None):
        """
        GRF 궤적 임피던스 제어 루프 (blocking).

        forceS 와 차이점:
          - q_r 이 시간에 따라 변하는 궤적을 추종 (self._waypoints)
          - Impulse 블록: 토크 센싱 → GRF 추정 → τ_grf = J^T · grf_est
          - 제어 법칙: τ_off = τ_g − τ_grf + τ_imp
              τ_g   : 중력 보상 (feedforward)
              τ_grf : GRF 역변환 — 지면이 발을 지지하는 만큼 모터 토크 감산
              τ_imp : Cartesian 임피던스 보정

        종료 조건: HomeEvent, JumpEvent, 또는 궤적 완료
        """
        waypoints = getattr(self, '_waypoints', [])
        if not waypoints:
            if log_cb:
                log_cb('forceT: 궤적 없음 — 중단')
            return

        if log_cb:
            log_cb(
                f'forceT 시작: {len(waypoints)} waypoints'
                f' / dt={self.traj_dt*1000:.1f}ms'
            )

        self._in_movel   = True
        self._ctrl_ready = False
        dt = self.traj_dt   # 200 Hz

        try:
            # 속도 추정용 이전 발끝 위치 초기화
            x_a_prev, _, _ = _compute_kinematics(self._mcx.actual_positions)
            x_r_prev       = np.array(forward_kinematics(list(waypoints[0]))[-1])

            t0 = time.monotonic()
            for idx, q_r in enumerate(waypoints):
                if self._home_ev.is_set() or self._jump_ev.is_set():
                    if log_cb:
                        log_cb('forceT: HomeEvent/JumpEvent 수신 — 조기 종료')
                    break

                q_r   = list(q_r)
                q_now = self._mcx.actual_positions
                x_a, J, tau_g = _compute_kinematics(q_now)   # DH 1회만 계산

                # 발끝 속도 추정 (위치 미분)
                x_r_cart = np.array(forward_kinematics(q_r)[-1])
                dx_a     = (x_a     - x_a_prev) / dt
                dx_r     = (x_r_cart - x_r_prev) / dt
                x_a_prev = x_a.copy()
                x_r_prev = x_r_cart.copy()

                # Impulse: 토크 센싱 → GRF 추정
                tau_actual = self._mcx.actual_torque
                grf_est    = compute_grf(tau_actual, tau_g, J)

                # Impedance: Cartesian 공간 오차 + 속도 보정
                tau_imp = compute_impedance_torque(x_r_cart, x_a, J, dx_r=dx_r, dx_a=dx_a)

                # 합산: 중력보상 − GRF역변환 + 임피던스
                tau_grf = J.T @ grf_est
                tau_off = (tau_g - tau_grf + tau_imp).tolist()

                self._mcx.set_target_positions(q_r)
                self._mcx.set_target_torques(tau_off)
                with self._lock:
                    self._last_cmd_pos = list(q_r)

                if log_cb and idx % max(1, int(1.0 / dt)) == 0:
                    log_cb(
                        f'forceT [{idx}/{len(waypoints)}] '
                        f'GRF=({grf_est[0]:.1f},{grf_est[1]:.1f},{grf_est[2]:.1f})N  '
                        f'τ=({tau_off[0]:.1f},{tau_off[1]:.1f},'
                        f'{tau_off[2]:.1f},{tau_off[3]:.1f})Nm'
                    )

                sleep_t = t0 + (idx + 1) * dt - time.monotonic()
                if sleep_t > 0:
                    time.sleep(sleep_t)

        finally:
            self._mcx.set_target_torques([0.0] * N_AXES)
            self._in_movel   = False
            self._ctrl_ready = True
            if log_cb:
                log_cb('forceT 종료 — 토크 오프셋 초기화')

    # ── 궤적 실행 공통 내부 함수 ──────────────────────────────────────────────
    def load_trajectory(self) -> list:
        return load_trajectory(self.traj_file)

    def _send_cst(self, waypoints: list, dt: float, label: str,
                  torques: list = None, log_cb=None):
        """
        joint 공간 waypoints를 MCX에 전송 (blocking).
        waypoints : list of list[float]  [rad]          — q_r
        torques   : list of list[float]  [Nm] or None   — τ_offset (τ_ff + τ_imp)
                    None이면 토크 오프셋 0으로 전송
        dt        : waypoint 간격 [s]
        """
        self._ctrl_ready = False
        self._in_movel   = True

        time.sleep(0.05)
        try:
            self._mcx.get_target_positions()
        except Exception:
            pass

        zero_torque = [0.0] * N_AXES
        if log_cb:
            log_cb(f'{label} 시작: {len(waypoints)} waypoints / dt={dt*1000:.1f}ms')

        t0 = time.monotonic()
        for idx, wp in enumerate(waypoints):
            tau = torques[idx] if (torques and idx < len(torques)) else zero_torque
            self._mcx.set_target_positions(list(wp))
            self._mcx.set_target_torques(tau)
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

        self._mcx.set_target_torques(zero_torque)   # 완료 후 토크 오프셋 초기화
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

        self._send_cst(dense, dt, 'moveJ', log_cb)

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
            n = max(1, int(dist / (MOVEL_VEL * dt)))

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

        self._send_cst(dense_j, dt, 'moveL', log_cb)

    # ── 홈 복귀 궤적 ─────────────────────────────────────────────────────────────
    def move_to_home(self, max_vel: float = HOME_MAX_VEL,
                     max_acc: float = HOME_MAX_ACC, log_cb=None):
        """
        현재 actual 위치 → [0, 0, 0, 0] additive 위치 명령으로 홈 복귀 (blocking).
        hostInJointAdditivePosition2 사용.

        HomeEvent 발생마다 그 시점의 actual 위치를 기준으로
        사다리꼴 프로파일로 0° 수렴. 완료 후 실측 기반 1회 보정.
        속도는 HOME_SPEED_DEG [°/s] 상수로 조정.
        """
        # HomeEvent 발생 시점 실제 위치 읽기 — 매번 fresh하게 기준점 설정
        start_pos = self._mcx.get_actual_positions_snapshot()
        self._mcx.set_base_pos(start_pos)   # additive 기준 = 현재 위치

        if all(abs(p) < HOME_THRESHOLD_RAD for p in start_pos):
            if log_cb:
                log_cb('홈 복귀: 이미 홈 위치 — 스킵.')
            return

        max_dist = max(abs(p) for p in start_pos)
        profile  = trapezoid_profile(max_dist, max_vel, max_acc, self.traj_dt)
        n_steps  = len(profile)
        t_total  = n_steps * self.traj_dt

        if log_cb:
            log_cb(
                f'홈 복귀 시작: {n_steps} steps / {t_total:.2f}s '
                f'({HOME_SPEED_DEG:.1f}°/s)  '
                + ', '.join(f'j{i+1}={math.degrees(start_pos[i]):+.1f}°→0°'
                            for i in range(N_AXES))
            )

        self._ctrl_ready = False
        self._in_movel   = True

        try:
            t0 = time.monotonic()
            for k, p in enumerate(profile):
                # 각 축을 start_pos → 0 비례 보간
                target = [start_pos[i] * (1.0 - p / max_dist) for i in range(N_AXES)]
                self._mcx.set_additive_positions(target)
                with self._lock:
                    self._last_cmd_pos = target

                sleep_t = t0 + (k + 1) * self.traj_dt - time.monotonic()
                if sleep_t > 0:
                    time.sleep(sleep_t)

            # 실측 위치 기반 1회 보정
            time.sleep(0.2)   # 궤적 완료 후 정착 대기
            actual = self._mcx.actual_positions
            correction = [-p for p in actual]
            self._mcx.set_additive_positions(correction, blocking=True)

            # hostInJointPosition2도 0으로 동기화 — interp_loop 재개 시 원위치 복귀 방지
            self._mcx.set_target_positions([0.0] * N_AXES, blocking=True)
            with self._lock:
                self._last_cmd_pos = [0.0] * N_AXES

        finally:
            self._in_movel = False
            with self._lock:
                self._interp_prev   = [0.0] * N_AXES
                self._interp_target = [0.0] * N_AXES
            self._ctrl_ready = True

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
                mode     = self._ctrl_mode
                sub_mode = self._action_sub_mode

                if mode == CTRL_MODE_ACTION and sub_mode == ACTION_RL:
                    # RL 명령 추종: prev → target 선형 보간 (50Hz → 200Hz)
                    elapsed = time.monotonic() - self._interp_time
                    t = min(elapsed / CMD_PERIOD, 1.0)
                    positions = [
                        self._interp_prev[i] + t * (self._interp_target[i] - self._interp_prev[i])
                        for i in range(N_AXES)
                    ]
                elif mode == CTRL_MODE_ACTION and sub_mode == ACTION_MPC:
                    # TODO: MPC 계산 구현
                    positions = list(self._last_cmd_pos)
                else:
                    # standby / leg_test: 마지막 명령 위치 유지
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
