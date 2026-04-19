---
name: motion-dev
description: "CM_HL 로봇의 모션 개발 스킬. quintic polynomial moveL, Bezier gait 발걸음, forceT/forceS 임피던스 제어, _send_cst 단일 출력 경로, homing 시퀀스, 이벤트 루프 설계를 수행한다. 새로운 동작을 만들거나, 궤적을 수정하거나, 타이밍 문제를 분석하거나, 임피던스 제어를 조정할 때 반드시 이 스킬을 사용할 것."
---

# Motion Dev — 모션 개발 스킬

## 아키텍처 원칙: 단일 출력 경로

모든 MCX 명령은 `_send_cst` 하나를 통해서만 나간다. 이 함수 밖에서 `set_target_positions`를 직접 호출하지 않는다.

```python
_send_cst(
    waypoints,          # iterable[list[float]] — MCX offset 관절각 [rad]
    dt,                 # 스텝 간격 [s]
    label,              # 로그용 레이블
    cartesian_refs=None,# iterable[ndarray(3,)] — forceS 임피던스 계산용
    stop_fn=None,       # callable → bool — True 시 즉시 종료 (forceT 등)
    log_cb=None,
)
```

`_send_cst` 내부 구조:
- 진입: `_ctrl_ready=False`, `_in_movel=True`, 50ms sleep
- 루프: waypoint 전송 + (cartesian_refs 있으면) FK/J/τ_g 계산 + forceS 체크
- 종료(finally): zero torque, `_in_movel=False`, `_ctrl_ready=True`, interp 상태 복구

## 궤적 타입별 구현 가이드

### 1. moveL — Quintic Polynomial (Cartesian 직선)

```python
move_l(x_target, phi=None, v0=None, vf=None, duration=None, dt=None)
```

**경계조건:** 위치(x₀→xf) + 속도(v₀→vf, 기본=0) + 가속도(0→0)
**이동 시간 자동 계산:** `duration = max(dist / MOVEL_VEL, 3*dt)` (MOVEL_VEL=0.05 m/s)
**샘플 생성:**
```python
coeffs = [_quintic_coeffs(0, T, x0[i], v0[i], 0, xf[i], vf[i], 0) for i in range(3)]
t_arr  = np.linspace(0, T, n+1)[1:]    # t=0 제외, t[-1]=T
cart_pts = list(np.column_stack([t_arr**k for k in range(6)]) @ np.array(coeffs).T)
```

연속 moveL (v₀≠0, vf≠0)로 보행 위상 전환 시 가속도 연속성 보장 가능.

### 2. gait — Cubic Bezier (발걸음 패턴)

```python
_run_gait()  # 이벤트 루프에서 직접 호출
```

Bezier 제어점: `[p_cur, p_start(+x), p_peak(+z), p_end(-x)]`
- `MOVEL_STEP_X = 0.020 m` (x축 반진폭)
- `MOVEL_STEP_H = 0.030 m` (z축 step height)

샘플 수: `n = max(10, int(chord / (MOVEL_VEL * dt)))`

### 3. forceT — 발끝 고정 + GRF 추정

```python
_run_force_t()  # stop_fn으로 종료 제어
```

- 목표: 현재 발끝 위치 + X+10mm 고정
- `itertools.repeat(q_r_mcx)` + `itertools.repeat(x_r)` → 무한 루프
- 종료 조건: `not _force_t_active or _home_ev.is_set() or _jump_ev.is_set()`

### 4. forceS — 동적 임피던스 토글

`_force_s_active` 플래그가 True일 때 `_send_cst` 내부에서 매 step마다:
```python
tau_actual = mcx.actual_torque
grf_est    = compute_grf(tau_actual, tau_g, J)
tau_imp    = compute_impedance_torque(x_r, x_a, J, dx_a=dx_a)
tau = (tau_g - J.T @ grf_est + tau_imp).tolist()
```

`cartesian_refs`가 있으면 `_force_s_active=False`이어도 `_compute_kinematics()`를 실행해 `x_a_prev`를 유지한다. (forceS 전환 시 속도 추정 연속성 보장)

## IK 공통 헬퍼

```python
_ik_trajectory(cart_pts, phi, prev_phy, log_cb=None, label='IK') → list
```

IK 실패 시 이전 해 유지 + 로그 출력. `_run_gait()`와 `move_l()` 모두 이 헬퍼를 사용한다.

## 이벤트 루프 패턴

이벤트가 발화하면 `_event_loop`에서 처리:
```python
# 공통 드레인 패턴 (재트리거 방지)
self._in_movel = True       # 이벤트 핸들러 차단
self._mcx.reset_{event}()   # MCX에 리셋 전송
time.sleep(0.05)            # 버퍼 드레인
self._{event}_ev.clear()    # 이벤트 클리어
self._in_movel = False
```

**gait/jump 이벤트**: `_in_movel=True`를 액션 함수 호출 **전**에 설정한다.
**forceT**: 레벨 기반 (value=1→시작, value=0→종료), `_on_force_t_stop()`에서 `_force_t_active=False`.

## 타이밍 표준

| 루프 | Hz | dt | 사용처 |
|------|----|----|-------|
| `_send_cst` 루프 | 200 | 0.005 s | 모든 궤적 실행 |
| `_interp_loop` | 200 | 0.005 s | RL trot 보간 |
| `/joint_states` 발행 | 50 | 0.020 s | RViz |
| RL 명령 수신 | 50 | 0.020 s | `/low_cmd` |

절대 시각 기준 타이밍 (드리프트 방지):
```python
t0 = time.monotonic()
for idx, wp in enumerate(waypoints):
    send(wp)
    sleep_t = t0 + (idx + 1) * dt - time.monotonic()
    if sleep_t > 0:
        time.sleep(sleep_t)
```

## 홈 위치 (Q_HOME_RAD)

| 조인트 | 각도 | 채널 |
|-------|------|------|
| th1 (thigh_r) | 0° | ch0 |
| th2 (thigh_p) | -150° | ch1 |
| th3 (knee_p) | -90° | ch2 |
| th4 (ankle_p) | +90° | ch3 |

MCX offset 변환: `q_mcx = q_phy − Q_HOME_RAD`

## 임피던스 제어 파라미터

```python
KP_IMP  = [800.0, 800.0, 800.0]   # Cartesian 강성 [N/m]
KD_IMP  = [ 40.0,  40.0,  40.0]   # Cartesian 감쇠 [N·s/m]
MU_DAMP = 1e-3                     # Jacobian 댐핑 (특이점 방지)
```

## 레퍼런스

- 모션 제어기: `motion_controller.py`
- ROS2 브릿지: `joint_state_bridge.py`
- 시뮬레이션: `/home/jsh/leg_sim/leg_sim_v4.py`
- 궤적 파일: `/home/jsh/leg_sim/trajectory_jump.txt`
