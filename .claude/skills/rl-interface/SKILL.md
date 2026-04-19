---
name: rl-interface
description: "CM_HL 로봇의 강화학습(RL) 인터페이스 스킬. /low_cmd, /low_state, /rl_gains 토픽 연동, Kp/Kd 게인 튜닝, RL 정책 디버깅을 수행한다. RL 정책을 연결하거나, 게인을 조정하거나, 관측/행동 공간을 수정할 때 반드시 이 스킬을 사용할 것."
---

# RL Interface — 강화학습 인터페이스 스킬

## 토픽 사양

### /low_cmd (구독, Unitree LowCmd 호환)

| 필드 | MCX 전달 | 설명 |
|------|---------|------|
| position [4] | ✅ 200Hz 보간 후 전달 | 목표 관절각 [rad] |
| velocity [4] | ❌ | 보간에만 활용 |
| effort [4] | ❌ 현재 미연결 | CST additive offset 구조 지원, `_cmd_tau` 저장됨 |

### /low_state (발행, Unitree LowState 호환)

- 현재 4개 조인트 실제 위치 [rad]
- 업데이트 주기: 50Hz

### /rl_gains (구독, Float64MultiArray)

```
data: [kp0, kp1, kp2, kp3, kd0, kd1, kd2, kd3]
기본값: [20.0, 20.0, 20.0, 20.0, 0.5, 0.5, 0.5, 0.5]
```

## MCX 토크 구조

MCX는 CST 모드로 동작하며, `set_target_torques(tau)`가 additive torque offset을 전달한다.
임피던스 제어(forceS/forceT)는 `_send_cst` 내에서 이 경로로 토크를 전달한다.
`_interp_loop`(RL trot 추적)에서 effort forwarding을 활성화하려면:
```python
# _interp_loop 내 set_target_positions 호출 후 추가
with self._lock:
    tau = list(self._cmd_tau)
self._mcx.set_target_torques(tau)
```

## 명령 처리 구현

```python
# joint_state_bridge.py — /low_cmd 콜백
def _on_low_cmd(self, msg: JointState):
    q   = list(msg.position[:N_AXES])
    dq  = list(msg.velocity[:N_AXES]) if len(msg.velocity) >= N_AXES else None
    tau = list(msg.effort[:N_AXES])   if len(msg.effort)   >= N_AXES else None
    kp  = list(self._rl_kp)
    kd  = list(self._rl_kd)
    self._ctrl.set_command(q=q, dq=dq, kp=kp, kd=kd, tau=tau)
```

`set_command()` (motion_controller.py)는 현재 보간 진행률로 `_interp_prev`를 갱신하여 불연속을 방지한다.

## 게인 튜닝 가이드

- **Kp 증가**: 위치 추종 강화 → 진동/오버슈트 위험
- **Kd 증가**: 진동 감쇠 → 과도하면 응답 지연
- 권장 순서: Kp 점진적 증가 → 진동 시 Kd 증가
- forceS ON 시 임피던스 게인(KP_IMP, KD_IMP)이 별도로 작용함을 인지할 것

## 레퍼런스

- RL 인터페이스 구현: `joint_state_bridge.py`
- RL 명령 처리 (`set_command`): `motion_controller.py`
- 시뮬레이션 레퍼런스: `/home/jsh/leg_sim/leg_sim_v4.py`
