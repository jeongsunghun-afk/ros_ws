---
name: hardware-debug
description: "CM_HL 로봇의 Motorcortex(MCX) 하드웨어 디버깅 스킬. MCX 연결 문제, 드라이브 상태 확인, 인코더 캘리브레이션, ENGAGED/PAUSED/JOG 상태머신 진단을 수행한다. MCX 연결이 안 될 때, 드라이브가 응답하지 않을 때, 인코더 값이 이상할 때 반드시 이 스킬을 사용할 것."
---

# Hardware Debug — MCX 하드웨어 진단 스킬

## 진단 워크플로우

### Step 1: 연결 진단
1. WSS URL (`wss://192.168.2.100`) 및 인증서 경로 확인
2. `motorcortex_interface.py`의 `connect()` 반환값 확인
3. 네트워크 레이어 진단: `ping 192.168.2.100`

### Step 2: 드라이브 상태 확인
1. `root/MachineControl/` 경로의 현재 상태 읽기
2. 상태머신 전환 시퀀스 확인: PAUSED → JOG → ENGAGED
3. ch4/ch5 비활성화 (`disable_drives`) 여부 확인

### Step 3: 모드 및 인코더 확인

인코더 값 차이는 버그가 아니라 **의도된 모드 설계**:

| 모드 | COUNTS_PER_REV | 쓰기 경로 | 값 형식 |
|------|---------------|----------|---------|
| sim  | 1,048,576 (2^20) | `root/Simulator/targetPosition` | 절대 ticks |
| prod | 4,096 | `root/MachineControl/hostInJointAdditivePosition1` | additive rad |

- 변환 공식: `rad = counts × 2π / COUNTS_PER_REV`

### Step 4: 파라미터 경로 검증

#### MCX 읽기/쓰기 경로

| 파라미터 | MCX 경로 | 방향 |
|---------|---------|------|
| 실제 위치 | `root/AxesControl/actuatorControlLoops/actuatorControlLoop{01-05}/motorPositionActual` | 읽기 |
| 목표 위치 (prod) | `root/MachineControl/hostInJointAdditivePosition2` | 쓰기 |
| 목표 토크 | `root/MachineControl/hostInJointTorqueOffset` | 쓰기 (CST additive) |
| 실제 토크 | `root/AxesControl/actuatorControlLoops/actuatorControlLoop{01-05}/actuatorTorqueActual` | 읽기 |
| JOG 모드 | `root/MachineControl/gotoJogMode` | 쓰기 |
| PAUSE 모드 | `root/MachineControl/gotoPauseMode` | 쓰기 |

#### UserParameters 이벤트 경로

| 이벤트 | MCX 경로 | 트리거 조건 |
|--------|---------|-----------|
| jump | `root/UserParameters/jump_event` | value=1 |
| gait | `root/UserParameters/gait_event` | value=1 |
| moveL | `root/UserParameters/moveL_event` | value=1 |
| forceT | `root/UserParameters/forceT_event` | value=1(시작), value=0(종료) — 레벨 기반 |
| forceS | `root/UserParameters/forceS_event` | value=1 (토글) |
| home | `root/UserParameters/home_event` | value=1 |
| sitting | `root/UserParameters/sitting_event` | value=1 |
| standing | `root/UserParameters/standing_event` | value=1 |
| rl_trot | `root/UserParameters/rl_trot_event` | value=1 |
| fall_recovery | `root/UserParameters/fall_recovery_event` | value=1 |

**forceT는 레벨 기반**: 다른 이벤트와 달리 value=0도 의미가 있다 (GRF 제어 종료).
`subscribe_force_t_event(on_start, on_stop)` — `_subscribe_level_event()` 헬퍼 위임.

## 채널 구성

| 채널 | 조인트 | 제어 가능 |
|------|-------|---------|
| ch0 | HL_joint2_thigh_r | ✅ |
| ch1 | HL_joint3_thigh_p | ✅ |
| ch2 | HL_joint4_knee_p | ✅ |
| ch3 | HL_joint5_ankle_p | ✅ |
| ch4 | HL_joint6_toe_p | 읽기 전용 |
| ch5 | (미사용) | 비활성화 |

## 레퍼런스

- 저수준 통신 구현: `motorcortex_interface.py`
- MCX 클라이언트 예제: `/home/jsh/mcx-client-app-template/mcx-client-app_v4.py`
- MCX 인증서: `/home/jsh/mcx-client-app-template/mcx.cert.crt`
