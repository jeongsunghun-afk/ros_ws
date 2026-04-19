---
name: orchestrate
description: "CM_HL 로봇 하네스의 오케스트레이터 스킬. hardware-expert, motion-expert, rl-expert, qa-agent를 감독자(supervisor) 패턴으로 조율한다. 복잡한 로봇 개발 요청(하드웨어+모션, 모션+RL 등 2개 이상의 도메인이 엮인 작업)이 들어오면 반드시 이 스킬을 사용할 것."
---

# Orchestrate — CM_HL 로봇 하네스 오케스트레이터

## 팀 구성

| 에이전트 | 담당 |
|---------|------|
| supervisor | 요청 분석 및 라우팅 (이 스킬 담당) |
| hardware-expert | MCX 연결·드라이브·인코더·이벤트 경로 |
| motion-expert | quintic moveL·Bezier gait·forceT/S·_send_cst·IK |
| rl-expert | RL 정책·게인·/low_cmd·/low_state 연동 |
| qa-agent | 경계면 통합 검증 |

## 실행 모드: 에이전트 팀

```
[supervisor]
    ├── TeamCreate(team_name, members)
    ├── TaskCreate(tasks with dependencies)
    ├── 팀원 자체 조율 (SendMessage)
    ├── 결과 수집 (_workspace/ 파일)
    └── 통합 보고서 생성
```

## 라우팅 의사결정

| 요청 유형 | 팀 구성 | 실행 순서 |
|---------|--------|---------|
| MCX 연결 + 홈 이동 | hw + motion | 순차 |
| quintic moveL 설계 | motion | 단독 |
| gait Bezier 파라미터 조정 | motion | 단독 |
| forceT/S 임피던스 튜닝 | motion + qa | 순차 |
| RL 정책 연동 | rl + motion | 병렬 후 qa |
| 보행 시퀀스 전체 설계 | motion + rl + qa | 파이프라인 |
| 전체 시스템 진단 | hw + motion + rl + qa | 병렬 후 qa 순차 |

## 보행 제어 워크플로우

CM_HL 보행 로봇의 대표적인 개발 시나리오:

### Phase-based Gait 개발
```
1. motion-expert: gait(Bezier) ↔ moveL(quintic) 위상 전환 설계
   - 입각기: forceS=ON, moveL로 발끝 이동
   - 유각기: forceS=OFF, gait Bezier 발걸음
2. motion-expert: forceT 임피던스 파라미터 튜닝 (KP_IMP, KD_IMP)
3. rl-expert: 위상 감지 → /low_cmd 명령 매핑 설계
4. qa-agent: 위상 전환 시 torque offset 연속성 검증
```

### MCX → RL 통합 테스트
```
1. hardware-expert: MCX 연결 + ENGAGED 상태 확인
2. motion-expert: 홈 위치 → RL trot 모드 전환
3. rl-expert: /low_cmd 수신 → set_command() → _interp_loop 200Hz 확인
4. qa-agent: /low_state 지연 측정, 보간 오차 검증
```

## 데이터 전달 프로토콜

- **중간 산출물**: `_workspace/{phase}_{agent}_{artifact}.{ext}`
- **팀 통신**: SendMessage (실시간) + TaskCreate (작업 추적)
- **최종 산출물**: 대화 응답 또는 사용자 지정 경로

## 에러 핸들링

- 에이전트 실패: 1회 재시도 → 재실패 시 결과 없이 진행, 보고서에 명시
- 상충 결과: 삭제하지 않고 출처 병기
- 타임아웃: 에이전트당 5분

## 테스트 시나리오

### 정상 흐름 — 보행 시퀀스 설계
```
요청: "MCX 연결하고 홈 잡은 뒤 gait + forceS 조합으로 한 발 걸음 테스트해줘"
1. supervisor → hw-expert: MCX ENGAGED 확인
2. supervisor → motion-expert: move_to_home() 실행
3. supervisor → motion-expert: gait 이벤트 + forceS 토글 시퀀스 설계
4. supervisor → qa-agent: torque offset 연속성 + IK 실패율 검증
5. supervisor: 통합 결과 보고
```

### 에러 흐름 — forceT IK 실패
```
요청: "forceT 실행 중 IK 실패가 반복돼"
1. supervisor → motion-expert: _run_force_t() x_r 목표 좌표 분석
2. motion-expert 실패 (실제 발끝 위치 필요)
3. supervisor → hw-expert: actual_positions 스냅샷 수집
4. supervisor → motion-expert: 재시도 (실제 좌표 기반 IK 가역성 확인)
5. supervisor: 해결책 제시 (phi 재계산 또는 x_r 오프셋 축소)
```
