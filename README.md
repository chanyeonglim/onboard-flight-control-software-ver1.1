# UAM_Flight_control Project

###개발목표###
Lift & Cruise 형식 VTOL 기체 모델링 및 비행제어기 개발 프로젝트  
(MATLAB Simulink UAV Toolbox 기반)

###파일구조###
UAM_Flight_control/
├─ Models/                # Simulink 모델 파일 저장
│   ├─ UAM_Flight_control.slx   (Top 모델, 시스템 전체 구성)
│   ├─ UAM_Status.slx           (Status Estimator)
│   ├─ UAM_Controller.slx       (Flightcontroller)
│   ├─ UAM_FlightMode.slx       ( Mode Manager)
│   └─ UAM_Actuator.slx         (Actuator Mixer, 모터/서보 명령 생성)
│   └─ UAM_Plant.slx            (Simulation 전용 Plant Model, 검증용)
│
├─ Data/                  # 파라미터 및 초기화 스크립트
│   ├─ UAM_Params.m             (모델 파라미터 설정)
│   └─ UAM_Init.m               (시뮬레이션 초기 설정)
│
├─ Scripts/               # 시뮬레이션 실행 / 분석 스크립트
│   ├─ runSimulation.m          (시뮬레이션 실행 스크립트)
│   └─ plotFlightLog.m          (로그 데이터 시각화 스크립트)
│
├─ Docs/                  # 설계 문서 및 정리 자료
│   └─ 설계 메모, 모델 구성도, 회의록 등
│
├─ Images/                # 그림자료 저장 폴더
│   └─ 기체 Layout, 블록도, 구조도 등
│
└─ README.md              # 프로젝트 개요 및 폴더 구조 설명
