# onboard-flight-control-software ver 1.1

<<<<<<< HEAD
## 📌 개발 목표
=======
## 개발목표

Lift & Cruise 형식 VTOL 기체를 위한 MATLAB Simulink 기반 모델링 및 비행제어기 설계 프로젝트입니다.  
UAV Toolbox와 Model-Based Design(MBD)을 활용해 실기체 적용이 가능한 고신뢰성 제어 시스템 개발을 목표로 합니다.

---

## 🔀 브랜치 구성

| 브랜치 이름 | 담당자         | 역할 설명                     |
|-------------|----------------|------------------------------|
| `dev`       | navyblue95     | 메인 통합 브랜치             |
| `dev-1`     | tndus0825      | 서브 모듈 1 개발             |
| `dev-2`     | chanyeonglim   | 시스템 통합 및 컨트롤러 개발 |
| `dev-3`     | ghpark0922     | 센서 및 가이던스 관련 개발   |

> 각 개발자는 본인 브랜치에서 작업 후 `dev` 브랜치로 Pull Request를 통해 병합합니다.

---

## 📁 디렉토리 구조

UAM_Flight_control/
├── Models/                             # 주요 Simulink 모델 파일  
│   ├── UAM_Flight_control.slx         # 최상위 모델 (Top-Level System Integration)  
│   ├── UAM_Status.slx                 # 센서 데이터 처리 (Sensor Data Handling)  
│   ├── UAM_FlightModeManger.slx       # 비행 모드 전환 로직 (Flight Mode Change Logic)  
│   ├── UAM_GuidanceandNavigation.slx  # 센서 추정 및 경로 생성 (Sensor Estimation & Guidance)  
│   ├── UAM_Controller.slx             # 비행 제어기 (Flight Controller)  
│   ├── UAM_FlightMode.slx             # 비행 모드 관리자 (Mode Manager)  
│   ├── UAM_Actuator.slx               # 액추에이터 믹서 (Actuator Mixer / 모터·서보 명령 생성)  
│   └── UAM_Plant.slx                  # 시뮬레이션용 플랜트 모델 (Plant Model for Simulation)  
│
├── Data/                               # 파라미터 및 초기화 스크립트  
│   ├── UAM_data.sldd                  # 모델 변수 정의용 데이터 딕셔너리  
│   ├── UAM_Params.m                   # 모델 파라미터 설정 스크립트  
│   └── UAM_Init.m                     # 시뮬레이션 초기 설정 스크립트  
│
├── Scripts/                            # 시뮬레이션 실행 및 분석 스크립트  
│   ├── runSimulation.m                # 시뮬레이션 실행 스크립트  
│   └── plotFlightLog.m                # 로그 데이터 시각화 스크립트  
│
├── Docs/                               # 설계 문서 및 정리 자료  
│   └── 설계 메모, 모델 구성도, 회의록 등  
│
├── Images/                             # 그림 자료 저장 폴더  
│   └── 기체 Layout, 블록도, 구조도 등  
│
└── README.md                           # 프로젝트 개요 및 폴더 구조 설명 파일

---

## 🔧 사용 툴 및 환경

- MATLAB R2024b  
- Simulink  
- UAV Toolbox  
- Git 버전 관리 (GitHub 연동)  

---

## 📈 향후 계획

- 모듈별 기능 고도화 및 상태 기반 테스트 추가  
- SIL/HIL 통합을 통한 실제 하드웨어 연동  
- DO-178C 기반 개발 프로세스 정비 및 요구사항 추적  

---

※ 프로젝트에 대한 문의 또는 기여 제안은 아래 저장소를 통해 전달해주세요:  
https://github.com/chanyeonglim/UAM-Flight-control-software.git
