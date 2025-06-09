# Mobile Robotics Project

이 프로젝트는 ROS2를 기반으로 한 모바일 로봇 제어 시스템입니다. TurtleBot3를 사용하여 다양한 미션을 수행합니다.

## 미션 목록

### 1. 매핑 (Mapping)
- SLAM을 사용하여 환경의 지도를 생성합니다.
- 로봇의 위치 추적 및 환경 인식 기능을 구현합니다.

### 2. 주차 (Parking)
- `mission2_parking.py`를 통해 구현
- 목표 지점으로의 정확한 주차 기능
- PID 제어를 통한 정밀한 위치 제어
- SLAM 기반의 위치 추적

### 3. 웨이포인트 추적 및 장애물 회피 (Waypoint Tracking with Obstacle Avoidance)
- `mission3_waypoint_tracking.py`: 기본 웨이포인트 추적
  - PID 제어를 통한 정밀한 경로 추적
  - 여러 웨이포인트를 순차적으로 방문
  - 자동 속도 조절 기능

- `mission3_waypoint_tracking_avoidance.py`: 장애물 회피 기능이 추가된 웨이포인트 추적
  - LiDAR 센서를 이용한 장애물 감지
  - 실시간 경로 재계획
  - 안전 영역 계산 및 시각화
  - 동적 속도 조절

## 프로젝트 구조

```
.
├── src/
│   └── tb3_control/          # TurtleBot3 제어 패키지
│       ├── tb3_control/      # 소스 코드
│       │   ├── mission2_parking.py                    # 주차 미션
│       │   ├── mission3_waypoint_tracking.py          # 웨이포인트 추적
│       │   ├── mission3_waypoint_tracking_avoidance.py # 장애물 회피 웨이포인트 추적
│       │   ├── pose_publisher.py                      # 로봇 포즈 발행 노드
│       │   ├── SETTINGS.py                           # 전역 설정 파일
│       │   └── utils/        # 유틸리티 모듈
│       │       ├── Robot.py                           # 로봇 상태 관리 클래스
│       │       ├── GoalPlanner.py                     # 목표 지점 계획 클래스
│       │       └── autunomous_module.py              # 자율 주행 관련 유틸리티 함수
│       ├── resource/         # 리소스 파일
│       ├── setup.py         # 패키지 설정
│       ├── package.xml      # 패키지 정보
│       └── setup.cfg        # 빌드 설정
├── build/                    # 빌드 디렉토리
├── install/                  # 설치 디렉토리
└── log/                     # 로그 디렉토리
```

## 유틸리티 모듈 설명

### Robot.py
로봇의 상태를 관리하는 클래스입니다.
- 위치(x, y)와 방향(theta) 정보 관리
- Odometry와 PoseStamped 메시지로부터 로봇 상태 업데이트
- 각도 변환 (라디안 <-> 도) 기능

### GoalPlanner.py
목표 지점으로의 이동을 계획하는 클래스입니다.
- 목표 위치와 방향 설정
- PID 제어를 위한 파라미터 설정
- 속도 계산 (선속도, 각속도)
- 각도 정규화

### autunomous_module.py
자율 주행을 위한 다양한 유틸리티 함수들을 제공합니다.
- LiDAR 데이터 처리 및 매핑
- 안전 영역 계산
- 최적 경로 계획
- 장애물 감지 및 회피
- 목표 지점 도달 확인
- 각도 정규화 및 비용 함수

## 요구사항

- ROS2 (Humble 또는 최신 버전)
- Python 3
- TurtleBot3 관련 패키지
- NumPy
- Matplotlib

## 사용 방법

### SLAM 실행 (매핑)
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

### Pose Publisher 실행
```bash
ros2 run tb3_control pose_publisher
```

### 주차 미션 실행
```bash
python3 src/tb3_control/tb3_control/mission2_parking.py
```

### 웨이포인트 추적 실행
```bash
python3 src/tb3_control/tb3_control/mission3_waypoint_tracking.py
```

### 장애물 회피 웨이포인트 추적 실행
```bash
python3 src/tb3_control/tb3_control/mission3_waypoint_tracking_avoidance.py
```

## 주요 기능

- SLAM 기반 위치 추적
- PID 제어를 통한 정밀한 모션 제어
- LiDAR 기반 장애물 감지 및 회피
- 실시간 경로 재계획
- 시각화 도구 (장애물 회피 미션)

## 주요 파일 설명

### pose_publisher.py
로봇의 현재 위치와 자세를 발행하는 노드입니다.
- TF(Transform)를 사용하여 map 프레임에서 base_link 프레임으로의 변환을 수행
- 로봇의 현재 위치와 방향을 PoseStamped 메시지로 발행
- 0.1초 간격으로 로봇의 포즈 정보 업데이트
- SLAM과 함께 사용되어 로봇의 정확한 위치 추적에 사용

### SETTINGS.py
프로젝트의 전역 설정값을 관리하는 파일입니다.
- AVOID_RANGE: 장애물 회피를 위한 안전 거리 (0.8m)
- BOAT_WIDTH: 로봇의 폭 (1m)
- MAX_RANGE: LiDAR의 최대 감지 거리 (100m)
- GAIN_PSI: 방향 제어를 위한 게인 값 (1.0)
- GAIN_DISTANCE: 거리 제어를 위한 게인 값 (0.5)