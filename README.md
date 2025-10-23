# 마! 뜨급나?
물류 창고 화재 예방·모니터링 플랫폼 `마! 뜨급나?`는 SLAM 기반 맵과 자율주행 로봇, 열화상 리포트, 실시간 알림을 하나로 통합하여 제공하는 산업용 모니터링 시스템입니다. 

> **🚨 기본 프로젝트 정보**
> - 배포 URL: http://are-u-hot.kro.kr/
> - 총 기획·개발 기간: 25.07.14 ~ 25.08.18 
> 
> **🚨 TEST 계정 정보**
> - id: ssafy
> - password: hot123

## 프로젝트 소개 
**🚨 문제**  
물류 창고는 밀집 적재·열원·사각지대 등으로 초기 발화 징후 파악이 어려워 탐지 지연이 발생하고, 이로 인해 큰 피해를 입을 수 있습니다.

**🚨 해결**  
`마! 뜨급나?`는 SLAM 기반 자율주행 로봇이 창고 내 촬영 스팟을 정기 순찰하며 열화상 데이터를 수집·분석하고, 이상 온도를 감지하면 실시간 알림/지도 시각화로 즉각 대응을 유도합니다.  
프론트는 PWA로, 관리자·작업자는 모바일에서 알림을 받고 현장까지의 정확한 위치 안내 및 처리 이력 등록이 가능합니다.

**🚨 핵심 가치**  
- 발화 전 단계의 조기 징후 탐지로 피해 최소화  
- 알림/처리 이력의 표준화로 대응 시간 단축  
- 원격-현장 협업(관리자↔작업자)을 통한 오퍼레이션 안정성 향상

**🚨 구성/흐름(요약)**  
- 로봇: SLAM 맵 업로드·순찰·열화상 촬영 → 이상 시 리포트 발행
- 백엔드: Spring Boot + MyBatis, 세션 인증, WebSocket(STOMP) 브로드캐스트, S3 presigned URL로 이미지 안전 제공  
- 프론트: React, PWA, 실시간 위치·리포트 지도 시각화, 처리 이력 기록

**🚨 주요 시나리오**  
1) 로봇이 스팟 도달 → 열화상 촬영/분석 → 이상 온도 감지 시 리포트 전송  
2) 서버 저장(S3 키) → presigned URL 변환 → WebSocket으로 실시간 알림  
3) 관리자/작업자: 모바일에서 위치·이미지 확인 → 처리 이력 등록(세션 인증)

## 빠른 시작 
```bash
# 0. ROS2 개발 환경 구축
https://docs.ros.org/en/humble/Installation.html#binary-packages

# 1. 워크스페이스 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. 프로젝트 클론
git clone <this-repo-url> robot
cd ..

# 3. 빌드 및 환경설정
colcon build --symlink-install
source /opt/ros/humble/setup.bash 
source install/setup.bash

# 4. SLAM 실행 (Cartographer)
ros2 launch my_cartographer now_slam_test.launch.py

# 5. 네비게이션 실행 (Nav2)
ros2 launch my_cartographer bringup_ackermann_nav2.launch.py

# 6. 키보드 조작 (Teleop)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 핵심 기능 

- SLAM 기반 지도 생성 및 업로드
    - my_cartographer, map_uploader 패키지를 통해 실시간 맵 생성 및 AWS 업로드
- 자율 주행 및 모터 제어
    - nav2 기반 목표 지점 이동, motor_controller 패키지를 통한 저수준 모터 제어
- 센서 융합 및 보정
    - ydlidar_ros2_driver, ros2_mpu6050_driver로 LiDAR/IMU 데이터 수집
    - my_imu_calibration_pkg, my_imu_corrector_pkg로 캘리브레이션 수행
- 위치 및 상태 전송
    - robot_location_publisher를 통해 로봇 좌표를 서버에 실시간 송신
- 탐색 및 다중 로봇 확장
    - m-explore-ros2로 frontier 기반 탐색 및 다중 로봇 확장성 제공
- 수동 조작
    - teleop_twist_keyboard를 통한 heuristic/manual 제어 지원
- 실시간 로봇 위치 트래킹 
    - WebSocket(STOMP + SockJS)으로 창고 단위 토픽 구독 
- 위험 리포트(Alert)와 처리 이력(Processing)
    - 처리 내역 등록/수정, 타임 스탬프 반영(updatedAt)
- SLAM 맵 버저닝 
    - 로봇이 업로드한 맵을 저장하고 활성 버전을 관리 

## 개발 환경 
**임베디드**
- 로봇 플랫폼: Jetson Orin Nano, Raspberry Pi 5

- 센서: YDLiDAR X4 Pro, MPU6050 IMU, 열화상 카메라, Pi Camera

- 프레임워크:

    - ROS 2 Humble
    - Cartographer (SLAM)
    - Nav2 (Navigation2)

- 빌드/도구: Maven 3.9+, Docker 24+

- OS & 런타임: Ubuntu 22.04 LTS, Python 3.10.11, Java 21, Node 18+

**프론트엔드**
- Node.js 20
- React 19.1.0
- Zustand 5.0.6
- PWA
- Prettier 3.6.2
- VS Code

**백엔드**
- JDK 21.0.7 LTS
- Spring Boot 3.5.4 (Web, Security, WebSocket)
- MyBatis 3.0.3 + MySQL 8.x (Connector/J)
- MQTT (Eclipse Paho 1.2.5)
- AWS SDK for S3 v2.24.12 (presigned URL)
- Jackson (JSR-310)
- Lombok 1.18.30, DevTools
- BouncyCastle (PEM/TLS)
- IntelliJ: IntelliJ IDEA 2023.3.8 (Ultimate Edition)
- Cursor: 1.4.5

**DB**
- MySQL 8.0.37

**Infra**
- Docker 28.3.2 (Docker Engine - Community)
- AWS EC2 22.04.4 LTS
- Jenkins jenkins:lts
- Nginx nginx/1.18.0 (Ubuntu)

## 아키텍처 

## 기술 스택 
- Embedded: Ubuntu 22.04 LTS, ROS2 Humble, Python 3.10.11, Cartographer, Navigation2
- Frontend: JavaScript, React 19.1.0, Zustand 5.0.6, Tailwind 4.1.11
- Backend: Python 3.10.11, Java 21, Spring Boot 3.5.3, Spring Security 6, MyBatis, Lombok, WebSocket(STOMP)
- DB: MySQL 8.0.37
- Storage: AWS S3 (presigned URL)
- Messaging/Broker: MQTT 

## 프로젝트 구조 
```
root/
├── server/                  # Spring Boot 백엔드 (MyBatis)
│   └── src/main/java/kr/kro/areuhot/
│       ├── alert/           # 도메인 패키지(예: alert)
│       │   ├── controller/  │ REST API
│       │   ├── service/     │ 도메인 서비스
│       │   ├── mapper/      │ MyBatis Mapper
│       │   ├── dto/         │ 요청/응답 DTO
│       │   └── model/       │ DB 매핑 모델
│       ├── map/
│       ├── rack/
│       ├── spot/
│       ├── adminauth/
│       ├── warehouse/
│       ├── rackspot/
│       ├── robot/
│       └── common/          # 공통 Util, Error, WebSocket 등
├── frontend/               # 프론트엔드 (React.js, PWA)
│   ├── nginx/              # 배포용 Nginx 설정 파일
│   ├── public/
│   └── src/
│       ├── api/            # 서버와 통신하는 API 모듈
│       │   └── index.js
│       ├── assets/         # 이미지, 아이콘, 폰트 등 정적 리소스
│       ├── components/     # 재사용 가능한 UI 컴포넌트
│       │   ├── common/
│       │   └── realtime/
│       ├── firebase/       # Firebase 설정 및 연동 코드
│       ├── hooks/          # 커스텀 React Hooks
│       ├── mocks/          
│       ├── pages/          # 실제 라우트 페이지 컴포넌트
│       ├── stores/         # 전역 상태 관리
│       ├── utils/          # 공용 유틸리티 함수
│       ├── App.css
│       ├── App.jsx
│       ├── index.css
│       └── main.jsx
└── robot/
    ├── maps/                                           # SLAM을 통한 지도 생성 및 저장
    └── src/
        ├── my_cartographer/                            # SLAM 및 자율주행(nav2 활용)
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── config/                                 # 파라미터, yaml 설정 모음
        │   │   ├── my_robot.lua                        # Cartographer SLAM 주요 설정
        │   │   ├── nav2_params.yaml                    # Navigation2 파라미터 (controller, planner, costmap 등)
        │   │   ├── my_robot_localization.yaml          # Localization 모드용 설정
        │   │   ├── camera.yaml
        │   │   ├── imu.yaml
        │   │   └── ekf.yaml
        │   ├── launch/                                 # 실행용 launch 파일
        │   │   ├── now_slam_test.launch.py             # LiDAR + IMU + Cartographer 통합 실행
        │   │   ├── bringup_ackermann_nav2.launch.py    # Ackermann + Nav2 실행 (map_server 포함)
        │   │   ├── my_robot.launch.py                  # 제자리 Cartographer 실행
        │   │   └── check_imu.launch.py                 # imu 디버깅 노드 실행
        │   ├── urdf/                                   # 로봇 구조 및 TF 정의
        │   │   └── robot.urdf
        │   ├── rviz/                                   # rviz 프로파일
        │   │   ├── cartographer_config.rviz            # Cartographer rviz 설정
        │   │   ├── nav2_slam_view.rviz                 # Cartographer + Navigation rviz 설정
        │   │   └── imu_test.rviz                       # IMU 설정
        │   └── maps/                                   # Cartographer로 생성된 지도 저장소
        │       ├── my_map.pgm
        │       ├── my_map.yaml
        │       └── my_map.pbstream                     # Cartographer pbstream (Localization 용)
        ├── map_uploader/                               # 생성된 지도 AWS 서버로 업로드
        │   ├── setup.py
        │   └── motor_controller/
        │       └── motor_node.py
        ├── motor_controller/                           # 로봇 모터 제어
        │   ├── setup.py
        │   └── map_uploader/
        │       └── map_uploader_node.py
        ├── my_imu_calibration_pkg/                     # IMU 정적 캘리브레이션 데이터 수집
        │   ├── setup.py
        │   └── my_imu_calibration_pkg/
        │       └── imu_calibrator.py
        ├── my_imu_corrector_pkg/                       # IMU 정적 캘리브레이션 보정 적용
        │   ├── setup.py
        │   └── my_imu_calibration_pkg/
        │       └── imu_corrector_node.py
        ├── robot_location_publisher/                   # 로봇 위치를 서버로 전송
        │   ├── setup.py
        │   └── robot_location_publisher/
        │       └── location_publisher.py
        ├── m-explore-ros2/                             # 다중 로봇 탐사 및 frontier 기반 탐색
        ├── ros2_mpu6050_driver/                        # MPU6050 IMU 드라이버
        ├── teleop_twist_keyboard/                      # 키보드로 모터 조작 (Heuristic 제어)
        └── ydlidar_ros2_driver/                        # YDLiDAR 드라이버

```

## 팀원 구성 및 역할 분배 
| 이름 | 역할 | 담당 영역 | 주 사용 스택 | 
|:----:|:----:|-----------|--------------|
| 김준엽 | Embedded | RPi, Jetson 제어·통신 구현 | Python, Paho MQTT, ROS2(Cartographer, Navigation2) |
| 한경훈 | Embedded | 팀장, SLAM, 자율주행 | Python, Paho MQTT, ROS2(Cartographer, Navigation2) |
| 유아름 | Backend | 비즈니스 로직 구현, DB 설계, design, 영상 포트폴리오  | Java, Spring Boot, MyBatis, MySQL, WebSocket |
| 황상하 | Backend | 발표, 맵 데이터 전처리, MQTT, DB | Java, Spring Boot, MyBatis, MySQL, Python, MQTT |
| 남여경 | Frontend | 컴포넌트 기반 페이지 구조 설계·구현, 발표 자료 제작 | JavaScript, React, Zustand |
| 최재명 | Frontend, Infra | 인프라, 전역 상태 관리 구조 설계·구현 | JavaScript, React, Zustand, Docker |

