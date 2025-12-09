# MSCKF VIO ROS2 - 아키텍처 문서

## 프로젝트 구조

이 프로젝트는 명확한 계층 구조로 재구성되었습니다.

### 디렉토리 구조

```
MSCKF_VIO_ROS2/
├── include/msckf_vio/          # 헤더 파일 (테마별 분류)
│   ├── filter/                 # MSCKF 필터 관련
│   │   └── msckf_vio.h
│   ├── image/                  # 이미지 처리 관련
│   │   └── image_processor.h
│   ├── state/                  # 상태 정의
│   │   ├── cam_state.h         # 카메라 상태
│   │   ├── feature.hpp         # 특징점
│   │   └── imu_state.h         # IMU 상태
│   └── utils/                  # 유틸리티
│       ├── math_utils.hpp      # 수학 함수 (quaternion, rotation)
│       └── utils.h             # 일반 유틸리티
│
├── src/
│   ├── core/                   # [향후 확장] 순수 알고리즘 (ROS 독립)
│   └── server/                 # ROS2 노드 구현
│       ├── msckf_vio_node.cpp          # MSCKF 필터 노드
│       ├── image_processor_node.cpp    # 이미지 처리 노드
│       ├── utils.cpp                   # 유틸리티 구현
│       ├── server_msckf_vio.cpp        # MSCKF main 진입점
│       └── server_image_processor.cpp  # Image Processor main 진입점
│
├── config/                     # 설정 파일
├── launch/                     # ROS2 launch 파일
├── msg/                        # ROS2 메시지 정의
└── test/                       # 단위 테스트
```

## 설계 원칙

### 1. 헤더 파일 구조 (include/)

**테마별 분류**로 관련 기능을 그룹화:

- **filter/** - MSCKF 필터 알고리즘
- **image/** - 특징점 추출 및 추적
- **state/** - 시스템 상태 정의 (IMU, Camera, Feature)
- **utils/** - 수학 유틸리티 및 헬퍼 함수

### 2. 소스 파일 구조 (src/)

**역할별 분리**:

#### server/ (ROS2 인터페이스 계층)
현재 모든 구현이 여기 위치:
- ROS2 Node 클래스
- ROS 콜백 함수
- 메시지 변환
- 파라미터 로딩
- 퍼블리셔/서브스크라이버

#### core/ (향후 확장 예정)
ROS 독립적인 순수 알고리즘:
- 칼만 필터 로직
- 특징점 추적 알고리즘
- 상태 추정 알고리즘

## 빌드 구조

### 라이브러리
1. **msckf_vio_lib** - MSCKF 필터 구현
   - `msckf_vio_node.cpp`
   - `utils.cpp`

2. **image_processor_lib** - 이미지 처리 구현
   - `image_processor_node.cpp`
   - `utils.cpp`

### 실행 파일
1. **msckf_vio_node** - MSCKF VIO 서버
   - `server_msckf_vio.cpp` (main)

2. **image_processor_node** - 이미지 처리 서버
   - `server_image_processor.cpp` (main)

## 데이터 흐름

```
카메라 이미지 → ImageProcessor → 특징점 추출 → MsckfVio → 상태 추정 → Odometry 출력
                     ↑                                ↑
                  IMU 데이터 ────────────────────────┘
```

## 향후 개선 계획

### 단계 1: 인터페이스 정의 ✅ 완료
- 테마별 헤더 구조화
- Server/Core 디렉토리 분리

### 단계 2: 점진적 리팩토링 (계획)
1. ROS 독립적인 데이터 타입 정의
2. 핵심 알고리즘을 core/로 점진적 이동
3. Server는 ROS 래퍼 역할만 수행

### 단계 3: 완전한 분리 (장기 목표)
- Core: 순수 C++ 알고리즘
- Server: ROS2 인터페이스
- 재사용성 및 테스트 용이성 향상

## 주요 클래스

### MsckfVio (msckf_vio.h)
Multi-State Constraint Kalman Filter 구현
- IMU 및 특징점 관측치 처리
- 상태 추정 및 업데이트
- 카메라 상태 관리

### ImageProcessor (image_processor.h)
특징점 검출 및 추적
- 스테레오 이미지 처리
- FAST 특징점 검출
- KLT 광류 추적
- RANSAC 아웃라이어 제거

## 빌드 및 실행

```bash
# 빌드
cd ~/catkin_msckf
colcon build --packages-select msckf_vio

# 실행
ros2 launch msckf_vio msckf_vio_euroc.launch.py
```

## 참고 문헌

- Mourikis, A. I., & Roumeliotis, S. I. (2007). A multi-state constraint Kalman filter for vision-aided inertial navigation.
- Original Implementation: https://github.com/KumarRobotics/msckf_vio
