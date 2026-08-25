# Biped Bike Robot 자율주행 실행 및 이해 가이드

이 문서는 처음 프로젝트를 실행하는 사람도 현재 시스템의 목적과 명령어의
의미를 이해하고, Gazebo에서 저장 지도를 이용한 목표점 자율주행을 실행할 수
있도록 정리한 안내서이다.

Notion에서 사용하려면 이 Markdown 파일을 Notion의 `Import → Markdown & CSV`로
가져오면 된다.

---

## 1. 현재 프로젝트의 목적

로봇을 바이크 형태로 변신시킨 뒤 다음 작업을 수행하는 것이 현재 목표이다.

1. Gazebo 미로에서 로봇과 라이다를 시뮬레이션한다.
2. 미리 저장한 지도를 불러온다.
3. 지도에서 로봇의 현재 위치를 사용한다.
4. RViz에서 목표 위치와 도착 방향을 지정한다.
5. Nav2가 벽을 통과하지 않는 경로를 계산한다.
6. 로봇이 라이다로 충돌을 감시하면서 목표까지 이동한다.

현재 구현은 **Gazebo 시뮬레이션용**이다. 실물 Raspberry Pi, OpenCR,
OpenRB-150 및 실제 Dynamixel 모터 연결은 이후 별도 검증이 필요하다.

---

## 2. 전체 구조

```text
RViz의 Nav2 Goal
        ↓
Nav2 BT Navigator
        ↓
전역 경로 계획기 (A*)
        ↓
Regulated Pure Pursuit 경로 추종기
        ↓
/cmd_vel (직진속도, 회전속도)
        ↓
cmd_vel_to_wheels.py
        ↓
/wheel_velocity_controller/commands
        ↓
Gazebo 좌우 바퀴
```

센서와 위치 정보는 반대 방향으로 전달된다.

```text
Gazebo 로봇 실제 위치 ─→ /odom ─→ Nav2
Gazebo 라이다 ─────────→ /scan ─→ 비용지도와 충돌 감시
저장 지도 ─────────────→ /map  ─→ 전역 경로 계획
```

---

## 3. 현재 구현 상태

### 완료된 항목

- 축소된 지그재그 미로 실행
- 로봇의 바이크 자세 변신
- 2D 라이다 `/scan` 출력
- 저장 지도 `maze_half` 불러오기
- Gazebo 실제 위치 기반 `/odom`
- `map → odom → base_footprint → base_link` TF 연결
- Nav2 목표점 한 개 자율주행
- 실제 비대칭 로봇 외곽선 기반 충돌 검사
- `/cmd_vel`을 좌우 바퀴 속도로 변환
- 명령이 끊기면 0.5초 후 바퀴 자동 정지

### 다음에 구현할 항목

- 고정 경유점들을 자동 실행하는 `fixed_maze_route.py`
- 완성된 전체 미로 지도
- 실제 라이다와 IMU 연결
- 실제 바퀴 엔코더와 IMU 센서 융합
- Raspberry Pi에서 OpenCR/OpenRB-150 제어
- 실물 속도·가속도·비상 정지 검증

---

## 4. 주요 파일의 역할

| 파일 | 역할 |
|---|---|
| `worlds/simple_maze.sdf` | Gazebo 미로의 바닥과 벽 정의 |
| `launch/gazebo_maze.launch.py` | 미로와 로봇을 함께 실행하고 초기 위치를 보정 |
| `launch/gazebo.launch.py` | Gazebo, 로봇, 컨트롤러, 라이다 브리지 실행 |
| `scripts/bike_teleop.py` | 사람 키보드 조종이 아니라, 로봇을 바이크 자세로 변신시키는 궤적 발행 |
| `maps/maze_half.yaml` | 저장 지도의 해상도, 원점, 이미지 파일 정보 |
| `maps/maze_half.pgm` | 저장된 점유격자 지도 이미지 |
| `launch/nav2_saved_map.launch.py` | 저장 지도와 Nav2를 함께 실행 |
| `launch/nav2_slam_navigation.launch.py` | Nav2 외곽선, 속도, 경로 추종기 설정 |
| `scripts/cmd_vel_to_wheels.py` | `/cmd_vel`을 좌우 바퀴 각속도로 변환 |
| `scripts/ground_truth_attitude.py` | Gazebo 실제 위치와 기울기를 `/odom` 및 TF에 반영 |
| `scripts/wheel_odometry.py` | 바퀴 회전량 기반 오도메트리 실험용. 현재 저장 지도 시뮬레이션에서는 사용하지 않음 |

---

## 5. 처음부터 실행하는 방법

모든 명령은 Ubuntu 노트북에서 실행한다. 각 번호의 실행 명령은 별도 터미널에서
실행한다.

### 5.1 코드 변경 후 한 번만 빌드

```bash
cd ~/biped_bike_ws
colcon build --packages-select biped_bike_runtime
```

`colcon build`는 `src` 안의 코드와 설정을 ROS 2가 실행할 수 있는 `install`
공간에 반영한다. 코드가 바뀌지 않았다면 매번 실행할 필요는 없다.

### 5.2 터미널 1: Gazebo 미로 실행

```bash
bike
ros2 launch biped_bike_runtime gazebo_maze.launch.py
```

- `bike`: ROS 2 Jazzy, ROS_DOMAIN_ID, 현재 워크스페이스 환경을 활성화한다.
- `ros2 launch`: 여러 ROS 노드를 런치 파일 정의에 따라 함께 실행한다.
- `gazebo_maze.launch.py`: 축소 미로, 로봇, 라이다, 바퀴 컨트롤러를 실행한다.

Gazebo에서 미로와 로봇이 보일 때까지 기다린다.

### 5.3 터미널 2: 바이크 자세로 변신

```bash
bike
ros2 run biped_bike_runtime bike_teleop.py
```

`ros2 run`은 패키지 안의 실행 파일 하나를 실행한다. 이 스크립트는 7단계 관절
궤적을 한 번 전송한 뒤 종료된다. 두 바퀴가 바닥에 닿고 로봇 자세가 안정될
때까지 기다린다.

### 5.4 터미널 3: 저장 지도와 Nav2 실행

```bash
bike
ros2 launch biped_bike_runtime nav2_saved_map.launch.py
```

이 명령은 다음 기능을 실행한다.

- `map_server`: `maze_half.yaml`과 `maze_half.pgm`을 `/map`으로 발행
- `map_to_odom`: 시뮬레이션 지도와 Gazebo 오도메트리 좌표를 일치시킴
- `controller_server`: 계산된 경로를 실제 속도 명령으로 변환
- `planner_server`: 시작점에서 목표점까지 벽을 피하는 경로 계산
- `bt_navigator`: 계획, 주행, 실패 복구 순서를 관리
- `collision_monitor`: 라이다로 충돌 가능성을 검사

다음 로그가 나오면 Nav2 준비가 끝난 것이다.

```text
Managed nodes are active
```

현재 시뮬레이션은 Gazebo 실제 위치를 사용하므로 RViz Navigation 2 패널에
`Localization: inactive`가 표시되는 것이 정상이다. 실물에서는 AMCL 또는 센서
융합 위치추정을 사용해야 한다.

### 5.5 터미널 4: RViz 실행

```bash
bike
rviz2
```

RViz에서 다음 항목을 설정한다.

#### Global Options

```text
Fixed Frame: map
```

#### Map 추가

```text
Topic: /map
Reliability Policy: Reliable
Durability Policy: Transient Local
```

`No map received`가 나오면 대부분 `Durability Policy`가 `Volatile`로 되어 있는
것이 원인이다.

#### RobotModel 추가

```text
Description Topic: /robot_description
Reliability Policy: Reliable
Durability Policy: Transient Local
```

#### LaserScan 추가

```text
Topic: /scan
```

#### Navigation 2 패널 추가

```text
Panels → Add New Panel → nav2_rviz_plugins/Navigation 2
```

#### Nav2 Goal 도구 추가

```text
상단 + 버튼 → nav2_rviz_plugins/GoalTool
```

일반 `2D Goal Pose`가 아니라 `Nav2 Goal`을 사용해야 한다.

---

## 6. 목표 위치를 지정하는 방법

RViz의 화살표 전체가 목표 자세이다.

- 화살표가 시작되는 점: 로봇이 도착할 위치
- 화살표가 향하는 방향: 도착 후 로봇이 바라볼 방향

사용 순서:

1. 상단 `Nav2 Goal`을 선택한다.
2. 지도에서 흰색 통로 중앙을 마우스 왼쪽 버튼으로 누른다.
3. 버튼을 누른 채 원하는 최종 방향으로 조금 드래그한다.
4. 버튼을 놓으면 목표가 전송된다.

처음에는 현재 로봇과 같은 통로의 0.5~1 m 앞을 선택한다. 정상 주행을 확인한
후 코너 너머의 연결된 통로에 목표를 지정한다.

목표가 접수되면 Navigation 2 패널이 다음처럼 변한다.

```text
Feedback: active
Distance remaining: 0보다 큰 값
```

---

## 7. 정지 방법

### 일반 정지

RViz의 Navigation 2 패널에서 `Cancel`을 누른다.

### 긴급 정지

Nav2를 실행한 터미널에서 다음 키를 누른다.

```text
Ctrl + C
```

`cmd_vel_to_wheels.py`는 새 명령이 0.5초 동안 들어오지 않으면 좌우 바퀴에
`[0.0, 0.0]`을 발행한다.

Gazebo 실행 터미널의 `Ctrl + C`는 시뮬레이션 전체를 종료한다.

---

## 8. 현재 속도 설정

현재 값은 **Gazebo 시험 전용**이다.

```text
목표 직진속도: 0.50 m/s
목표 제자리 회전속도: 1.50 rad/s
바퀴 최대 각속도: 20 rad/s
회전 가속도: 3.0 rad/s²
```

바퀴 반지름은 약 `0.0309258 m`이며 직선속도 관계는 다음과 같다.

```text
직선속도 = 바퀴 반지름 × 바퀴 각속도
```

경로가 고정되어 있어도 바퀴 한계, 가속도, 제동거리, 라이다 주기 및 충돌
가능성은 사라지지 않는다. 실물에서는 현재 Gazebo 값을 그대로 사용하면 안 된다.

---

## 9. 현재 로봇 외곽선

바이크 자세에서 측정하고 4 cm 안전 여유를 추가한 외곽선이다.

```text
앞:  +0.2535 m
뒤:  -0.3510 m
왼쪽: +0.0776 m
오른쪽: -0.2305 m
```

Nav2에 적용된 다각형:

```text
[[0.2535, 0.0776],
 [0.2535, -0.2305],
 [-0.3510, -0.2305],
 [-0.3510, 0.0776]]
```

로봇 기준점이 외곽선 중앙에 있지 않으므로 단순 원형 로봇으로 계산하면 뒤쪽이나
오른쪽이 벽에 부딪힐 수 있다.

---

## 10. 중요한 토픽

| 토픽 | 메시지 | 의미 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 라이다 거리값 |
| `/map` | `nav_msgs/OccupancyGrid` | 저장된 2D 지도 |
| `/odom` | `nav_msgs/Odometry` | 로봇 위치와 속도 |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2가 계산한 원본 속도 |
| `/cmd_vel_smoothed` | `geometry_msgs/Twist` | 속도 완화기를 통과한 명령 |
| `/cmd_vel` | `geometry_msgs/Twist` | 충돌 감시 후 최종 속도 |
| `/wheel_velocity_controller/commands` | `std_msgs/Float64MultiArray` | 좌우 바퀴 각속도 |
| `/joint_states` | `sensor_msgs/JointState` | 모든 관절의 위치와 속도 |

속도가 이상할 때는 다음 명령으로 단계별 값을 비교한다.

```bash
ros2 topic echo --once /cmd_vel_nav
```

```bash
ros2 topic echo --once /cmd_vel
```

- `/cmd_vel_nav`부터 0에 가까움: 경로 추종기 또는 비용지도 문제
- `/cmd_vel_nav`는 크지만 `/cmd_vel`이 0: 충돌 감시기가 정지시킴
- `/cmd_vel`은 크지만 바퀴가 안 움직임: 바퀴 변환 또는 컨트롤러 문제

---

## 11. TF 좌표 구조

```text
map
 └─ odom
     └─ base_footprint
         └─ base_link
             └─ lidar_link 및 모든 관절 링크
```

- `map`: 저장 지도의 고정 좌표
- `odom`: Gazebo 실제 평면 위치
- `base_footprint`: 바닥 위 로봇 기준점
- `base_link`: 높이와 기울기를 포함한 본체 기준점
- `lidar_link`: 라이다 기준점

연결 확인:

```bash
ros2 run tf2_ros tf2_echo map base_footprint --ros-args -p use_sim_time:=true
```

Gazebo 실제 위치 확인:

```bash
ros2 topic echo --once /gazebo/ground_truth_odom --no-arr
```

현재 저장 지도 시뮬레이션에서는 두 위치의 `x`, `y`, `yaw`가 같아야 한다.

---

## 12. 자주 발생하는 문제

### `No map received`

RViz Map의 QoS를 다음처럼 설정한다.

```text
Reliability: Reliable
Durability: Transient Local
```

### 지도 벽과 라이다 점이 어긋남

목표를 보내지 말고 `map → base_footprint`와 Gazebo 실제 위치를 비교한다.

```bash
ros2 run tf2_ros tf2_echo map base_footprint --ros-args -p use_sim_time:=true
```

```bash
ros2 topic echo --once /gazebo/ground_truth_odom --no-arr
```

### `Invalid frame ID`가 처음 한 번 표시됨

TF 노드가 아직 발견되지 않은 순간적인 메시지일 수 있다. 이후 좌표가 반복해서
출력되면 정상이다.

### `Navigation: active`, `Localization: inactive`

현재 Gazebo 저장 지도 모드에서는 정상이다. AMCL 대신 Gazebo 실제 위치와
`map → odom` 고정 변환을 사용한다.

### 목표가 active인데 움직이지 않음

먼저 원본 속도를 확인한다.

```bash
ros2 topic echo --once /cmd_vel_nav
```

그다음 최종 속도를 확인한다.

```bash
ros2 topic echo --once /cmd_vel
```

### Nav2와 텔레옵을 동시에 실행함

두 노드가 동시에 `/cmd_vel`을 발행하면 명령이 충돌할 수 있다. 저장 지도
자율주행에서는 `teleop_twist_keyboard`와 `slam_toolbox`를 실행하지 않는다.

---

## 13. 고정 경로 주행의 다음 단계

목표와 미로가 항상 같다면 다음 경유점들을 미리 저장할 수 있다.

```text
시작점
→ 직선 통로 중앙
→ 코너 진입점
→ 코너 통과점
→ 다음 직선 통로 중앙
→ 최종 목표
```

시험 방법:

1. Navigation 2 패널에서 `Waypoint / Nav Through Poses Mode`를 선택한다.
2. `Nav2 Goal`로 경유점을 순서대로 추가한다.
3. `Start Nav Through Poses`로 실행한다.
4. 경로가 정상이라면 `Save WPs`로 저장한다.
5. 저장된 좌표를 `fixed_maze_route.py`에서 자동 실행하도록 연결한다.

고정 경유점은 잘못된 통로 선택과 불필요한 경로 탐색을 줄여주지만, 충돌 검사와
코너 감속은 계속 유지해야 한다.

---

## 14. 저장 지도 다시 만들기

현재 지도는 반쪽만 작성된 `maze_half`이다. 전체 지도를 새로 만들 때만
`slam_toolbox`와 키보드 텔레옵을 사용한다.

SLAM:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

수동 주행:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.18 -p turn:=0.8
```

지도 저장:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/biped_bike_ws/src/biped_bike_runtime/maps/maze_half \
  --ros-args -p save_map_timeout:=15.0
```

---

## 15. 실물 전환 시 달라지는 부분

Gazebo 전용 구조:

```text
Gazebo 실제 위치 → /odom
map → odom 고정 변환
```

실물 구조는 다음처럼 바뀌어야 한다.

```text
바퀴 엔코더 + IMU → robot_localization EKF → odom
저장 지도 + 실제 라이다 → AMCL → map → odom
Nav2 /cmd_vel → Raspberry Pi → OpenCR → Dynamixel 바퀴
```

실물에서는 반드시 별도로 확인해야 한다.

- `/dev/serial/by-id`를 사용한 OpenCR/OpenRB 고정 구분
- `dialout` 그룹과 시리얼 포트 권한
- 실제 모터 방향과 ID
- Dynamixel 속도 단위와 감속비
- 하드웨어 비상 정지
- 저속부터 단계적으로 속도 증가
- 엔코더/IMU 오도메트리 정확도
- 라이다 설치 위치 TF

현재 `0.50 m/s`, `20 rad/s`는 실물에 바로 적용할 값이 아니다.

---

## 16. 종료 순서

1. Navigation 2 패널에서 `Cancel`
2. Nav2 터미널에서 `Ctrl + C`
3. RViz 터미널에서 `Ctrl + C`
4. Gazebo 터미널에서 `Ctrl + C`

종료 후 남은 노드 확인:

```bash
ros2 node list
```

아무 노드도 나오지 않으면 전체 종료가 완료된 것이다.

---

## 참고 문서

- [Nav2 Getting Started](https://docs.nav2.org/getting_started/index.html)
- [Navigate Through Poses](https://docs.nav2.org/behavior_trees/trees/nav_through_poses_recovery.html)
- [Nav2 Simple Commander API](https://docs.nav2.org/commander_api/index.html)
- [Nav2 Route Tool](https://docs.nav2.org/tutorials/docs/route_server_tools/navigation2_route_tool.html)

