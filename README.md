# Mando Autonomous Driving Competition
만도 자율주행 경진대회

## Member Roles

| 이름         | 담당                                         |
| ------------ | :------------------------------------------- |
| 고세람(팀장) | SLAM과 Stanley를 이용한 Path tracking 기능 중 미션구간 인덱스 및 PID 값 조정 |
| 김진우       | Cartographer의 파라미터 값 튜닝 |
| 조영진       | SLAM과 Stanley를 이용한 Path tracking 기능 중 차량의 실제 속도값 측정하여 최적화 작업 진행 |

## Goal
---
![KakaoTalk_20211206_141136870](https://user-images.githubusercontent.com/65532515/144790999-3bffbf42-f2da-48ef-b9c9-e5645188192c.jpg)

### 1. 차선인식
![image](https://user-images.githubusercontent.com/65532515/144791107-56968305-e1d5-4041-84b9-0a0866456d60.png)

- lane은 검정색 바탕에 노란색  또는 흰색으로 20mm의 폭(오차 2mm)를 구성이 되어 있으며 line사이의 폭은 400mm       (오차 5%)이내 구성되어 있다
- 라인 검출 능력을 보기 위하여 일부 오염된 구간이 존재 하거나 끊어진 구간이 존재 할 수 있으며, 라인과 라인 사이에 
- 다른 라인이 일부 존재할 수 있음
- 최소 곡률 반경은 800mm 임

### 2. 교차로 및 신호등 인식
![image](https://user-images.githubusercontent.com/65532515/144791621-958b5759-ca03-496b-959a-7206b62b0c80.png)

- 교차로가 존재하며 인공지능으로 인식하는 신호등을 인식하여 신호의 흐름에 맞도록 주행해야 한다.
  - (신호등은 가로식과 세로식 으로 구성 할 수 있음 , 각 신호등의 크기는 30mm 이내, 신호등은 연습주행에서 뽑기로 선정함)
- 신호드의 위치는 350mm 이하에서 위치 함(이동함)
- 교차로에서는 반드시 정지라인 전에 정지 하여야 함(정지 미션)

### 3. 혼잡 추월 구간
![image](https://user-images.githubusercontent.com/65532515/144791744-8e1c31c4-2e95-4693-9ed4-e3759fd3fbf5.png)
- 2차선으로 구성된 구간에서는  정차된 차량을 추월 주행 해야 함
- 차량의 위치는 랜덤으로 정해짐

### 4. 출발 및 주차
![image](https://user-images.githubusercontent.com/65532515/144977947-c9387738-0b23-4a24-9247-ae8232a192d8.png)
- 주차장에서 출발하여 주차장에 도착함
- 세로 주차 미션

### 5. 장애물 감지 및 도로 표지판
- 주행중 특정 구역에서 사람이 등장 또는 정지 표지판이 나올 수 있음
- 표지판의 위치는 350mm 이하에서 위치 함

### 6. 라이다 주행 구간
![KakaoTalk_20211207_152612522](https://user-images.githubusercontent.com/65532515/144977677-a68b5ac7-a550-4ae0-8f33-c5010a2a3ea8.jpg)
- lane이 없는 구간은 라이다와 odometry로 주행을 해야 함( 라이다를 위해서 벽 또는 cone을 구성할 예정임)
- cone의 3D프린팅을 위한 도면 제공 예정임 / 벽의 높이는 350mm임

## Environment
---
- Ubuntu 18.04
- ROS Melodic
- 1/10 scale model car
- Jetson Nano 4GB
- Arduino Mega2560

## Structure
---
~~~
arduino
  └─ motor_control_2ch
  │   └─ motor_control_2ch.ino
camera_test
  └─ src
  │   └─ csi_camera.py
handsfree_ros_imu
  └─ launch
  │   └─ handsfree_imu.launch
  └─ scripts
  │   └─ hfi_b9_ros.py
sc_mini
  └─ include
  │   └─ sc_mini
  │       └─ sc_mini.h
  └─ launch
  │   └─ sc_mini_test.launch
  └─ src
  │   └─ sc_mini.cpp
  │   └─ serial_port.cpp
xycar_slam
  └─ config
  │   └─ mapping_try1.lua
  │   └─ xycar_localization.lua
  └─ launch
  │   └─ final.launch
  │   └─ localization.launch
  │   └─ online_mapping.launch
  └─ maps
  │   └─ draw_path.py
  │   └─ mando_11_27_2.pbstream
  │   └─ mando_11_27_2.pkl
  │   └─ rosbag_to_map.py
  └─ rviz
  │   └─ localization.rviz
  │   └─ offline_mapping.rviz
  └─ src
  │   └─ main_yolo.py
  │   └─ mando_yolo.py
  │   └─ stanley.py
  │   └─ stanley_follower.py
  │   └─ yolo.py
  └─ urdf
  │   └─ xycar.urdf
  
~~~

## Usage
---
~~~bash
$ roslaunch xycar_slam final.launch
~~~

## Procedure
---
### mapping
![image (2)](https://user-images.githubusercontent.com/65532515/134633108-9ed5957a-f9e4-4f48-b7af-3e2ac3cf81aa.png)
- mapping 전용 lua 파일을 이용하여 주행할 맵을 매핑한다.
### localization
![image](https://user-images.githubusercontent.com/65532515/134635003-8f8fad1a-d4a3-4ae6-8a53-b621d457782a.png)
- 위에서 제작한 맵을 토대로 localization을 진행한다. 
### path tracking & control
![image](https://user-images.githubusercontent.com/65532515/134119319-62f924a7-be56-4271-8923-5a333136f601.png)
![KakaoTalk_20211207_191617935](https://user-images.githubusercontent.com/65532515/145010684-ebfa61f6-5a06-46d8-bd66-c2d325950e13.jpg)
- 맵의 x, y 좌표와 차의 현재 위치(x, y)좌표를 비교하여 heading error와 cross track error(cte) 를 구하고, steering angle 값을 도출하여 reference path대로 따라갈 수 있도록 path planning 진행.
### Stopline
![KakaoTalk_20211207_190655471_02](https://user-images.githubusercontent.com/65532515/145009646-b211f54c-0c41-460b-9c2d-c527660e981f.png)
- localization을 통한 차량의 현재위치를 구해 reference path 상의 정지선 위치에서 정차하도록 구현.
### interrupt
![KakaoTalk_20211207_190655471_03](https://user-images.githubusercontent.com/65532515/145009761-c2244aac-f54a-40f0-b4f6-71ffb0cf849f.png)
- 1. 라이다를 이용한 장애물 회피 주행
- 2. 장애물을 피해가는 reference path를 딴 뒤, stanley method를 이용하여 path tracking
### Lidar Driving
![KakaoTalk_20211207_190655471_04](https://user-images.githubusercontent.com/65532515/145010422-765b48eb-351d-45c0-933e-a1bab12e6611.png)
- 1. 라이다를 이용한 장애물 회피 주행
- 2. 장애물을 피해가는 reference path를 딴 뒤, stanley method를 이용하여 path tracking
### T_parking
![image](https://user-images.githubusercontent.com/65532515/134636638-3afb65bd-5c98-4a9f-81e9-0ddcf431cffb.png)
- AR 태그를 인식해 차량의 yaw값을 구해 그 값에 10.0 만큼 곱한 값을 angle값을 사용하여 주차공간에 주차.


## Limitations & Try
---
### Stanley
- 라이다 주행 구간에서 콘을 치는 문제가 있었음
  - 크기가 작은 콘이 촘촘하게 있어 카토그래퍼가 콘들을 벽으로 오인식하여 localization이 틀어졌을 것이라고 추정.
    - 해결 방안 -> 문제 해결
      - lua 파일의 파라미터 값 중 voxel filter 값을 키워 크기가 작은 콘은 무시하도록 함.
- 차량이 주행 하는 경로가 reference path를 정확히 따라가지 않고, 살짝 옆으로 치우쳐서 주행함.
  - stanley method의 cte term에 들어가는 v(속도)값이 정확하지 않아서 일어난 문제로 추정함.
    - 해결 방안 -> 문제 해결
      - (속도 = 거리 / 시간) 공식을 이용함.
      - 시간 = 코드의 루프를 한번 도는 시간(rate) 
        거리 = cartographer의 tracked_pose 좌표를 이용하여 이전 발행주기의 차량의 (x, y)좌표와 다음 발행주기의 차량의(x, y)좌표를 np.hypot을 이용하여 두 좌표간 거리를 구함.
### Obstacle Avoidance & Rotary Mission
- 라이다의 노이즈값 (0값)이 매우 많아 장애물을 정확히 측정할 수 없는 문제가 있었음.
  - 라이다 측정값 중 0인 값을 필터링하여 해결.

## What I've learned
---
- SLAM을 이용한 자율주행을 경험해보니, 연산량을 줄이는 작업이 굉장히 중요하고, 파라미터 튜닝이 굉장히 중요하다는 점을 배웠다.
- Stanley method의 cte term에 들어가는 속도값 등 각종 파라미터 값을 중요하게 생각하지 않았는데, 이 작은 값 하나 때문에 전체적인 차량의 경로가 틀어질 수 있다는 것을 배움.
# 파라미터 튜닝도 매우 중요하다!
