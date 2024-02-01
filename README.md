# Simulatin data logging with ROS   

 시뮬레이션 (e.g., Gazebo, MuJoCo, ... ) 결과를 저장할 때, 각 timestep 마다 나오는 값(관절 각도, 속도, 토크 등...)을 데이터(topic) 형태로 저장 

 - 시뮬레이션 소프트웨어에서 데이터를 재생하기만 하면, real-time 으로 할 때 움직였던 그대로 로봇이 움직임

 - 모니터링, 디버깅 등에도 유용

 - real-time 시뮬레이션과 녹화를 동시에 하면, 스크린 녹화가 리소스를 잡아먹어서 시뮬레이션에 영향이 갈 수 있음

 - 따라서 데이터를 저장해놓고, 재생을 할 때 스크린 녹화를 하는 것이 효율적 (구도같은 것도 잡기 편함)

 - ROS 와 시뮬레이션 소프트웨어가 연동되어있으면, rosbag을 이용하여 쉽게 데이터를 기록하는 것이 가능함 (시뮬레이션 소프트웨어의 API등을 통하여 직접 데이터를 저장할 수도 있겠지만, 시뮬레이션마다 API가 다 다르므로, 이것을 공부해야하고 복잡할 수 있음. rosbag을 쓰면 software-specific한 지식에 시간을 쏟지 않아도 됨.)

 - 다만 ros 와 시뮬레이션간의 interfacing 이 필요한데, 이것이 잘 안되어있으면 rosbag을 쓰는 게 더 복잡하므로, 본인의 상황에 맞게 사용

 - 필자는 ros와의 interfacing 이 잘 돼있는 Gazebo를 사용하였음

 - ROS1 기준으로 작성하였으나, ROS2에서도 (아마도?) 크게 다르지 않을 것
 

## How to use logged data in Matlab

- 시뮬레이션 할 때 .bag file 저장 (`rosbag record --all`)

- `rosbag_example.m` 파일과 같은 디렉토리로 .bag 파일을 옮김

- Matlab 상에서 `rosbag_example.m` 을 열고 섹션별로 run section 

- `.bag` 에서 topic 별로 추출한 변수를 `.mat` 으로 저장하여, 본인이 작성할 코드에 사용

