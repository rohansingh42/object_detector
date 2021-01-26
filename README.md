# Cylindrical Object detection from 2D scan

## Install
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone https://github.com/rohansingh42/object_detector.git
cd ..
catkin build
```


## Run
Set laserscan topic in the _config/params.yaml_ file and play recorded rosbag(/live data) along with launching _launch/run.launch_. By default, the launch file runs a demo rosbag, but can be shutdown by setting __enable_rosbag_demo__ argument to false in launch file. The topic __/circleOdom__ publishes detected circle odometry.