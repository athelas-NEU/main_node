# main_node
Main ROS node on the Jetson Nano

# Setup
Follow [these instruction](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS Desktop install on the Jetson Nano.

Install dependencies.
```
pip3 install empy
pip3 install catkin_pkg
```

Create ROS workspace.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

Clone this repo and the pose_estimation repo.
```
cd ~/catkin_ws/src
git clone https://github.com/athelas-NEU/main_node.git
git clone https://github.com/athelas-NEU/pose_estimation.git
```

Build files.
```
cd ~/catkin_ws
catkin_make
```