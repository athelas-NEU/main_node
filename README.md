# main_node
Main ROS node on the Jetson Nano

# Setup
Follow [these instruction](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS Desktop install on the Jetson Nano.

Install dependencies.
```
pip3 install empy
pip3 install catkin_pkg
pip3 install rospkg
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

Make scripts executable.
```
chmod +x ~/catkin_ws/src/main_node/scripts/get_keypoint_client.py
chmod +x ~/catkin_ws/src/pose_estimation/scripts/get_keypoint_server.py
```

Build files.
```
cd ~/catkin_ws
catkin_make
```

# Run 
In separate terminals run:
```
roscore
rosrun pose_estimation get_keypoint_service.py
rosrun main_node main_node_run.py
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```