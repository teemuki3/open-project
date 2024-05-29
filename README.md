### Cooperative multi-robot system

Simulation that uses turtlebot and tello_drone to navigate to goal target from a maze.

### Installation
**Make sure that ROS2 Galactic, Gazebo, etc are installed**
1. Create a workspace folder, preferrably something that ends in _ws, and a /src folder inside of it.
```
mkdir -p test_ws/src
cd test_ws/src
```
2. Clone this and two other repositories to the /src, and move back to the ws folder.
```
git clone https://github.com/teemuki3/open-project.git
git clone https://github.com/clydemcqueen/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ..
```
3. Export needed models
```
export GAZEBO_MODEL_PATH=src/open_project/models/:$GAZEBO_MODEL_PATH
export TURTLEBOT3_MODEL=burger
```
4. Source and build the project in the _ws folder
```
source /opt/ros/galactic/setup.bash
colcon build
source install/setup.bash
source /usr/share/gazebo/setup.sh
```
5. Launch the project with:
```
ros2 launch open_project open_project_launch.py
```

### Errors
There might be some ros-galactic packages missing when trying to launch for the first time. If 'LookupError' occurs, check the packages name from the error and install it.

For example with this error: 
```
LookupError: Could not find the resource 'turtlebot3_gazebo' of type 'packages'
```
This command will install the missing package:
```
sudo apt install ros-galactic-turtlebot3-gazebo
```

### Authors
- Teemu Mäkinen
- Lauri Jussinmäki
- Luuka Lindgren
