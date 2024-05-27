Run these in this project's root directory:
```
git clone https://github.com/clydemcqueen/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git
```

Before building the project with colcon build, in workspace directory run: 
```
export GAZEBO_MODEL_PATH=src/open_project/models/:$GAZEBO_MODEL_PATH
export TURTLEBOT3_MODEL=burger
```
Launch the project with:
```
source install/setup.bash; ros2 launch open_project open_project_launch.py
```
