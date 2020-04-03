# chargingbot

## Installation

Go to your workspace/src
```bash
git clone https://github.com/zimougao/chargingbot.git
cd ..
catkin_make
source devel/setup.bash
```
Install the ROS controllers
```bash
sudo apt-get install ros*controller*
```

## Modify model path to your workspace

Go to your workspace/src
```bash
git clone https://github.com/zimougao/chargingbot.git
cd ..
```

## Starting Gazebo Simulation

The simulation is launched by:
```bash
$ roslaunch husky_ur5_world husky_artrack_world.launch
```

## Starting MoveIt control

Launch the MoveIt control by:
```bash
$ roslaunch ur5ehuskyg_moveit demo.launch
```
You now should see the same posture in the Gazebo as the MoveIt environment.
Set in Rviz the `MotionPlanning/Planning Request/Planning Group` to manipulator to drag the arm around. 
By `Plan and Execute` the arm will move to its new position.


## Control your ur5 with robotiq gripper85

You can send commands to the joints or to the gripper.
The simulation is launched by:
```bash
$ rosrun pick_test pick_test_scripts_pick_up_demo.py
```
