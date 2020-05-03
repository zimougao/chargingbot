# chargingbot

**Husky + UR5e + Robotiq 85 Gripper in Simulation**

IKFast Plugin, Yolov3 & SSD Real-time Object Detection, Lidar & Visual SLAM.

Tested in Ubuntu 18.04 ROS Melodic. If you use ROS Kinetic, modify *$robotiq_gazebo/src/mimic_joint_plugin.cpp* to suit Gazebo 7


## Installation

Install the ROS controllers
```bash
sudo apt-get install ros*controller*
```
Install dependencies
```bash
sudo apt install ros-melodic-pointcloud-to-laserscan ros-melodic-rosbridge-server
sudo apt-get install ros-melodic-robot-localization
sudo apt-get install ros-melodic-interactive-marker-twist-server
sudo apt-get install ros-melodic-ar-track-alvar
```
Go to your workspace/src
```bash
git clone https://github.com/zimougao/chargingbot.git
cd chargingbot
git clone https://github.com/JenniferBuehler/gazebo-pkgs
sudo apt-get install ros-melodic-object-recognition-msgs
git clone https://github.com/JenniferBuehler/general-message-pkgs.git
cd ../..
catkin_make
source devel/setup.bash
```

## Download the urdf file from Google Drive
Visit Google Drive: https://drive.google.com/drive/u/0/folders/1XtgpTrNwrcQiswtwLbZn0oNSMuTtQXNs  
Go to File: Charging robot materials / Team files / Bingkun / parking_lot / car_with_port  
Download **meshes** file.
```bash
$ roscd husky_ur5_world/
```
Replace **meshes** file with the new one.


## Modify model path to your workspace

Change 2 lines of <uri> paths in test_05.world:

```bash
$ roscd husky_ur5_world/worlds/
$ gedit test_05.world
```
Replace "<uri>/home/zbk/catkin_ws/src/car_with_port/meshes/station01.dae</uri>"

"<uri>/home/zbk/catkin_ws/src/car_with_port/meshes/EVcharger.dae</uri>"

"<uri>/home/zbk/catkin_ws/src/car_with_port/meshes/parkinglot.dae</uri>"

"<uri>/home/zbk/catkin_ws/src/car_with_port/meshes/Mercedes_AMG.dae</uri>"

to "<uri>**your path to husky_ur5_world pkg**/meshes/station01.dae</uri>"

"<uri>**your path to husky_ur5_world pkg**/meshes/EVcharger.dae</uri>"

"<uri>**your path to husky_ur5_world pkg**/meshes/parkinglot.dae</uri>"

"<uri>**your path to husky_ur5_world pkg**/meshes/Mercedes_AMG.dae</uri>"
   
   
## Starting Gazebo Simulation

Open a new terminal. The simulation is launched by:
```bash
$ roslaunch husky_ur5_world husky_artrack_world.launch
```

## Starting MoveIt control

Open a new terminal. Launch the MoveIt control by:
```bash
$ roslaunch ur5ehuskyg_moveit demo.launch
```
You now should see the same posture in the Gazebo as the MoveIt environment.
Set in Rviz the `MotionPlanning/Planning Request/Planning Group` to manipulator to drag the arm around. 
By `Plan and Execute` the arm will move to its new position.

## Control Husky to trace ar track

Open a new terminal. Launch the QR code detector by:
```bash
$ roslaunch husky_ur5_world husky.launch
```
Open a new terminal. Launch the tracker by:
```bash
$ rosrun husky_ur5_world carrot.py
```
You should see the Husky tracing the QR code

## Control your ur5 with robotiq gripper85
Before controlling the arm, please shut off the carrot.py after the robot reaches the point.  
You can send commands to the joints or to the gripper.  
The robot needs several steps to grab the gun.  
Step one: Let the robot arm be prepared
```bash
$ roscd pick_test/scripts
$ chmod +x pick_up_prepere.py cartisan_ur5.py
$ rosrun pick_test pick_up_prepere.py
```
Step two: Shut off the husky.launch and use the camera on the robot arm to detect the tag
```bash
$ roslaunch husky_ur5_world manipulator.launch
```

Step three: Move the gripper to the proper position.
```bash
$ rosrun pick_test cartisan_ur5.py
```
Step four, grap and go.
