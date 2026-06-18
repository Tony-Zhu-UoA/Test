# ROS Control
This project is used to control the UR5e + Linear rail in Linux system completed by Industrial AI Research Team.

Follow the instruction step by step to turn on and set up the devices, including robot, gripper, camera, ft300s sensor.
## 1. Power on all devices and set up the robot
1.1 Sep up the robot

1.1.1 Power on the robot
   
1.1.2 Change the mode from "Automatic" to "Manual" and the passport of the robot is "1234"
   
1.1.3 Load the program "corerobotcontrol/ros_control.urp", choose "Discard Save" when you turn off the robot, don't make mistakes!!!

1.1.4 Check the Ip address in "Installation-URCaps-External Control", make sure that this IP address is same as the computer“192.168.12.5”.
 
1.1.5 Turn on the robot:chose "Power off", touch "on" and wait, then touch "START"


2. Check the connection of the network cable, gripper, ft300s sensor and camera.

## 2. Clone the project
```bash
# create a catkin workspace
mkdir -p HRC_robot/src && cd HRC_robot/src

# clone the project noetic branch
https://github.com/uoa-iai/corerobotcontrol.git
```

## 3. Dependencies
### For ur_ikfast package
Original from https://github.com/cambel/ur_ikfast

```bash
sudo apt-get install libblas-dev liblapack-dev

git clone https://github.com/cambel/ur_ikfast.git
cd ur_ikfast
pip install -e .
```

### 4. For using Azure Kinect Sensor SDK
Original from https://gist.github.com/madelinegannon/c212dbf24fc42c1f36776342754d81bc

1.Add source using curl

```bash
sudo apt install curl
curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
```

2.Modify `/etc/apt/sources.list`. At the bottom of the file, change from:

```bash
deb https://packages.microsoft.com/ubuntu/18.04/prod bionic main
# deb-src https://packages.microsoft.com/ubuntu/18.04/prod bionic main
```

to:

```bash
deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main
# deb-src [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main
deb [arch=amd64] https://packages.microsoft.com/ubuntu/20.04/prod focal main

```

3.Rerun `sudo apt-get update`

4.Install Kinect Packages

```bash
sudo apt install libk4a1.4=1.4.1
sudo apt install libk4a1.4-dev=1.4.1
sudo apt install k4a-tools=1.4.1
sudo apt install libk4abt1.1-dev
# make sure the version is correct

```

5.Finish Device Setup

[Finish device setup](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#linux-device-setup) by setting up udev rules:

- Copy '[scripts/99-k4a.rules](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules)' into '/etc/udev/rules.d/'.

```bash
sudo cp ~/HRC_robot/src/Azure_Kinect_ROS_Driver/99-k4a.rules /etc/udev/rules.d/99-k4a.rules
```
- Detach and reattach Azure Kinect devices if attached during this process.


6.Verify installation by call it directly in the terminal:
```bash
k4aviewer
```

### 5. For ROS packages

```bash
# install dependencies
sudo apt update -qq
rosdep update
cd /HRC_robot
rosdep install --from-paths src/Universal_Robots_ROS_Driver/ --ignore-src -y
rosdep install --from-paths src/fmauch_universal_robot/ --ignore-src -y
rosdep install --from-paths src/robotiq_85_gripper/ --ignore-src -y
```

## 6. Building the project
```
# default building type is RelWithDebInfo, set in the toplevel CMakeList.txt
catkin build 

# activate the workspace (ie: source it)
echo 'source $HOME/HRC_robot/devel/setup.bash' >> ~/.bashrc 
source ~/.bashrc
```

## 7. Run following commands in sequence to run the pick up demo
Disable firewall to communicate with robot
```bash
sudo ufw disable
```

7.1 If you want to use Top camera
Create a new terminal, launch the robot and camera.
```bash
cd .../src/labcodes/launch
```
```bash
roslunch run_top_camera_and_URrobot_.launch
```
After lunching the robot when no errers occur, touch the "play" and "Robot Program" on the robot terminal. Make sure the Mode is 1 or 2 or 3 or 4 on the robot terminal.

Then bulid a new terminal, launch the ft300sensor and gripper.
```bash
roslunch run_ft300sensor_and_gripper.launch
```

7.2 If you want to use gripper camera
cd .../src/labcodes/launch
```bash
roslunch run_gripper_camera_and_URrobot.launch
```
After lunching the robot when no errers occur, touch the "play" and "Robot Program" on the robot terminal. Make sure the Mode is 1 or 2 or 3 or 4 on the robot terminal.

Then bulid a new terminal, launch the ft300sensor and gripper.
```bash
roslunch run_ft300sensor_and_gripper.launch
```
Bulid a new terminal, launch yolov5 for gripper camera.
```bash
roslunch run_yolov5_gripper_camera.launch
```


## 8. Trouble shooting
8.1 Having trouble with communicate with robot

```bash
# disable firewall to communicate with robot
sudo ufw disable
```

8.2 Having trouble with controlling the gripper

```bash
chmod +x /home/***/***/src/driver/robotiq_85_gripper/robotiq_85_bringup/launch/robotiq_85.launch

# change permission for /tmp/ttyUR
sudo chmod -t /tmp
sudo chmod 777 /tmp/ttyUR
sudo chmod +t /tmp
```

8.3 Having trouble with controlling the Force/Torque sensor

```bash
# change permission for /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB0
```

