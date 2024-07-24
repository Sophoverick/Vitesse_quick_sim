# IQ Simulations

This repo hosts quick simulation for the drone in ROS environment. It contains the two drones, both will be using ardupilot with only one having the camera, the one that shall simulate the drone with mechanism while the other is the target drone.

## Dependencies 

Take a look at these tutorials to setup ardupilot, gazebo and the ardupilot gazebo plugin 

[Installing Ardupilot and MAVProxy](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot.md)

[Installing QGroundControl](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_qgc.md)

[Installing Gazebo and ArduPilot Plugin](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md)

Installing x-term is recommended as it allows the ardupilot sitl interface to run in a terminal that will cleanly close when closing you sitl instance
```
sudo apt install xterm
```
Python ROS Packages for catkin packages:
```
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon
```
## Setup

First make the base catkin directory:
```
mkdir -p ~/catkin_ws/src && cd catkin_ws/src
```
Then we setup mavros and mavlink prior:
```
cd ~/catkin_ws
wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```
Put it in bashrc:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
Source it:
```
source ~/.bashrc
```
Then install the geographic lib dataset:
```
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```
Finally we approach to the actual package:
```
cd ~/catkin_ws/src
git clone https://github.com/Sophoverick/Vitesse_quick_sim.git
```
Now add to gazebo setup path, our sim file location:
```
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc
```
Now we build:
```
cd ~/catkin_ws
catkin build
```
Now we update the variables:
```
source ~/.bashrc
```
## Drone World

This repo contains the file for the simulation:

```
Vitesse.world
```
 

### Running Drone Simulations 

Launch the simulation with the following command.
```
roslaunch iq_sim Vitesse.launch
``` 
Launch the ardupilot instance of first drone by running:
```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris -I0
```
And in another terminal the other one:
```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris -I1
```

For some information, take a look at the corresponding tutorials [here](https://github.com/Intelligent-Quads/iq_tutorials)


## IQ_SIM Models

### drone_with_camera 

Example drone with a forward facing camera. The camera published a `sensor_msgs/Image` ROS msg which can be used to view or do image processing on. 

![drone_with_camera](docs/imgs/drone_with_camera.png)
