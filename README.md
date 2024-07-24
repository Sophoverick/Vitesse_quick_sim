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
