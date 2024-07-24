#!/bin/bash

gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris -I0" 
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris -I1"
