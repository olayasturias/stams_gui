# Stams GUI ROS Package

STAMS GUI is the user inteface used with all the devices from STAMS project.
It will allow easier manoeuvre of the different devices, and convenient visualization of the sensor readings.


## Setting up

You must clone this repository as `tritech_profiler` into your catkin workspace:

```bash
git clone https://github.com/olayasturias/stams_gui
```

## Dependencies

Before proceeding, make sure to install all the dependencies by running:

```bash
rosdep update
rosdep install stams_gui
```

## Compiling

You **must** compile this package before being able to run it. You can do so
by running:

```bash
catkin_make
```

from the root of your workspace.

## RPIM GUI

To run the RPIM package run the node with:

```bash
rosrun stams_gui rpim.py
```


