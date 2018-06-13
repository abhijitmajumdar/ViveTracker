# ViveTracker
A ROS package to use the HTC VIVE Tracker and Lighthouse for high speed pose estimation. This solution offers an economical solution for nearly precise pose and orientation, without the need of calculated positioning of several cameras in a large area. This module was tested on an Odroid XU4 to run a Kobuki robot, without the need for any kind of communication, obtaining a computed pose estimation rate of ~300Hz.

![ViveTrackerDemoChair](/Images/ViveTrackerDemoChair.gif?raw=true "Learning")

## Usage
#### Clone the repositories
```sh
git clone --recurse-submodules https://github.com/abhijitmajumdar/ViveTracker.git ~/ViveTracker
cd ~/ViveTracker
```
#### Compile libsurvive
It is advised to follow instructions from the original repository https://github.com/cnlohr/libsurvive and also use it for troubleshooting. A summary of the instructions performed here are:
```sh
cd ~/ViveTracker/libsurvive
cp useful_files/81-vive.rules to /etc/udev/rules.d/
make
```
Might need a reboot here before connecting the tracker and running make

#### Compiling ROS package
```sh
cd ~/ViveTracker
source vive_tracker_ros/source_this.sh
./vive_tracker_ros/run_to_build.sh
```
Sourcing the file asks for root password to access permissions for USB

#### Running the package
Publishes position and orientations with TF, after calibration
```sh
roslaunch vive_tracker_ros_package vive_tracker_ros.launch
```
Uses the position from the tacker to keep the Kobuki at its initialized position
```sh
roslaunch vive_tracker_ros_package kobuki_keep_ground.launch
```

If ROS does not find the package, source the `source_this.sh` file again, or you could add it to `.bashrc` to do this automatically and not have to do this every time:
```sh
echo "source ~/ViveTracker/vive_tracker_ros/source_this.sh" >> ~/.bashrc
```

## Details
This package is meant to be used with individual robots, running as a standalone module. We aimed at locally processing localization data for a robot, to avoid the need for (wireless) communication (hence avoiding issues related to it like delays), and being self sufficient in obtaining its own position.

A Lighthouse can be mounted at any raised platform without restrictions on the orientation, except that the field of view of the Lighthouse should enclose the robot(s). Each robot (Kobuki) that uses this package is intended to have a Tracker mounted making sure there are as little obstruction as possible (though the Trackers have a wide sphere of coverage ~270 degrees), connected to Odroid XU4 on the Kobuki over USB. The package is individually compiled and run on each robot to provide positioning data, which can then be used for localization. The `kobuki_keep_ground` launch file uses the processed pose estimation to maintain the initial position that it was started at.

This can then be extended to perform formation control with several robots by publishing to each robot the desired destination position over a communication channel.
