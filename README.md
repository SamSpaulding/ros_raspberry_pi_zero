# Introduction
The Raspberry Pi Zero is a small, affordable, and powerfull single board computer.  With ROS Kinetic, we can use this as a convenient I/O expander and remote camera, integrated with the rest of our robot via the Rasbperry Pi Zero USB Network Gadget.  The Raspberry Pi camera is hardware accelerated, enabling efficient VGA video streaming with the ROS Camera interfaces.

The following steps will enable the compilation and installation of ROS Kinetic on a Raspberry Pi Zero, running Raspian Jessie Lite.

Many thanks for the helpful documentation sets and Wiki instructions referenced below.  They provided several missing steps and gaps I was missing.

# Installation

## Installing Raspbian Jessie on Rasberry Pi Zero
To begin, download the last version of the Raspbian OS from [this link](http://www.raspberrypi.org/downloads/)

In our case we downloaded the *Jessie Lite* version at https://downloads.raspberrypi.org/raspbian_lite_latest

Plug the Rasp SD card in your Linux based PC and copy the image.
```bash
$ sudo umount /dev/sdb
$ sudo dd bs=1M if=2016-11-25-jessie-lite.img of=/dev/sdb
```
**NOTE:** you can see other methods in http://www.raspberrypi.org/documentation/installation/installing-images/README.md 

### First boot on your Raspberry Pi Zero
Once you are connecting to your Raspberry you must prepare it.
```bash
$ sudo raspi-config
```
You could:
* Expand filesystem.
* Change your password.
* Set up language and regional settion
* ...

### Increase Swap Space
In order to ensure there is enough swap space to compile ROS, make the following modificationb below, then reboot the Raspberry Pi Zero so the change takes affect.

```bash
sudo vi /etc/dphys-swapfile 
```
and set the variable CONF_SWAPSIZE=1024

## ROS Kinetic on Raspberry Pi Zero
### Setup ROS Repositories
While we are going to build ROS Kinetic from source on our Raspberry Pi Zero, the ROS repository for Jessie contains many architectural independent packages we'll need to run ROS.
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu jessie main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
```
Now update the Debian package index with the new repository:
```bash
$ sudo apt-get update
$ sudo apt-get upgrade
```

### ROS bootstrap dependencies
```bash
$ sudo apt-get install python-setuptools git checkinstall cmake libboost-system-dev libboost-thread-dev
$ sudo easy_install pip
$ sudo pip install -U rosdep rosinstall_generator wstool rosinstall
```

### Initializing rosdep
```bash
$ sudo rosdep init
$ rosdep update
```

### Create a catkin Workspace

In order to build the core packages, you will need a catkin workspace.
```bash
$ mkdir ~/ros_catkin_ws
$ cd ~/ros_catkin_ws
```
Next we will want to fetch the core packages so we can build them. We will use wstool for this.
We will install ROS-Comm (ROS package, build, and communication libraries without GUI tools) and additional packages used for the camera support:
```bash
$ rosinstall_generator ros_comm diagnostics bond_core dynamic_reconfigure nodelet_core rosserial class_loader image_common vision_opencv image_transport_plugins pluginlib --rosdistro kinetic --deps --wet-only --exclude roslisp --tar > kinetic-robot-wet.rosinstall
$ wstool init -j8 src kinetic-robot-wet.rosinstall
```
**NOTE:** If wstool init fails or is interrupted, you can resume the download by running:
```
$ wstool update -j8 -t src
```
### libconsole-bridge-dev
We need install libconsole-bridge-dev for ROS_Comm, as it is not yet available in the Jessie repositories and must be built manually.
The packages can be built from source in a new directory (Also install checkinstall and cmake):
```bash
$ mkdir ~/ros_catkin_ws/external_src
$ cd ~/ros_catkin_ws/external_src
$ git clone https://github.com/ros/console_bridge.git
$ cd console_bridge
$ cmake .
$ sudo checkinstall make install
```
When check-install asks for any changes, the name (2) needs to change from "console-bridge" to "libconsole-bridge-dev" otherwise the rosdep install wont find it.

### Resolving Dependencies with rosdep
The remaining dependencies should be resolved by running rosdep:

```bash
$ cd ~/ros_catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r --os=debian:jessie
```

### Building the catkin Workspace
Once you have completed downloading the packages and have resolved the dependencies, you are ready to build the catkin packages.

**NOTE:** Building time takes about 9 hours on a Raspberry Pi Zero.  Before invoking the build step, you may be interesting in using a program like screen to close your ssh connection and allow the build on the Raspberry Pi to continue uninterrupted.
http://raspi.tv/2012/using-screen-with-raspberry-pi-to-avoid-leaving-ssh-sessions-open

Invoke catkin_make_isolated:
```bash
$ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic
```
Now ROS should be installed! Remember to source the new installation:
```bash
$ echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc
```
Then you must reboot or execute:
```bash
$ source /opt/ros/kinetic/setup.bash
```

## Raspicam node
The following instructions enable the Raspberry Pi camera as a fully functional ROS camera node, using the ROS Kinetic base built above.

### Enable RaspiCam
First of all we must to enable cam in our Raspberry Pi.

To do this just:
```bash
$ sudo raspi-config
```
Activate option -> *5: Enable Camera*

Finish and reboot raspberry.

### Adding ARM libraries for interfacing to Raspberry Pi GPU
The raspicam node software below requires the following GPU interface code be checked out at the location /home/pi/userland, so we download this repository here:
```bash
$ git clone https://github.com/raspberrypi/userland.git /home/pi/userland
```

### Install raspicam node

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/fpasteau/raspicam_node.git raspicam
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

### Running the camera node

```bash
$ roscore &
$ rosrun raspicam raspicam_node &
$ rosservice call /raspicam_node/camera/start_capture
```

### Viewing the camera node

```bash
$ rqt_image_view image:=/raspicam_node/camera/image/compressed
```

# References
* https://github.com/antdroid-hexapod/antdroid/wiki/Installation
* http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi
* http://wiki.ros.org/kinetic/Installation/Source
