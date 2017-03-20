# INSTALL
The Grow Unit Control (GUC) system utilizes the ROS (http://ros.org) for its underlying framework. All mechanism for interacting with the hardware clients, sequences and state machines are written on top of that.

## Install the ROS workspace

#### 1.
For installing the ROS, follow the documentation [here](http://wiki.ros.org/ROS/Installation).
(for Raspberry Pi follow [this](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi))
Make sure that the command `source /opt/ros/kinetic/setup.bash` has been executed (preferrably placed in the bottom of `.bashrc`).

Add the extra package:

```
sudo apt-get install ros-kinetic-robot-upstart
```

#### 2.
Download the GUC's GIT repository to an appropriate location, like the user's home directory:

```
cd ~
git clone https://github.com/Nextfood/grow-unit-control.git
```
#### 3.
Prepare the ROS workspace:

```
cd ~/grow-unit-control/ros-workspace/src
catkin_init_workspace
```

#### 4.
Build the packages:

```
cd ~/grow-unit-control/ros-workspace
catkin_make
```
If all goes well, run the command `source . ~/grow-unit-control/ros-workspace/devel/setup.bash`. Preferrably add it to the bottom of `.bashrc` too.
 
#### 5.
The normal user does not have access to the TTY devices (USB serial port) on Ubuntu. Change the permission of the devices with this command:

```
sudo echo 'KERNEL=="ttyUSB[0-9]*",NAME="tts/USB%n",SYMLINK+="%k",GROUP="uucp",MODE="0666"' > /etc/udev/rules.d/50-ttyusb.rules
```
For this to take effect, either reconnect all the USB devices or reboot.

##### Alternatively:

Adding user to the `dialout` group so the user has access to the serial ports:

```
 sudo usermod -a -G dialout <your-login-name>
```
Log out and log in again.

User the command `id` to verify the user has been added to the `dialout` group.

#### 6.
Add the ROS system to start up during the bootup of Linux:

```
cd ~/grow-unit-control/ros-workspace
rosrun robot_upstart install nextfood_ros/launch/nextfood_ros.launch 
sudo systemctl daemon-reload && sudo systemctl start nextfood
```

