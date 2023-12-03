# TurtleBot3 Logo Follower

This ROS Melodic project simulates (in Gazebo environment) or executes in real life a simple logo follower module for TurtleBot3. The control of the robot is performed using the input of its real-time camera where vibrant colors (in order to make a difference from other colors in the environment) are detected. It also enables to record the robot camera with and without the detection bounding rectangle shown below.

All recordings are stored in _recordings_ directory, saving two files:
1. Raw camera view without editions: _raw_view.avi_
2. Camera view with a printed green bounding rectangle of the detection made: _detection.avi_

## Dependencies

`
rospy
std_msgs
message_generation
opencv-python
numpy
catkin
`

## How does it work?

Image processing techniques are performed from the images obtained from the robot camera, and the output is shown in the screen when executing the module. The image is scanned and processed calculating the Center Of Gravity (COG) of the detected colors in order to make decisions:
* Centered COG: straight
* Left COG: turn left
* Right COG: turn right
* No detection: stop

## install dependency
```
sudo apt-get install ros-melodic-cv-bridge
sudo apt-get update
sudo apt-get install libgl1-mesa-glx
export LIBGL_ALWAYS_SOFTWARE=1
sudo apt-get install ros-melodic-turtlebot3-description
sudo apt install python-catkin-tools
```

## edit bashrc
```
sudo nano ~/.bashrc
```

add following command
```
export LIBGL_ALWAYS_SOFTWARE=1
```

## Build

If you have no workspace directory, create one using the next comands in your command line:

```console
user@ubuntu:~$ mkdir your_ws
user@ubuntu:~$ cd your_ws
user@ubuntu:~/your_ws$ mkdir src
user@ubuntu:~/your_ws$ cd src
```

After creating your workspace directory you are ready to go. Clone this directory into your _src_ folder and build it using Catkin building packages. Run the next commands in your terminal:

```console
user@ubuntu:~/your_ws/src$ git clone https://github.com/rhkrdkdms123/turtlebot-logo-tracker.git
user@ubuntu:~/your_ws/src$ cd ..
user@ubuntu:~/your_ws$ catkin build
```

## Launch

Once your _src_ folder is build, the package is ready to use. To simply launch the TurtleBot3 in a real life environment you can run the next command:

```console
user@ubuntu:~/your_ws$ roslaunch turtlebot3_line_follower line_follower.launch
```
If roscore is already running, you need to shut it down before launching.


However, in order to make a more complex launch of the module, some arguments can be added. In this case, the arguments are related to the simulation environment (Gazebo), recording and position/orientation of the TurtleBot3.

```
Arguments:
  * sim: runs Gazebo simulation (default=false)
  * rec: records raw camera view video, detection view is always recorded (default=false)
  * x: X axis position (default=0.025)
  * y: Y axis position (default=0)
  * z: Z axis position (default=0)
  * roll: roll angle orientation (default=0)
  * pitch: pitch angle orientation (default=0)
  * yaw: yaw angle orientation (default=1.570796)
```

Example of command line used to simulate in Gazebo environment and recording the output of the camera:
```console
user@ubuntu:~/your_ws$ roslaunch turtlebot3_line_follower line_follower.launch sim:=true rec:=true
```

## Publish image topic on your turtlebot
Here is an example.
```
cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch vision imshow.launch
```
the launch file "imshow.launch" lauch the node that publish "camera/image" topic.
Make sure the camera connected on turtlebot available.
