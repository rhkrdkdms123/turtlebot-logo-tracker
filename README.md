# TurtleBot3 Logo Follower
`This project refers to John's line follower project.`
This ROS Melodic project executes in real life a simple logo follower module for TurtleBot3. The control of the robot is performed using the input of its real-time camera where logo images are detected. It also enables to record the robot camera with and without the detection bounding rectangle shown below.


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

Image processing techniques are performed from the images obtained from the robot camera, and the output is shown in the screen when executing the module. The image is scanned and processed calculating the Center Of Gravity (COG) of the detected logo in order to make decisions:
* Centered COG: straight
* Left COG: turn left
* Right COG: turn right
* No detection: stop

## install dependency
```
sudo apt-get install ros-melodic-cv-bridge
sudo apt-get update
sudo apt-get install libgl1-mesa-glx
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

To apply the changes, use the source command.
```
source ~/.bashrc
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
user@ubuntu:~/your_ws$ source devel/setup.bash
user@ubuntu:~/your_ws$ roslaunch turtlebot3_line_follower line_follower.launch
```
if launch is sucessed, you can see the image rqt view window which shows the image from rapsicam on turtlebot.


If you want to simulate this project in Gazebo environment, make your own world in the 'world' direcotry, and specify the 'sim' parameter as 'true' when launching the file.
```
roslaunch turtlebot3_line_follower line_follower.launch sim:=true
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

## Raspicam setting
If Raspberry Camera recognition is not possible, execute the following commands.

Let's check if the Pi Camera is properly connected. If both the 'supported' and 'detected' values are 1, the camera connection has been successfully established.
```
vcgencmd get_camera
```

Load the bcm2835-v4l2 module into the kernel using the modprobe command.
```
sudo modprobe bcm2835-v4l2
```
Depending on the Raspberry Pi environment, you may need to run this command at every boot.

And then, check the video*.
```
 ls /dev/video*
```
Use one of the outputs as the video index.
