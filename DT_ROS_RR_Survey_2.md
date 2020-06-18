# Duckiebot ROS and RR Survey
## Introduction
In this survey, we want students to learn and compare robotics middleware: widely used [Robot Operating System](http://wiki.ros.org/) (ROS) and [Robot Raconteur](https://www.robotraconteur.com/) (RR) designed by RPI alumni. Given a duckiebot with [Ubuntu Mate (18.04)](https://ubuntu-mate.org/blog/ubuntu-mate-bionic-final-release/) image, the goal is to use ROS and Robot Raconteur to achieve joystick control with a little image processing.
On the computer/duckiebot, please follow instructions on [RR website](https://github.com/robotraconteur/robotraconteur/wiki/Download) (Ubuntu Xenial python2 version) and [ROS website](http://wiki.ros.org/melodic/Installation/Ubuntu) (Desktop version) to install both of them.
In order to edit files on duckiebot, you can either use `vim`/`nano` on terminal, or `scp` command to copy the script to duckiebot. Example:
`scp <files on laptop> <duckiebot username>@<duckiebot IP>:<location on duckiebot>`
## Prerequisites
Ubuntu 18.04 

Python2

pygame
`pip install pygame`

### SSH Mounting
```
sudo apt-get install sshfs
cd ~
mkdir duckie
sshfs duckiebot@10.42.0.50:/home/duckiebot duckie
```
This way the duckiebot is mounted on your computer, so you can modify the files more easily.



## Duckiebot
[Duckiebot](https://www.duckietown.org/) is a wheeled robot with 2 motors, one Raspberry Pi, a motor HAT and a Pi Camera. You have the control over 2 motor wheels and read image or video from Pi Cam. To access the duckiebot, use **ssh** command with given *username*, *password* and *IP address*:
```
ssh <username>@<IP>
```
First, clone this repository to the root directory on both the duckiebot and computer
```
cd ~
git clone https://github.com/hehonglu123/Duckiebot_Survey_ROS_RR.git
```

## Robot Raconteur Survey
Robot Raconteur is an object oriented Service-Client middleware. An RR service generally runs with a hardware (e.g. sensors,actuators) attached to a robot/computer to have direct communication between them. An RR client can receive messages that are sent from services and can call object functions in the service to command the robot. In this survey, we’ll first demonstrate how RR works with virtual duckiebot keyboard control. 
### RR Service:
First go to `Duckiebot_Survey_ROS_RR/RobotRaconteur/Example`, there is RR service script `service.py`. Simply run it by `python service.py`, and it will bring up a `pygame` window, with a virtual Duckiebot at upper left corner. Now open `service.py` in an Editor, and see how the RR service actually works. 
In order to use Robot Raconteur library as RR service, it’s necessary to import RR library at start:
```
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
```
Right after that, there's the object definition:
```
minimal_create_interface="""
#Service to provide virtual interface to the Duckiebot Drive
service experimental.minimal_create
stdver 0.9
object create_obj
     function void Drive(single x_vel, single y_vel)
end object
"""
```
In this example, there's only one object called `create_obj`, with a void function called `Drive` which takes in 2 single precision float variables, correspoinding to horizontal and vertical velocity. Now take a look at the python class `create_impl` below. Upon initialization, most part is pygame initialization and also virtual Duckiebot location initialization. The function `Drive` is the actual function that moves the virtual Duckiebot, and note the `Drive` function defined in RR object definition before refers to this function.
Inside main function, the RR service node is created
```
with RR.ServerNodeSetup("experimental.minimal_create", 52222):     
```
at port 52222 (you can choose any port as long as not used by other services). 
And the service type is registered by 
```
RRN.RegisterServiceType(minimal_create_interface)         
```
Now we create a python class object `create_inst=create_impl(0.,0.)`, and register the service:
```
RRN.RegisterService("Create","experimental.minimal_create.create_obj",create_inst)
```
with service name `Create`. Even though `create_inst` is the python class object, the object received on client side only has attributes defined in RR object definition earlier.

### RR Client:
As for the RR client `client.py`, the first step is also to import RR client library:
```
from RobotRaconteur.Client import *
```
Most part of this script is pygame interfaces, so the RR client part is inside main function,
```
url='rr+tcp://localhost:52222/?service=Create'
obj=RRN.ConnectService(url)
```
The client connects to `Create` service at localhost port 52222. If the client is on another machine, simply swap localhost with IP address (or hostname) the servie is on. The username and password have to match with the ones on service side, otherwise you'll get authentification error. Note that username and password are optional, so if the service doesn't have them, just drop those two argument in `ConnectService`.
Then the `obj` object is passed to function `loop()`, which detects the keyboard arrow keys pressed or not. For example, if left arrow key is pressed, then 
```
obj.Drive(-0.5,0)
```
Try running this client on another terminal and play with it. Keep in mind the service always has to start before the client.

### Task 1: Motor Driving
On duckiebot, inside `~/Duckiebot_Survey_ROS_RR/RobotRaconteur/`, there's a script called `Example_Drive.py`. This script can run directly, and makes the motor drive straight for 5 seconds. The motor drivers are located in the same directory, and the task is to fill in `#TODO` section to make this an RR motor drive service. Take a look at the pygame example to get a sense how to start. Start with a service definition, and continue to the service. After that, try create an RR client script on your laptop to drive the duckiebot motor remotely.

### Task 2: Keyboard Control
This task is an extension on the pygame extension on duckiebot. With the motor control service you have from task 1, combine the client and the pygame keyboard control example to make the client can use keyboard to teleop the duckiebot. On your computer, copy the file `client.py` inside `~/Duckiebot_Survey_ROS_RR/RobotRaconteur/Example` to `~/Duckiebot_Survey_ROS_RR/RobotRaconteur`, and start modify it to connect to the service from Task 1.

### Task 3: PiCam Streaming
Given examples for webcam service (`SimpleWebcamService.py`) and client (`SimpleWebcamClient.py`), try runnning them on your computer to get a sense. Then write RR service and client for the Picam on duckiebot, so that on the computer side you can get video streaming from the Picam. Picam python package is already installed, and their API is listed here: https://picamera.readthedocs.io/en/release-1.10/recipes1.html. Take a look at how to capture OpenCV object. The service has to run on the duckiebot side, and the client may be on any device over the network.

### Task 4: Lane Following
Based on the Picam client from Task 3 and motor control client from Task 1, the goal is to integrate them together with a little image processing. You are provided with a skeleton example, and the task is to fill in the `TODO` blocks based on the comments. Make sure the final client is running on the duckiebot to avoid network lagging.

## ROS Survey
The structure of ROS is a little different from Robot Raconteur. First it has the Publisher-Subscriber relationship between different nodes. The example for ROS is turtlesim, a turtle vehicle simulator came with ROS installation. Go to `~/Duckiebot_Survey_ROS_RR/ROS/Example` first, then open up 3 terminals on the computer, with one typed in `$ roscore`, one typed in `$ python subscriber.py`, one typed in `python publisher.py`. 
Each ROS communication requires one and only one `roscore` running. The subscriber brings up the turtlesim environment as well as a ROS **subscriber** subscribing **rostopic** `motor_command`. The publisher is known as the ROS **publisher** that publish the message to the same **rostopic**. 
Open up `publisher.py` and take a look at how it's done. First the ROS libraries and messages are imported:
```
import rospy
from geometry_msgs.msg import Twist
```
The [ROS message](http://wiki.ros.org/msg) are a standardized series of messages that used for communication between publisher and subscriber, and here to publish velocity data, we are using [gemoetry_msgs/Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html). Inside `main`, the ROS node, publisher and node are initialized:
```
pub = rospy.Publisher('motor_command', Twist, queue_size=0)
rospy.init_node('motor_command', anonymous=True)
```
The message type is initialized on line 31:
```
velocity = Twist()
```
After the script reads in user keyboard press, the message is initialized and formed by setting values of each component (`linear` and `angular`). The program keeps running inside the `while` loop, and at each iteration, the message is published to **rostopic** `motor_command` by `pub.publish(velocity)`. 
Now open the script `subscriber.py` in editor; similarly, the ROS library and messages are imported at the top:
```
import rospy
from geometry_msgs.msg  import Twist
```
, and the initialization looks similar except there's a subscriber now:
```
rospy.init_node('motor_control', anonymous=True)    
rospy.Subscriber("motor_command", Twist, callback, queue_size = 1, buff_size=2**24)
```
This subscriber subscribes to the **rostopic** `motor_command` with `Twist`. Everytime a message is subscribed, the `callback()` function will be triggered, so that the virtual duckiebot will move based on the command. At last, `rospy.spin()` on line 45 keeps this script runs indefinitely until interruption.

### Task 1: Motor Driving
The first task will ask you to write a subscriber that subscribes to a **rostopic** with `Twist` type of message and drive the motor accordingly (set linear.x for left motor and linear.y for right motor). First open three terminals `ssh` into the duckiebot, with one terminal running `roscore`. Inside `~/Duckiebot_Survey_ROS_RR/ROS/`, there's a scirpt called `Example_Drive.py`. This script can run directly, and makes the motor drive straight for 5 seconds. The motor drivers are located in the same directory, and the task is to fill in `#TODO` section to make this a ROS subscriber. After that, run the script directly, and in the other terminal, run `$ rostopic pub <topic-name> <topic-type> [data...]` (replace `<topic-name>` with your **rostopic** name and keep pressing `TAB`, then modify `linear.x` and `linear.y`). You can always monitor the message by `rostopic echo <topic-name>`.

Once the duckiebot responds to the publisher, let's try doing this remotely by setting **ROS MASTER** and **ROS IP**. Stop all scripts including `roscore` on Duckiebot. 
To initiate a ROS communication from computer to the Duckiebot, it’s necessary to identify which one is **ROS MASTER** and the device's IP. Usually we set the master to the computer side. This needs to be done on both side by 
`$ export ROS_MASTER_URI=http://<computer's IP>:11311`
, where the <computer's IP> is the laptop’s IP address. Once this is done, you can look up this value by 
`$ echo $ROS_MASTER_URI`
to make sure it’s set. 
The other environment variable needs to be set is **ROS IP**:
`$ export ROS_IP=<device's IP>`
, where the <device's IP> is the IP address of the device your terminal is in (If you ssh into the duckiebot, then this is the IP of duckiebot, otherwise it's the IP of your computer). Once this is done, you can look up this value by 
`$ echo $ROS_IP`
to make sure it’s set. 

Note you need to do this for every terminal opened, and you only need one roscore running in one Master-Slave setup, which should be on the laptop side. This only needs to be done if you need ROS communication among different machines.
So for teleop, we have communication between Duckiebot and computer. On the computer side, make sure all terminals are set with **ROS_MASTER_URI** and **ROS_IP**, and with one running `roscore`. Then run your subscriber on duckiebot side, and try the `$ rostopic pub` command on the computer side, and check if the duckiebot receives the messages or not. 

### Task 2: Keyboard Control
This task is an extension of the turtlesim keyboard control on duckiebot. With the motor subscriber you have from task 1, combine it with the keyboard control example to make the user can use keyboard to teleop the duckiebot. You can copy the publisher in previous example and start from there. Make sure to set the **ROS_MASTER_URI** and **ROS_IP** on all terminals on both sides.

### Task 3: PiCam Streaming
Given examples for webcam publisher and subscriber (`Example_Webcam_Pub.py` and `Example_Webcam_Subscriber.py`), write another ROS publisher and subscriber for the Picam on Duckiebot, so that on the computer side you can get video streaming from the Picam. Picam python package is already installed, and their API is listed here: https://picamera.readthedocs.io/en/release-1.13/api_streams.html. The publisher has to run on the duckiebot side, and the subscriber may be on any device over the network.


### Task 4: Lane Following
Based on the Picam subscriber from Task 3 and motor publisher from Task 1, the goal is to integrate them together with a little image processing. You are provided with a skeleton example, and the task is to fill in the `TODO` blocks based on the comments. Make sure the final code is running on the duckiebot to avoid network lagging.
