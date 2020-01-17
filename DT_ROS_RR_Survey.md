# Duckiebot ROS and RR Survey
## Introduction
In this survey, we want students to learn and compare robotics middleware: widely used [Robot Operating System](http://wiki.ros.org/) (ROS) and [Robot Raconteur](https://www.robotraconteur.com/) (RR) designed by RPI alumni. Given a duckiebot with Ubuntu Mate (18.04) image, the goal is to use ROS and Robot Raconteur to achieve lane following.
## Duckiebot
[Duckiebot](https://www.duckietown.org/) is a wheeled robot with 2 motors, one Raspberry Pi, a motor HAT and a Pi Camera. You have the control over 2 motor wheels and read image or video from Pi Cam. Please follow Duckiebot Setup Guide before proceeding.
## Robot Raconteur Tutorial
Robot Raconteur is an object oriented Service-Client middleware. An RR service generally runs with a sensor/robot to have communication directly with them. An RR client usually can receive sensor messages from service and call object function to command the robot. Take a look into given webcam examples on https://github.com/robotraconteur/RobotRaconteur_Python_Examples. Try connecting a webcam to your Ubuntu laptop, running SimpleWebcamService.py and SimpleWebcamClient_streaming.py to get a live window streaming your webcam. 
To get a good understanding of Robot Raconteur, we’ll demonstrate how RR works on duckiebot with keyboard teleop. 
### RR Service:
The RR service for duckiebot is to control both motors. Inside `duckiebot/RobotRaconteur/DuckiebotRR-Service-Drive.py`, there is RR robdef called `drive_servicedef`
```
drive_servicedef="""
	#Service to provide sample interface to the Duckiebot Drive
	service experimental.duckiebot
	
	stdver 0.9
	
	object Drive
	    property int8 LEFT_MOTOR_MIN_PWM 
	    property int8 LEFT_MOTOR_MAX_PWM 
	    property int8 RIGHT_MOTOR_MIN_PWM
	    property int8 RIGHT_MOTOR_MAX_PWM 
	    property int8 SPEED_TOLERANCE 
	
	    function void setWheelsSpeed(double v_left, double v_right)
	
	end object
	"""
```
  You could consider this as a python class object declaration with variables and functions, but it’s also necessary to create an actual python class object including those variables/functions or some others that you don’t need on client side. In short, variables and functions inside **robdef** are the ones you have access to on client side. Inside class object **DaguWheelsDriver**, the PWM output is specified here to control the motor speed. Note that the motor doesn’t have a wheel encoder, so the values here doesn’t mean the actual velocity of the wheel speed. 
  At the bottom of the file, which is the main part for Robot Raconteur, we have `with RR.ServerNodeSetup("Drive_Service",2356) as node_setup:`
`Drive_Service` here is the node name, and 2356 is the port for TCP communication. The object is initialized by `obj=DaguWheelsDriver()`
The major difference for Robot Raconteur is that it has security over service. The password is hashed and a username is also required to connect to the service. 
`authdata="cats be7af03a538bf30343a501cb1c8237a0 objectlock"`
Here the username is **cats** and password is **cats111!**. 
To expose the service over network, the service is registered through
```
RRN.RegisterServiceType(drive_servicedef)
RRN.RegisterService("Drive","experimental.duckiebot.Drive",obj,security)
```
So in this case the service name is called `Drive`, and the object type, the actual object and security is exposed together with the service.
`raw_input()` is just a function to hold the service run indefinitely, and if the user wants to terminate the service, just press `Enter` key and the service will shutdown. In order to use Robot Raconteur library as RR service, it’s necessary to import RR library at start:
```
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
```
### RR Client
