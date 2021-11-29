#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from pickle import FALSE, FLOAT
from typing import get_type_hints
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from Phidget22.Phidget import *
from Phidget22.Devices.DCMotor import *
from Phidget22.Devices.Encoder import *
from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.Devices.TemperatureSensor import *
from Phidget22.Devices.CurrentInput import *
import time


# Constants:
wheel_circumference = 0.508     # m
ticks_per_rev = 5000.0          # Ticks pro Radumdrehung
wheelmount_radius = 0.1683		# m

# global Variables:
sensorticks_left = 0.0
sensorticks_right = 0.0
v_sensor_left = 0.1
v_sensor_right = 0.2
v_setpoint_left = 0.1
v_setpoint_right = 0.1
dutycycle_set_left = 0.1
dutycycle_set_right = 0.1

# #Create your Phidget channels
# dcMotor0 = DCMotor()
# encoder0 = Encoder()
# voltageRatioInput0 = VoltageRatioInput()
# temperatureSensor0 = TemperatureSensor()
# currentInput0 = CurrentInput()
# dcMotor1 = DCMotor()
# encoder1 = Encoder()
# voltageRatioInput1 = VoltageRatioInput()
# temperatureSensor1 = TemperatureSensor()
# currentInput1 = CurrentInput()

# #Declare any event handlers here. These will be called every time the associated event occurs.
# def onPositionChange(self, positionChange, timeChange, indexTriggered):
# 	print("PositionChange: " + str(positionChange))
# 	print("TimeChange: " + str(timeChange))
# 	print("IndexTriggered: " + str(indexTriggered))
# 	print("----------")

# def onVoltageRatioChange(self, voltageRatio):
# 	print("VoltageRatio: " + str(voltageRatio))

# def onTemperatureChange(self, temperature):
# 	print("Temperature: " + str(temperature))

# def onCurrentChange(self, current):
# 	print("Current: " + str(current))

# def init_motorDriver():
# 	print(".                Entering [init_motorDriver]")

# 	#Set addressing parameters to specify which channel to open (if any)
# 	dcMotor0.setHubPort(0)
# 	dcMotor0.setDeviceSerialNumber(635334)
# 	encoder0.setHubPort(0)
# 	encoder0.setDeviceSerialNumber(635334)
# 	voltageRatioInput0.setHubPort(0)
# 	voltageRatioInput0.setDeviceSerialNumber(635334)
# 	temperatureSensor0.setHubPort(0)
# 	temperatureSensor0.setDeviceSerialNumber(635334)
# 	currentInput0.setHubPort(0)
# 	currentInput0.setDeviceSerialNumber(635334)
# 	dcMotor1.setHubPort(1)
# 	dcMotor1.setDeviceSerialNumber(635334)
# 	encoder1.setHubPort(1)
# 	encoder1.setDeviceSerialNumber(635334)
# 	voltageRatioInput1.setHubPort(1)
# 	voltageRatioInput1.setDeviceSerialNumber(635334)
# 	temperatureSensor1.setHubPort(1)
# 	temperatureSensor1.setDeviceSerialNumber(635334)
# 	currentInput1.setHubPort(1)
# 	currentInput1.setDeviceSerialNumber(635334)

# 	#Assign any event handlers you need before calling open so that no events are missed.
# 	encoder0.setOnPositionChangeHandler(onPositionChange)
# 	voltageRatioInput0.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
# 	temperatureSensor0.setOnTemperatureChangeHandler(onTemperatureChange)
# 	currentInput0.setOnCurrentChangeHandler(onCurrentChange)
# 	encoder1.setOnPositionChangeHandler(onPositionChange)
# 	voltageRatioInput1.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
# 	temperatureSensor1.setOnTemperatureChangeHandler(onTemperatureChange)
# 	currentInput1.setOnCurrentChangeHandler(onCurrentChange)

# 	#Open your Phidgets and wait for attachment
# 	dcMotor0.openWaitForAttachment(1000)
# 	encoder0.openWaitForAttachment(1000)
# 	voltageRatioInput0.openWaitForAttachment(1000)
# 	temperatureSensor0.openWaitForAttachment(1000)
# 	currentInput0.openWaitForAttachment(1000)
# 	dcMotor1.openWaitForAttachment(1000)
# 	encoder1.openWaitForAttachment(1000)
# 	voltageRatioInput1.openWaitForAttachment(1000)
# 	temperatureSensor1.openWaitForAttachment(1000)
# 	currentInput1.openWaitForAttachment(1000)

# 	#Activate Fan for "Driver up and running" signaling
# 	dcMotor0.setFanMode(True)
# 	dcMotor1.setFanMode(True)
# 	print(".                Leaving [init_motorDriver]")

# def terminate_motorDriver():
# 	dcMotor0.setTargetVelocity(0.0)
# 	dcMotor1.setTargetVelocity(0.0)
# 	dcMotor0.close()
# 	encoder0.close()
# 	voltageRatioInput0.close()
# 	temperatureSensor0.close()
# 	currentInput0.close()

# 	dcMotor1.close()
# 	encoder1.close()
# 	voltageRatioInput1.close()
# 	temperatureSensor1.close()
# 	currentInput1.close()
# 	print(".            Driver terminated!")

# def motorDriver(vel_left, vel_right):
# 	#Do stuff with your Phidgets here or in your event handlers.
# 	print(".                Entering [motorDriver]")
# 	dcMotor0.setTargetVelocity(vel_left)
# 	dcMotor1.setTargetVelocity(vel_right)
	
# 	# try:
# 	# 	input("Press Enter to Stop")
# 	# except (Exception, KeyboardInterrupt):
# 	# 	pass

# 	# Close your Phidgets once the program is done.
# 	# terminate_motorDriver()

# 	print(".                Leaving [motorDriver]")

def messUpDummy():
	print("Entering Mess-Up - Dummy")
	global v_sensor_left
	global v_sensor_right
	v_sensor_left 	= v_setpoint_left + 0.1
	v_sensor_right 	= v_setpoint_right - 0.1


def ticks_to_vel(sensorticks, delta_t):
	global wheel_circumference
	dist = (sensorticks / ticks_per_rev) * wheel_circumference
	vel = dist/delta_t
	return vel 

def diff_to_twist():
	global wheelmount_radius
	move = Twist()
	move.linear.x = (v_sensor_right + v_sensor_left) / 2					# velocity of platform centerpoint
	move.angular.z = (v_sensor_left - v_sensor_right)/ wheelmount_radius	# angular velocity (omega)
	return move

def driveController():
	global dutycycle_set_left
	global dutycycle_set_right
	global v_setpoint_left
	global v_setpoint_right
	global v_sensor_left
	global v_sensor_right
    
	e_left = v_setpoint_left - v_sensor_left
	e_right = v_setpoint_right - v_sensor_right

	Kp = 0.5    # Proportional adjustment factor
	eTol= 0.1   # Tolerance around setpoint, creating envelope

	if (e_left > eTol or e_left < eTol):
		dutycycle_set_left = dutycycle_set_left + e_left*Kp

	if (e_right > eTol or e_right < eTol):
		dutycycle_set_right = dutycycle_set_right + e_right*Kp

def main():
	print("Entering [main]")
	#variable definitions:

	#Function callups:
	#-----------------------------------------------------------------------------------------------------------------------------
	# motorDriver(0.3,0.1)
	messUpDummy()

	#main logic:
	print("Leaving [main]")


# Subscriber callbacks:
def get_targetVel_left(data):
	global v_setpoint_left
	v_setpoint_left = data.data
    # rospy.loginfo("Node Input for left Motor: %f", v_setpoint_left)

def get_targetVel_right(data):
	global v_setpoint_right
	v_setpoint_right = data.data
    # rospy.loginfo(rospy.get_caller_id() + "Node Input for right Motor: %f", v_setpoint_right)

# Subscribers:
subLeft_vel = rospy.Subscriber("motor/right/set_duty_cycle", Float64, get_targetVel_left)
subRight_vel = rospy.Subscriber("motor/left/set_duty_cycle", Float64, get_targetVel_right)

# Publishers:
pubLeft_ticks = rospy.Publisher('motor/left/sensor/ticks', Float64, queue_size=50)
pubRight_ticks = rospy.Publisher('motor/right/sensor/ticks', Float64, queue_size=50)
pubLeft_vel = rospy.Publisher('motor/left/sensor/is_vel', Float64, queue_size=50)
pubRight_vel = rospy.Publisher('motor/right/sensor/is_vel', Float64, queue_size=50)
pubTwistFeedback = rospy.Publisher('motor/common/sensor/twist', Twist, queue_size=50)
feedback_twist = Twist()

# ROS: typical initialisation, parametrization and program loop
rospy.init_node('drive_feedback', anonymous=False)
rate = rospy.Rate(20) # 20hz

if __name__ == '__main__':
	#------------------------------------------------------------------------------------------------------------------------
	# init_motorDriver()

	try:
		while not rospy.is_shutdown():
			main()
			# Investigation:
			# print(encoder0.getDataInterval)
			# print(encoder0.getEnabled)
			# print(encoder0.getIndexPosition)
			# print(encoder0.getPosition)
			# rospy.loginfo()
			pubLeft_ticks.publish(sensorticks_left)
			pubRight_ticks.publish(sensorticks_right)
			pubLeft_vel.publish(v_sensor_left)
			pubRight_vel.publish(v_sensor_right)
			pubTwistFeedback.publish(feedback_twist)
			rate.sleep()

		
		rospy.spin()								# Avoid python from closing the ROS node while beeing up and executed
		rospy.on_shutdown(print("TERMINATION FUNCTION EXECUTED"))# erminate_motorDriver)	# Terminate hardware functions of native "Phidgets Drivers" driver (Python)
	except rospy.ROSInterruptException:
		pass