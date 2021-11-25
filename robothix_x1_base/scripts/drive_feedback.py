#!/usr/bin/env python
# license removed for brevity
from pickle import FALSE, FLOAT
from typing import get_type_hints
import geometry_msgs
import rospy
import std_msgs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from Phidget22.Phidget import *
from Phidget22.Devices.DCMotor import *
from Phidget22.Devices.Encoder import *
from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.Devices.TemperatureSensor import *
from Phidget22.Devices.CurrentInput import *
import time

sensorticks_left = 0.0
sensorticks_right = 0.0
wheel_circumference = 0.508     # m
ticks_per_rev = 5000.0          # Ticks pro Radumdrehung
v_sensor_left = 0.1
v_sensor_right = 0.2
v_setpoint_left = 0.3
v_setpoint_right = 0.4
v_setMotor_left = 0.1
v_setMotor_right = 0.1

dcMotor0 = DCMotor()
encoder0 = Encoder()
voltageRatioInput0 = VoltageRatioInput()
temperatureSensor0 = TemperatureSensor()
currentInput0 = CurrentInput()

dcMotor1 = DCMotor()
encoder1 = Encoder()
voltageRatioInput1 = VoltageRatioInput()
temperatureSensor1 = TemperatureSensor()
currentInput1 = CurrentInput()

#Declare any event handlers here. These will be called every time the associated event occurs.
def onPositionChange(self, positionChange, timeChange, indexTriggered):
	print("PositionChange: " + str(positionChange))
	print("TimeChange: " + str(timeChange))
	print("IndexTriggered: " + str(indexTriggered))
	print("----------")

def onVoltageRatioChange(self, voltageRatio):
	print("VoltageRatio: " + str(voltageRatio))

def onTemperatureChange(self, temperature):
	print("Temperature: " + str(temperature))

def onCurrentChange(self, current):
	print("Current: " + str(current))

def motorDriver():
	#Create your Phidget channels



	#Set addressing parameters to specify which channel to open (if any)
	dcMotor0.setHubPort(0)
	dcMotor0.setDeviceSerialNumber(635334)
	encoder0.setHubPort(0)
	encoder0.setDeviceSerialNumber(635334)
	voltageRatioInput0.setHubPort(0)
	voltageRatioInput0.setDeviceSerialNumber(635334)
	temperatureSensor0.setHubPort(0)
	temperatureSensor0.setDeviceSerialNumber(635334)
	currentInput0.setHubPort(0)
	currentInput0.setDeviceSerialNumber(635334)

	dcMotor1.setHubPort(1)
	dcMotor1.setDeviceSerialNumber(635334)
	encoder1.setHubPort(1)
	encoder1.setDeviceSerialNumber(635334)
	voltageRatioInput1.setHubPort(1)
	voltageRatioInput1.setDeviceSerialNumber(635334)
	temperatureSensor1.setHubPort(1)
	temperatureSensor1.setDeviceSerialNumber(635334)
	currentInput1.setHubPort(1)
	currentInput1.setDeviceSerialNumber(635334)


	#Assign any event handlers you need before calling open so that no events are missed.
	encoder0.setOnPositionChangeHandler(onPositionChange)
	voltageRatioInput0.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
	temperatureSensor0.setOnTemperatureChangeHandler(onTemperatureChange)
	currentInput0.setOnCurrentChangeHandler(onCurrentChange)

	encoder1.setOnPositionChangeHandler(onPositionChange)
	voltageRatioInput1.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
	temperatureSensor1.setOnTemperatureChangeHandler(onTemperatureChange)
	currentInput1.setOnCurrentChangeHandler(onCurrentChange)


	#Open your Phidgets and wait for attachment
	dcMotor0.openWaitForAttachment(5000)
	encoder0.openWaitForAttachment(5000)
	voltageRatioInput0.openWaitForAttachment(5000)
	temperatureSensor0.openWaitForAttachment(5000)
	currentInput0.openWaitForAttachment(5000)

	dcMotor1.openWaitForAttachment(5000)
	encoder1.openWaitForAttachment(5000)
	voltageRatioInput1.openWaitForAttachment(5000)
	temperatureSensor1.openWaitForAttachment(5000)
	currentInput1.openWaitForAttachment(5000)


	#Do stuff with your Phidgets here or in your event handlers.
	dcMotor0.setTargetVelocity(v_setpoint_left)
	dcMotor1.setTargetVelocity(v_setpoint_right)

	try:
		input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

	#Close your Phidgets once the program is done.
	terminateDriver()

def terminateDriver():
	dcMotor0.close()
	encoder0.close()
	voltageRatioInput0.close()
	temperatureSensor0.close()
	currentInput0.close()

	dcMotor1.close()
	encoder1.close()
	voltageRatioInput1.close()
	temperatureSensor1.close()
	currentInput1.close()

# def ticks_to_vel(sensorticks):
#     v_sensor = sensorticks /ticks_per_rev
#     return 

# def diff_to_twist():
    # mean_lin_x = 0.0
    # mean_ang_z = 0.0
#     move_cmd = Twist()
#     move_cmd.linear.x = 0.0
#     move_cmd.angular.z = 0.0


def controller():
    global v_setMotor_left
    global v_setMotor_right
    # global v_setpoint_left
    # global v_setpoint_right
    # global v_sensor_left
    # global v_sensor_right
    
    e_left = v_setpoint_left - v_sensor_left
    e_right = v_setpoint_right - v_sensor_right

    Kp = 0.5    # Proportional adjustment factor
    eTol= 0.1   # Tolerance around setpoint, creating envelope

    if (e_left > eTol or e_left < eTol):
        v_setMotor_left = v_setMotor_left + e_left*Kp

    if (e_right > eTol or e_right < eTol):
        v_setMotor_right = v_setMotor_right + e_right*Kp

 
def get_targetVel_left(data):
    v_setpoint_left= data.data
    rospy.loginfo("Node Input for left Motor: %f", v_setpoint_left)

def get_targetVel_right(data):
    v_setpoint_right = data.data
    rospy.loginfo(rospy.get_caller_id() + "Node Input for right Motor: %f", v_setpoint_right)

pubLeft_vel = rospy.Publisher('motorLeft_vel', std_msgs.msg.Float64, queue_size=50)
pubRight_vel = rospy.Publisher('motorRight_vel', std_msgs.msg.Float64, queue_size=50)

subLeft_vel = rospy.Subscriber("motor/right/set_duty_cycle", std_msgs.msg.Float64, get_targetVel_left)
subRight_vel = rospy.Subscriber("motor/left/set_duty_cycle", std_msgs.msg.Float64, get_targetVel_right)

rospy.init_node('drive_feedback', anonymous=False)

if __name__ == '__main__':
    try:        
        rate = rospy.Rate(4) # 20hz
        while not rospy.is_shutdown():
            rospy.loginfo(v_sensor_left)
            rospy.loginfo(v_sensor_right)
            motorDriver()
            controller()
            pubLeft_vel.publish(v_setpoint_left)
            pubRight_vel.publish(v_setpoint_right)
            pubLeft_vel.publish(v_sensor_left)
            pubRight_vel.publish(v_sensor_right)
            rate.sleep()
			

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
		
    except rospy.ROSInterruptException:
        pass



 
    
