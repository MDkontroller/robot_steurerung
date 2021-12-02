#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import time
from pickle import FALSE, FLOAT
from typing import get_type_hints
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from Phidget22.Phidget import *
from Phidget22.Devices.DCMotor import *
from Phidget22.Devices.Encoder import *
from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.Devices.TemperatureSensor import *
from Phidget22.Devices.CurrentInput import *


# Constants:
wheel_circumference = 505.0    		# mm
ticks_per_rev 		= 1200.0        # pulses per revolution (of the engine rotor shaft before gear)
wheelmount_radius 	= 185.0			# mm
trans_factor		= 18.0			# Transmission factor: wheel rpm / engine rotor rpm

# Global Variables:
setpointTwist = Twist()
v_setpoint_left 	= 0.0			# [m/s]
v_setpoint_right 	= 0.0			# [m/s]
dutycycle_set_left 	= 0.0			# [%] 1.0 = 100% / 0.0 = 0%
dutycycle_set_right = 0.0			# [%] 1.0 = 100% / 0.0 = 0%
v_sensor_left 		= 0.0			# [m/s]
v_sensor_right 		= 0.0			# [m/s]

#Create your Phidget channels
dcMotor0 			= DCMotor()
dcMotor1 			= DCMotor()
encoder0 			= Encoder()
encoder1 			= Encoder()
voltageRatioInput0 	= VoltageRatioInput()
voltageRatioInput1 	= VoltageRatioInput()
temperatureSensor0 	= TemperatureSensor()
temperatureSensor1 	= TemperatureSensor()
currentInput0 		= CurrentInput()
currentInput1 		= CurrentInput()



#Declare any event handlers here. These will be called every time the associated event occurs.
def onPositionChange0(self, positionChange, timeChange, indexTriggered):
	global v_sensor_left
	v_sensor_left = ticks_to_vel((-positionChange), timeChange, wheel_circumference, trans_factor, ticks_per_rev)

	# print("PositionChange: " + str(positionChange))
	# print("TimeChange: " + str(timeChange))
	# print("IndexTriggered: " + str(indexTriggered))
	# print("----------")

def onPositionChange1(self, positionChange, timeChange, indexTriggered):
	global v_sensor_right
	v_sensor_right = ticks_to_vel(positionChange, timeChange, wheel_circumference, trans_factor, ticks_per_rev)

def onVoltageRatioChange(self, voltageRatio):
	print("VoltageRatio: " + str(voltageRatio))

def onTemperatureChange(self, temperature):
	print("Temperature: " + str(temperature))

def onCurrentChange(self, current):
	print("Current: " + str(current))

def init_motorDriver():
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
	encoder0.setOnPositionChangeHandler(onPositionChange0)
	voltageRatioInput0.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
	temperatureSensor0.setOnTemperatureChangeHandler(onTemperatureChange)
	currentInput0.setOnCurrentChangeHandler(onCurrentChange)
	encoder1.setOnPositionChangeHandler(onPositionChange1)
	voltageRatioInput1.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
	temperatureSensor1.setOnTemperatureChangeHandler(onTemperatureChange)
	currentInput1.setOnCurrentChangeHandler(onCurrentChange)

	#Open your Phidgets and wait for attachment
	dcMotor0.openWaitForAttachment(500)
	encoder0.openWaitForAttachment(500)
	voltageRatioInput0.openWaitForAttachment(500)
	temperatureSensor0.openWaitForAttachment(500)
	currentInput0.openWaitForAttachment(500)
	dcMotor1.openWaitForAttachment(500)
	encoder1.openWaitForAttachment(500)
	voltageRatioInput1.openWaitForAttachment(500)
	temperatureSensor1.openWaitForAttachment(500)
	currentInput1.openWaitForAttachment(500)

	#Activate Fan for "Driver up and running" signaling
	dcMotor0.setFanMode(True)
	dcMotor1.setFanMode(True)
	encoder0.setDataInterval(100)
	encoder1.setDataInterval(100)

def terminate_motorDriver():
	dcMotor0.setTargetVelocity(0.0)
	dcMotor0.close()
	encoder0.close()
	voltageRatioInput0.close()
	temperatureSensor0.close()
	currentInput0.close()

	dcMotor1.setTargetVelocity(0.0)
	dcMotor1.close()
	encoder1.close()
	voltageRatioInput1.close()
	temperatureSensor1.close()
	currentInput1.close()
	print(".            Driver terminated!")

def limiter(dutycycle_val):
	limited_dutycycle_val = 0.0

	if (dutycycle_val > 1.0):
		limited_dutycycle_val = 1.0
	elif (dutycycle_val < -1.0):
		limited_dutycycle_val = -1.0
	else:
		limited_dutycycle_val = dutycycle_val

	return limited_dutycycle_val
	
# Transformations	
def ticks_to_vel(ticks, delta_t, wheel_circumference, transmissionfactor, ticks_per_rot): # [none], [ms], [mm], [rotor_revolution/wheel_revolution], [1/rotor_revolution]
	ticks_per_time = ticks/delta_t											# [1/ms]
	dist_per_ticks = wheel_circumference/(transmissionfactor*ticks_per_rot)	# [mm/1]

	vel_tangetial  = ticks_per_time * dist_per_ticks						# [mm/ms] = [m/s]						

	return vel_tangetial													# [m/s]

def diff_to_twist(v_left, v_right, wheelmount_radius):	# [m/s], [m/s], [mm]
	move = Twist()

	move.linear.x  = (v_right + v_left) / 2									# velocity of platform centerpoint
	move.angular.z = (v_right - v_left) / (wheelmount_radius*0.002) 		# angular velocity (omega)

	return move

def twist_to_diff(twistmsg, wheelmount_radius): 		# [m/s] resp. [rad/s], [mm]
	global v_setpoint_left
	global v_setpoint_right

	vel_tan_wheel = twistmsg.angular.z * (wheelmount_radius/1000)

	v_setpoint_left 	= twistmsg.linear.x - vel_tan_wheel
	v_setpoint_right	= twistmsg.linear.x + vel_tan_wheel


def driveController():
	global dutycycle_set_left
	global dutycycle_set_right
	global v_setpoint_left
	global v_setpoint_right
	global v_sensor_left
	global v_sensor_right

	# Determination of control aberrations to be corrected for:
	e_left = v_setpoint_left - v_sensor_left
	e_right = v_setpoint_right - v_sensor_right
	print("Motor Controller:   e_left: ", e_left, "e_right: ", e_right)

	# Control parameters:
	Kp 	 = 0.07    	# Proportional adjustment factor
	eTol = 0.0   	# Deadband around setpoint

	# catching control exception: 
	# At a duty cycle of 1.0, the speed of this motor cannot be increased any more. However, there is a possibility that the contol algorithm tries to do so
	# In that case, the other motor needs to be slowed down:
	# if (dutycycle_set_left == 1.0):				
	# 	if (v_sensor_left < v_sensor_right):
	# 		dutycycle_set_right = dutycycle_set_right * ((1 - (v_sensor_right - v_sensor_left)))
	# elif (dutycycle_set_right == 1.0):
	# 	if (v_sensor_right < v_sensor_left):	
	# 		dutycycle_set_left = dutycycle_set_left * ((1 - (v_sensor_left - v_sensor_right)))
	# if (dutycycle_set_left == -1.0):				
	# 	if (v_sensor_left > v_sensor_right):
	# 		dutycycle_set_right = dutycycle_set_right * (((v_sensor_right - v_sensor_left) - 1))
	# elif (dutycycle_set_right == -1.0):
	# 	if (v_sensor_right > v_sensor_left):	
	# 		dutycycle_set_left = dutycycle_set_left * (((v_sensor_left - v_sensor_right) - 1))
	# else:		
	if (abs(e_left) > eTol):
		dutycycle_set_left = dutycycle_set_left + e_left*Kp

	if (abs(e_right) > eTol):
		dutycycle_set_right = dutycycle_set_right + e_right*Kp

	# Duty cycle output limiter (allowed range: -1.0 ... 1.0)
	dutycycle_set_left = limiter(dutycycle_set_left)
	dutycycle_set_right = limiter(dutycycle_set_right)

	# This function does the control system calculations
# and sets output to the duty cycle that the motor needs to run at.

# void PID()
# {
#     # Calculate how far we are from the target
#     errorlast = error;
#     error = distance360(input, feedback);


#     # If the error is within the specified deadband, and the motor is moving slowly enough
#     # Or if the motor's target is a physical limit and that limit is hit
#     # (within deadband margins)
#     if ((Math.Abs(error) <= deadBand && Math.Abs(output) <= (double)vminTxt.Value))
#     {
#         # Stop the motor
#         output = 0;
#         error = 0;
#     }
#     else
#     {
#         # Else, update motor duty cycle with the newest output value
#         # This equation is a simple PID control loop
#         output = ((Kp * error) + (Ki * integral)) + (Kd * derivative);

#         # output = Kp * error;

#         errorTxt.Text = error.ToString();
#     }


#     # Prevent output value from exceeding maximum output specified by user, 
#     # And prevent the duty cycle from falling below the minimum velocity (excluding zero)
#     # The minimum velocity exists because some DC motors with gearboxes will not be able
#     # to overcome the detent torque of the gearbox at low velocities.
#     if (output >= maxOutput)
#         output = maxOutput;
#     else if (output <= -maxOutput)
#         output = -maxOutput;
#     else if (output < (double)vminTxt.Value && output > 0)
#         output = (double)vminTxt.Value;
#     else if (output > ((double)vminTxt.Value)*(-1) && output < 0)
#         output = ((double)vminTxt.Value)*(-1);
#     else
#         integral += (error * dt);

#     # Calculate the derivative for the next iteration
#     derivative = (error - errorlast) / dt;

#     # Record the previous encoder value for the next iteration of the control loop
#     feedbacklast = feedback;

# }


def main():
	global v_setpoint_left
	global v_setpoint_right
	global dutycycle_set_left
	global dutycycle_set_right

	# Main Logic:
	#		driveController()
	dutycycle_set_left 	= v_setpoint_left
	dutycycle_set_right = v_setpoint_right

	# Command motor rotation
	dcMotor0.setTargetVelocity(limiter(dutycycle_set_left))
	dcMotor1.setTargetVelocity(limiter(dutycycle_set_right))


# Subscriber callbacks:
def get_targetVel_left(data):
	global v_setpoint_left
	v_setpoint_left = data.data

def get_targetVel_right(data):
	global v_setpoint_right
	v_setpoint_right = data.data

def get_targetTwist_common(data):
	global setpointTwist
	global wheelmount_radius
	setpointTwist = data
	twist_to_diff(setpointTwist, wheelmount_radius)

# Subscribers:
# 		subLeft_vel = rospy.Subscriber("motor/left/set_duty_cycle", Float64, get_targetVel_left)
# 		subRight_vel = rospy.Subscriber("motor/right/set_duty_cycle", Float64, get_targetVel_right)
subCommon_twist = rospy.Subscriber("twist_mux/cmd_vel", Twist, get_targetTwist_common)

# Publishers:
pubLeft_ticks 	= rospy.Publisher('motor/left/sensor/ticks', Float64, queue_size=50)
pubRight_ticks 	= rospy.Publisher('motor/right/sensor/ticks', Float64, queue_size=50)
pubLeft_vel 	= rospy.Publisher('motor/left/sensor/is_vel', Float64, queue_size=50)
pubRight_vel	= rospy.Publisher('motor/right/sensor/is_vel', Float64, queue_size=50)
pubTwistFeedback = rospy.Publisher('motor/common/sensor/twist', Twist, queue_size=50)
feedback_twist 	= Twist()

# ROS: typical initialisation, parametrization and program loop
rospy.init_node('drive_feedback', anonymous=False)
rate = rospy.Rate(20) 									# 20hz

if __name__ == '__main__':
	init_motorDriver()

	try:
		while not rospy.is_shutdown():
			main()
			# print(encoder0.getPosition)
			# print(encoder1.getPosition)
			pubLeft_vel.publish(v_sensor_left)
			pubRight_vel.publish(v_sensor_right)
			pubTwistFeedback.publish(diff_to_twist(v_sensor_left, v_sensor_right, wheelmount_radius))
			# pubTwistFeedback.publish(diff_to_twist(v_setpoint_left, v_setpoint_right, wheelmount_radius))
			rate.sleep()
		
		rospy.spin()								# Avoid python from closing the ROS node while beeing up and executed
		rospy.on_shutdown(terminate_motorDriver())	# Terminate hardware functions of native "Phidgets Drivers" driver (Python)
	except rospy.ROSInterruptException:
		pass