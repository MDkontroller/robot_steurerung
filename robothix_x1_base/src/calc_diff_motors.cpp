#include    <ros/ros.h>
#include    <std_msgs/Float64.h>
#include    <geometry_msgs/Twist.h>

#include    "robothix_x1_base/calc_diff_motors.h"

namespace DiffController
{

TwistToDiffDrive::TwistToDiffDrive(float wheel_distance, float wheel_diameter,
        float transmission_factor, float motor_direction_left, float motor_direction_right)
{
    _wheel_distance         = wheel_distance;
    _wheel_diameter         = wheel_diameter;
    _trans_factor           = transmission_factor;
    _motor_direction[LEFT]  = motor_direction_left;
    _motor_direction[RIGHT] = motor_direction_right;
}

float TwistToDiffDrive::calcMotorSpeed(Diff_Motor side, float lin_x, float ang_z)
{
    float _motor_speed = _motor_direction[side] * _trans_factor * ( lin_x + _sideFactor[side] * ( ang_z * _wheel_distance / 2.0 )) / ( _wheel_diameter * M_PI );
    return _motor_speed;
}

} // namespace DiffController