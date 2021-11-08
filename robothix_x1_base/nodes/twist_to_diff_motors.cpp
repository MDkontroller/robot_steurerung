#include    <ros/ros.h>
#include    <std_msgs/Float64.h>
#include    <geometry_msgs/Twist.h>

#include    "robothix_x1_base/smoothing_exponential.h"
#include    "robothix_x1_base/calc_diff_motors.h"


DiffController::TwistToDiffDrive  diff_drive(0.37, 0.165, 0.51, 1.0, 1.0);
Filter::SmoothingExponential      smooth_left(0.6);
Filter::SmoothingExponential      smooth_right(0.6);

ros::Publisher _pub_motor_left, _pub_motor_right;


void cb_twist(const geometry_msgs::Twist::ConstPtr& twist)
{
    std_msgs::Float64 _motor_left, _motor_right;
    
    _motor_left.data  = smooth_left.getSmoothedValue(diff_drive.calcMotorSpeed(DiffController::LEFT, twist->linear.x, twist->angular.z));
    _motor_right.data = smooth_right.getSmoothedValue(diff_drive.calcMotorSpeed(DiffController::RIGHT, twist->linear.x, twist->angular.z));

    _pub_motor_left.publish(_motor_left);
    _pub_motor_right.publish(_motor_right);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_to_diff_motors");

    ros::NodeHandle nh;

    ros::Subscriber _sub_twist = nh.subscribe<geometry_msgs::Twist>("twist_mux/cmd_vel", 1, cb_twist);

    _pub_motor_left  = nh.advertise<std_msgs::Float64>("motor/left/set_duty_cycle", 1);
    _pub_motor_right = nh.advertise<std_msgs::Float64>("motor/right/set_duty_cycle", 1);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}