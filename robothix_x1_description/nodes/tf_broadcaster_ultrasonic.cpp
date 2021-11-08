#include <ros/ros.h>
#include <tf/transform_broadcaster.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_tf_publisher_Us");
    ros::NodeHandle nh;

    ros::Rate tf_rate(100);

    tf::TransformBroadcaster broadcaster;

    while (ros::ok())
    {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, -45, 1), tf::Vector3(0.23, 0.17, 0.065)),
                ros::Time::now(), "base_link", "ultrasonic/FL"));

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.29, 0.0, 0.065)),
                ros::Time::now(), "base_link", "ultrasonic/FM"));

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 45, 1), tf::Vector3(0.20, -0.17, 0.065)),
                ros::Time::now(), "base_link", "ultrasonic/FR"));

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 210), tf::Vector3(-0.21, 0.19, 0.065)),
                ros::Time::now(), "base_link", "ultrasonic/RL"));

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.29, 0.0, 0.065)),
                ros::Time::now(), "base_link", "ultrasonic/RM"));

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 120), tf::Vector3(-0.21, -0.19, 0.065)),
                ros::Time::now(), "base_link", "ultrasonic/RR"));

        tf_rate.sleep();
    }
}
