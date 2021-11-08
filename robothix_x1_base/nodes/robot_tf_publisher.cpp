#include    <ros/ros.h>
#include    <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle nh;

    ros::Rate tf_rate(100);

    tf::TransformBroadcaster broadcaster;

    while (ros::ok())
    {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.24, 0.0, 0.23)),
                ros::Time::now(),"base_link", "base_laser_front"));
        
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.22, 0.0, 0.685)),
                ros::Time::now(),"base_link", "base_camera_servo"));

        tf_rate.sleep();
    }
    
}