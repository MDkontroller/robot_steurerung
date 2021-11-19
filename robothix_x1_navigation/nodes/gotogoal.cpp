#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <stdlib.h>
#include <tf/tf.h>
#include <math.h>
using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;	// to determine the position for turning the robot in an absolute orientation --> in the setDesiredOrientation fn
ros::Subscriber goal_pose_subscriber;
turtlesim::Pose turtlesim_pose;
turtlesim::Pose goal_pose;

double max_vel = 0.3;
double max_angular_vel = 0.8;

double factor_vel_lin = 1;
double factor_vel_angular = 4;

bool original = false;
bool robot = false;


void poseCallbackTurtle(const turtlesim::Pose::ConstPtr & pose_message);
void poseCallbackRobot(const nav_msgs::Odometry & odometry_message);	//Callback fn everytime the turtle pose msg is published over the /turtle1/pose topic.
void goalPoseCallback(const turtlesim::Pose::ConstPtr & pose_message_goal);
void moveGoal(double distance_tolerance);	//this will move robot to goal
double getDistance(double x1, double y1, double x2, double y2);


int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlesim_cleaner");
	ros::NodeHandle n;
	

	
	if(robot){
		pose_subscriber = n.subscribe("/odometry/filtered/global", 10, goalPoseCallback);
		velocity_publisher = n.advertise<geometry_msgs::Twist>("/navigation/cmd_vel", 1000);
	}else{
		pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallbackTurtle);	//call poseCallback everytime the turtle pose msg is published over the /turtle1/pose topic.
		velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	}
	goal_pose_subscriber = n.subscribe("/turtle1/goal_pose", 10, goalPoseCallback);
	ros::Rate loop_rate(0.5);

	ROS_INFO("\n\n\n ********START TESTING*********\n");

	
	/*int x, y;
	cout<<"enter x: ";
	cin>>x;
	
	cout<<"enter y: ";
	cin>>y;*/
	
	
	goal_pose.x = 1;
	goal_pose.y = 9;
	//while(true){
		cout<<"loop begin"<<endl;
		moveGoal(0.2);
		cout<<"one while loop"<<endl;
		loop_rate.sleep();	
	//}

	ros::spin();

	return 0;
}


void poseCallbackTurtle(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

void poseCallbackRobot(const nav_msgs::Odometry & odometry_message){
	turtlesim_pose.x = odometry_message.pose.pose.position.x;
    	turtlesim_pose.y = odometry_message.pose.pose.position.y;
    	
    	double x = odometry_message.pose.pose.orientation.x;
    	double y = odometry_message.pose.pose.orientation.y;
    	double z = odometry_message.pose.pose.orientation.z;
    	double w = odometry_message.pose.pose.orientation.w;
    	
    	turtlesim_pose.theta = atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
}

void goalPoseCallback(const turtlesim::Pose::ConstPtr & pose_message_goal){
	goal_pose.x=pose_message_goal->x;
	goal_pose.y=pose_message_goal->y;
}



double velocity_calculation(){
	double vel = factor_vel_lin * getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
	if(original){
		return vel;
	}else{
		if (vel > 0){
			return (min(fabs(vel),max_vel));
		}else{
			return (-min(fabs(vel),max_vel));
		}
		//return max_vel;
	}
	
}

double angular_vel_calculation(){
	double angular_vel = factor_vel_angular*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta);
	if(original){
		return angular_vel;
	}else{
		if(angular_vel > 0){
			return (min(fabs(angular_vel), max_angular_vel));
		}else{
			return (-min(fabs(angular_vel), max_angular_vel));
		}
	}
	
	
}

void moveGoal(double distance_tolerance){
//void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){
	//We implement a Proportional Controller. We need to go from (x,y) to (x',y'). Then, linear velocity v' = K ((x'-x)^2 + (y'-y)^2)^0.5 where K is the constant and ((x'-x)^2 + (y'-y)^2)^0.5 is the Euclidian distance. The steering angle theta = tan^-1(y'-y)/(x'-x) is the angle between these 2 points.
	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(10);
	do{
		//linear velocity 
		vel_msg.linear.x = velocity_calculation();
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		//angular velocity
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = angular_vel_calculation();

		velocity_publisher.publish(vel_msg);
		
		printf("dist: %f\n",getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y));

		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
	cout<<"end move goal"<<endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}

