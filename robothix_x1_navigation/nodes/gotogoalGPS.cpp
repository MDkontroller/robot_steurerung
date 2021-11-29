#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>
#include <stdlib.h>
#include <tf/tf.h>
#include <math.h>
//#include <unistd.h>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;	// to determine the position for turning the robot in an absolute orientation --> in the setDesiredOrientation fn
ros::Subscriber pose_imu_subscriber;
ros::Subscriber goal_pose_subscriber;
turtlesim::Pose turtlesim_pose;
turtlesim::Pose goal_pose;

double max_vel = 0.3;
double max_angular_vel = 0.4;

double factor_vel_lin = 1;
double factor_vel_angular = 0.3;

double latLongDegInMeter = 111.1944;
double latLongDegInMeterEastWest = 74.403; 

bool navGPS = true;
bool original = false;
bool robot = true;


void poseCallbackTurtle(const turtlesim::Pose::ConstPtr & pose_message);
void poseCallbackRobot(const nav_msgs::Odometry & odometry_message);	//Callback fn everytime the turtle pose msg is published over the /turtle1/pose topic.
void poseCallbackRobotGPS(const sensor_msgs::NavSatFix & NavSatFix_message);
void poseCallbackRobotIMU(const sensor_msgs::Imu & Imu_message);
void goalPoseCallback(const turtlesim::Pose::ConstPtr & pose_message_goal);
void moveGoal(double distance_tolerance);	//this will move robot to goal
double getDistance(double x1, double y1, double x2, double y2);

double longitudeInX (double lon);
double latitudeInY (double lat);


int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlesim_cleaner");
	ros::NodeHandle n;
	

	
	if(robot){
		if(navGPS){
			pose_subscriber = n.subscribe("/ublox/fix", 10, poseCallbackRobotGPS);
			pose_imu_subscriber = n.subscribe("/imu/data", 10, poseCallbackRobotIMU);
		}else{
			pose_subscriber = n.subscribe("/odom", 10, poseCallbackRobot);
		}
		velocity_publisher = n.advertise<geometry_msgs::Twist>("/navigation/cmd_vel", 1000);
	}else{
		pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallbackTurtle);	//call poseCallback everytime the turtle pose msg is published over the /turtle1/pose topic.
		velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	}
	goal_pose_subscriber = n.subscribe("/turtle1/goal_pose", 10, goalPoseCallback);
	ros::Rate loop_rate(20);

	ROS_INFO("\n\n\n *********************************\n");

	int mode= 0;
	cout<<"Wähle Eingabe Modus 1 oder 2 und drücke Enter\n 1: Eingabe in Latitude and Longitude (Dezimalgrad WGS84)\n 2: Eingabe in Meter X und Y \n";
	cin>>mode;

	ros::spinOnce();

	printf("\naktuelle Position pos x: %f pos y: %f\n\n",turtlesim_pose.x,turtlesim_pose.y);
	
	double x_goal, y_goal;
	
	if(mode == 1){
		cout<<"Eingabe in Latitude and Longitude (Dezimalgrad WGS84): \n";
		cout<<"Longitude goal position: ";
		cin>>x_goal;
	
		cout<<"Latitude goal position: ";
		cin>>y_goal;
		
	}else if (mode == 2){
		cout<<"Eingabe in Meter X und Y: \n";
		cout<<"x goal position: ";
		cin>>x_goal;
	
		cout<<"y goal position: ";
		cin>>y_goal;
	}else{
		printf("nicht erlaubte eingabe\n");
		return 0;
	}
	
	if(mode == 2){
		goal_pose.x = x_goal;
		goal_pose.y = y_goal;
	}else if(mode == 1){
		goal_pose.x = longitudeInX(x_goal);
		goal_pose.y = latitudeInY (y_goal);
	}
	
	
	cout<<"loop begin"<<endl;
	moveGoal(1);
	cout<<"one while loop"<<endl;
	loop_rate.sleep();	
	

	ros::spin();

	return 0;
}

double longitudeInX (double lon){
	double ergX = latLongDegInMeterEastWest * lon * 1000;
	return ergX;
}

double latitudeInY (double lat){
	double ergY = latLongDegInMeter * lat * 1000;
	return ergY;
}


void poseCallbackTurtle(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

void poseCallbackRobotGPS(const sensor_msgs::NavSatFix & NavSatFix_message){
	double lat = NavSatFix_message.latitude;
	double lon = NavSatFix_message.longitude;
	
	//double x_meter = latLongDegInMeter * lon * cos(lat*M_PI/180);
	double x_meter = longitudeInX(lon);
	double y_meter = latitudeInY (lat);
	
	//printf("x: %f y: %f\n",x_meter, y_meter);
	
	turtlesim_pose.x = x_meter;
	turtlesim_pose.y = y_meter;
	
}
void poseCallbackRobotIMU(const sensor_msgs::Imu & Imu_message){
	double x = Imu_message.orientation.x;
    	double y = Imu_message.orientation.y;
    	double z = Imu_message.orientation.z;
    	double w = Imu_message.orientation.w;
    	
    	double e1 = atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
    	double e2 = asin(-2.0 * (x*z - w*y));
    	double e3 = atan2(2*(y*z + w*x), w*w - x*x - y*y + z*z);
    	//printf("e1: %f e2: %f e3: %f\n",e1,e2,e3);

	turtlesim_pose.theta = e1;
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
		
		double dist = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		printf("dist: %f pos x: %f pos y: %f goal x: %f goal y: %f\n",dist,turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y);

		ros::spinOnce();
		loop_rate.sleep();

	}while(ros::ok() && getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);

	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;

	for(int i = 0; i < 3; i++){
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	cout<<"end move goal"<<endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}

