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
#include <cmath>
//#include <unistd.h>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;	// to determine the position for turning the robot in an absolute orientation --> in the setDesiredOrientation fn
ros::Subscriber pose_imu_subscriber;
ros::Subscriber goal_pose_subscriber;
turtlesim::Pose turtlesim_pose;
turtlesim::Pose goal_pose;

double latitude_akt = 0;
double longitude_akt = 0;

double max_vel = 0.3;					//maximalwert der Geschwindigkeit in x Richtung "Fahrgeschwindigkeit"
double max_angular_vel = 0.4;				//maximalwert der Rotationsgeschwindigkeit in Rad/s

double factor_vel_lin = 0.2;				//P-Faktor für x Geschwindigkeit Abstand[m] * Fakror => soll v
double factor_vel_angular = 0.3;			//P-Faktor für z Rot-Ges  Winkelabstadt[rad] * Faktor => soll RotGes um z in rad/s

double imu_correction_angle = 0;			//Magnetometer offset nur positive Winkel erlaubt

double latLongDegInMeter = 111.1944;			//entsprechungen von 1Grad Longitude in Metern in Ingolstadt
double latLongDegInMeterEastWest = 74.403; 		//entsprechungen von 1Grad Latitude in Metern in Ingolstadt

bool navGPS = true;					//wenn true fährt der Roboter auf den Daten des GPS-Sensors zur Posotionsbestimmung und Imu als Kompass
							//bei false fährt der Roboter auf der der gefilterten Odometrie der Datenfusion
bool robot = true;					//bei true wird der Roboter gesteuert
							//bei false wird die Turtle in Turtlesim gesteuert

bool original = false;					//bei False werden die Geschwindigkeiten durch den Maximalwert gekappt bei True nicht


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
			pose_subscriber = n.subscribe("/ublox/fix", 10, poseCallbackRobotGPS);				//Gps message des Sensors
			pose_imu_subscriber = n.subscribe("/imu/data", 10, poseCallbackRobotIMU);			//orientierung des Roboters aus Imu
		}else{
			pose_subscriber = n.subscribe("/odom", 10, poseCallbackRobot);					//Daten aus lokaler gefilterter Odometrie
		}
		velocity_publisher = n.advertise<geometry_msgs::Twist>("/navigation/cmd_vel", 1000);			//Schnittstelle zur Vorgabe der Sollgeschwindigkeit des Roboters
	}else{
		pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallbackTurtle);					//Pos der Turtle
		velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);			//soll ges der Turtle
	}
	goal_pose_subscriber = n.subscribe("/turtle1/goal_pose", 10, goalPoseCallback);					//auf dieses topic kann eine neue Zielkoordinate über ein Topic vorgegeben werden Konzept des Fahens/ warten auf Zieleingabe etc nicht implementiert
	ros::Rate loop_rate(20);

	ROS_INFO("\n\n\n *********************************\n");

	int mode= 0;
	cout<<"Wähle Eingabe Modus 1 oder 2 und drücke Enter\n 1: Eingabe in Latitude and Longitude (Dezimalgrad WGS84)\n 2: Eingabe in Meter X und Y \n";
	cin>>mode;

	ros::spinOnce();

	
	
	double x_goal, y_goal;
	
	if(mode == 1){		//eingabemodus lat long
		printf("\naktuelle Position Latitude: %f Longitude: %f\n\n",latitude_akt,longitude_akt);
		cout<<"Eingabe in Latitude and Longitude (Dezimalgrad WGS84): \n";
		cout<<"Latitude goal position: ";
		cin>>y_goal;

		cout<<"Longitude goal position: ";
		cin>>x_goal;
		
	}else if (mode == 2){		//eingabemodus x y 
		printf("\naktuelle Position pos x: %f pos y: %f\n\n",turtlesim_pose.x,turtlesim_pose.y);
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
	moveGoal(1.2);
	cout<<"loop ended"<<endl;
	loop_rate.sleep();	
	

	ros::spin();

	return 0;
}

double longitudeInX (double lon){					//Umrechnung Longitude in meter Erde als Fläche
	double ergX = latLongDegInMeterEastWest * lon * 1000;		//aktuell hardgecodet für benutzung beim 48 latitude
	return ergX;
}

double latitudeInY (double lat){					//Umrechnung Latitude in meter Erde als Fläche
	double ergY = latLongDegInMeter * lat * 1000;
	return ergY;
}


void poseCallbackTurtle(const turtlesim::Pose::ConstPtr & pose_message){		//einlesen der Orientierung der Turtle vgl Tutorial Gotogoal
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

void poseCallbackRobotGPS(const sensor_msgs::NavSatFix & NavSatFix_message){		//einlesen der lat long werte von gps sensor und umrechnung in Meter
	double lat = NavSatFix_message.latitude;
	double lon = NavSatFix_message.longitude;

	latitude_akt = lat;
	longitude_akt = lon;
	
	//double x_meter = latLongDegInMeter * lon * cos(lat*M_PI/180);
	double x_meter = longitudeInX(lon);
	double y_meter = latitudeInY (lat);
	
	//printf("x: %f y: %f\n",x_meter, y_meter);
	
	turtlesim_pose.x = x_meter;
	turtlesim_pose.y = y_meter;
	
}
void poseCallbackRobotIMU(const sensor_msgs::Imu & Imu_message){			//Einlesen der Orientierung aus Imu (Quaternion) Umrechnung in Eulerwinkel weitergebe des Eulerwinkels um z als th des Roboters
	double x = Imu_message.orientation.x;
    	double y = Imu_message.orientation.y;
    	double z = Imu_message.orientation.z;
    	double w = Imu_message.orientation.w;
    	
    	if(isnan(x)){
    		printf("Error: IMU orientation in nan\n");				//Sensor liefert manchmal keine Orientierung => Fehlermeldung
    	}
    	
    	double e1 = atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
    	double e2 = asin(-2.0 * (x*z - w*y));
    	double e3 = atan2(2*(y*z + w*x), w*w - x*x - y*y + z*z);
    	//printf("e1: %f e2: %f e3: %f\n",e1,e2,e3);
    	
    	
    	e1 = e1 + imu_correction_angle;							//imu offset
    	if(e1 > M_PI){									//normierung auf Wertebereich von -Pi bis pi
    		e1 = e1 - 2*M_PI;
    	}

	turtlesim_pose.theta = e1;
}

void poseCallbackRobot(const nav_msgs::Odometry & odometry_message){			//einlesen x y th aus odometry message
	turtlesim_pose.x = odometry_message.pose.pose.position.x;
    	turtlesim_pose.y = odometry_message.pose.pose.position.y;
    	
    	double x = odometry_message.pose.pose.orientation.x;
    	double y = odometry_message.pose.pose.orientation.y;
    	double z = odometry_message.pose.pose.orientation.z;
    	double w = odometry_message.pose.pose.orientation.w;
    	
    	turtlesim_pose.theta = atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
}

void goalPoseCallback(const turtlesim::Pose::ConstPtr & pose_message_goal){		//einlesen des Goal Position
	goal_pose.x=pose_message_goal->x;
	goal_pose.y=pose_message_goal->y;
}



double velocity_calculation(){		
	double vel = factor_vel_lin * getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);  //Regelfunktion des P-Reglers Faktor * Abstand in Meter
	if(original){
		return vel;											//einfach Sollwert durchreichen
	}else{
		if (vel > 0){											//Sollwert auf maximalwert begrenzen
			return (min(fabs(vel),max_vel));
		}else{
			return (-min(fabs(vel),max_vel));
		}
	}
	
}

double angular_vel_calculation(){
	double angular_vel = factor_vel_angular*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta);	//Reglelfunktion der Rotation Winkelabstand * Faktor => Soll RotGes
	if(original){
		return angular_vel;							//wert durchreichen
	}else{
		if(angular_vel > 0){							//Begrenzung auf Maximalwerte
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
		vel_msg.linear.x = velocity_calculation();				//Geschwindigkeitswert in lin.x vorgeben
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		//angular velocity
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = angular_vel_calculation();				//Rotationsgeschwindigkeit in ang.z vorgeben
	
		velocity_publisher.publish(vel_msg);					//Twist mgs publischen
		
		double dist = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);	//Abstand berechnen für Print
		printf("dist: %f pos x: %f pos y: %f goal x: %f goal y: %f\n",dist,turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y);

		ros::spinOnce();
		loop_rate.sleep();

	}while(ros::ok() && getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);		//Knoten bricht ab wenn node gekillt wurde oder der Roboter sich im Zielradius befindet

	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;

	for(int i = 0; i < 3; i++){						//bei Beendigung des Regelalgorithmus werden noch 3 "0 Messages" gesendet damit der Roboter auch wirklich stoppt schein nötig zu sein
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	cout<<"end move goal"<<endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

double getDistance(double x1, double y1, double x2, double y2){		 //Berechnung des Abstands zwischen 2 x-y Paaren
	return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}

