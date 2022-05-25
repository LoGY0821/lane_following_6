#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <iostream>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

ros::Publisher cmdVelPub;
ros::Publisher marker_pub;
geometry_msgs::Point current_point;
geometry_msgs::Quaternion current_qua;
geometry_msgs::Pose pose_list[4];  //a pose consisting of a position and orientation in the map frame.

geometry_msgs::Point setPoint(double _x, double _y, double _z);
geometry_msgs::Quaternion setQuaternion(double _angleRan);
void init_goalList();
void shutdown(int sig);
void init_markers(visualization_msgs::Marker *marker);
void activeCb();
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
double computeDistance(geometry_msgs::Point& m_current_point, geometry_msgs::Point& m_goal);
double computerAngle(geometry_msgs::Quaternion & m_current_quaternion, geometry_msgs::Quaternion & m_goal);


void init_goalList()
{
	//How big is the square we want the robot to navigate?
	double square_size = 1.5;

	//Create a list to hold the target quaternions (orientations)
	geometry_msgs::Quaternion quaternions;
	geometry_msgs::Point point;

	point = setPoint(4.742, 5.408, 0.000);
	quaternions = setQuaternion( -1.619 );
	pose_list[0].position = point;
	pose_list[0].orientation = quaternions;

	point = setPoint(0.583, 5.808, 0.000);
	quaternions = setQuaternion( -1.692  );
	pose_list[1].position = point;
	pose_list[1].orientation = quaternions;

	//point = setPoint(6.66478586197, -6.98255062103, 0.000);
	//quaternions = setQuaternion( 0.105 );
	point = setPoint(0.339, 1.766, 0.000);
	quaternions = setQuaternion( 1.570 );
	pose_list[2].position = point;
	pose_list[2].orientation = quaternions;

	//point = setPoint(2.67574381828, -8.69232368469, 0.000);
	//quaternions = setQuaternion( 1.622 );
	point = setPoint(4.406, 1.296, 0.000);
	quaternions = setQuaternion( 1.624  );
	pose_list[3].position = point;
	pose_list[3].orientation = quaternions;
}

geometry_msgs::Point setPoint(double _x, double _y, double _z)
{
	geometry_msgs::Point m_point;
	m_point.x = _x;
	m_point.y = _y;
	m_point.z = _z;
	return m_point;
}

geometry_msgs::Quaternion setQuaternion(double _angleRan)
{
	geometry_msgs::Quaternion m_quaternion;
	m_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, _angleRan);
	return m_quaternion;
}

// Shutdown
void shutdown(int sig)
{
	cmdVelPub.publish(geometry_msgs::Twist());
	ros::Duration(1).sleep(); // sleep for  a second
	ROS_INFO("move_base_square_hit.cpp ended!");
	ros::shutdown();
}

// Init markers
void init_markers(visualization_msgs::Marker *marker)
{
	marker->ns = "waypoints";
	marker->id = 0;
	marker->type = visualization_msgs::Marker::CUBE_LIST;
	marker->action = visualization_msgs::Marker::ADD;
	marker->lifetime = ros::Duration();//0 is forever
	marker->scale.x = 0.2;
	marker->scale.y = 0.2;
	marker->color.r = 1.0;
	marker->color.g = 0.7;
	marker->color.b = 1.0;
	marker->color.a = 1.0;

	marker->header.frame_id = "map";
	marker->header.stamp = ros::Time::now();
}


// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("Goal Received");
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
//	ROS_INFO("Got base_position of Feedback");
	current_point.x = feedback->base_position.pose.position.x;
	current_point.y = feedback->base_position.pose.position.y;
	current_point.z = feedback->base_position.pose.position.z;
	current_qua.x = feedback->base_position.pose.orientation.x;
	current_qua.y = feedback->base_position.pose.orientation.y;
	current_qua.z = feedback->base_position.pose.orientation.z;
	current_qua.w = feedback->base_position.pose.orientation.w;
}

double computeDistance(geometry_msgs::Point& m_current_point, geometry_msgs::Point& m_goal)
{
	double m_distance;
	m_distance = sqrt(pow(fabs(m_goal.x - m_current_point.x), 2) + pow(fabs(m_goal.y - m_current_point.y), 2));
	return m_distance;
}

double computerAngle(geometry_msgs::Quaternion & m_current_quaternion, geometry_msgs::Quaternion & m_goal){
	double m_angle,row2, pitch2, yaw2, row1, pitch1, yaw1;
	tf::Quaternion quat1;
	tf::quaternionMsgToTF(m_goal, quat1);
	tf::Matrix3x3(quat1).getRPY(row1, pitch1, yaw1);
	//ROS_INFO("yaw1 is : [%s]", yaw1);

	tf::Quaternion quat2;
	tf::quaternionMsgToTF(m_current_quaternion, quat2);
	tf::Matrix3x3(quat2).getRPY(row2, pitch2, yaw2);
	cout << "*************************************" << endl;
	cout << "*************************************" << endl;
	cout << "yaw2 is :" << yaw2 << endl;
	m_angle = fabs(yaw1 - yaw2);
	cout << "m_angle is : " << m_angle << endl;
	return m_angle;

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_move_base");
	ros::NodeHandle node;

	ros::Rate loop_rate(10);

	//Subscribe to the move_base action server
	//Client ac("move_base", true);
        Client ac("move_base", true);

	//Define a marker publisher.
	marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_markers", 10);

	signal(SIGINT, shutdown);
	//ROS_INFO("move_base_square.cpp start...");

	//Initialize the list of goal
	init_goalList();

	//for init_markers function
	visualization_msgs::Marker  marker_list;

	//Initialize the visualization markers for RViz
	init_markers(&marker_list);

	//Set a visualization marker at each waypoint
	for (int i = 0; i < 4; i++)
	{
		marker_list.points.push_back(pose_list[i].position);
	}

	//Publisher to manually control the robot (e.g. to stop it, queue_size=5)
	cmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

	ROS_INFO("Waiting for move_base action server...");

	//Wait 60 seconds for the action server to become available
	if (!ac.waitForServer(ros::Duration(60)))
	{
		ROS_INFO("Can't connected to move base server");
		return 1;
	}

	ROS_INFO("Connected to move base server");
	ROS_INFO("Starting navigation test");

	//Initialize a counter to track waypoints
	int count = 0;
	double distance = 0.0;
	double angle = 0.0;

	//Intialize the waypoint goal
	move_base_msgs::MoveBaseGoal goal;

	//Use the map frame to define goal poses
	goal.target_pose.header.frame_id = "map";

	//Set the time stamp to "now"
	goal.target_pose.header.stamp = ros::Time::now();

	//Set the goal pose to the i-th waypoint
	goal.target_pose.pose = pose_list[count];

	//Start the robot moving toward the goal
	ac.sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);
			
	//Cycle through the four waypoints
	while (ros::ok())
	{
		//Update the marker display
		marker_pub.publish(marker_list);

		distance = computeDistance(current_point, goal.target_pose.pose.position);
		angle = computerAngle(current_qua, goal.target_pose.pose.orientation);
		//ROS_INFO("distance = %f", distance);

		if (distance <= 0.2 && angle <= 0.15)
		{
			count++;
			if (4 == count)
			{
				count = 0;
			}

			//Use the map frame to define goal poses
			goal.target_pose.header.frame_id = "map";

			//Set the time stamp to "now"
			goal.target_pose.header.stamp = ros::Time::now();

			//Set the goal pose to the i-th waypoint
			goal.target_pose.pose = pose_list[count];

			ac.sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);
		}

		loop_rate.sleep();
	}

	//ROS_INFO("move_base_square_lx.cpp end...");
	return 0;
}
