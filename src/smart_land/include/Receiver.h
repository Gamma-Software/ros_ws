/**
 * @author Valentin Rudloff +33 625 163 722
 */

#ifndef RECEIVER_H
#define RECEIVER_H

//---------------------------------
//	Inclusions et definitions
//---------------------------------
#include <stdlib.h>
#include <iostream>
#include <vector>
#include "string.h"

//Inclue le librairie pour les convertions d'angles
#include <tf/transform_datatypes.h> 


#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Bool.h"
#include "std_srvs/Trigger.h"

//Inclue les messages type de MavRos
#include "ros/ros.h"
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>


using namespace mavros_msgs;
using namespace tf;
using namespace std;

//@Description Cette classe permet de définir les callbacks du node
class Receiver
{
public:
	//Callbacks
	Receiver();
	void stateCallback(const State::ConstPtr& msg);
	void rcCallback(const RCIn::ConstPtr& msg);
	void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	bool processSmartLanding(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
	
	//Methods
	void setNodeHandle(ros::NodeHandle _nh);
	bool isFcConnected();

	//Variables 
	bool armed;
	bool connected; 
	bool modeReseted;
	string currentMode;
	string currentModeBeforeLand;
	int currentLandingState;
	bool landingStarted;
	bool landingRequested;
	
	//Variable de position angulaire
	double pitch;
	double roll;
	double yaw;

	double local_x;
	double local_y;
	double local_z;
	double local_yaw;

	int currentChannelsValue[9];
	bool channelHasChangedDuringDrawing;

	ros::ServiceClient land_service;
	ros::Publisher path_pub;
	ros::Publisher control_pub;
	ros::Publisher state_pub;
};

#endif
