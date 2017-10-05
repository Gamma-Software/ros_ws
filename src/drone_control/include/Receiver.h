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
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>


//Definition de la voie d'activation du declenchement de la sequence photo de la radio
#define PI 3.141592

enum Shape{
	NONE = 0,
	CIRCLE = 1,
	HEART = 2,
	FIGURE8 = 3,
	CREPE = 4
};


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
	
	//Methods
	void setNodeHandle(ros::NodeHandle _nh);
	bool isFcConnected();
	bool processFigure(Shape shape, double radius, double increment, int nbRotation);
	vector<geometry_msgs::PoseStamped> get3DPath(Shape which, double radius, double increment, int nbTour);

	//Variables state
	bool armed; //Etat arme de l'autopilote
	bool connected; //Etat connecte de l'autopilote
	bool modeReseted;
	string currentMode;
	string currentModeBeforeLand;
	int currentLandingState;
	bool landingStarted;
	bool stopScanBecauseAngle; //Stop le declenchement si le drone est dans une mauvaise posture
	
	//Variable de position GPS
	float latitude;
	float longitude;
	float altitude;
	
	//Variable de position angulaire
	double pitch;
	double roll;
	double yaw;

	double local_x;
	double local_y;
	double local_z;

	int currentChannelsValue[9];
	bool channelHasChangedDuringDrawing;

	Shape currentFigureToDraw;

	ros::ServiceClient land_service;
	ros::Publisher path_pub;
	ros::Publisher control_pub;
	ros::Publisher state_pub;
};

#endif
