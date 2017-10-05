/**
 * @author Valentin Rudloff +33 625 163 722
 */

#ifndef SMART_LAND_H
#define SMART_LAND_H

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
#include "Receiver.h"

#include <mavros_msgs/CommandTOL.h>


//Definition de la voie d'activation du declenchement de la sequence photo de la radio
#define PI 3.141592
#define SMART_LAND_HEIGHT_MIN 2

enum LandShape{
	CIRCLE = 0,
	YAW_ROT = 1
}

using namespace mavros_msgs;
using namespace tf;
using namespace std;

//@Description Cette classe permet de définir les callbacks du node
class SmartLand
{
public:	
	//Variables
	Receiver* receiver;
	vector<geometry_msgs::PoseStamped> poses; //List of 3D positions to finally return
	int currentPose;
	bool landingStarted;
	bool nextPoseReached;
	LandShape currentLandShape;

	SmartLand();
	//Methods
	bool calculate3DPath(double radius, double speedOfDescent, double increment);
	bool processLand();
	bool checkIfCanLand();
};

#endif
