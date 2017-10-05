/**
 * @brief Code principal du noeud de control du drone
 * @author Valentin Rudloff +33 625 163 722
 */


//---------------------------------
//	Inclusions et definitions
//---------------------------------
//Inclue les librairies standards
#include <stdio.h>
#include <ctime>
#include <iostream>
#include <string>

//Inclue les librairies ROS
#include "ros/ros.h"
#include "ros/time.h"


//Inclue les librairies de messages
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
//#include <geometry_msgs/PoseStamped.h>
//#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/StreamRate.h"

//Inclue la librairie du projet
#include "Receiver.h"
#include "SmartLand.h"


//---------------------------------
//	Declarations des espaces de nom
//---------------------------------
using namespace std;

//---------------------------------
//	Variables
//---------------------------------
Receiver receiver;

//---------------------------------
//	Fonctions Secondaires
//---------------------------------
bool changeStreamRate(ros::NodeHandle nh)
{
	mavros_msgs::StreamRate streamRateMsgs;
	streamRateMsgs.request.stream_id = 0;
	streamRateMsgs.request.message_rate = 10;
	streamRateMsgs.request.on_off = true;
	ros::ServiceClient service = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
	return service.call(streamRateMsgs);
}


//---------------------------------
//	Fonction Principale
//---------------------------------
int main(int argc, char **argv)
{
	//Initialise ROS
	ros::init(argc, argv, "drone_control_node");
	ROS_INFO_STREAM("SMART_LANDING: ROS / Init ROS");
	
	//Connection du node au node ROS master
	ros::NodeHandle nh;
	
	double inc(0.0); nh.param("increment", inc, double(10.0));
	double radius(0.0); nh.param("radius", radius, double(10.0));
	int speedOfDescent(0); nh.param("speed_of_descent", speedOfDescent, int(1));
	
	ROS_INFO("inc %f, radius %f, speed_of_descent %d", inc, radius, speedOfDescent);

	//Initialisation des recepteurs
	receiver.setNodeHandle(nh);
	ros::Subscriber state_sub = nh.subscribe("/mavros/state", 1, &Receiver::stateCallback, &receiver);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1000, &Receiver::localPositionCallback, &receiver);
	ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 1, &Receiver::rcCallback, &receiver);
	ros::ServiceServer smart_land_service = nh.advertiseService("/smart_land/land", &Receiver::processSmartLanding, &receiver);

	//ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 10);
	

	// Attend la connection de l'autopilote
	//Ralentis ROS à 10hz
    	ros::Rate rate(10);
	while(ros::ok())
    	{
	    	if(receiver.isFcConnected())
	    	{
	    		ROS_INFO_STREAM("SMART_LANDING: MAVROS / FC connected");
	    		break;
	    	}

	    	ROS_INFO_STREAM("SMART_LANDING: MAVROS / Wait for FC");
		ros::spinOnce();
		rate.sleep();
	}
    
	//Modifie le parametre de vitesse du stream du drone pour recevoir ses infos
	if(!changeStreamRate(nh)) //Si le service n'a pas fonctionne alors
	{
		//Termine le programme
  		ros::shutdown();
  		return 0;
	}
	
	//Create the smartLand object
	SmartLand smartLand; smartLand.receiver = &receiver;
	ros::Rate rate2(1);
	while(ros::ok())
	{
		if(smartLand.calculate3DPath(10.0, 10.0, 7.2)) //Radius of 10 meters, descend of 10 meters, increment of 7.2 so 50 iteration per revolution
			break;
		rate2.sleep();
		ros::spinOnce();
	}

    	ros::Rate rate3(1);
	while(ros::ok())
	{
		//Retrive params every seconds
		//receiver.getParams(nh);
		if(receiver.landingRequested && smartLand.checkIfCanLand()){
			smartLand.landingStarted = true;
			receiver.landingRequested = false;
		}
		if(smartLand.landingStarted){
			smartLand.processLand();
		}
		rate3.sleep();
		ros::spinOnce();
	}

  	//Ferme proprement ROS
  	ros::shutdown();
	return 0;
}
