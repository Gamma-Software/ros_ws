/**
 * @author Valentin Rudloff +33 625 163 722
 */



//---------------------------------
//	Inclusions et definitions
//---------------------------------
#include "Receiver.h"



/**
     * Contructeur
     *
     * @param void
     * @return void
     */
Receiver::Receiver()
{
	armed = false;
	connected = false;
	modeReseted = true;
	latitude = 0;
	longitude = 0;
	altitude = 0;
	pitch = 0;
	roll = 0;
	yaw = 0;
	local_x = 0;
	local_y = 0;
	local_z = 0;
	currentMode = "";
	currentModeBeforeLand = "";
	currentLandingState = 0;
	landingStarted = false;
	currentFigureToDraw = NONE;
	channelHasChangedDuringDrawing = false;
}

/**
     * SetNodeHandle.
     * Sets the node handler to the receiver class so that it can publish topics
     * @param NodeHandle
     * @return void
     */
void Receiver::setNodeHandle(ros::NodeHandle nh)
{
	state_pub = nh.advertise<mavros_msgs::State>("/mavros/state", 1);
	control_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
	path_pub = nh.advertise<nav_msgs::Path>("/control/path", 1);

	land_service = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
}

/**
     * Callback the state's drone
     *
     * @param mavros_msgs::State
     * @return void
     */
void Receiver::stateCallback(const State::ConstPtr& msg)
{
	armed = msg -> armed;
	connected = msg -> connected;
	currentMode = msg -> mode;
}

/**
     * Callback the current position of the drone in the local frame (Home is the origin)
     *
     * @param geometry_msgs::PoseStamped
     * @return void
     */
void Receiver::localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::Pose pose = msg->pose;
	geometry_msgs::Point position = pose.position;
	local_x = position.x;
	local_y = position.y;
	local_z = position.z;
}

/**
     * Callback RC values
     *
     * @param mavros_msgs::RCIn
     * @return void
     */
void Receiver::rcCallback(const RCIn::ConstPtr& msg)
{
	ROS_INFO("1: %d, 1current: %d", msg->channels[0], currentChannelsValue[0]);
	for(int channel = 0; channel <= 8; channel++){
		if(msg->channels[channel] != currentChannelsValue[channel] && landingStarted){
			ROS_INFO_STREAM("input");
			channelHasChangedDuringDrawing = true;
		}
		currentChannelsValue[channel] = msg->channels[channel];
	}
}

/**
     * Converts the Roll Pitch and Yaw eulerien representation to quaternion representation
     *
     * @param double r p y
     * @return tf::quaternion quaternion
     */
tf::Quaternion convertRPYtoQ(double roll, double pitch, double yaw){
	tf::Matrix3x3 obs_mat;
	obs_mat.setEulerYPR(yaw,pitch,roll);
	tf::Quaternion qToSend;
	obs_mat.getRotation(qToSend);
	return qToSend;
}

/**
     * Get the angle between to points
     *
     * @param double x1 x2 y1 y2
     * @return double angle
     */
double getYaw(double x1, double x2, double y1, double y2){
	double yawToReturn = atan2((y2-y1),(x2-x1));
	if(yawToReturn < 0.0f){
		yawToReturn += 2*PI;
	}
	return yawToReturn;
}

/**
     * Get the 3D path of the shape
     *
     * @param Shape s, double radius increment nbRotation
     * @return vector<geometry_msgs::PoseStamped> listOf3DPosition
     */
vector<geometry_msgs::PoseStamped> Receiver::get3DPath(Shape s, double radius, double increment, int nbTour){

	vector<geometry_msgs::PoseStamped> posesToReturn; //List of 3D positions to finally return
	double offset_x = 0; //Offsets
	double offset_y = 0;
	double offset_z = 0;

	//Construction du path
	switch(s){
		case CIRCLE:
			offset_x = -radius + local_x;
			offset_y = local_y;
			offset_z = local_z;
			ROS_INFO("DRONE_CONTROL CIRCLE");
			for(int i = 0; i <= nbTour*360 ; i+=increment){
				geometry_msgs::PoseStamped poseToAdd;
				poseToAdd.pose.position.x = radius*cos(i*2*PI/360) + offset_x;
				poseToAdd.pose.position.y = radius*sin(i*2*PI/360) + offset_y;
				poseToAdd.pose.position.z = offset_z;
				
				//Calculate the yaw orientation based with the 2 positions
				if(!posesToReturn.empty()){
					poseToAdd.pose.orientation = tf::createQuaternionMsgFromYaw(getYaw(posesToReturn.back().pose.position.x, poseToAdd.pose.position.x, posesToReturn.back().pose.position.y, poseToAdd.pose.position.y));
				}else{
					poseToAdd.pose.orientation = tf::createQuaternionMsgFromYaw(getYaw(offset_x, poseToAdd.pose.position.x, offset_y, poseToAdd.pose.position.y));
				}
				
				posesToReturn.push_back(poseToAdd);
			}
			break;
		case HEART:
			offset_x = local_x;
			offset_y = -2*radius*cos(-1) + local_y;
			offset_z = local_z;
			ROS_INFO("DRONE_CONTROL HEART");
			for(int t = 0; t < nbTour; t++){
				for(double i = 0; i <= 2 ; i+=increment){
					geometry_msgs::PoseStamped poseToAdd;
					poseToAdd.pose.position.x = 2*radius*cos(pow(i-1, 3))*sin(pow(i-1, 3))*log(abs(pow(i-1, 3))) + offset_x;
					poseToAdd.pose.position.y = 2*radius*sqrt(abs(pow(i-1, 3)))*cos(pow(i-1, 3)) + offset_y;
					poseToAdd.pose.position.z = offset_z;
					
					//Calculate the yaw orientation based with the 2 positions
					if(!posesToReturn.empty()){
						poseToAdd.pose.orientation = tf::createQuaternionMsgFromYaw(getYaw(posesToReturn.back().pose.position.x, poseToAdd.pose.position.x, posesToReturn.back().pose.position.y, poseToAdd.pose.position.y));
					}else{
						poseToAdd.pose.orientation = tf::createQuaternionMsgFromYaw(getYaw(offset_x, poseToAdd.pose.position.x, offset_y, poseToAdd.pose.position.y));
					}
					posesToReturn.push_back(poseToAdd);
				}
			}
			break;
		case FIGURE8:
			offset_x = -radius + local_x;
			offset_y = local_y;
			offset_z = local_z;
			ROS_INFO("DRONE_CONTROL FIGURE8");
			for(double i = 0; i <= nbTour*360 ; i+=increment){
				geometry_msgs::PoseStamped poseToAdd;
				poseToAdd.pose.position.x = radius*cos(i*2*PI/360) + offset_x;
				poseToAdd.pose.position.y = radius*cos(i*2*PI/360)*sin(i*2*PI/360) + offset_y;
				poseToAdd.pose.position.z = offset_z;

				//Calculate the yaw orientation based with the 2 positions
				if(!posesToReturn.empty()){
					poseToAdd.pose.orientation = tf::createQuaternionMsgFromYaw(getYaw(posesToReturn.back().pose.position.x, poseToAdd.pose.position.x, posesToReturn.back().pose.position.y, poseToAdd.pose.position.y));
				}else{
					poseToAdd.pose.orientation = tf::createQuaternionMsgFromYaw(getYaw(offset_x, poseToAdd.pose.position.x, offset_y, poseToAdd.pose.position.y));
				}
				posesToReturn.push_back(poseToAdd);
			}
			break;
		case CREPE:
			offset_x = -radius + local_x;
			offset_y = local_y;
			offset_z = local_z;
			ROS_INFO("DRONE_CONTROL CREPE");
			for(double i = 0; i <= nbTour*360 ; i+=increment){
				geometry_msgs::PoseStamped poseToAdd;
				poseToAdd.pose.position.x = radius*cos(i*2*PI/360) + offset_x;
				poseToAdd.pose.position.y = radius*sin(i*2*PI/360) + offset_y;
				poseToAdd.pose.position.z = radius*sin(2*i*2*PI/360) + offset_z;

				//Checks if the height of the 3D shape won't let the drone to crash
				if(poseToAdd.pose.position.z <= 0){
					ROS_INFO_STREAM("DRONE_CONTROL ERROR: Can't draw the shape because of the height, try to go higher !");		
			  		ros::shutdown(); 
				}
	
				//Calculate the yaw orientation based with the 2 positions
				if(!posesToReturn.empty()){
					poseToAdd.pose.orientation = tf::createQuaternionMsgFromYaw(getYaw(posesToReturn.back().pose.position.x, poseToAdd.pose.position.x, posesToReturn.back().pose.position.y, poseToAdd.pose.position.y));
				}else{
					poseToAdd.pose.orientation = tf::createQuaternionMsgFromYaw(getYaw(offset_x, poseToAdd.pose.position.x, offset_y, poseToAdd.pose.position.y));
				}
				posesToReturn.push_back(poseToAdd);
			}
			break;
	}

	return posesToReturn;
}

/**
     * Process the shape in the sky
     *
     * @param Shape s, double radius increment nbRotation
     * @return bool if the shape has been succesfully drawn
     */
bool Receiver::processFigure(Shape shape, double radius, double increment, int nbRotation)
{
	//Check if the figure can be processed
  	if(!armed || shape == NONE || currentMode != "GUIDED" || radius == 0 || increment == 0 || nbRotation == 0) //Add if the drone is in the sky
	{			
		ROS_INFO_STREAM("DRONE_CONTROL ERROR: Can't draw the shape");		
  		ros::shutdown(); 
		return false;
	}

	//Check if the UAV is corretly positioned in the local frame
	ros::Rate rate2(1);
	int i = 0;
	while(local_z == 0.0){
		ROS_INFO_STREAM("DRONE_CONTROL WAIT: Wait for the drone to be located");
		ros::spinOnce();
		rate2.sleep();
		i++;
		if(i == 10) break; // 10 tries
	} if(i == 10) ros::shutdown();
	
	//Calculate all the 3D positions to then send to the drone
	ROS_INFO_STREAM("DRONE_CONTROL START: Starting to calculate the shape");
	vector<geometry_msgs::PoseStamped> pathToDraw = get3DPath(shape, radius, increment, nbRotation);
	nav_msgs::Path path; //Path to draw variable send in a topic to then see on Rviz
	path.poses = pathToDraw;
	path.header.frame_id=1;
	path_pub.publish(path);	

	//Draw the shape with the 3D Position previously calculated
	ROS_INFO_STREAM("DRONE_CONTROL START: Starting to draw the shape in the sky");
	ros::Rate rate(1);
	i = 0; //Reset the variable
	while(ros::ok() && i < pathToDraw.size())
	{
		control_pub.publish(pathToDraw.at(i));
		i++;
		rate.sleep();
		if(channelHasChangedDuringDrawing){
			ROS_INFO_STREAM("DRONE_CONTROL STOP: Draw stopped because of RC input");
  			ros::shutdown(); //Shutdown the node	
		}
	}

	shape = NONE; //Reset

	ROS_INFO_STREAM("DRONE_CONTROL DONE: done drawing the shape");	
  	ros::shutdown(); //Shutdown the node
	return true;
}

/**
     * Give the info : is ROS connected to the drone ?
     *
     * @param msg
     * @return bool if ROS is connected or not
     */
bool Receiver::isFcConnected()
{
	return connected;
}
