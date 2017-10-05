/**
 * @Brief This Class is used to store and manage every topic involved in the node
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
	local_x = 0;
	local_y = 0;
	local_z = 0;
	currentMode = "";
	currentModeBeforeLand = "";
	currentLandingState = 0;
	landingStarted = false;
	channelHasChangedDuringDrawing = false;
	landingRequested = false;
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
	path_pub = nh.advertise<nav_msgs::Path>("/smart_land/path", 1);

	land_service = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
}

/**
     * processSmartLanding.
     * Start the landing process
     * @param std_srvs::Trigger::Request and std_srvs::Trigger::Respond
     * @return void
     */
bool Receiver::processSmartLanding(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
	landingRequested = true;
	res.success = true;
	return true;
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
void cc::localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::Pose pose = msg->pose;
	geometry_msgs::Point position = pose.position;
	local_x = position.x;
	local_y = position.y;
	local_z = position.z;
}

/**
     * Callback the orientation of the drone
     *
     * @param sensor_msgs::Imu
     * @return void
     */
void Receiver::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	tf::Quaternion q;
	quaternionMsgToRF(msg->orientation, q);
	Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	local_yaw *= 360 / (2 * PI);
}

/**
     * Callback RC values
     *
     * @param mavros_msgs::RCIn
     * @return void
     */
void Receiver::rcCallback(const RCIn::ConstPtr& msg)
{
	//ROS_INFO("1: %d, 1current: %d", msg->channels[0], currentChannelsValue[0]);
	for(int channel = 0; channel <= 8; channel++){
		if(msg->channels[channel] != currentChannelsValue[channel] && landingStarted){
			ROS_INFO_STREAM("input changed");
			channelHasChangedDuringDrawing = true;
		}
		currentChannelsValue[channel] = msg->channels[channel];
	}
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
