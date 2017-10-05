/**
 * @Brief This Class is used to manage the smart landing of the drone
 * @author Valentin Rudloff +33 625 163 722
 */



//---------------------------------
//	Inclusions et definitions
//---------------------------------
#include "SmartLand.h"



/**
     * Contructeur
     *
     * @param Receiver
     * @return void
     */
SmartLand::SmartLand()
{
	currentPose = 0;
	landingStarted = false;
	nextPoseReached = true; //TODO add the enslavment of the position reached
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
     * The speedOfDescent Parameter is how much meters you lose by one turn in meters
     * If the radius equals to 0 then the drone descend by rotation around the yaw axis TODO
     * @param Shape s, double radius speedOfDescent increment nbRotation
     * @return boolean if the calculation succeded or not
     */
bool SmartLand::calculate3DPath(double radius, double speedOfDescent, double increment){

	if(increment == 0 || receiver->local_z == 0.0)
	{
		if(receiver->local_z == 0.0)
			ROS_INFO_STREAM("SMART_LANDING WAIT: Wait for the drone to be located");
		else
			ROS_INFO_STREAM("SMART_LANDING ERROR: The smart land parameters are not set");
		return false;
	}

	poses.clear(); //Reset the 3D positions
	currentPose = 0; //Reset the current 3D position to process

	ROS_INFO_STREAM("SMART_LANDING CALCULATE: Starting to calculate the shape to follow");

	double offset_x = -radius + receiver->local_x; //Offsets
	double offset_y = receiver->local_y;
	double offset_z = receiver->local_z;
	double offset_yaw = receiver->local_yaw;
	double i = 0;
	bool calculateDone = false;
	while(!calculateDone){
		geometry_msgs::PoseStamped poseToAdd;
		switch(currentLandShape){
			case CIRCLE:
				poseToAdd.pose.position.x = radius*cos((i/360)*2*PI) + offset_x;
				poseToAdd.pose.position.y = radius*sin((i/360)*2*PI) + offset_y;
				poseToAdd.pose.position.z = -speedOfDescent*(i/360)  + offset_z;
				//Calculate the yaw orientation based with the 2 positions
				if(!poses.empty()){
					poseToAdd.pose.orientation = tf::createQuaternionMsgFromYaw(getYaw(poses.back().pose.position.x, poseToAdd.pose.position.x, poses.back().pose.position.y, poseToAdd.pose.position.y));
				}else{
					poseToAdd.pose.orientation = tf::createQuaternionMsgFromYaw(getYaw(offset_x, poseToAdd.pose.position.x, offset_y, poseToAdd.pose.position.y));
				}
				break;
			case YAW_ROT:
				poseToAdd.pose.position.x = offset_x;
				poseToAdd.pose.position.y = offset_y;
				poseToAdd.pose.position.z = -speedOfDescent*(i/360)  + offset_z;
				//Calculate the yaw orientation based with the 2 positions
				offset_yaw -= 10;
				if(offset_yaw <= 0) offset_yaw = 360;
				poseToAdd.pose.orientation = tf::createQuaternionMsgFromYaw(offset_yaw);
				break;
		}
		
		if(poseToAdd.pose.position.z <= SMART_LAND_HEIGHT_MIN) //Stop the smart landing SMART_LAND_HEIGHT_MIN meters away from the ground and then land normaly
			calculateDone = true;
		
		
		
		poses.push_back(poseToAdd)
		i+=increment;
	}
	
	//Publish the shape to follow (to later see in Rviz)
	nav_msgs::Path path;
	path.poses = poses;
	path.header.frame_id=1;
	receiver->path_pub.publish(path);	

	return true;
}

/**
     * Process the shape in the sky
     *
     * @param Shape s, double radius increment nbRotation
     * @return bool if the shape has been succesfully drawn
     */
bool SmartLand::processLand()
{
	checkIfCanLand();

	//Application du path
	if(currentPose < poses.size())
	{
		receiver->control_pub.publish(poses.at(currentPose)); //Send the next position of the drone to follow
		currentPose++;
		if(receiver->channelHasChangedDuringDrawing){
			ROS_INFO_STREAM("SMART_LANDING STOP: Draw stopped because of RC input");
  			ros::shutdown(); //Shutdown the node	
		}
	}else{ //Then it means that the drone is SMART_LAND_HEIGHT_MIN meters above the ground and then it has to land normaly
		mavros_msgs::CommandTOL landSrv;
		receiver->land_service.call(landSrv);
		landingStarted = false;
		ROS_INFO_STREAM("SMART_LANDING FINISHED: the drone is now landing safely above the ground");
		ros::shutdown();
	}
	return true;
}

bool SmartLand::checkIfCanLand(){
	//Check if the figure can be processed
  	if(!receiver->armed || !nextPoseReached || poses.empty() || currentPose == poses.size()-1 || receiver->currentMode != "GUIDED" )
	{	
		if(!receiver->armed)		
			ROS_INFO_STREAM("SMART_LANDING ERROR: The drone is not armed");	
		if(poses.empty())		
			ROS_INFO_STREAM("SMART_LANDING ERROR: The shape to follow has not been calculated");	
		if(currentPose == poses.size()-1)		
			ROS_INFO_STREAM("SMART_LANDING ERROR: The drone is already landed smartly");	
		if(receiver->currentMode != "GUIDED")		
			ROS_INFO_STREAM("SMART_LANDING ERROR: The drone is not in Guided Mode");
		return false;
	}
	return true;
}


