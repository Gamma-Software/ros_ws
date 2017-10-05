#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 #include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <vector>
#include <stdio.h>
#include <iostream>

#include <inttypes.h>

using namespace std; 
using namespace cv;

/* INTERNAL CAM TEST CODE */
/* REF: https://www.youtube.com/watch?v=HqNJbx0uAv0 */

class CamProc{


  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  bool first;
  cv::Mat img1, img2;
  cv::Mat diff;

  public:
	CamProc() : it_(nh_)
	{
		first = true;
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &CamProc::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		//ROS_INFO("I heard: %u", msg->width);

		cv_bridge::CvImagePtr cv_ptr;

		try
		{
		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		if(first){
			first = false;
			img1 = cv_ptr->image;
			resize(img1, img1, Size(640/8, 480/8), 0, 0, INTER_CUBIC);
		}else{

			img2 = cv_ptr->image;
			resize(img2, img2, Size(640/8, 480/8), 0, 0, INTER_CUBIC);

			absdiff(img1, img2, diff);

			cvtColor(diff, diff, cv::COLOR_BGR2GRAY);

			imshow("image", diff);
			waitKey(3);


			// Output modified video stream
			image_pub_.publish(cv_ptr->toImageMsg());

			img1 = img2;
		}
	}

	~CamProc()
	{
		cvDestroyWindow("Camera_Output"); // Destroy Window
	}
};


class CamFlow{

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher image_pub_x;
	ros::Publisher image_pub_y;
	ros::Publisher image_pub_x_filter;
	ros::Publisher image_pub_y_filter;

	Mat flow, frame;
	UMat  flowUmat, prevgray;
	vector <float> meanMagnitudeX;
	vector <float> meanMagnitudeY;
	float meanMagnitudeXFiltered;
	float meanMagnitudeYFiltered;
	std_msgs::Float32 meanMagnitudeXFiltermsg, meanMagnitudeYFiltermsg;
	std_msgs::Float32 meanMagnitudeXmsg, meanMagnitudeYmsg;

	public:
		CamFlow() : it_(nh_)
		{
			image_sub_ = it_.subscribe("/camera/image_raw", 1, &CamFlow::imageCb, this);
			image_pub_ = it_.advertise("/image_converter/output_video_flow", 1);
			image_pub_x = nh_.advertise<std_msgs::Float32>("/image_converter/output_flow_mag_x", 1);
			image_pub_x_filter = nh_.advertise<std_msgs::Float32>("/image_converter/output_flow_mag_x/filtered", 1);
			image_pub_y = nh_.advertise<std_msgs::Float32>("/image_converter/output_flow_mag_y", 1);
			image_pub_y_filter = nh_.advertise<std_msgs::Float32>("/image_converter/output_flow_mag_y/filtered", 1);

			for(int i = 0; i < 4; i++){
				meanMagnitudeX.push_back(0);
				meanMagnitudeY.push_back(0);
			}
		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg)
		{
			Mat img;
			Mat original;

			cv_bridge::CvImagePtr cv_ptr;

			try
			{
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e)
			{
			  ROS_ERROR("cv_bridge exception: %s", e.what());
			  return;
			}

			img = cv_ptr->image;
			img.copyTo(original);
			resize(img, img, Size(640/4, 480/4), 0, 0, INTER_CUBIC);
			blur(img, img, Size(10, 10));

			cvtColor(img, img, COLOR_BGR2GRAY);

		    if (prevgray.empty() == false ) {

   				// calculate optical flow 
    			calcOpticalFlowFarneback(prevgray, img, flowUmat, 0.4, 1, 12, 2, 8, 1.2, 0);
    			// copy Umat container to standard Mat
    			flowUmat.copyTo(flow);

    			float magnitudeX = 0;
    			float magnitudeY = 0;
    			// By y += 5, x += 5 you can specify the grid 
    			for (int y = 0; y < img.rows; y += 20) {
    				for (int x = 0; x < img.cols; x += 20){
              			// get the flow from y, x position * 10 for better visibility
            			const Point2f flowatxy = flow.at<Point2f>(y, x);
            			float currentMagX = norm(cvRound(x)-cvRound(flowatxy.x)) - x;
            			
            			if(currentMagX < -0.5 || currentMagX > 0.5){
	            			magnitudeX += currentMagX;
	       				}

            			float currentMagY = norm(cvRound(y)-cvRound(flowatxy.y)) - y;
            			if(currentMagY < -0.5 || currentMagY > 0.5){
	            			magnitudeY += currentMagY;
	       				}
    				}
				}

				meanMagnitudeX.push_back(magnitudeX / (img.cols/20));
				meanMagnitudeY.push_back(magnitudeY / (img.rows/20));

				// Output value x and y
				meanMagnitudeXmsg.data = meanMagnitudeX.back();
				meanMagnitudeYmsg.data = meanMagnitudeY.back();
				image_pub_x.publish(meanMagnitudeXmsg);
				image_pub_y.publish(meanMagnitudeYmsg);

				// Output value x and y filtered
				if(lpf(meanMagnitudeX, meanMagnitudeXFiltered, 2)){
					meanMagnitudeXFiltermsg.data = meanMagnitudeXFiltered;
					image_pub_x_filter.publish(meanMagnitudeXFiltermsg);
				}

				if(lpf(meanMagnitudeY, meanMagnitudeYFiltered, 2)){
					meanMagnitudeYFiltermsg.data = meanMagnitudeYFiltered;
					image_pub_y_filter.publish(meanMagnitudeYFiltermsg);
				}

            	/*
            	line(original, Point(original.cols/2, original.rows/2), Point((original.cols/2 + meanMagnitudeX.back()*2), original.rows/2), Scalar(255,0,0), 5);
				line(original, Point(original.cols/2, original.rows/2), Point(original.cols/2, (original.rows/2 + meanMagnitudeY.back()*2)), Scalar(0,0,255), 5);
				line(original, Point(original.cols/2, original.rows/2), Point((original.cols/2 + meanMagnitudeX.back()*2), (original.rows/2 + meanMagnitudeY.back()*2)), Scalar(0,255,0), 5);
				*/

				line(original, Point(original.cols/2, original.rows/2), Point((original.cols/2 + meanMagnitudeXFiltered*2), original.rows/2), Scalar(255,0,0), 5);
				line(original, Point(original.cols/2, original.rows/2), Point(original.cols/2, (original.rows/2 + meanMagnitudeYFiltered*2)), Scalar(0,0,255), 5);
				line(original, Point(original.cols/2, original.rows/2), Point((original.cols/2 + meanMagnitudeXFiltered*2), (original.rows/2 + meanMagnitudeYFiltered*2)), Scalar(0,255,0), 5);
				
				
				// draw the results
    			namedWindow("prew", WINDOW_NORMAL);
    			imshow("prew", original);
    			// fill previous image again
    			img.copyTo(prevgray);
   			}else {
				img.copyTo(prevgray);
    		}

    		waitKey(3);


			// Output modified video stream
			image_pub_.publish(cv_ptr->toImageMsg());
		}

		~CamFlow()
		{
			cvDestroyAllWindows();
		}

		bool lpf(vector<float> & data_in, float & data_out, int obs)
		{
		  	// Stop if their is not enough values
		  	if(data_in.size() < obs) return false;

			for(uint32_t i = 0; i < data_in.size(); i++)
				data_out += data_in.at(i);
			
			data_out /= data_in.size();
			
			data_in.clear();
			return true;
		}
};

int main(int argc, char **argv)
{
  //SetUP ROS.
  ros::init(argc, argv, "opencv_tutorial");
  //CamProc cam_object;
  CamFlow cam_object;
  ros::spin();
  

}
