
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
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

class TrackObject{
	//Publisher and Subscriber param
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

	// Images
	Mat img, original, flow;
	UMat  flowUmat, prevgray;


  public:
	TrackObject() : it_(nh_)
	{
		// Init Publisher and Subscriber
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &TrackObject::callbackImg, this);
	}

	~TrackObject()
	{
		cvDestroyAllWindows();
	}

	void callbackImg(const sensor_msgs::ImageConstPtr& msg)
	{
		//Image converter
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
		resize(original, original, Size(640/4, 480/4), 0, 0, INTER_CUBIC);
		blur(img, img, Size(10, 10));

		cvtColor(img, img, COLOR_BGR2GRAY);

	    if (prevgray.empty() == false ) {

			// calculate optical flow 
			calcOpticalFlowFarneback(prevgray, img, flowUmat, 0.4, 1, 12, 2, 8, 1.2, 0);
			// copy Umat container to standard Mat
			flowUmat.copyTo(flow);



			for (int y = 0; y < original.rows; y += 5) {
				for (int x = 0; x < original.cols; x += 5){
          			// get the flow from y, x position * 10 for better visibility
        			const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;

        			//circle(original, Point(x, y), 2, Scalar(0, 255, 0),-1);

        			//line(original, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,0,0));
					

        			
        			float currentMagX = norm(cvRound(x)-cvRound(flowatxy.x)) - x;
        			
        			if(currentMagX < -5 || currentMagX > 5){
            			//magnitudeX += currentMagX;
            			circle(original, Point(x, y), 2, Scalar(0, 255, 0),-1);
       				}

        			float currentMagY = norm(cvRound(y)-cvRound(flowatxy.y)) - y;
        			if(currentMagY < -5 || currentMagY > 5){
            			//magnitudeY += currentMagY;
            			circle(original, Point(x, y), 2, Scalar(0, 255, 0),-1);
       				}		
				}
			}
			
			namedWindow("prew", WINDOW_NORMAL);
    		imshow("prew", original);
			
			// fill previous image again
		}
		// Copy current Image to previous image
		img.copyTo(prevgray);
	}
};


void onKeyboard(int key){
	if(key == 102){
		
	}
}

// Callback function used to track the mouse events
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    switch( event ) {
    case CV_EVENT_LBUTTONDOWN: {

    }
    break;  
  }
}

/**
@param:
**/
int main(int argc, char **argv)
{
  	//SetUP ROS.
  	ros::init(argc, argv, "opencv_track");

  	// Initialise OpenCV windows
	namedWindow( "original", 0 );
	// Set callback to the mouse when on the "focus" window
	setMouseCallback( "original", onMouse, 0 );

	// Init object detectFocus
	TrackObject cam_object = TrackObject();

	// Set looprate to 30 Hz
	ros::Rate loop_rate(30);

	while (ros::ok()){
		int key = waitKey(3);
		onKeyboard(key);

		// Let loop ROS
		ros::spinOnce();
		loop_rate.sleep();
	}

	// Destroy object and thus windows name
	//delete(&cam_object);
	
}


