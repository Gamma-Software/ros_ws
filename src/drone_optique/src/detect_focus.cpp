
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

class DetectFocus{
	//Publisher and Subscriber param
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher focus_pub;

	// Images
	Mat img, original, crop;

	// Crop param
  	Point firstPt, secondPt;
  	Rect cropRect;

  	// Focus var
  	double focusLap, focusGradSobel;
  	bool inFocus;
	std_msgs::Bool focusData;

	// Focus test var
	double maxFocusValue;
	double ratio;
	bool inRangeFocusTest;


  public:
	DetectFocus() : it_(nh_)
	{
		// Init crop param
		firstPt = Point(100, 100);
		secondPt = Point(100 + 100, 100 + 100);
		cropRect = Rect(firstPt.x, firstPt.y, secondPt.x - firstPt.x, secondPt.y - firstPt.y);

		// Init range focus test
		inRangeFocusTest = false;
		ratio = 0.7;
		maxFocusValue = -1;

		// Init Publisher and Subscriber
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &DetectFocus::callbackImg, this);
		focus_pub = nh_.advertise<std_msgs::Bool>("/detect_focus/varLap", 1);
	}

	~DetectFocus()
	{
		cvDestroyWindow("focus"); // Destroy Window
		cvDestroyWindow("cropped"); // Destroy Window
	}


	void setFirstPoint(int x, int y){
		firstPt = Point(x, y);
	}
	void setSecondPoint(int x, int y){
		secondPt = Point(x, y);
	}
	void setTestRangeFocus(bool boolean){
		//Reset value
		inRangeFocusTest = boolean;
		if(inRangeFocusTest) maxFocusValue = -1;
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


		// Check if crop rect is valid
		if(firstPt.x > 0 && firstPt.y > 0 && firstPt.x < img.cols && firstPt.y < img.rows &&
			secondPt.x > 0 && secondPt.y > 0 && secondPt.x < img.cols && secondPt.y < img.rows &&
			(secondPt.y - firstPt.y) != 0 && (secondPt.x - firstPt.x) != 0){
			if((secondPt.y - firstPt.y) < 0){
				//swap
				int temp = secondPt.y;
				secondPt.y = firstPt.y;
				firstPt.y = temp;
			}
			if((secondPt.x - firstPt.x) < 0){
				//swap
				int temp = secondPt.x;
				secondPt.x = firstPt.x;
				firstPt.x = temp;
			}

			cropRect = Rect(firstPt.x, firstPt.y, secondPt.x - firstPt.x, secondPt.y - firstPt.y);
		}

		/* Crop the original image to the defined ROI */
	    crop = img(cropRect);
	    imshow("cropped", crop);  

	    // Display rect of the ROI
	    rectangle(original, firstPt, secondPt, Scalar(255, 0, 0), 10);

		//Measure Focus with the variance of the Laplacian
		focusLap = varianceOfLaplacian(crop);
		original.copyTo(img);

		//Measure Focus with the gradient of the sobel edge detection
		focusGradSobel = tenengrad(img, 3);
		original.copyTo(img);

		if(inRangeFocusTest){
			if(focusLap > maxFocusValue) maxFocusValue = focusLap;
			putText(original, "Center button to valid the focus range", Point(10, original.rows - 50), FONT_HERSHEY_SCRIPT_SIMPLEX, 1.5, Scalar(100,100,100), 4);
		}else{
			// Detect if the focus is good -> have to be tweaked a bit
			//inFocus = (focusLap > 450 || focusGradSobel > 8500);
			if(maxFocusValue == -1){
				putText(original, "Check the focus Range please", Point(10, original.rows - 50), FONT_HERSHEY_SCRIPT_SIMPLEX, 1.5, Scalar(100,100,100), 4);
			}else{
				inFocus = (focusLap > maxFocusValue*ratio);

				// Display text if image is in focus or not
				if(inFocus) putText(original, "In focus !", Point(10, original.rows - 50), FONT_HERSHEY_SCRIPT_SIMPLEX, 1.5, Scalar(100,100,100), 4);
				else putText(original, "Not in focus !", Point(10, original.rows - 50), FONT_HERSHEY_SCRIPT_SIMPLEX, 1.5, Scalar(100,100,100), 4);

				// Publish data focus
				focusData.data = inFocus;
				focus_pub.publish(focusData);
			}
		}

		// Display original image
		imshow("focus", original);

	}

	// OpenCV port of 'LAPV' algorithm (Pech2000)
	double varianceOfLaplacian(const Mat& src)
	{
	    Mat lap;
	    Laplacian(src, lap, CV_64F);

	    Scalar mu, sigma;
	    meanStdDev(lap, mu, sigma);

	    double focusMeasure = sigma.val[0]*sigma.val[0];
	    return focusMeasure;
	}

	// OpenCV port of 'TENG' algorithm (Krotkov86)
	double tenengrad(const Mat& src, int ksize)
	{
	    Mat Gx, Gy;
	    Sobel(src, Gx, CV_64F, 1, 0, ksize);
	    Sobel(src, Gy, CV_64F, 0, 1, ksize);

	    Mat FM = Gx.mul(Gx) + Gy.mul(Gy);

	    double focusMeasure = mean(FM).val[0];
	    return focusMeasure;
	}
};




Point firstPt, secondPt;
bool leftMouseDown = false;
bool populateROI = false;
bool testRangeFocus = false;
bool eventKeyButton = false;


void onKeyboard(int key){
	if(key == 102){ // ascii : 'f'
		testRangeFocus = !testRangeFocus;
		printf("range test: %s\n", testRangeFocus ? "true" : "false");
     	eventKeyButton = true;
	}
}

// Callback function used to track the mouse events
void onMouse(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN ){
     	firstPt = Point(x, y);
    	leftMouseDown = true;
     }
     else if(leftMouseDown && event == EVENT_LBUTTONUP){
     	leftMouseDown = false;
     	secondPt = Point(x, y);
     	populateROI = false;
     }else if(!leftMouseDown && event == EVENT_LBUTTONUP){
     	leftMouseDown = false;
     }
}


int main(int argc, char **argv)
{
  	//SetUP ROS.
  	ros::init(argc, argv, "opencv_detectFlow");

  	// Initialise OpenCV windows
	namedWindow( "focus", 0 );
	namedWindow( "cropped", 0 );
	// Set callback to the mouse when on the "focus" window
	setMouseCallback( "focus", onMouse, 0 );

	// Init object detectFocus
	DetectFocus cam_object = DetectFocus();

	// Set looprate to 30 Hz
	ros::Rate loop_rate(30);

	while (ros::ok()){

		// Populate the rect to check focus
		if(!populateROI){
			cam_object.setFirstPoint(firstPt.x, firstPt.y);
			cam_object.setSecondPoint(secondPt.x, secondPt.y);
     		populateROI = true;
		}

		if(eventKeyButton){
			cam_object.setTestRangeFocus(testRangeFocus);
			eventKeyButton = false;
		}



		int key = waitKey(3);
		onKeyboard(key);

		// Let loop ROS
		ros::spinOnce();
		loop_rate.sleep();
	}


	// Destroy object and thus windows name
	delete &cam_object;
	
}


