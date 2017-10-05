
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Polygon.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 #include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <vector>
#include <stdio.h>
#include <iostream>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <inttypes.h>

using namespace std; 
using namespace cv;

class DetectPatern{
	//Publisher and Subscriber param
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher pattern_pub;

	geometry_msgs::Polygon shapePattern;

	// Images
	Mat img, img_gray, pattern, result, original, crop;

	// Crop param
  	Rect cropRect;

  	Mat camMatrix, distCoeffs;

	Ptr<aruco::DetectorParameters> detectorParams;

	vector<Point2f> lines;
	vector<int> linesWeight;
	int mode;


  public:
	DetectPatern() : it_(nh_)
	{

		camMatrix = Mat::zeros(3, 3, CV_32F);
		camMatrix.Mat::at<float>(0,0) = 1373.086440;
		camMatrix.Mat::at<float>(0,1) = 0.000000;
		camMatrix.Mat::at<float>(0,2) = 288.853252;
		camMatrix.Mat::at<float>(1,0) = 0.000000;
		camMatrix.Mat::at<float>(1,1) = 1375.254391;
		camMatrix.Mat::at<float>(1,2) = 247.528029;
		camMatrix.Mat::at<float>(2,0) = 0.000000;
		camMatrix.Mat::at<float>(2,1) = 0.000000;
		camMatrix.Mat::at<float>(2,2) = 1.000000;

		distCoeffs = Mat::zeros(1, 5, CV_32F);
		distCoeffs.Mat::at<float>(0,0) = -0.232728;
		distCoeffs.Mat::at<float>(0,1) = 0.838863;
		distCoeffs.Mat::at<float>(0,2) = 0.000156;
		distCoeffs.Mat::at<float>(0,3) = -0.001388;
		distCoeffs.Mat::at<float>(0,4) = 0.00000;


		mode = 0;
		
		// Init pattern
		pattern = imread("/home/altametris/catkin_ws/src/drone/src/pattern.jpg");
		resize(pattern, pattern, Size(64, 64), 0, 0, INTER_NEAREST);

		// Init Publisher and Subscriber
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &DetectPatern::callbackImg, this);
		pattern_pub = nh_.advertise<geometry_msgs::Polygon>("/detect_pattern/shape", 1);


		detectorParams = aruco::DetectorParameters::create();
	}

	~DetectPatern()
	{
		cvDestroyWindow("Camera_Output"); // Destroy Window
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


		// Create dictionnary
	  	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);


        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
		vector< Vec3d > rvecs, tvecs;

		// detect markers and estimate pose
        aruco::detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);

        if(ids.size() > 0){
        	//aruco::estimatePoseSingleMarkers(corners, 0.05, camMatrix, distCoeffs, rvecs,tvecs);
        	//aruco::drawAxis(img, camMatrix, distCoeffs, rvecs, tvecs, 1);
        }




        if(ids.size() > 0) {

        	if(ids.size() == 4){
        		vector<Point2f> centers;

        		for(int i = 0; i < ids.size(); i++){
		            Point2f corner1 = (corners.at(i)).at(0);
		            Point2f corner2 = (corners.at(i)).at(1);
		            Point2f corner3 = (corners.at(i)).at(2);
		            Point2f corner4 = (corners.at(i)).at(3);

		            centers.push_back(Point2f(0, 0));
		            intersection(corner1, corner3, corner2, corner4, centers.at(i));
	            	circle(img, centers.at(i), 2, Scalar(0, 255, 0), 2);
        		}
        		
        		Point2f inter;
        		intersection(centers.at(0), centers.at(1), centers.at(2), centers.at(3), inter);
        		circle(img, inter, 2, Scalar(0, 0, 255), 2);

        	}else{
	            Point2f corner1 = (corners.at(0)).at(0);
	            Point2f corner2 = (corners.at(0)).at(1);
	            Point2f corner3 = (corners.at(0)).at(2);
	            Point2f corner4 = (corners.at(0)).at(3);


	            float w = sqrt( (corner1 - corner2).ddot(corner1 - corner2));

	            circle(img, corner1, 2, Scalar(0, 255, 0), 2);
	            circle(img, corner2, 2, Scalar(0, 255, 0), 2);
	            circle(img, corner3, 2, Scalar(0, 255, 0), 2);
	            circle(img, corner4, 2, Scalar(0, 255, 0), 2);

	            line(img, corner1, corner3, Scalar(0, 255, 0));
	            line(img, corner2, corner4, Scalar(0, 255, 0));



	            Point2f inter;
	            bool interBool = intersection(corner1, corner3, corner2, corner4, inter);


	            
	            if(mode == 0){
	            	if(interBool){
		            	lines.push_back(inter);
		            	linesWeight.push_back(w/40);
		            }

		            if(lines.size() > 1){
		            	for(int i = 0; i < lines.size() - 2; i++){
		            		line(img, lines.at(i), lines.at(i+1), Scalar(0, 0, 255), linesWeight.at(i));
		            	}
		            }
	            }else
	            	line(img, Point2f(img.cols/2, img.rows/2), inter, Scalar(0, 0, 255), 3);
		        

	            circle(img, inter, 2, Scalar(255, 0, 0), 2);
        	}
            
           		
            
 


           	//for(unsigned int i = 0; i < ids.size(); i++)
                //aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
		}
		imshow("out", img);
	  	

	    /*
		// Publish position of pattern
		shapePattern.points[0] = Point(0, 0);
		shapePattern.points[1] = Point(10, 10);
		pattern_publish(shapePattern);

		/* Crop the original image to the defined ROI */
	    
	    /*
	    crop = img(cropRect);
	    imshow("cropped", crop);  

	    // Display rect of the ROI
	    rectangle(original, firstPt, secondPt, Scalar(255, 0, 0), 10);


		// Display original image
		imshow("image", original);


		*/




		
	    //imshow("marker2", boardImage);



    	onKeyboard(waitKey(3));

	}



	void onKeyboard(int key){
		if(key == 109){
			if(mode == 0)
				mode = 1;
			else
				mode = 0;
		}
	}

	bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r)
	{
	    Point2f x = o2 - o1;
	    Point2f d1 = p1 - o1;
	    Point2f d2 = p2 - o2;

	    float cross = d1.x*d2.y - d1.y*d2.x;
	    if (abs(cross) < /*EPS*/1e-8)
	        return false;

	    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
	    r = o1 + d1 * t1;
	    return true;
	}
};



int main(int argc, char **argv)
{
  	//SetUP ROS.
  	ros::init(argc, argv, "opencv_detect_pattern");

  	// Initialise OpenCV windows
	namedWindow( "image", 0 );
	namedWindow( "pattern", 0 );

	// Init object detectFocus
	DetectPatern cam_object = DetectPatern();

	ros::spin();
	
}


