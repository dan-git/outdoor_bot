#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "OpenCV Image";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_blobs");
  cv::Mat img = cv::imread("../images/foo.jpg");
  cv::SimpleBlobDetector::Params sbdParams;
  // sbdParams.filterByArea = false;
  // sbdParams.filterByCircularity = false;
  // sbdParams.filterByColor = true;
  // sbdParams.blobColor = 255;
  // sbdParams.filterByInertia = false;
  // sbdParams.filterByConvexity = false;
  
  std::vector<cv::KeyPoint> keyPoints;
  #if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2

		// Set up detector with params
		SimpleBlobDetector detector(sbdParams);

		// Detect blobs
		detector.detect( img, keyPoints);
	#else 

		// Set up detector with params
		Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(sbdParams);   

		// Detect blobs
		detector->detect( img, keyPoints);
	#endif
  //cv::SimpleBlobDetector det(sbdParams);
  //det.detect(img, keyPoints);
  ROS_INFO("Found %zu key points.", keyPoints.size());
  // cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
  // cv::imshow(OPENCV_WINDOW, img);
  // cv::waitKey();
  // cv::destroyWindow(OPENCV_WINDOW);
}
