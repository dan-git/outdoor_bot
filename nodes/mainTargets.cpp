#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <string>
#include <vector>
#include "outdoor_bot/mainTargets_service.h"
#include "outdoor_bot/mainTargets_msg.h"
#include "outdoor_bot/mainTargetsCommand_msg.h"
#include "outdoor_bot/mainTargets_imageReceived_msg.h"
#include "outdoor_bot_defines.h"
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include "/home/dbarry/Dropbox/outdoor_bot/software/ros/outdoor_bot/src/robustMatcher.h"
//#include <opencv2/legacy/legacy.hpp>

using namespace std;
using namespace cv;


#define MAX_NUM_TARGETS 50
#define PIX_PER_CM_AT_3M 8.
#define FRONT_CAMERA_FULLZOOM_FIRST_TARGET_RANGE_PARAMETER 1000000
#define BLUE 0
#define RED 1
#define BLUE_RED 2

int getUserInputInteger()
{
   int myNumber = 0;
   while (true)
   {
      string input = "";
      cout << "Please enter a valid number: " << endl << endl;
      getline(cin, input);

      // This code converts from string to number safely.
      stringstream myStream(input);
      if (myStream >> myNumber)
        break;
      cout << "Invalid number, please try again" << endl;
   }
   cout << "You entered: " << myNumber << endl << endl;
   return myNumber;
}

bool writeToFile(std::string filename, Mat imageFrame)
{
   if (imageFrame.empty()) return false;
   vector<int> compression_params; //vector that stores the compression parameters of the image
   compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique
   compression_params.push_back(98); //specify the compression quality
   bool bSuccess = imwrite(filename, imageFrame, compression_params); //write the image to file
   if (bSuccess) 
   {
      ROS_INFO("image written to file");
      return true;
   }
   else ROS_ERROR("error in writing image to file");
   return false;
}

class mainTargets
{
private:
   ros::NodeHandle nh_;
   ros::Publisher image_received_pub_, target_center_pub_;
   ros::Subscriber analyzeImage_sub_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber subWebcam_, subDigcam_;
   std::string filename_;
   float rangeSquared_, approxRange_;
	int centerX_, centerY_, numTargetsDetected_, cameraType_;
   bool newDigcamImageReceived_, newWebcamImageReceived_;
   Mat image_, gray_, newDigcamImage_, newWebcamImage_, showImg_;
   CvSize sz_;

public:
mainTargets(ros::NodeHandle &nh)
   :  nh_(nh), it_(nh)
{
      analyzeImage_sub_ = nh.subscribe("mainTargets_cmd", 10, &mainTargets::commandCallback, this);
      target_center_pub_ = nh_.advertise<outdoor_bot::mainTargets_msg>("target_center", 50);
      image_received_pub_ = nh_.advertise<outdoor_bot::mainTargets_imageReceived_msg>("target_image_received", 5);
      subDigcam_ = it_.subscribe("digcam_image", 1, &mainTargets::digcamImageCallback, this);
      subWebcam_ = it_.subscribe("webcam_image", 1, &mainTargets::webcamImageCallback, this);

      centerX_ = -2;
      centerY_ = -2;
      rangeSquared_ = 400;
      approxRange_ = 20.;
      numTargetsDetected_ = 0;
      newDigcamImageReceived_ = false;
      newWebcamImageReceived_ = false;
}

   // commanded to analyze image

   void commandCallback(const outdoor_bot::mainTargetsCommand_msg msg)
   {
      bool firstTarget = msg.firstTarget;
      cameraType_ = msg.cameraType;
      if (cameraType_ == DIGCAM && newDigcamImageReceived_)
      {
         ROS_INFO("mainTargets is using a digcam image via image transport.");
         image_ = newDigcamImage_.clone();
         newDigcamImageReceived_ = false;
      }
      else if (cameraType_ == WEBCAM && newWebcamImageReceived_)
      {
         image_ = newWebcamImage_.clone();
         newWebcamImageReceived_ = false;
      }
      else return;

      approxRange_ = msg.approxRange;

      outdoor_bot::mainTargets_msg output_msg;
      output_msg.cameraType = cameraType_; 
      output_msg.totalX = image_.cols;
 
      if (detectBlobs(image_, firstTarget))
      {
         output_msg.centerX = centerX_;
         output_msg.centerY = centerY_;
         output_msg.range = sqrt(rangeSquared_);
         target_center_pub_.publish(output_msg);
         ROS_INFO("possible target found, center and rangeSquared = %d, %d, %f", centerX_, centerY_, rangeSquared_);
      }
      else
      {
         output_msg.centerX = -1;
         output_msg.centerY = -1;
         output_msg.range = 0.;
         target_center_pub_.publish(output_msg);
         ROS_INFO("sending back main target response: -1, -1, -1.");
      }  
}      

   void digcamImageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
     ROS_INFO("digcam image received in main targets");
     try
     {
       //cv::imshow("keypoints", cv_bridge::toCvShare(msg, "bgr8")->image);
       //cv::waitKey(30);
       newDigcamImage_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
       newDigcamImageReceived_ = true;
       outdoor_bot::mainTargets_imageReceived_msg imMsg;
       imMsg.cameraType = DIGCAM;
       image_received_pub_.publish(imMsg);   // publish that we received a digcam image
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
   }
   
   void webcamImageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
     ROS_INFO("webcam image received in MainTargets");
     try
     {
       //cv::imshow("keypoints", cv_bridge::toCvShare(msg, "bgr8")->image);
       //cv::waitKey(30);
       newWebcamImage_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
       newWebcamImageReceived_ = true;
       outdoor_bot::mainTargets_imageReceived_msg imMsg;
       imMsg.cameraType = WEBCAM;
       image_received_pub_.publish(imMsg);   // publish that we received a webcam image
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
   }

void thresholdGray(Mat img, int level)
{
   Mat gray(sz_, CV_8UC1); // depth 8, 1 channel, gray version
   cvtColor(img, gray, CV_BGR2GRAY);
   threshold(gray, gray, level, 255, CV_THRESH_BINARY );
   gray_ = gray;
}

/*
bool matchFeatures()
{
   int numKeypointsORB = 1500, minHessianSURF = 10;
   cv::Ptr<cv::FeatureDetector> pfd= new cv::SurfFeatureDetector(minHessianSURF); // use this detector for SURF
   //cv::Ptr<cv::FeatureDetector> pfd= new cv::OrbFeatureDetector(numKeypointsORB);      // use this detector for ORB

        // Read input images
   //string filenameTemplate = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/christmastree_sunlight_normzoom.JPG";
   //string filenameImage = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/christmastree_shade_normzoom.JPG";
   //string filenameTemplate = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/pi_sunlight_normzoom.JPG";
   //string filenameImage = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/pi_shade_normzoom.JPG";

   string filenameTemplate = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/ceiling_image101.jpg";
   string filenameImage = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/ceiling_image102.jpg";

        cv::Mat image1= cv::imread(filenameTemplate,0);
        cv::Mat image2= cv::imread(filenameImage,0);
        if (!image1.data || !image2.data)
                return false; 

    // Display the images
        //cv::namedWindow("Right Image");
        //cv::imshow("Right Image",image1);
        //cv::namedWindow("Left Image");
        //cv::imshow("Left Image",image2);

        // Prepare the matcher
        RobustMatcher rmatcher;
        rmatcher.setConfidenceLevel(0.98);
        rmatcher.setMinDistanceToEpipolar(1.0);
        rmatcher.setRatio(0.65f);
 

        rmatcher.setFeatureDetector(pfd);

        // Match the two images
        std::vector<cv::DMatch> matches;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cout << "matching" << endl;
        cv::Mat fundemental= rmatcher.match(image1,image2,matches, keypoints1, keypoints2);
        // draw the matches
        cv::Mat imageMatches;
        cv::drawMatches(image1,keypoints1,  // 1st image and its keypoints
                            image2,keypoints2,  // 2nd image and its keypoints
                                        matches,                        // the matches
                                        imageMatches,           // the image produced
                                        cv::Scalar(255,255,255)); // color of the lines
        cv::namedWindow("Matches");
        cv::imshow("Matches",imageMatches);     
        // Convert keypoints into Point2f       
        std::vector<cv::Point2f> points1, points2;
        
        for (std::vector<cv::DMatch>::const_iterator it= matches.begin();
                         it!= matches.end(); ++it) {

                         // Get the position of left keypoints
                         float x= keypoints1[it->queryIdx].pt.x;
                         float y= keypoints1[it->queryIdx].pt.y;
                         points1.push_back(cv::Point2f(x,y));
                         cv::circle(image1,cv::Point(x,y),3,cv::Scalar(255,255,255),3);
                         // Get the position of right keypoints
                         x= keypoints2[it->trainIdx].pt.x;
                         y= keypoints2[it->trainIdx].pt.y;
                         cv::circle(image2,cv::Point(x,y),3,cv::Scalar(255,255,255),3);
                         points2.push_back(cv::Point2f(x,y));
        }
        
        // Draw the epipolar lines
        if (points1.size() < 1 || points2.size() < 1 || points1.size() != points2.size())
        {
            cout << "no matches found in calcORB" << endl;  
            return false;
        }
        std::vector<cv::Vec3f> lines1; 
        cv::computeCorrespondEpilines(cv::Mat(points1),1,fundemental,lines1);
                
        for (vector<cv::Vec3f>::const_iterator it= lines1.begin();
                         it!=lines1.end(); ++it) {

                         cv::line(image2,cv::Point(0,-(*it)[2]/(*it)[1]),
                                             cv::Point(image2.cols,-((*it)[2]+(*it)[0]*image2.cols)/(*it)[1]),
                                                         cv::Scalar(255,255,255));
        }

        std::vector<cv::Vec3f> lines2; 
        cv::computeCorrespondEpilines(cv::Mat(points2),2,fundemental,lines2);
        
        for (vector<cv::Vec3f>::const_iterator it= lines2.begin();
                     it!=lines2.end(); ++it) {

                         cv::line(image1,cv::Point(0,-(*it)[2]/(*it)[1]),
                                             cv::Point(image1.cols,-((*it)[2]+(*it)[0]*image1.cols)/(*it)[1]),
                                                         cv::Scalar(255,255,255));
        }

    // Display the images with epipolar lines
        cv::namedWindow("Right Image Epilines (RANSAC)");
        cv::imshow("Right Image Epilines (RANSAC)",image1);
        cv::namedWindow("Left Image Epilines (RANSAC)");
        cv::imshow("Left Image Epilines (RANSAC)",image2);
      std::stringstream buffer, buffer1, buffer2;
      buffer << "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/RANSAC_image0.jpg";
      std::string filenm = buffer.str();
      writeToFile(filenm, image1);

      buffer1 << "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/RANSAC_image1.jpg";
      filenm = buffer1.str();
      writeToFile(filenm, image2);


      buffer2 << "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/imageMatches0.jpg";
      filenm = buffer2.str();
      writeToFile(filenm, imageMatches);
        //cv::waitKey();
        return true;
}




// set parameters

int numKeyPoints = 1500;

//Instantiate robust matcher

RobustMatcher rmatcher;

//instantiate detector, extractor, matcher

detector = new cv::OrbFeatureDetector(numKeyPoints);
extractor = new cv::OrbDescriptorExtractor;
matcher = new cv::BruteForceMatcher<cv::HammingLUT>;

rmatcher.setFeatureDetector(detector);
rmatcher.setDescriptorExtractor(extractor);
rmatcher.setDescriptorMatcher(matcher);

//Load input image detect keypoints

cv::Mat img1;
std::vector<cv::KeyPoint> img1_keypoints;
cv::Mat img1_descriptors;
cv::Mat img2;
std::vector<cv::KeyPoint> img2_keypoints
cv::Mat img2_descriptors;
std::vector<std::vector<cv::DMatch> > matches;
   string filenameTemplate = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/pi_sunlight_normzoom.JPG";
   string filenameImage = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/clubmed/digcam/zoom/pi_shade_normzoom.JPG";
   img1 = cv::imread(filenameTemplate, CV_LOAD_IMAGE_GRAYSCALE);
   img2 = cv::imread(filenameImage, CV_LOAD_IMAGE_GRAYSCALE);

rmatcher.match(img1, img2, matches, img1_keypoints, img2_keypoints); 
*/
  //vector<KeyPoint> keypoints;
   //ORB_(imageORB, &keypoints, cv::noArray(), false);

/*
   detector.detect(img1, img1_keypoints);
   detector.detect(img2, img2_keypoints);
   extractor.compute(img1, img1_keypoints, img1_descriptors);
   extractor.compute(img2, img2_keypoints, img2_descriptors);

   //Match keypoints using knnMatch to find the single best match for each keypoint
   //Then cull results that fall below given distance threshold

   std::vector<std::vector<cv::DMatch> > matches;
   matcher.knnMatch(img1_descriptors, img2_descriptors, matches, 1);
   int matchCount=0;
   for (int n=0; n<matches.size(); ++n) {
       if (matches[n].size() > 0){
           if (matches[n][0].distance > distThreshold){
               matches[n].erase(matches[n].begin());
           }else{
               ++matchCount;
           }
       }
   }
}
*/

bool detectBlobs(Mat im_original, bool firstTarget)
{
   double maxArea = 0.;
   // create a working copy
   Mat im = im_original.clone();
   CvSize sz_ = cvSize(im_original.cols, im_original.rows);
   long totalArea = im_original.cols * im_original.rows;
   cout << endl << endl << "total image area for blob detection = " << totalArea << endl;
   
   // down-scale and upscale the image to filter out some noise
   Mat pyr(cvSize(sz_.width/2, sz_.height/2), CV_8UC3); // depth 8, 3 channels for downscaling
   pyrDown( im, pyr, cvSize(sz_.width/2, sz_.height/2)  );
   pyrUp( pyr, im, sz_ );

   vector<Mat> planes_BGR;
   split(im, planes_BGR); // split into separate color planes (B G R)
   
   Mat timg_B = planes_BGR[0];
   Mat timg_G = planes_BGR[1];
   Mat timg_R = planes_BGR[2];

   // leave out the green, so we don't see grass
   Mat im_BR, im_analyze;
   addWeighted( timg_B, 0.5, timg_R, 0.5, 0.0, im_BR);


   // invert the image if we want to detect dark objects instead of light (see if shadows matter here)
   // remember to change the filterbycolor (actually intensity) parameter if you change here
   //bitwise_not(im_BR, im_BR);

   // reduce noise with erosion, note if we invert the image, this must change to dilate
   //erode(im_BR, im_BR, Mat(), Point(-1,-1), 1);

   // Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;
   for (int colorCounter = 0; colorCounter < 3; colorCounter++)
   {
      if (firstTarget)
      {
         im_analyze = im_BR;
         // threshold a bit
         //threshold(im_analyze, im_analyze, 200, 255, CV_THRESH_BINARY);
         // If we threshold here, then in blob thresholding, we can just do the minimum we can get away with
         // which is a single thresholding at 254.
         // Note that this means minRepeatability has to be = 1, since we are only doing a single threshold 

	      // Set thresholds to check. We have to specify values or it will use the defaults (50,220)
	      params.minThreshold = 154;  // inclusive. 
	      params.maxThreshold = 255;  // exclusive.
         params.thresholdStep = 10;  // 10 is the default.  Doesn't matter here.
         params.minDistBetweenBlobs = im_analyze.cols / 100.; // we want lots of small blobs when there is noise, and we will
                                                      // select the blob with max area.
         params.minRepeatability = 5;  // how many centers show up

         // Filter by Area.
	      params.filterByArea = true;
	      //params.minArea = 7327; // 7327 corresponds to a size value (~radius?) of 47.8009 for the keypoint
                              // if that was a radius, it would correspond to an area of 7178.3
                              // even a radius of 48 corresponds to an area of only 7238.2
         if (approxRange_ > 1.)
         {
            float predictedArea = FRONT_CAMERA_FULLZOOM_FIRST_TARGET_RANGE_PARAMETER / (approxRange_ * approxRange_);
            cout << "mainTargets predicted Area = " << predictedArea << endl;
            params.minArea = predictedArea / 2.;
            params.maxArea = predictedArea * 5;
         }
         else
         {
            params.minArea = 5000;
            params.maxArea = totalArea / 3.;
         }
	      // Filter by Color-- this is not actually color, it is intensity and goes from 0 to 255
         // the blob has to match this intensity exactly, so it is only useful if you have thresholded
         // to a specific value, which we do, so it is really just choosing 
         // if you want blobs above (bright) or below (dark) the threshold
         params.filterByColor = true;
         //params.blobColor = 0;   // detects black blobs
         params.blobColor = 255; // detects white blobs

         // Filter by Circularity
	      params.filterByCircularity = false;
	      params.minCircularity = 0.1;

	      // Filter by Convexity
	      params.filterByConvexity = false;
	      params.minConvexity = 0.87;

	      // Filter by Inertia
	      params.filterByInertia = false;
	      params.minInertiaRatio = 0.01;
      }
      else  
      {

         cout << endl << endl;
         if (colorCounter == BLUE) cout << "For blue image: " << endl;
         else if (colorCounter == RED) cout << "For red image: " << endl;
         else if (colorCounter == BLUE_RED) cout << "For blue plus red image: " << endl;

         if (colorCounter == 0) im_analyze = timg_B;
         else if (colorCounter == 1) im_analyze = timg_R;
         else if (colorCounter == 2) im_analyze = im_BR;

         // threshold a bit
         //threshold(im_analyze, im_analyze, 200, 255, CV_THRESH_BINARY);
         // If we threshold here, then in blob thresholding, we can just do the minimum we can get away with
         // which is a single thresholding at 254.
         // Note that this means minRepeatability has to be = 1, since we are only doing a single threshold 

	      // Set thresholds to check. We have to specify values or it will use the defaults (50,220)
	      params.minThreshold = 154;  // inclusive. 
	      params.maxThreshold = 255;  // exclusive.
         params.thresholdStep = 10;  // 10 is the default.  Doesn't matter here.
         params.minDistBetweenBlobs = im_analyze.cols / 100.; // we want lots of small blobs when there is noise, and we will
                                                      // select the blob with max area.
         params.minRepeatability = 5;  // how many centers show up

         // Filter by Area.
	      params.filterByArea = true;
	      //params.minArea = 7327; // 7327 corresponds to a size value (~radius?) of 47.8009 for the keypoint
                              // if that was a radius, it would correspond to an area of 7178.3
                              // even a radius of 48 corresponds to an area of only 7238.2
         params.minArea = 100; //totalArea / 500;
         params.maxArea = totalArea / 4;

	      // Filter by Color-- this is not actually color, it is intensity and goes from 0 to 255
         // the blob has to match this intensity exactly, so it is only useful if you have thresholded
         // to a specific value, which we do, so it is really just choosing 
         // if you want blobs above (bright) or below (dark) the threshold
         params.filterByColor = true;
         //params.blobColor = 0;   // detects black blobs
         params.blobColor = 255; // detects white blobs

         // Filter by Circularity
	      params.filterByCircularity = false;
	      params.minCircularity = 0.1;

	      // Filter by Convexity
	      params.filterByConvexity = false;
	      params.minConvexity = 0.87;

	      // Filter by Inertia
	      params.filterByInertia = false;
	      params.minInertiaRatio = 0.01;
    }

      // Storage for blobs
      vector<KeyPoint> keypoints;
      #if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2

			// Set up detector with params
			SimpleBlobDetector detector(params);

			// Detect blobs
			detector.detect( im_analyze, keypoints);
		#else 

			// Set up detector with params
			Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);   

			// Detect blobs
			detector->detect( im_analyze, keypoints);
		#endif 

      cout << "Found " << keypoints.size() << " keypoints" << endl;

      // Show blobs
      double maxKeypointArea = -1.;
      int maxIndex = 0;
      for (unsigned int i=0; i < keypoints.size(); i++)
      {
         double keypointArea = keypoints[i].size * keypoints[i].size * 3.14;
         cout << "keypoint center: " << keypoints[i].pt << endl;
         cout << "keypoint radius: " << keypoints[i].size << endl;
         cout << "keypoint area: " <<  keypointArea << endl;
         //cout << "keypoint response: " << keypoints[i].response << endl;
         //cout << "keypoint angle: " << keypoints[i].angle << endl;
         //cout << "keypoint class_id: " << keypoints[i].class_id << endl;
         //cout << "keypoint octave: " << keypoints[i].octave << endl;

         if (keypointArea > maxKeypointArea)
         {
            maxKeypointArea = keypointArea;
            maxIndex = i;
         }
       }

       Point keyCenter(0,0);
       if (maxKeypointArea > maxArea )
       {
         cout << "Analyzing filename: " << filename_ << endl;
         cout << "containing an image with rows, cols, area = " << im_original.rows << ", " << im_original.cols << ", "
         << im_original.rows * im_original.cols << endl;
         maxArea = maxKeypointArea;
         cout << "Possible target found, from " << keypoints.size() << " keypoints: " << endl;
         cout << "Target center: " << keypoints[maxIndex].pt << endl;
         cout << "Target radius: " << keypoints[maxIndex].size << endl;
         cout << "Target area: " <<  maxKeypointArea << endl;
         //cout << "Target response: " << keypoints[maxIndex].response << endl;
         cout << "Target angle: " << keypoints[maxIndex].angle << endl;
         //cout << "Target class_id: " << keypoints[maxIndex].class_id << endl;
         //cout << "Target octave: " << keypoints[maxIndex].octave << endl;
         centerX_ = (int) keypoints[maxIndex].pt.x;
         centerY_ = (int) keypoints[maxIndex].pt.y;
         rangeSquared_ = FRONT_CAMERA_FULLZOOM_FIRST_TARGET_RANGE_PARAMETER / maxKeypointArea;
         keyCenter.x = centerX_;
         keyCenter.y = centerY_;
       } // closes if > maxArea
       else
       {
         if (keypoints.size() > 0)
         {
            cout << keypoints.size() << " keypoints found, but no better targets" << endl;
            cout << "best candidate from this color was: " << endl;
            cout << "Target center: " << keypoints[maxIndex].pt << endl;
            cout << "Target radius: " << keypoints[maxIndex].size << endl;
            cout << "Target area: " <<  maxKeypointArea << endl;
            keyCenter.x = (int) keypoints[maxIndex].pt.x;
            keyCenter.y = (int) keypoints[maxIndex].pt.y;
         }
         else cout << "no keypoints found" << endl;
       }
      //if (colorCounter == 0) 
      showImg_ = im_original.clone();
      //blankImg.setTo(cv::Scalar(0,0,0));
      if (colorCounter == BLUE)
      {
         circle(showImg_, keyCenter, 25, Scalar(255,0,0), 10);
         //drawKeypoints(showImg, keypoints, showImg, Scalar(255,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      }
      else if (colorCounter == RED) 
      {
         circle(showImg_, keyCenter, 25, Scalar(0,0,255), 10);
         //drawKeypoints(showImg, keypoints, showImg, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      }
      else if (colorCounter == BLUE_RED) 
      {
         circle(showImg_, keyCenter, 25, Scalar(255,0,255), 10);
         //drawKeypoints(showImg, keypoints, showImg, Scalar(255,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      }
      
      Mat resizedImg;
      Size dsize(0,0); //round(fx*src.cols), round(fy*src.rows))}
      resize(showImg_, resizedImg, dsize, 0.25, 0.25, CV_INTER_AREA);
      imshow("keypoints", resizedImg);
      //waitKey(0);
      if (firstTarget) break; // we don't do all the colors for the known white target
   } // closes for loop with colors
   if (maxArea > 0) return true;
   return false;
}

bool sendCenter(outdoor_bot::mainTargets_service::Request  &req, outdoor_bot::mainTargets_service::Response &resp)
{
   bool firstTarget = req.firstTarget;
   filename_ = req.image_filename;  
   if (filename_.compare("memory")) // non-zero return means the compare was not true, so we will use the filename
   {
      image_ = imread(filename_, CV_LOAD_IMAGE_UNCHANGED);
      resp.newWebcamImageReceived = false;   // we dont even check for this if we are using a filename
      resp.newDigcamImageReceived = false;
   }
   else  // if the filename = memory, then we try to read an image that has arrived over image transport
         // rather than reading from disk
   {
      if (newDigcamImageReceived_)
      {
         resp.newDigcamImageReceived = true;
         newDigcamImageReceived_ = false;
         ROS_INFO("mainTargets is using a digcam image via image transport.");
         image_ = newDigcamImage_.clone();
      }
      else if (newWebcamImageReceived_)
      {
         resp.newWebcamImageReceived = true;
         newWebcamImageReceived_ = false;
         image_ = newWebcamImage_.clone();
      }
      else
      {
         resp.newDigcamImageReceived = false;
         resp.newWebcamImageReceived = false;
         resp.centerX = -3;
         resp.centerY = -3;
         resp.rangeSquared = 0;
         ROS_INFO("mainTargets has not received a new image to analyze.");
         return true;
      }

   }
  approxRange_ = req.approxRange;  
   if (detectBlobs(image_, firstTarget))
   {
      resp.centerX = centerX_;
      resp.centerY = centerY_;
      resp.rangeSquared = rangeSquared_;
      ROS_INFO("possible target found, center and rangeSquared = %d, %d, %f", centerX_, centerY_, rangeSquared_);
   }
   else
   {
      resp.centerX = -1;
      resp.centerY = -1;
      resp.rangeSquared = -1.;
      ROS_INFO("sending back main target response: -1, -1, -1.");
   }  
   return true;
}

};  

int main(int argc, char** argv)
{
   ros::init(argc, argv, "mainTargets");
   ros::NodeHandle nh;
   mainTargets mT(nh);
   cv::namedWindow("keypoints", CV_WINDOW_AUTOSIZE);
   cv::startWindowThread();

   ros::ServiceServer service = nh.advertiseService("mainTargetsService", &mainTargets::sendCenter, &mT);
   //if (!mT.matchFeatures()) cout << "no features matched" << endl;
   //else cout << "all done" << endl;
   ros::spin();
   cv::destroyWindow("keypoints");
   return EXIT_SUCCESS;
}
