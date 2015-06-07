#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "outdoor_bot/NavTargets_service.h"
#include "outdoor_bot/NavTargets_msg.h"
#include "outdoor_bot_defines.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define MAX_NUM_TARGETS 50
//#define TARGET_HEIGHT_IN_PIX_AT_3M 150. // small target
#define TARGET_HEIGHT_IN_PIX_AT_10M 700. //large target, digcam, zoom = 7
//#define TARGET_HEIGHT_IN_PIX_AT_10M 64. //large target, webcam
//#define TARGET_HEIGHT_IN_PIX_AT_10M 1211. //large target, zoom_digcam, zoom = 7
//#define PIX_PER_CM_AT_3M 8.
//#define TARGET_HEIGHT_CM 78. //19.

#define NUM_VERTICES 6
#define MAX_MATCH_VALUE 0.1
#define BIGGEST_SMALL_ANGLE_COS 0.42
#define SMALLEST_SMALL_ANGLE_COS 0.28
#define BIGGEST_BIG_ANGLE_COS 0.63
#define SMALLEST_BIG_ANGLE_COS 0.50
#define MIN_AREA 800.
#define MAX_AREA 12000.
#define MIN_RATIO_AREA_ARCLENGTH 5. //10.
#define MAX_RATIO_AREA_ARCLENGTH 120. 
#define MIN_ASPECT_RATIO 0.55
#define MAX_ASPECT_RATIO 0.75

class NavTargets
{
private:
   ros::NodeHandle nh_;
   //ros::ServiceServer service_;
   ros::Publisher home_center_pub_, image_received_pub_;
   ros::Subscriber analyzeImage_sub_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber home_image_sub_;
	//void FindNavTargets(int numVertices, double MinAngle, double MaxAngle, int MinArea, int MaxArea);
	double angle_( Point* pt1, Point* pt2, Point* pt0 );
   float range_;
	int centerX_, centerY_, height_, width_, areaAvg_, numPolygons_;
	int circleCenterX_, circleCenterY_;
	bool foundMultipleTargets_, newHomeImageReceived_;
   std::string filename_;
   Mat newHomeImage_, image_;
	//IplImage * CreateTemplate(IplImage *InternalTarget);
	//bool FindTemplate(IplImage * TargetImage, IplImage * TemplateImage, float Threshold);
	//bool FindDarkSquare(IplImage * TargetImage, int SquareSizeFactor, float Threshold);

public:

NavTargets(ros::NodeHandle &nh)
   :  nh_(nh), it_(nh)
{
      //service_ = nh.advertiseService("NavTargetsService", &NavTargets::sendCenter, this);
   	analyzeImage_sub_ = nh.subscribe("NavTargets_cmd", 10, &NavTargets::commandCallback, this);
      home_image_sub_ = it_.subscribe("home_target_image", 1, &NavTargets::homeImageCallback, this);
   	home_center_pub_ = nh_.advertise<outdoor_bot::NavTargets_msg>("Home_target_center", 50);
      image_received_pub_ = nh_.advertise<std_msgs::Int32>("nav_target_image_received", 5);
      
      range_ = 0.;
      //newHomeImageReceived_ = false;
      //filename_ = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0";
      //filename_ = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/hexagon_cropped";
      //std::string filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam0.jpg";
      //std::string filename = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/hexagon_cropped.jpg";
      //Mat image = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
      //waitKey(30);
      // GetNavTargets parameters are:
      // Mat NavImage, int numVertices, double MaxMatchValue, BiggestSmallAngleCos, SmallestSmallAngleCos,
      // BiggestBigAngleCos, SmallestBigAngleCos, MinArea, MaxArea, MinRatioAreaArcLength,
      // MaxRatioAreaArcLength, MinAspectRatio, MaxAspectRatio,
		// bool MultipleTargets, bool InsideTarget)
      // double maxArea = image.rows * image.cols * 0.75;  // scales with image area
      //double minArea = 5000.; //maxArea / 400.;
      //if (minArea < 500.) minArea = 500;
      //double minRatioAreaArcLength = .004 * (image.rows + image.cols); // this ratio scales with length
      //double maxRatioAreaArcLength = 0.1 * (image.rows + image.cols); 
      //cout << "maxArea, minArea, minRatio, maxRatio = " << maxArea << ", " << minArea << ", "
      //   << minRatioAreaArcLength << ", " << maxRatioAreaArcLength << endl;
      //GetNavTargets(image, 6, 0.05, 0.42, 0.28, 0.63, 0.5, minArea, maxArea,
      //   minRatioAreaArcLength, maxRatioAreaArcLength, 0.55, 0.72, false, false);
      //GetNavTargets(image, 6, 0.05, 0.42, 0.28, 0.63, 0.50, 5000., 3000000.,
      //     10., 120., 0.55, 0.75, false, false);

      //GetNavTargets(image, 6, 0.2, 0.45, 0.28, 0.7, 0.49, 100., 2000000.,
    //     1., 1000., 0.3, 1.1, false, false);
}
~NavTargets(void)
{
}

void homeImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("home image received in NavTargets");
  try
  {
    //cv::imshow("keypoints", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv::waitKey(30);
    newHomeImage_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    newHomeImageReceived_ = true;
    std_msgs::Int32 imMsg;
    imMsg.data = HOMECAM;
    image_received_pub_.publish(imMsg);   // publish that we received a home image
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


void commandCallback(const std_msgs::String::ConstPtr& msg)
{
        cout << "analysing image for home target in NavTargets" << endl;
        newHomeImageReceived_ = false;
        
        //if (GetNavTargets(newHomeImage_, 6, 0.05, 0.42, 0.28, 0.63, 0.50, 5000., 3000000.,10., 120., 0.55, 0.75, false, false))
        
        if (GetNavTargets(newHomeImage_, NUM_VERTICES, MAX_MATCH_VALUE, BIGGEST_SMALL_ANGLE_COS,
	SMALLEST_SMALL_ANGLE_COS, BIGGEST_BIG_ANGLE_COS, SMALLEST_BIG_ANGLE_COS,
	MIN_AREA, MAX_AREA, MIN_RATIO_AREA_ARCLENGTH, MAX_RATIO_AREA_ARCLENGTH,
	MIN_ASPECT_RATIO, MAX_ASPECT_RATIO, false, false))
        {
           ROS_INFO("Found home target: (x,y), range = (%d, %d), %f", centerX_, centerY_, range_);
        }
        else ROS_INFO("Center not found in home target image");
        // note that getNavTargets publishes the results, so we do not have to do that here
}

/*
bool sendCenter(outdoor_bot::NavTargets_service::Request  &req, outdoor_bot::NavTargets_service::Response &resp)
{
   filename_ = req.image_filename;   // this might be a ROS string, not a std string
   //std::string filename = req.image_filename.data.c_str();   // this might be a ROS string, not a std string
   //std::string filename = req.image_filename.c_str();   // this might be a ROS string, not a std string

   Mat image = imread(filename_, CV_LOAD_IMAGE_UNCHANGED);
   if (GetNavTargets(image, 6, 0.05, 0.42, 0.28, 0.63, 0.50, 5000., 3000000.,10., 120., 0.55, 0.75, false, false))
   {
      resp.centerX = centerX_;
      resp.centerY = centerY_;
      resp.range = range_;
      ROS_INFO("sending back nav target response: %d, %d, %f", centerX_, centerY_, range_);
   }
   else
   {
      resp.centerX = -1;
      resp.centerY = -1;
      resp.range = -1.;
      ROS_INFO("sending back nav target response: -1, -1, -1.");
   }
   
   return true;
}
 
   // got an image of the home target.  analyze it and publish the center point
   void homeTargetCallback(const sensor_msgs::ImageConstPtr& msg)
   {
      //ROS_INFO("image received");
      Mat home_image;
      try
      {
       home_image = cv_bridge::toCvShare(msg, "bgr8")->image;
       //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
       //cv::waitKey(30);
      }
      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
         return;       
      }
      if (GetNavTargets(home_image, 6, 0.05, 0.42, 0.28, 0.63, 0.50, 5000., 3000000.,10., 120., 0.55, 0.75, false, false))
      {
         ROS_INFO("Found home target: (x,y), range = (%d, %d), %f", centerX_, centerY_, range_);
      }
      else ROS_INFO("Center not found in home target image");
   }
*/ 
double angle( Point* pt1, Point* pt2, Point* pt3 )
{
    double dx1 = pt1->x - pt2->x;
    double dy1 = pt1->y - pt2->y;
    double dx2 = pt2->x - pt3->x;
    double dy2 = pt2->y - pt3->y;
    return (dx1*dx2 + dy1*dy2)/(sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2)) + 1e-10);
}

/*
void GetNavParameters(int *pXcenter, int *pYcenter, int *pHeight, int *pWidth, int *pArea, int *pNumPolygons, bool *pMultipleTargets)
{
	 *pXcenter = centerX_;   // assign the values to the passed parameters
    *pYcenter = centerY_;	// if no polygons detected, these values will be -1, the initial values
    *pHeight = height_;
    *pWidth = width_;
    *pArea = areaAvg_;
    *pNumPolygons = numPolygons_;
    *pMultipleTargets = m_FoundMultipleTargets;
}



CvRect NavTargets::GetTargetRectangle()
{
	CvRect TargetRectangle;
	int HalfWidth = (int) ( (width_ - ((int) (0.1 * width_))) /2);  // get rid of borders
	TargetRectangle.x = centerX_ - HalfWidth;  // left is minimum x
	TargetRectangle.y = centerY_ - HalfWidth;	// top is minimum y, we want a square, so use Width here
	TargetRectangle.width = 2 * HalfWidth;
	TargetRectangle.height = 2 * HalfWidth;  // again, use width to get a square
	return TargetRectangle;
}

*/

bool GetNavTargets(Mat NavImage, int numVertices, double MaxMatchValue,
   double BiggestSmallAngleCos, double SmallestSmallAngleCos,
   double BiggestBigAngleCos, double SmallestBigAngleCos,
   int MinArea, int MaxArea,
   double MinRatioAreaArcLength, double MaxRatioAreaArcLength,
   double MinAspectRatio, double MaxAspectRatio,
	bool MultipleTargets, bool InsideTarget)
{
   if (numVertices < 1) {
      ROS_ERROR("Number of Vertices requested is less than 1, exiting NavTargets \n");
      return false;
   }

   vector<Point> hexagon; //contours_poly[i].size());
   // image data, cosines = 0.56 (x4) and 0.37 (x2)
   Point p0(184,26);
   Point p1(40, 124);
   Point p2(40, 421);
   Point p3(184, 519);
   Point p4(328, 421);
   Point p5(328, 124); 
   /*
   // test angles, cosines = -0.87, -0.5, -0.87, 0.5, -0.71, 0.71
   Point p0(0,0);
   Point p1(1000, 0);
   Point p2(0, 577);
   Point p3(0,0);
   Point p4(-577, 1000);
   Point p5(-1000, 1000);
 
   // model home target, aspect ratio = .667, width = 200, center = (320,240)
   // cosines = 0.45 (x4) and 0.6 (x2)
   Point p0(320,390);
   Point p1(420, 340);
   Point p2(420, 140);
   Point p3(320, 90);
   Point p4(220, 140);
   Point p5(220, 340);
   */
   hexagon.push_back(p0);
   hexagon.push_back(p1);
   hexagon.push_back(p2);
   hexagon.push_back(p3);
   hexagon.push_back(p4);
   hexagon.push_back(p5);

   vector<vector<Point> > target_contour;
   Point target_center[MAX_NUM_TARGETS];
   double target_radius[MAX_NUM_TARGETS];
   Rect target_Rect[MAX_NUM_TARGETS];
   double target_match_value[MAX_NUM_TARGETS];
   //double target_aspectRatio[MAX_NUM_TARGETS];
   double target_area[MAX_NUM_TARGETS];
   //int target_threshold[MAX_NUM_TARGETS], target_color[MAX_NUM_TARGETS];
   //double target_bigAngleCos[MAX_NUM_TARGETS], target_smallAngleCos[MAX_NUM_TARGETS];

   vector<vector<Point> > contours;
   int numTargetsFound = 0;
   vector<Vec4i> hierarchy;
   int NumberOfThresholdLevelsToCheck = 11, NumberOfColorsToCheck = 1;  // blue, geen, red
   int UpperCannyThreshold = 50, LowerCannyThreshold = 0;  // LowerCannyThreshold = 0 forces edges merging

   CvSize sz = cvSize( NavImage.cols, NavImage.rows);
   Mat img = NavImage.clone(); // make a clone of input image so we can freely change the image itself
   //imshow("view", img);
   Mat pyr(cvSize(sz.width/2, sz.height/2), CV_8UC3); // depth 8, 3 channels for downscaling
   Mat drawing = Mat::zeros(sz, CV_8UC3 ); // for showing images

   // down-scale and upscale the image to filter out some noise
   pyrDown( img, pyr, cvSize(sz.width/2, sz.height/2)  );
   pyrUp( pyr, img, sz );

   Mat img_YUV(sz, CV_8UC3); // to hold the YUV version of the image
   //Mat gray(sz, CV_8UC1); // depth 8, 1 channel, gray version
   //cvtColor(img, gray, CV_BGR2GRAY);
   cvtColor(img, img_YUV, COLOR_BGR2YCrCb);
   vector<Mat> planes_YUV;
   //vector<Mat> planes_BGR;
   split(img_YUV, planes_YUV);  // split into separate color planes (Y U V)
   //split(img, planes_BGR);       // split into separate color planes (B G R)
   /*  
   Mat timg_B = planes_BGR[0];
   Mat timg_G = planes_BGR[1];
   Mat timg_R = planes_BGR[2];
   Mat timg_Y = planes_YUV[0];
   Mat timg_U = planes_YUV[1];
   Mat timg_V = planes_YUV[2];


   imshow("view", NavImage);
   waitKey(0);
   imshow("view", timg_B);
   waitKey(0);
   imshow("view", timg_G);
   waitKey(0);
   imshow("view", timg_R);
   waitKey(0);
   imshow("view", timg_Y);
   waitKey(0);
   imshow("view", timg_U);
   waitKey(0);
   imshow("view", timg_V);
   waitKey(0);
   */ 

   //namedWindow( "YUV", CV_WINDOW_AUTOSIZE );
   //namedWindow( "BGR", CV_WINDOW_AUTOSIZE );

   // find polygons
   for(int c = 0; c < NumberOfColorsToCheck; c++ )
   {
      // extract the c-th color plane
      Mat YUV = planes_YUV[c];
      //Mat BGR = planes_BGR[c];

      // try several threshold levels
      
      
      
      //***********************************************************************************
      // skipping canny because too many contours from digcam makes the routine take too long***************
      //***************************************************************
      
      
      for( int l = 1; l < NumberOfThresholdLevelsToCheck - 2; l++ ) 
      // also skip first and last few thresholds.  By doing this instead of changing
      // NumberOfThresholdLevelsToCheck we keep the distance between checked thresholds smaller
      {
         //printf("For color = %d, threshold = %d \n",c,l);
         Mat YUVdup = YUV.clone();
         //Mat BGRdup = BGR.clone();
         //  imshow("BGR", BGRdup);
         // hack: use Canny instead of zero threshold level.
         // Canny helps to catch squares with gradient shading
         if( l == 0 )
         {
            // apply Canny. Take the thresholds from UpperCannyThreshold and LowerCannyThreshold
            // LowerCannyThreshold = 0 forces edges merging
            Canny( YUVdup, YUVdup, LowerCannyThreshold, UpperCannyThreshold);
            //Canny( BGRdup, BGRdup, LowerCannyThreshold, UpperCannyThreshold);
            // dilate canny output to remove potential holes between edge segments
            dilate( YUVdup, YUVdup, Mat()); //NULL);
            //dilate( BGRdup, BGRdup, NULL);
         }
         else
         {
             threshold( YUVdup, YUVdup, (l+1)*255/NumberOfThresholdLevelsToCheck, 255, CV_THRESH_BINARY );
             //threshold( BGRdup, BGRdup, (l+1)*255/NumberOfThresholdLevelsToCheck, 255, CV_THRESH_BINARY );
         }

         // show canny or threshold image
         //imshow("YUV", YUVdup);


         Mat YUVcontours = YUVdup.clone();
         // find contours and store them all as a list
         findContours( YUVcontours, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

         // Draw contours
         namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
         RNG rng(12345);
         
         for(unsigned int i = 0; i < contours.size(); i++ )
         {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );      
         }

         // Show contours
         // imshow( "Contours", drawing );
         // printf("see contour, %d contours found \n", contours.size());

         for(unsigned int i = 0; i < contours.size(); i++ )
         {
            // Find closed polygons with accuracy proportional to the contour perimeter
            double  arc_length = arcLength(contours[i], true);
            vector<Point> contours_poly;
            approxPolyDP(contours[i], contours_poly, arc_length * 0.02, true );
            
            //cout << "contour " << i << " has " << contours[i].size() << " points, arc length =  "
            //   << arc_length << " and its polygon has " 
            //  << contours_poly.size() << " points" << endl;
            
            if ( ((int) contours_poly.size()) != numVertices)
            {
               //cout << "rejected contour for vertices " << endl;
               continue;
            }

            if (!isContourConvex(contours_poly))
            {
               //cout << "rejected contour for convexity " << endl;
               continue;
            }

            double aspectRatio;
            Rect boundRect = boundingRect(contours_poly); 
            if (boundRect.height > 0)
            {
               aspectRatio = ((double) boundRect.width) / ((double)boundRect.height);
               if (aspectRatio < MinAspectRatio || aspectRatio > MaxAspectRatio)
               {
                  //cout << "rejected contour for aspect ratio = " << aspectRatio << endl;
                  continue; 
               }
            }
            else continue;

            double areaContour = fabs(contourArea(contours_poly));
            if (areaContour < MinArea || areaContour > MaxArea
               || areaContour / arc_length < MinRatioAreaArcLength 
               || areaContour / arc_length  > MaxRatioAreaArcLength)
            {
               //cout << "rejected contour for area or area to arc length ratio " << endl;
               //cout << "area = " << areaContour << ", arc length ratio = " << areaContour / arc_length << endl;
               continue; 
            }  

            double match_value = matchShapes(hexagon, contours_poly, CV_CONTOURS_MATCH_I1, 0.);
            //printf("for contour %d, match value = %f \n", i, match_value);
            // note that polygons with just 2 sides return a match_value = 0, a very good match value
            
            if (match_value > MaxMatchValue) // only look for good matches
            {
               //cout << "rejected contour for match value =  " << match_value << endl;
               continue; 
            }
               
			   // find cos of angles between joint edges
            std::vector<double> cos_vertices;
            for (int j=0; j < numVertices; j++)
            {            
               // Get the angle (in cosines) of all vertices
               Point p0 = contours_poly[j];
               Point p1 = contours_poly[(j+1)%numVertices];
               Point p2 = contours_poly[(j+2)%numVertices];
               double result = fabs(angle(&p0, &p1, &p2));
               cos_vertices.push_back(result);
               //cout << "for j = " << j << ", angle = " << result << endl;
            }
            // Sort ascending the corner degree values
            double smallAngles = 0., bigAngles = 0.;
            std::sort(cos_vertices.begin(), cos_vertices.end());
            for (int j=0; j < 2; j++) smallAngles += cos_vertices[j];
            for (int j=2; j < numVertices; j++) bigAngles += cos_vertices[j];
            if (smallAngles < SmallestSmallAngleCos * 2. ||
               smallAngles > BiggestSmallAngleCos * 2. ||
               bigAngles < SmallestBigAngleCos * 4.  ||
               bigAngles > BiggestBigAngleCos * 4.)
            {
               //cout << "contour rejected for angles: small, big = "
               //   << smallAngles /2. << ", " << bigAngles / 4. << endl;
               continue;
            }
               
            //cout << "this contour looks like a target: c, l = " << c << ", " << l << endl;
            Point2f centerContour;
            float radiusContour;
            minEnclosingCircle(contours_poly, centerContour, radiusContour);
            if (numTargetsFound < MAX_NUM_TARGETS)
            {                                             
               target_contour.push_back(contours_poly);
               target_center[numTargetsFound] = centerContour;
               target_radius[numTargetsFound] = radiusContour;
               target_Rect[numTargetsFound] = boundRect;
               target_match_value[numTargetsFound] = match_value;
               //target_aspectRatio[numTargetsFound] = aspectRatio;
               target_area[numTargetsFound] = areaContour;
               //target_threshold[numTargetsFound] = l;
               //target_color[numTargetsFound] = c;
               //target_bigAngleCos[numTargetsFound] = bigAngles / 4.;
               //target_smallAngleCos[numTargetsFound] = smallAngles / 2.;
            }
            numTargetsFound++;

            cout << "area, ratio = " << areaContour << ", " << areaContour / arc_length << endl;
            cout << "aspect ratio = " << aspectRatio << endl;
            cout << "angles: small, big = " << smallAngles / 2. << ", " << bigAngles / 4. << endl;
            cout << "match value = " << match_value << endl;
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( img, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            circle(img, centerContour, (int) radiusContour, color, 2, 8, 0);
            rectangle(img, boundRect.tl(), boundRect.br(), color, 2, 8, 0);
            // draw the individual vertex points
            for (int j=0; j < numVertices; j++)
            {
              circle(img, contours_poly[j], 5, color, 2, 8, 0);
               printf("vertex coordinates are: %d, %d \n", contours_poly[j].x, contours_poly[j].y);
            }                       
         }
      }
   }

   cout << "Found " << numTargetsFound << " possible contours for the target" << endl;
   if (numTargetsFound < 1)
   {
      cout << "No nav targets found" << endl;
      outdoor_bot::NavTargets_msg fail_message;
      fail_message.centerX = -1;
      fail_message.centerY = -1;
      fail_message.range = -1;
      home_center_pub_.publish(fail_message);
      return false;
   }
   // make sure that we are looking at a single target
   double avgCenterX = 0., avgCenterY = 0., totalCenterX = 0., totalCenterY = 0.;
   for (int i=0; i < numTargetsFound; i++)
   {
      totalCenterX += target_center[i].x;
      totalCenterY += target_center[i].y;
   }
   avgCenterX = totalCenterX / numTargetsFound;
   avgCenterY = totalCenterY / numTargetsFound;
   cout << "Average center x,y = (" << (int) avgCenterX << ", " << (int) avgCenterY << ")" << endl;

   double best_match_value = 100.;
   int bestContour = 0;
   for (int i=0; i < numTargetsFound; i++)
   {
      if (target_match_value[i] < best_match_value)
      {
         // the center should not be more than 10% away from the average center
         if (fabs(target_center[i].x - avgCenterX) < ((double)NavImage.cols) / 10.
            && fabs(target_center[i].y - avgCenterY) < ((double)NavImage.rows) / 10.)
         {
            best_match_value = target_match_value[i];
            bestContour = i;
         }
         else
         {
            cout << "Multiple targets detected" << endl;
            cout << "Average center x,y = " << avgCenterX << ", " << avgCenterY << endl;
            cout << "Target contour " << i << " center x,y = " << target_center[i].x << ", " 
               << target_center[i].y << endl;
            numTargetsFound--;
         }
      }
   }
   if (numTargetsFound < 1)
   {
      outdoor_bot::NavTargets_msg fail_message;
      fail_message.centerX = -1;
      fail_message.centerY = -1;
      fail_message.range = -1;
      home_center_pub_.publish(fail_message);
      cout << "No nav targets found" << endl;
      return false;
   }
   // now can publish targetCenter[bestContour] as a detected target center point
   centerX_ = target_center[bestContour].x;
   centerY_ = target_center[bestContour].y;
   int maxY = -1;
   int minY = 1000;
   for (int j=0; j < numVertices; j++)
   {
      if (target_contour[bestContour][j].y > maxY) maxY = target_contour[bestContour][j].y;
      else if (target_contour[bestContour][j].y < minY) minY = target_contour[bestContour][j].y;
   }
   range_ = 10. * (TARGET_HEIGHT_IN_PIX_AT_10M) / ((float)(maxY - minY));
   outdoor_bot::NavTargets_msg home_center_message;
   home_center_message.centerX = centerX_;
   home_center_message.centerY = centerY_;
   home_center_message.totalX = NavImage.cols;
   home_center_message.range = range_;
   home_center_pub_.publish(home_center_message);
   cout << "home center published = (" << centerX_ << ", " << centerY_ << ")" << endl;
   cout << "home range published = " << range_ << " m"  << endl;
   cout << "target area = " << target_area[bestContour] << endl;
   cout << "target match value = " << target_match_value[bestContour] << endl;  

   // draw it on top of the input image, along with the bounding rectangle and circle
   drawContours(img, target_contour, bestContour, Scalar(0,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
   circle(img, target_center[bestContour], (int) target_radius[bestContour], Scalar(0,0,255), 2, 8, 0);
   rectangle(img, target_Rect[bestContour].tl(), target_Rect[bestContour].br(), Scalar(0,0,255), 2, 8, 0);
   
   //imshow( "view", img );
 
   // here is an example of how to draw a single contour, for example vector<Point> bestContour
   //vector<vector<Point> > contourVec;
   //contourVec.push_back(target_contour[bestContour]);
   //drawContours( img, contourVec, 0, Scalar(0,0,255), 1, 8, vector<Vec4i>(), 0, Point() );

   std::string filename_contour = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/contour.jpg";
   vector<int> compression_params;
   compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
   compression_params.push_back(100);
   try {
      imwrite(filename_contour, img, compression_params);
   }
   catch (runtime_error& ex) {
      cout << "Exception converting image to JPG format: " <<  ex.what() << endl;
      return true; // we didn't write the file, but we did find a nav target
   }
   cout << "Saved image file, " << filename_contour << " with contours." << endl;
   //cvWaitKey(0);
   return true;
}

};

/*
int NavTargets::RecognizeTarget(IplImage *pMyImage)
{
	//cvNamedWindow("ROI",1);
	wxString TempString;
	bool PolygonPresent;
	//bool MultipleTargets = true;
	int i, QuadrantEdgeLength, ResultValue = 0, location = 0;
	CvRect ROI = GetTargetRectangle();
	IplImage *pImageROIQuadrant = NULL;

	cvSetImageROI(pMyImage, ROI);
	IplImage *pMyImageROI	= cvCreateImage(cvSize(ROI.width, ROI.height), 8, 3);
	IplImage *pMyDummyImage	= cvCreateImage(cvSize(ROI.width, ROI.height), 8, 3);
	cvCopyImage(pMyImage, pMyDummyImage);
	cvConvertImage(pMyDummyImage, pMyImageROI, 1);  // for some reason, image flips verically with cvCopy
									//	from captured images, but perhaps not from file images??????
									//	if so, we may have to flip images either after capture or before writing
									//	to file...........  For now, use the convert function  to flip vertically
									// also, subseqent cvCopy calls do not flip.  weird.
*/
	/*TempString << "\nX, Y = ";
	TempString << ROI.x;
	TempString << ", ";
	TempString << ROI.y;
	TempString << "\nWidth, Height = ";
	TempString << ROI.width;
	TempString << ", ";
	TempString << ROI.height;
	TempString << "\nArea = ";
	TempString << areaAvg_;
	std::cout << std::string(TempString.mb_str()) << std::endl;
	*/

/*
	//MultipleTargets = true;
	//int AreaAvgWholeTarget = areaAvg_;  // use this because GetNavTargets will change areaAvg_ to correspond
								// to the target square rather than the area inside the whole target
	//PolygonPresent = GetNavTargets(pMyImageROI,4,0,1,(int) (AreaAvgWholeTarget/100),(int) (AreaAvgWholeTarget/20),
		// 0.5, 1.5, MultipleTargets, true);

	//ShowNavTargets();
   // cvWaitKey(0);

   //IplImage *TemplateImage = CreateTemplate(pMyImageROI);
   //PolygonPresent = FindTemplate(pMyImageROI, TemplateImage, 1.5);
    PolygonPresent = FindDarkSquare(pMyImageROI, 8, 1.5);

	//cvShowImage("ROI", pMyImageROI);
	//cvWaitKey(0);
*/
	/*
	TempString = "\nTemplate Image";
	TempString << "\nWidth, Height = ";
	TempString << TemplateImage->width;
	TempString << ", ";
	TempString << TemplateImage->height;
	TempString << "\nArea = ";
	TempString << TemplateImage->width * TemplateImage->height;
	std::cout << std::string(TempString.mb_str()) << std::endl;
	cvShowImage("ROI",TemplateImage);
	cvWaitKey(0);
	*/

/*
	if (! PolygonPresent)
	{
		TempString = (_T("\nNo polygon in entire ROI"));
		std::cout << std::string(TempString.mb_str()) << std::endl;
		return location;
	}

	QuadrantEdgeLength = (int) (ROI.width/2);

	for (i=0; i<4; i++)
	{
		//m_pGeneralTextBox->AppendText("\nIn the ");
		switch (i)
		{
			case 0:
				ROI = cvRect(0, 0, QuadrantEdgeLength, QuadrantEdgeLength);   // UL quadrant
				//m_pGeneralTextBox->AppendText("upper left ");
				ResultValue = 1;
				break;
			case 1:
				ROI = cvRect(QuadrantEdgeLength - 1, 0, QuadrantEdgeLength, QuadrantEdgeLength);   // UR quadrant
				//m_pGeneralTextBox->AppendText("upper right ");
				ResultValue = 2;
				break;
			case 2:
				ROI = cvRect(QuadrantEdgeLength - 1, QuadrantEdgeLength - 1, QuadrantEdgeLength, QuadrantEdgeLength);   // LR quadrant
				//m_pGeneralTextBox->AppendText("lower right ");
				ResultValue = 8;
				break;
			case 3:
				ROI = cvRect(0, QuadrantEdgeLength - 1,QuadrantEdgeLength, QuadrantEdgeLength);   // LL quadrant
				//m_pGeneralTextBox->AppendText("lower left ");
				ResultValue = 4;
				break;
		}
		cvSetImageROI(pMyImageROI, ROI);
		pImageROIQuadrant	= cvCreateImage(cvSize(ROI.width, ROI.height), 8, 3);
		cvCopy(pMyImageROI, pImageROIQuadrant);
*/
	/*
	TempString = "\nQuadrant Image";
	TempString << "\nWidth, Height = ";
	TempString << pImageROIQuadrant->width;
	TempString << ", ";
	TempString << pImageROIQuadrant->height;
	TempString << "\nArea = ";
	TempString << pImageROIQuadrant->width * pImageROIQuadrant->height;
	std::cout << std::string(TempString.mb_str()) << std::endl;
	cvShowImage("ROI",pImageROIQuadrant);
	cvWaitKey(0);
	*/

/*


		//PolygonPresent = GetNavTargets(pImageROIQuadrant,4,0,0.2,(int) (AreaAvgWholeTarget/100),
		// 	(int) (AreaAvgWholeTarget/20), 0.8, 1.2, MultipleTargets, true);

		 //PolygonPresent = FindTemplate(pImageROIQuadrant, TemplateImage, 1.5);
		 PolygonPresent = FindDarkSquare(pImageROIQuadrant, 8, 1.5);

	//ShowNavTargets();
		if (PolygonPresent)
		{
			//m_pGeneralTextBox->AppendText("a polygon was detected.");
			//TempString = "\nIt's Area = ";
			//TempString << areaAvg_;
			//TempString << "\nWidth, Height = ";
			//TempString << width_;
			//TempString << ", ";
			//TempString << height_;
			//std::cout << std::string(TempString.mb_str()) << std::endl;

			location = location + ResultValue;  // based on powers of 2, so we have 16 locations
		}
		//else m_pGeneralTextBox->AppendText("there was no polygon.");
		cvReleaseImage(&pImageROIQuadrant);
	}

	//cvReleaseImage (&pImageROIQuadrant);
	cvReleaseImage (&pMyImageROI);
	cvReleaseImage (&pMyDummyImage);
	//cvDestroyWindow("ROI");
	return location;
}

bool NavTargets::FindTemplate(IplImage * TargetImage, IplImage * TemplateImage, float Threshold)
{
	//wxString TempString;
	int TemplateWidth = TemplateImage->width;
	int TemplateHeight = TemplateImage->height;
	int ResultWidth = (TargetImage->width - TemplateWidth) + 1;
	int ResultHeight = (TargetImage->height - TemplateHeight) + 1;
	int i,j, idelta, jdelta=0;
	float Value = 0, MinValue = 10e20, AvgValue = 0;
	CvPoint MinLocation;

	IplImage *TargetGray, *TemplateGray;
	if (TargetImage->nChannels == 1) TargetGray = cvCloneImage(TargetImage);   // gray image sent
	else
	{
		TargetGray = cvCreateImage(cvSize(TargetImage->width,TargetImage->height), IPL_DEPTH_8U, 1);
		cvCvtColor(TargetImage, TargetGray, CV_RGB2GRAY);
	}

	if (TemplateImage->nChannels == 1) TemplateGray = cvCloneImage(TemplateImage);  // gray image sent
	else
	{
		TemplateGray = cvCreateImage(cvSize(TemplateImage->width,TemplateImage->height), IPL_DEPTH_8U, 1);
		cvCvtColor(TemplateImage, TemplateGray, CV_RGB2GRAY);
	}

	BwImage Target(TargetGray);
	BwImage Template(TemplateGray);
	for (idelta = 0; idelta < ResultWidth; idelta++)
	{
		for (jdelta = 0; jdelta < ResultHeight; jdelta++)
		{
			// could use cvDotProduct with appropriate ROI instead of these two loops
			// then could get rid of BwImage stuff
			for (i=0; i < TemplateWidth; i++)
			{
				for (j=0; j < TemplateHeight; j++)
				{
					Value = Value + (Target[i + idelta][j + jdelta]) * (Template[i][j]);
				}
			}
			AvgValue = AvgValue + Value;
			if (Value < MinValue)
			{
				MinValue = Value;
				MinLocation.x = idelta + TemplateWidth/2;
				MinLocation.y = jdelta + TemplateHeight/2;
			}
			Value = 0;
		}
	}
*/
	/*AvgValue = AvgValue/(idelta*jdelta);
	TempString = "\nMin value = ";
	TempString << MinValue;
	TempString << "\nAverage Value = ";
	TempString << AvgValue;
	TempString << "\nRatio = ";
	TempString << MinValue/AvgValue;
	TempString << MinLocation.x;
	TempString << ", ";
	TempString << MinLocation.y;
	std::cout << std::string(TempString.mb_str()) << std::endl;
	*/
/*
	cvReleaseImage(&TargetGray);
	cvReleaseImage(&TemplateGray);
	if (AvgValue > Threshold * MinValue) return true;
	return false;
}

bool NavTargets::FindDarkSquare(IplImage * MyImage, int SquareSizeFactor, float Threshold)
{
	//wxString TempString;
	IplImage *TargetImage = cvCloneImage(MyImage);
	int TemplateWidth = (TargetImage->width)/SquareSizeFactor;
	int TemplateHeight = (TargetImage->height)/SquareSizeFactor;
	int ResultWidth = (TargetImage->width - TemplateWidth) + 1;
	int ResultHeight = (TargetImage->height - TemplateHeight) + 1;
	int idelta, jdelta=0;
	float Value = 0, MinValue = 10e20, AvgValue = 0;
	double NormValue = 0;
	CvPoint MinLocation;
	CvRect ROI = cvRect(0,0,TemplateWidth, TemplateHeight);

	for (idelta = 0; idelta < ResultWidth; idelta++)
	{
		for (jdelta = 0; jdelta < ResultHeight; jdelta++)
		{
			ROI.x = idelta;
			ROI.y = jdelta;
			cvSetImageROI(TargetImage, ROI);

			//NormValue = cvAvg(TargetImage);  // here NormValue has to be CvScalar becasue Avg returns
									// separate values for each channel
			NormValue = cvNorm(TargetImage, NULL, CV_L1);
			Value = (float) NormValue;
			AvgValue = AvgValue + Value;
			if (Value < MinValue)
			{
				MinValue = Value;
				MinLocation.x = idelta + TemplateWidth/2;
				MinLocation.y = jdelta + TemplateHeight/2;
			}
		}
	}

	AvgValue = AvgValue/(idelta*jdelta);
*/
	/*TempString = "\nMin value = ";
	TempString << MinValue;
	TempString << "\nAverage Value = ";
	TempString << AvgValue;
	TempString << "\nRatio = ";
	TempString << MinValue/AvgValue;
	TempString << "\nMin Location x,y = ";
	TempString << MinLocation.x;
	TempString << ", ";
	TempString << MinLocation.y;
	std::cout << std::string(TempString.mb_str()) << std::endl;
	*/
/*
	cvReleaseImage(&TargetImage);
	if (AvgValue > Threshold * MinValue) return true;
	return false;
}
*/
/*  not needed for standard nav target, since FindDarkSquare works better than FindTemplate, which uses this
IplImage * NavTargets::CreateTemplate(IplImage *InternalTarget)
{
	// template is a square 1/36 the area of the square internal target
	int Width = (int) ((InternalTarget->width)/8);
	int Height = (int) ((InternalTarget->height)/8);
	IplImage *Template = cvCreateImage(cvSize(Width,Height),IPL_DEPTH_8U,1);
	cvFloodFill(Template, cvPoint(0,0), cvScalarAll(255), cvScalarAll(255), cvScalarAll(255));  // make the entire image white
	//int i,j;
	//BwImage TemplatePixels(Template);
	//for (i=0;i<Width;i++) {for (j=0;j<Height;j++) TemplatePixels[i][j] = 255; }
	return Template;
}
*/

/*
bool NavTargets::GetCircleTarget(IplImage *img, int *pCircleCenterX, int *pCircleCenterY, int *pCircleRadius)
{
		wxString TempString;
		float* p;
		IplImage* gray = cvCreateImage( cvGetSize(img), 8, 1 );
        CvMemStorage* storage = cvCreateMemStorage(0);
        cvCvtColor( img, gray, CV_BGR2GRAY );
        cvSmooth( gray, gray, CV_GAUSSIAN, 9, 9 ); // smooth it, otherwise a lot of false circles may be detected
        CvSeq* circles = cvHoughCircles( gray, storage, CV_HOUGH_GRADIENT, 2, gray->height/4, 200, 100 );
        int i;
		TempString = _T("\nTotal number of circles = ");
		TempString << circles->total;
		std::cout << std::string(TempString.mb_str()) << std::endl;
		if (circles->total == 0) return false;
        for( i = 0; i < circles->total; i++ )
        {
             p = (float*)cvGetSeqElem( circles, i );
             cvCircle( img, cvPoint(cvRound(p[0]),cvRound(p[1])), 3, CV_RGB(0,255,0), -1, 8, 0 );
             cvCircle( img, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0 );
        }
        cvShowImage( "Nav Targets", img );
		*pCircleCenterX = (int) (p[0] + 0.5);
		*pCircleCenterY = (int) (p[1] + 0.5);
		*pCircleRadius = (int) (p[2] + 0.5);
		TempString = _T("\nCircle center x,y = ");
		TempString << *pCircleCenterX;
		TempString << _T(", ");
		TempString << *pCircleCenterY;
		TempString << _T("\nRadius = ");
		TempString << *pCircleRadius;
		std::cout << std::string(TempString.mb_str()) << std::endl;
    return true;
}

*/

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "NavTargets");
   ros::NodeHandle nh;

   NavTargets nt(nh);

   //cv::namedWindow("view");
   //cv::startWindowThread();

   /*
   while(nh.ok())
   {
   	//ros::spinOnce();  // check for incoming messages
   	//struct timespec ts;
      //ts.tv_sec = 0;
      //ts.tv_nsec = 10000000;
      //nanosleep(&ts, NULL); // update every 10 ms
		ros::Time last_time = ros::Time::now();
		while ( ros::Time::now.toSec() - last_time.toSec() < 0.01) ros::spinOnce(); // delay a bit
		
   }
   */
   
   ros::spin();
   //cv::destroyWindow("view");
   return EXIT_SUCCESS;
}

