// target tracking node

#include <string>
#include <stdio.h>
#include <stdlib.h>  
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdarg.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream> 

using namespace cv;
using namespace std;

#define LIGHT_HEIGHT 1000
#define LIGHT_WIDTH 500
#define MAX_NUM_POLYGONS 20
#define TOTAL_FRAMES 300
#define START_FRAME 60 // min value is 1 since we to use the 0th frame for setting up
#define VERTEX_DISTANCE_SQUARED_THRESHOLD 1000
#define CENTER_DISTANCE_SQUARED_THRESHOLD 700
#define CM_PER_PIXEL 2.67
#define DEGREES_PER_ASPECT_RATIO 51.4

VideoCapture camera_, camera1;
Mat frame_;

bool capture(Mat *image)
{
  bool bSuccess = camera_.read(frame_); // read a new frame from video
  if (!bSuccess) //if not success, break loop
  {
       printf("Cannot read a frame from video0 stream \n");
       return false;
  }
else printf("Successfully read a frame from video0 stream \n");
  *image = frame_;
  //cout << "Read a frame from video stream" << endl;
   waitKey(30);  // give highgui time to get the image

      imshow("view", frame_); //show the frame
      waitKey(0); //wait to give highGUI time to process
  return true;
}

bool capture1(Mat *image)
{
  bool bSuccess = camera1.read(frame_); // read a new frame from video
  if (!bSuccess) //if not success, break loop
  {
       printf("Cannot read a frame from video1 stream \n");
       return false;
  }
  else printf("Successfully read a frame from video1 stream \n");
  *image = frame_;
  //cout << "Read a frame from video stream" << endl;
   waitKey(30);  // give highgui time to get the image

      imshow("view1", frame_); //show the frame
      waitKey(0); //wait to give highGUI time to process
  return true;
}
int main(int argc, char* argv[])
{
   //Initialize ROS
   ros::init(argc, argv, "ceiling_tracker");
   ros::NodeHandle nh;
   ROS_INFO("ceiling_tracker starting");
   namedWindow("view1");
namedWindow("view");
   camera_.open(0);   // open the default webcam
   //camera_.open("/home/dbarry/Dropbox/outdoor_bot/media/image_processing/celing_data/top_video.webm");   
   //camera_.open("/home/dbarry/Dropbox/outdoor_bot/media/image_processing/ceiling_data/rotation.mp4");
   //camera_.open("/home/dbarry/Dropbox/outdoor_bot/media/image_processing/ceiling_data/rotation.mp4");

   if (camera_.isOpened())  printf("video0 opened \n");
   else
   {
      printf("Cannot open video0 \n");
      return 1;
   }

 camera1.open(1);   // open the default webcam
   //camera_.open("/home/dbarry/Dropbox/outdoor_bot/media/image_processing/celing_data/top_video.webm");   
   //camera_.open("/home/dbarry/Dropbox/outdoor_bot/media/image_processing/ceiling_data/rotation.mp4");
   //camera_.open("/home/dbarry/Dropbox/outdoor_bot/media/image_processing/ceiling_data/rotation.mp4");

   if (camera1.isOpened())  printf("video1 opened \n");
   else
   {
      printf("Cannot open video1 \n");
      return 1;
   }

// restricted to these formats or the logitech C310 uses up so much USB bandwidth that
// you can only use one of them.
camera_.set(CV_CAP_PROP_FRAME_WIDTH, 320);
camera_.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
camera1.set(CV_CAP_PROP_FRAME_WIDTH, 320);
camera1.set(CV_CAP_PROP_FRAME_HEIGHT, 240);


cout << "cam0 format is " << camera_.get(CV_CAP_PROP_FORMAT) << endl;
//camera1.get(CV_CAP_PROP_FORMAT);

// read and display some frames

   // read a frame
 for (int i=0; i < 4; i++)
{  Mat nextFrame0;
   capture(&nextFrame0);
   capture1(&nextFrame0);
}

return EXIT_SUCCESS;





   CvSize sz;   
   vector<int> compression_params;
   compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
   compression_params.push_back(100);
   RNG rng(12345);

   vector<Point> overhead_light; // model shape of a light fixture from video image
   Point p0(902, 120);
   Point p1(769, 153);
   Point p2(754, 322);
   Point p3(808, 475);
   Point p4(895, 504);
   Point p5(970, 454);
   Point p6(979, 363);

   overhead_light.push_back(p0);
   overhead_light.push_back(p1);
   overhead_light.push_back(p2);
   overhead_light.push_back(p3);
   overhead_light.push_back(p4);
   overhead_light.push_back(p5);
   overhead_light.push_back(p6);

   /*
   vector<Point> rectngl; // model shape of a light fixture
   Point p0(200, 200);
   Point p1(500, 200);
   Point p2(500, 400);
   Point p3(200, 400);

   rectngl.push_back(p0);
   rectngl.push_back(p1);
   rectngl.push_back(p2);
   rectngl.push_back(p3); 
   */
   
   vector<vector<Point> > polygon_contour;
   Point polygon_center[MAX_NUM_POLYGONS];
   double polygon_radius[MAX_NUM_POLYGONS];
   double polygon_match_value[MAX_NUM_POLYGONS];
   double polygon_aspectRatio[MAX_NUM_POLYGONS];
   double polygon_area[MAX_NUM_POLYGONS];

   vector<Point>Vertices;
   double vertexDistance[TOTAL_FRAMES];
   double centerDistance[TOTAL_FRAMES];
   double target_aspectRatio[TOTAL_FRAMES];
   Point target_center[TOTAL_FRAMES];
   int numFramesWithTargets = 0;

   

   //startWindowThread();

   //double dWidth_; // the width of frames of the video
   //double dHeight_;
   //dWidth_ = camera_.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
   //dHeight_ = camera_.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
   //printf("Frame size : %f x %f \n", dWidth_, dHeight_);

   // read a frame
   Mat nextFrame;
   capture(&nextFrame);
   waitKey(30);  // give highgui time to get the image
   sz = cvSize( nextFrame.cols, nextFrame.rows);
   Mat drawing = Mat::zeros(sz, CV_8UC3 ); // for showing contours

   int numFrames;
   double totalAspectAngle_ = 0., prevAspectAngle_ = 0.;
   for (numFrames = 1; numFrames < TOTAL_FRAMES + START_FRAME; numFrames++) // start with 1 since we already got one frame
   {
      if (capture(&nextFrame))
      {
         //cout << endl << "Frame " << numFrames << " captured" << endl;
         waitKey(30);  // give highgui time to get the image
      }
      else
      {
         //cout << endl << "Frame " << numFrames << " not captured" << endl;
         waitKey(30);
         break;   // probably just reached the end of the video file, so just exit loop
      }

      if (numFrames < START_FRAME) continue; // capture up to the start frame and then start analysis


      /*
      std::string filename_image = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/ceiling_data/ceiling_image.jpg";
      try {
         imwrite(filename_image, nextFrame, compression_params);
      }
      catch (runtime_error& ex) {
         cout << "Exception converting image to JPG format: " <<  ex.what() << endl;
         return true; // we didn't write the file
      }
      cout << "Saved image file, " << filename_image << " with ceiling image." << endl;
      */
      Mat gray(sz, CV_8UC1); // depth 8, 1 channel, gray version
      cvtColor(nextFrame, gray, CV_BGR2GRAY);
      //imshow("view", nextFrame); //show the frame
      //waitKey(0); // let user see it and dismiss it
      
      threshold(gray, gray, 245, 255, CV_THRESH_BINARY );

      //imshow("view", gray); //show the frame
      //waitKey(0); //wait to give highGUI time to process
      //destroyWindow("view"); //destroy the window

      Mat grayContours = gray.clone();
      vector<vector<Point> > contours;
      int numpolygonsFound = 0;
      vector<Vec4i> hierarchy;
      // find contours and store them all as a list
      findContours( grayContours, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) ); 

      // Draw contours

      namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
      Mat contour_drawing = Mat::zeros(sz, CV_8UC3 ); // for showing contours  
      
      for( int i = 0; i < contours.size(); i++ )
      {
         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         drawContours( contour_drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );      
      }
      //imshow("Contours", contour_drawing); //show the frame
      //waitKey(0); //wait to give highGUI time to process
      //destroyWindow("Contours"); //destroy the window
      

      //cout << "number of polygons = " << contours.size() << endl;
      for( int i = 0; i < contours.size(); i++ )
      {
         // Find closed polygons with accuracy proportional to the contour perimeter
         double  arc_length = arcLength(contours[i], true);
         vector<Point> contours_poly;
         approxPolyDP(contours[i], contours_poly, arc_length * 0.02, true );

         /*
         Rect contourRect = boundingRect(contours[i]);
         Point p0(contourRect.x, contourRect.y);
         Point p1(contourRect.x + contourRect.width, contourRect.y);
         Point p2(contourRect.x + contourRect.width, contourRect.y + contourRect.height);
         Point p3(contourRect.x, contourRect.y + contourRect.height);

         contours_poly.push_back(p0);
         contours_poly.push_back(p1);
         contours_poly.push_back(p2);
         contours_poly.push_back(p3);
         */

         double MinArea = 1500., MaxArea = 2500.;
         //double MinArea = 40000., MaxArea = 80000.;
         double MinRatioAreaArcLength = 4., MaxRatioAreaArcLength = 16.;
         double areaContour = fabs(contourArea(contours_poly));
         if (arc_length > 0.0001)
         {
            if (areaContour < MinArea || areaContour > MaxArea
               || areaContour / arc_length < MinRatioAreaArcLength 
               || areaContour / arc_length  > MaxRatioAreaArcLength)
            {
               //cout << "rejected contour " << i << " for area or area to arc length ratio " << endl;
               //cout << "area = " << areaContour << ", arc length ratio = " << areaContour / arc_length << endl;
               continue; 
            }  
            else
            {
               //cout << "accepted contour " << i << " with area or area to arc length ratio " << endl;
               //cout << "area = " << areaContour << ", arc length ratio = " << areaContour / arc_length << endl;
            }
         }
         else
         {
            //cout << "rejected contour " << i << " for arc length approx = 0" << endl;
            continue;
         }

         double match_value = matchShapes(overhead_light, contours_poly, CV_CONTOURS_MATCH_I1, 0.);
         double MaxMatchValue = 1.0;
         //cout << "contour " << i << " has match value = " << match_value << endl;
         // note that polygons with just 2 sides return a match_value = 0, a very good match value
         
         if (match_value > MaxMatchValue) // only look for good matches
         {
            //cout << "rejected contour for match value =  " << match_value << endl;
            continue; 
         }

         int numVertices = 4;
         if (contours_poly.size() < numVertices || contours_poly.size() > numVertices + 3 )
         {
            //cout << "rejected contour " << i << " for vertices =  " << contours_poly.size() << endl;
            continue;
         }

         if (!isContourConvex(contours_poly))
         {
            //cout << "rejected contour " << i << " for convexity " << endl;
            continue;
         }
        

         // this contour looks like a target
         Point2f centerContour;
         float radiusContour;
         minEnclosingCircle(contours_poly, centerContour, radiusContour);

         double aspectRatio;
         Rect boundRect = boundingRect(contours_poly); 
         if (boundRect.height > 0) aspectRatio = ((double) boundRect.width) / ((double)boundRect.height);

         if (numpolygonsFound < MAX_NUM_POLYGONS)
         {                                             
            polygon_contour.push_back(contours_poly);  
            polygon_center[numpolygonsFound] = centerContour; 
            polygon_radius[numpolygonsFound] = radiusContour;
            polygon_match_value[numpolygonsFound] = match_value;
            polygon_aspectRatio[numpolygonsFound] = aspectRatio;
            polygon_area[numpolygonsFound] = areaContour;
            //cout << " polygon " << numpolygonsFound << " has area, center = "
            //   << polygon_area[numpolygonsFound] << ", " <<  polygon_center[numpolygonsFound] << endl;
            //cout << "radius = " << polygon_radius[numpolygonsFound] << endl;
            numpolygonsFound++;
         }  

         /*
         // here is an example of how to draw a single contour, for example vector<Point> bestContour
         vector<vector<Point> > contourVec;
         contourVec.push_back(contours_poly);
         drawContours( drawing, contourVec, 0, Scalar(0,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
         */

      } // finishes contour loop

      //cout << "Found " << numpolygonsFound << " polygons" << endl;
      if (numpolygonsFound < 1)
      {
         //cout << "no targets found in frame " << numFrames << endl;
         continue;  // no targets found in this image
      }

      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      for( int i = 0; i < polygon_contour.size(); i++ ) // draw all the qualified target polygons in this frame
      {
         drawContours( drawing, polygon_contour, i, color, 1, 8, vector<Vec4i>(), 0, Point() );      
      }
      // we will use the contour with the largest area
      double maxArea = 0.;
      int chosenContour = 0;
      for (int i=0; i < numpolygonsFound; i++)
      {
         if (polygon_area[i] > maxArea)
         {
            maxArea = polygon_area[i];
            chosenContour = i; 
         }
      }
      //cout << "max polygon area = " << maxArea << " at index = " << chosenContour << endl;
      //waitKey(0);

      target_center[numFramesWithTargets] = polygon_center[chosenContour];
      target_aspectRatio[numFramesWithTargets] = polygon_aspectRatio[chosenContour];

      numFramesWithTargets++;

      // track the center
      if (numFramesWithTargets > 1)
      {
         Point diff = target_center[numFramesWithTargets - 1] - target_center[numFramesWithTargets - 2]; 
         double dist = ( (double) diff.x * (double) diff.x) + ((double) diff.y * (double) diff.y);
         if (dist < CENTER_DISTANCE_SQUARED_THRESHOLD) 
         {
            circle(drawing, target_center[numFramesWithTargets - 1], 10, color, 2, 8, 0);
            //cout << "center tracking, dist squared = " << dist << endl;
            centerDistance[numFramesWithTargets - 2] = dist;
         }
         else
         {
            //cout << "center tracking over threshold, dist squared = " << dist << endl;
            centerDistance[numFramesWithTargets - 2] = -1.;
         }

         // show the angle
         Point origin(nextFrame.cols/2, nextFrame.rows/2);
         double aspectAngle = 1.57 + (1.57 * ((target_aspectRatio[numFramesWithTargets - 1] - 0.45) / 1.75));
         totalAspectAngle_ += aspectAngle - prevAspectAngle_;
         prevAspectAngle_ = aspectAngle;
         double xCoord = cos(totalAspectAngle_) * 25.; 
         double yCoord = sin(totalAspectAngle_) * 25.;      
         Point endLine( ((int) xCoord) + (nextFrame.cols/2) , ((int) yCoord) + (nextFrame.rows/2) );
         line(drawing, origin, endLine, color, 2, 8, 0);
       } 
       //imshow("Contours",drawing);
      // waitKey(0); //wait for user to dismiss


      /*
      // track vertices, take the vertex closest to the center
      Point chosenVertex(0,0);
      Point imageCenterPoint(nextFrame.cols, nextFrame.rows);
      double minDistToCenter = 100000000.;
      //double tiltAngle;
      for (int i=0; i < polygon_contour[chosenContour].size(); i++)   // for each vertex in the chosen polygon
      {
         Point diff = polygon_contour[chosenContour][i] - imageCenterPoint;
         double dist = ((double)diff.x * (double) diff.x) + ((double) diff.y * (double)diff.y);  // dont bother with the sqrt
         if (dist < minDistToCenter)
         {
            chosenVertex = polygon_contour[chosenContour][i];
            minDistToCenter = dist;
         } 
         //cout << "vertex " << i << ": " << polygon_contour[chosenContour][i] << endl;
      }
      Vertices.push_back(chosenVertex);
      cout << "chosen vertex is at " << chosenVertex << endl;
      //cout << "area of this contour = "  << polygon_area[chosenContour] << endl;
      //cout << "center of this contour = " << polygon_center[chosenContour].x <<
      //       ", " << polygon_center[chosenContour].y << endl;
      //cout << "radius of this contour = " << polygon_radius[chosenContour] << endl;

      // track the vertex
      if (numFramesWithTargets > 1)
      {
         Point diff = Vertices[numFramesWithTargets - 1] - Vertices[numFramesWithTargets - 2]; 
         double dist = ( (double) diff.x * (double) diff.x) + ((double) diff.y * (double) diff.y);
         if (dist < VERTEX_DISTANCE_SQUARED_THRESHOLD) 
         {
            circle(drawing, Vertices[numFramesWithTargets - 1], 10, Scalar(0,0,255), 2, 8, 0);
            cout << "vertex tracking, dist squared = " << dist << endl;
            vertexDistance[numFramesWithTargets - 2] = dist;
         }
         else vertexDistance[numFramesWithTargets - 2] = -1.;
      }   
      */

      // here is an example of how to draw a single contour, for example vector<Point> bestContour
      vector<vector<Point> > contourVec;
      contourVec.push_back(polygon_contour[chosenContour]);
      drawContours( drawing, contourVec, 0, Scalar(0,0,255), 1, 8, vector<Vec4i>(), 0, Point() );

      //imshow("Contours",drawing);
      //waitKey(0); //wait for user to dismiss

      polygon_contour.clear(); // get ready to accept a new set of contours
       
   }  // finishes numFrames loop

   if (numFramesWithTargets < 1)
   {
      cout << "No targets found in entire video sequence" << endl;
      return 0;
   }

   cout << "List of centers, distances, aspect ratios and angles:" << endl;
   cout << "Frame 0 center, aspect angle: " << target_center[0] << ", " << target_aspectRatio[0]  << endl; 
   double total_pixels_moved = 0., total_angle_turned = 0.;
   for (int i=1; i < numFramesWithTargets; i++)
   {
      cout << "Frame " << i << " center, distance, aspectRatio, angle, delta angle: ";
      cout << target_center[i] << ", ";
      if (centerDistance[i-1] >= 0)
      {
         line(drawing, target_center[i], target_center[i-1], Scalar((i * 20)%255,255,0), 4);
         total_pixels_moved += sqrt(centerDistance[i-1]); 
         double angDiff = (target_aspectRatio[i] - target_aspectRatio[i-1]) * DEGREES_PER_ASPECT_RATIO;
         total_angle_turned += angDiff;   
         cout << sqrt(centerDistance[i-1]) << ", " << target_aspectRatio[i] << ", " <<
         total_angle_turned << ", " << angDiff  << endl;
      }
      else
      {
         cout << "too far away. ";
      }

      /*
      cout << " vertex: " << Vertices[i] << ", ";
      if (vertexDistance[i-1] >= 0)
      {
         line(drawing, Vertices[i], Vertices[i-1], Scalar((i * 20)%255,255,0), 4);
         cout << sqrt(vertexDistance[i-1]);
      }
      else
      {
         cout << "too far away";
      }
      */
      cout << endl;
   }

   cout << "last frame captured = " << numFrames - 1 << endl;
   cout << "total pixels moved = " << total_pixels_moved << endl;
   cout << "total cm moved = " << total_pixels_moved * CM_PER_PIXEL << endl;
   cout << "total angle turned = " << total_angle_turned << endl;

   imshow("Contours",drawing);
   waitKey(0); //wait for user to dismiss
   
   
   std::string filename_contour = "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/ceiling_data/ceiling_contour.jpg";
   //vector<int> compression_params;
   //compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
   //compression_params.push_back(100);
   try {
      imwrite(filename_contour, drawing, compression_params);
   }
   catch (runtime_error& ex) {
      cout << "Exception converting image to JPG format: " <<  ex.what() << endl;
      return true; // we didn't write the file
   }
   cout << "Saved image file, " << filename_contour << " with ceiling contours." << endl;
   

   destroyWindow("Contours"); //destroy the window
   while(nh.ok()) ros::spinOnce();  // check for incoming messages
   return EXIT_SUCCESS;
}
