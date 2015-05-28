// control cameras with gphoto2
// http://sepharads.blogspot.com/2011/11/camera-tethered-capturing-using.html
// list of cameras:
// gphoto2 --camera="Canon PowerShot A520 (PTP mode)" --port usb: --summary 
 
#include "digcams.h"
#include "outdoor_bot_defines.h"


#define POWERSHOT --camera="Canon PowerShot A520 (PTP mode)" --port usb:
#define CHECK(f) {int res = f; if (res < 0) {printf ("ERROR: %s\n", gp_result_as_string (res)); return (1);}} 

#define ZOOM_DIGCAM_NUMBER 1
#define REGULAR_DIGCAM_NUMBER 0

using namespace cv;
using namespace std;

class digcamControl
{
private:
   ros::NodeHandle nh_;
   ros::Publisher cam_pub_;
   ros::Subscriber cam_cmd_;
   image_transport::ImageTransport it_;
   image_transport::Publisher image_pub_, home_image_pub_;
   Camera **cams_;  
   GPContext *context_;
   CameraList	*list_;
	int numCams_;
	const char	*name_, *value_;
   bool cap_home_, zoomResult_, writeFileResult_;
   int retCapToMemory_, zoomCurrentZoom_, regularCurrentZoom_;
   
public:
  digcamControl(ros::NodeHandle &nh)
   :  nh_(nh), it_(nh)
   {
      cap_home_ = false;
      cam_pub_ = nh_.advertise<std_msgs::String>("digcam_file", 50); // advertise camera files
      home_image_pub_ = it_.advertise("home_target_image", 1); // advertise home target images
      image_pub_ = it_.advertise("digcam_image", 1);
      cam_cmd_ = nh_.subscribe("digcam_cmd", 50, &digcamControl::cameraCommandCallback, this); 
      retCapToMemory_ = -1;
      zoomCurrentZoom_ = 0;
      regularCurrentZoom_ = 0;
      zoomResult_ = false;
      writeFileResult_ = false;
   }

  ~digcamControl()
   {
      for (int i=0; i < numCams_; i++) gp_camera_exit(cams_[i], context_);
      //delete context_;
      //delete cams_;
      //delete list_;
      //delete name_
      //delete value_;
      //delete nh_;
   }

  bool initialize()
  {
      int ret;
    
      //printf ("Executing command to unmount cameras...\n");
      //ret = system("gvfs-mount -s gphoto2");
      //printf ("The value returned was: %d.\n",ret);

      //printf ("Executing command to verify that the camera was unmounted...\n");
      //ret = system("gvfs-mount -l");
      //printf ("The value returned was: %d.\n",ret);

      context_ = sample_create_context();

	   // Detect all the cameras that can be autodetected...
	   ret = gp_list_new (&list_);
	   if (ret < GP_OK)
      {
         printf("Error in creating new gphoto camera list.\n");
         return false;
      }
	   numCams_ = gp_camera_autodetect (list_, context_);
	   if (numCams_ < GP_OK) {
		   printf("No cameras detected.\n");
		   return false;
	   }

	   //Now open all cameras we autodected for usage 
	   printf("Number of cameras: %d\n", numCams_);
	   cams_ = (Camera**) calloc (sizeof (Camera*),numCams_);
      for (int i = 0; i < numCams_; i++)
      {
         gp_list_get_name  (list_, i, &name_);
         gp_list_get_value (list_, i, &value_);
		   ret = sample_open_camera (&cams_[i], name_, value_, context_);
		   if (ret >= GP_OK) fprintf(stderr,"Camera %s on port %s opened\n", name_, value_);
         else fprintf(stderr,"Camera %s on port %s failed to open\n", name_, value_);
       }	   
      return true;
   }

   void waitForCameraEvent(int camName)
   {
      int camNum;
      if (camName == ZOOM_DIGCAM) camNum = ZOOM_DIGCAM_NUMBER;
      else if (camName == REGULAR_DIGCAM) camNum = REGULAR_DIGCAM_NUMBER;
      else
      {
      	cout << "invalid digital camera requested in digcams capture.  camera number = " << camName << endl;
      	return;
      }
      int waittime = 2000;
      CameraEventType type;
      void *data;
      for (int i=0; i < 4; i++)  //wait for up to 8 seconds
      {
         gp_camera_wait_for_event(cams_[camNum], waittime, &type, &data, context_);
         if (type == GP_EVENT_TIMEOUT) {
            cout << "timed out waiting for digcam event, trying again" << endl;
         }
         else if (type == GP_EVENT_CAPTURE_COMPLETE) {
            printf("Capture completed event received from camera\n");
            break;
         }
         else if (type != GP_EVENT_UNKNOWN) {
            printf("Some event received from camera: %d\n", (int)type);
            break;
         }
      }
   }

 void capture_to_memory(Camera *camera, GPContext *context, const char **ptr, unsigned long int *size) {
	int retval;
	CameraFile *file;
	CameraFilePath camera_file_path;

	//printf("Capturing.\n");

	//NOP: This gets overridden in the library to /capt0000.jpg 
	strcpy(camera_file_path.folder, "/");
	strcpy(camera_file_path.name, "foo.jpg");

	retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
	//printf("  Retval: %d\n", retval);
   retCapToMemory_  = retval;

	//printf("Pathname on the camera: %s/%s\n", camera_file_path.folder, camera_file_path.name);

	retval = gp_file_new(&file);
	//printf("  Retval: %d\n", retval);
	retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name,
		     GP_FILE_TYPE_NORMAL, file, context);
	//printf("  Retval: %d\n", retval);

	gp_file_get_data_and_size (file, ptr, size);

	//printf("Deleting.\n");
	retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name,
			context);
	//printf("  Retval: %d\n", retval);
	//gp_file_free(file); 
}


   //void capture (int camNum, char* filename)
   void capture (int camName, bool writeFile = false)
   {
	   char* data;
      unsigned long size;
      int camNum;
      std_msgs::Header imgHeader;
      if (camName == ZOOM_DIGCAM)
      {
      	camNum = ZOOM_DIGCAM_NUMBER;
      	imgHeader.seq = zoomCurrentZoom_;
      	imgHeader.frame_id = "zoomDigcam";
      }
      else if (camName == REGULAR_DIGCAM)
      {
      	camNum = REGULAR_DIGCAM_NUMBER;
      	imgHeader.seq = regularCurrentZoom_;
      	imgHeader.frame_id = "regularDigcam";
      }
      else
      {
      	cout << "invalid digital camera requested in digcams capture.  camera number = " << camName << endl;
      	return;
      }
	   imgHeader.stamp = ros::Time::now();
      //capture_to_file(cams_[camNum], context_, filename);	  
      capture_to_memory(cams_[camNum], context_, (const char**)&data, &size);
      //waitForCameraEvent(camNum);
      cv::Mat imgbuf(cv::Size(1920, 1080), CV_8UC3, data);
      //cv::Mat imgbuf(cv::Size(640,480), CV_8UC3, data);
      cv::Mat img = cv::imdecode(imgbuf, CV_LOAD_IMAGE_COLOR);
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(imgHeader, "bgr8", img).toImageMsg();
      if (cap_home_)
      {
      	home_image_pub_.publish(msg);  // home target image
      	cout << "digcams published a home image";
      }
      else
      {
      	image_pub_.publish(msg);   // publish the image
      	cout << "digcams published a target image";
      }
   
      if (writeFile)
      {
         FILE 	*f;
         int retval;
         char filename[256];
         snprintf(filename, 256, "/home/dbarry/Dropbox/outdoor_bot/media/image_processing/digcam/digcam%d.jpg", camNum);
         f = fopen(filename, "wb");
         if (f) {
		      retval = fwrite (data, size, 1, f);
		      if (retval != (int) size) {
		        	printf("  fwrite size %ld, written %d, into filename %s\n", size, retval,filename);
		      }
		      fclose(f);
            printf("digcam image written to file %s\n", filename);

            std_msgs::String msg_name;
            msg_name.data = filename;       
            cam_pub_.publish(msg_name);  // publish the filename
            writeFileResult_ = true;
         }
         else
         {
            printf("fopen %s failed.\n", filename);
            writeFileResult_ = false;
         }
      }
	}

/*
   static int
   _lookup_widget(CameraWidget*widget, const char *key, CameraWidget **child) {
         int ret;
         ret = gp_widget_get_child_by_name (widget, key, child);
         if (ret < GP_OK)
               ret = gp_widget_get_child_by_label (widget, key, child);
         return ret;
   }
*/

bool setZoom(int camName, float value)
   {
      int camNum;
      if (camName == ZOOM_DIGCAM) camNum = ZOOM_DIGCAM_NUMBER;
      else if (camName == REGULAR_DIGCAM) camNum = REGULAR_DIGCAM_NUMBER;
      else
      {
      	cout << "invalid digital camera requested in digcams zoom.  camera number = " << camName << endl;
      	return false;
      }
            int ret;
      const char *key = "zoom";
      float newZoom = value, previousZoom;

      CameraWidget *widget = NULL, *child = NULL;
      ret = gp_camera_get_config (cams_[camNum], &widget, context_);
      if (ret < GP_OK)
      {
         cout << "camera_get_config failed" << endl;
         return false;
      }
      else cout << "camera_get_config succeeded" << endl;
      ret = _lookup_widget (widget, key, &child);
      if (ret < GP_OK)
      {
         cout << "_lookup widget failed" << endl;
         return false;
      }
      else cout << "lookup widget succeeded" << endl;
      ret = gp_widget_get_value (child, &previousZoom);
      if (ret < GP_OK)
      {
         cout << "get_value failed" << endl;
         return false;
      }
      else cout << "previous zoom value = " << previousZoom << endl;
                
      ret = gp_widget_set_value (child, &newZoom);
      if (ret < GP_OK)
      {
         cout << "set value failed" << endl;
         return false;
      }
      else cout << "set value succeeded, value = " << newZoom << endl;
      // This stores it on the camera again 
      ret = gp_camera_set_config (cams_[camNum], widget, context_);
      if (ret < GP_OK)
      {
         cout << "_lookup widget failed" << endl;
         return false;
      }
      else cout << "storing value on camera succeeded, value = " << newZoom << endl;
      //waitForCameraEvent(camNum);
      return true;
   } 

   /*void setCameraParameter(int camName, bool parameterSet = true, bool parameterGet = false)
   {
      int camNum;
      if (camName == ZOOM_DIGCAM) camNum = ZOOM_DIGCAM_NUMBER;
      else if (camName == REGULAR_DIGCAM) camNum = REGULAR_ REGULAR_DIGCAM_NUMBER;
      else
      {
      	cout << "invalid digital camera requested in digcams setCameraParameter.  camera number = " << camName << endl;
      	return;
      }
      
      const char* val;
      const char* stringToSet;
      int ret;
      const char *key = "zoom";
      float zoomval = 5;

      CameraWidget *widget = NULL, *child = NULL;
      CameraWidgetType type;
      ret = gp_camera_get_config (cams_[camNum], &widget, context_);
      if (ret < GP_OK) {
            fprintf (stderr, "camera_get_config failed: %d\n", ret);
            //return ret;
      }
      else cout << "camera_get_config succeeded" << endl;
      ret = _lookup_widget (widget, key, &child);
      if (ret < GP_OK) {
            fprintf (stderr, "lookup widget failed: %d\n", ret);
            //goto out;
      }
      else cout << "lookup widget succeeded" << endl;

      //This type check is optional, if you know what type the label
      //has already. If you are not sure, better check. 
      ret = gp_widget_get_type (child, &type);
      if (ret < GP_OK) {
            fprintf (stderr, "widget get type failed: %d\n", ret);
            //goto out;
      }
      else cout << "get type succeeded, type = " << type << endl;
      // the enum declaration for GP_WIDGET.... is here:
      // http://libgphoto2.sourcearchive.com/documentation/2.4.7/gphoto2-widget_8h-source.html

      switch (type) {
        case GP_WIDGET_MENU:  // value = 6
        case GP_WIDGET_RADIO: // value = 5
        case GP_WIDGET_TEXT:  // value = 2
            cout << "widget type corresponds to char *" << endl;
                 //This is the actual query call. Note that we just
                 // a pointer reference to the string, not a copy... 
            if (parameterGet)
            {
               ret = gp_widget_get_value (child, &val);
               if (ret < GP_OK) {
                  fprintf (stderr, "could not query widget value: %d\n", ret);
               }
               else cout << "get value succeeded, value = " << val << endl;
            }
            if (parameterSet)
            {
                // This is the actual set call. Note that we keep
                // ownership of the string and have to free it if necessary.
                
               ret = gp_widget_set_value (child, stringToSet);
               if (ret < GP_OK) {
                     fprintf (stderr, "could not set widget value: %d\n", ret);
                     //goto out;
               }
               else cout << "set value succeeded, value = " << val << endl;
               // This stores it on the camera again 
               ret = gp_camera_set_config (cams_[camNum], widget, context_);
               if (ret < GP_OK) {
                     fprintf (stderr, "camera_set_config failed: %d\n", ret);
               }
               else cout << "storing value on camera succeeded, value = " << endl;
            }
            break;
        case GP_WIDGET_RANGE: // this value = 3
            cout << "widget type corresponds to float " << endl;
            if (parameterGet)
            {
               ret = gp_widget_get_value (child, &zoomval);
               if (ret < GP_OK) {
                  fprintf (stderr, "could not query widget value: %d\n", ret);
               }
               else cout << "get value succeeded, value = " << zoomval << endl;
            }
            if (parameterSet)
            {
                // This is the actual set call. Note that we keep
                // ownership of the string and have to free it if necessary.
                
               ret = gp_widget_set_value (child, &zoomval);
               if (ret < GP_OK) {
                     fprintf (stderr, "could not set widget value: %d\n", ret);
                     //goto out;
               }
               else cout << "set value succeeded, value = " << zoomval << endl;
               // This stores it on the camera again 
               ret = gp_camera_set_config (cams_[camNum], widget, context_);
               if (ret < GP_OK) {
                     fprintf (stderr, "camera_set_config failed: %d\n", ret);
               }
               else cout << "storing value on camera succeeded, value = " << endl;
            }
            break;   
        default:
            fprintf (stderr, "widget has bad type %d\n", type);
            //ret = GP_ERROR_BAD_PARAMETERS;
            //return;
      } 
	   
      for (int i = 0; i < numCams_; i++)
      {		   
         CameraText	text;
         ret = gp_camera_get_summary (cams_[i], &text, context_);
		   if (ret < GP_OK)
         {
			   //fprintf (stderr, "Failed to get summary.\n");
            cout << endl << endl << "Failed to get summary, ret = " << ret << endl;
			   continue;
		   }
         else cout << endl << endl << "Summary obtained successfully, as follows: " << endl;
         
         gp_list_get_name  (list_, i, &name_);
         gp_list_get_value (list_, i, &value_);
         printf("%-30s %-16s\n", name_, value_);
		   printf("Summary:\n%s\n", text.text);
                 
		   // Query a simple string configuration variable.
         char *model; 
		   ret = get_config_value_string (cams_[i], "afdistance", &model, context_);

		   if (ret >= GP_OK) {
            cout << endl << endl << "Model obtained successfully: " << model << endl;

			   free (model);
		   } else {
			   printf("model: No model found.\n");
		   }
	   }     
   }
  */

   //void cameraCommandCallback(const std_msgs::String::ConstPtr& msg)
   void cameraCommandCallback(const outdoor_bot::digcams_custom::ConstPtr &msg)
   {
      std::string cameraCommand = msg->command; 
      if (!cameraCommand.compare("capture") || !cameraCommand.compare("cap_home"))
      {
         if (!cameraCommand.compare("cap_home")) cap_home_ = true;
         else cap_home_ = false;
         int cameraName = msg->cameraName;
         if (cameraName == ZOOM_DIGCAM || cameraName == REGULAR_DIGCAM)
         {
            if (msg->write_file) capture(cameraName, true);
            else capture(cameraName, false);
            cout << " from digcam" << cameraName << endl << endl;
         }
         else ROS_ERROR("invalid camera number requested for capture");
      }
      else if (!cameraCommand.compare("capAll"))
      {
         cout << "capturing from all cameras" << endl;
         cap_home_ = false;
         for (int i=0; i < numCams_; i++)
         {
            if (msg->write_file) capture(i, true);
            else capture(i, false);
         }
      }
      else if (!cameraCommand.compare("setZoom"))
      {
         int cameraName = msg->cameraName;
         float zoom = msg->zoomSet;
         cout << "Setting zoom on digcam" << cameraName << " to " << zoom << endl;
         if (setZoom(cameraName, zoom))
         {
         	if (cameraName == ZOOM_DIGCAM) zoomCurrentZoom_ = (int) zoom;
         	else if (cameraName == REGULAR_DIGCAM) regularCurrentZoom_ = (int) zoom;
         }
      } 
       
      else ROS_ERROR("unkown command sent to digcams");   
   }

   bool digcamService(outdoor_bot::digcams_service::Request  &req, outdoor_bot::digcams_service::Response &resp)
   {
      std::string cameraCommand = req.command; 
      if (!cameraCommand.compare("capture") || !cameraCommand.compare("cap_home"))
      {
         if (!cameraCommand.compare("cap_home")) cap_home_ = true;
         else cap_home_ = false;
         int cameraName = req.cameraName;
         if (cameraName == ZOOM_DIGCAM || cameraName == REGULAR_DIGCAM)
         {
            cout << "capturing from digcam" << cameraName << endl;
            if (req.write_file) capture(cameraName, true);
            else capture(cameraName, false);
         }
         else ROS_ERROR("invalid camera number requested for capture");
      }
      else if (!cameraCommand.compare("capAll"))
      {
         cout << "capturing from all cameras" << endl;
         cap_home_ = false;
         for (int i=0; i < numCams_; i++)
         {
            if (req.write_file) capture(i, true);
            else capture(i, false);
         }
      }
      else if (!cameraCommand.compare("setZoom"))
      {
         int cameraName = req.cameraName;
         float zoom = req.zoom;
         cout << "Setting zoom on digcam" << cameraName << " to " << zoom << endl;
         zoomResult_ = setZoom(cameraName, zoom);
      }         
      else ROS_ERROR("unkown command sent to digcams"); 

      resp.captureResult =  retCapToMemory_;
      resp.zoomResult = zoomResult_;
      resp.writeFileResult = writeFileResult_;
      return true;
   }

   int getNumCams()
   {
      return numCams_;
   }
};

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

int main (int argc, char *argv[])
{
   //Initialize ROS
   ros::init(argc, argv, "digcams");
   ros::NodeHandle nh;
   ROS_INFO("digcams starting");
   digcamControl camControl(nh);
   ros::ServiceServer service = nh.advertiseService("digcams_service", &digcamControl::digcamService, &camControl);
   
   #if CV_MAJOR_VERSION == 3
   cout << "cv3" << endl;
   
   #elif CV_MAJOR_VERSION == 2
   cout << "cv2" << endl;
   #endif

   //gp_log_add_func(GP_LOG_ERROR, errordumper, NULL);

   if (camControl.initialize())
   {
      ROS_INFO("camcontrol cameras initialized.");
      // take some shots      
      //int nCams = camControl.getNumCams();
      //for (int i = 0; i < nCams; i++) camControl.capture(i);
   }
   else ROS_INFO("camcontrol cameras failed to initialize.");
   //camControl.capture(0, true);

   //camControl.setZoom(0, 5.);

   //camControl.capture(0, true);
	/*
   while(nh.ok())
   {
   	ros::spinOnce();  // check for incoming messages
   	struct timespec ts;
      ts.tv_sec = 0;
      ts.tv_nsec = 10000000;
      nanosleep(&ts, NULL); // update every 10 ms
   }
   */
   ros::spin();
   return EXIT_SUCCESS;

}

