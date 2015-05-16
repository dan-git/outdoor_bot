#ifndef __OUTDOOR_BOT_DEFINES__H__
#define __OUTDOOR_BOT_DEFINES__H__

// sections
#define BOOTUP 0
#define FIRST_TARGET 1
#define TARGETS 2
#define HOME 3
#define PLATFORM 4

#define SERVO_ANTENNA_PAN 0
#define SERVO_ANTENNA_TILT 1
#define ZOOM_DIGCAM_PAN 2
#define DIGCAM_PAN 3
#define FRONT_WEBCAM_PAN 4
#define FRONT_WEBCAM_TILT 5
//#define REAR_DIGCAM_PAN 6
//#define REAR_DIGCAM_TILT 7
//#define SERVO_PWM_CHANNEL 8
//#define REAR_WEBCAM_PAN 9
//#define REAR_WEBCAM_TILT 10

#define ZOOM_DIGCAM_PAN_CENTER 127
#define DIGCAM_PAN_CENTER 127
#define FRONT_WEBCAM_PAN_CENTER 127
#define FRONT_WEBCAM_TILT_LEVEL 127 // this is where the camera points level
#define SERVO_MIN 0
#define SERVO_MAX 254
//#define SERVO_UNITS_TO_DEGREES_RATIO 1.27 // about 200 degrees coverage with a 0 - 254 range of units


// camera numbers
#define WEBCAM 0
//#define REAR_WEBCAM 1
#define LAPTOP_HAS_BUILTIN_WEBCAM 1	// 1 if it has a built in webcam, 0 if not

// camera names, cannot overlap camera numbers
#define REGULAR_DIGCAM 2
#define ZOOM_DIGCAM 3
#define HOMECAM 4

#define DIGCAM_PIXELS_WIDTH 2272
#define DIGCAM_PIXELS_HEIGHT 1704
#define WEBCAM_PIXELS_WIDTH 640
#define WEBCAM_PIXELS_HEIGHT 480
#define LARGE_TARGET_HEIGHT 0.78
#define LARGE_TARGET_WIDTH 0.445
#define WEBCAM_FOV 41.  // logitech 310 sees 7.5m at 10m -> has 40 degrees field of view
#define ORBITCAM_FOV	54.	// orbitcam sees 14m at 10m -> has 54 degree FOV
#define REGULAR_DIGCAM_FOV 14.  // powershot digcam with zoom = 7, sees 2.5m wide at 10m -> 14 degree FOV
#define ZOOM_DIGCAM_FOV 3.8 // zoom = 7, sees 0.66 m wide at 10m -> 4 degree FOV

//#define FRONT 1
//#define REAR 0
//#define DIGCAM 0

#define NUM_RADARS 3
#define LEFT_RADAR_NUMBER 101
#define RIGHT_RADAR_NUMBER 102
#define CENTER_RADAR_NUMBER 100
#define LEFT_RADAR_INDEX 0
#define RIGHT_RADAR_INDEX 1
#define CENTER_RADAR_INDEX 2
#define LEFT_RADAR_HOME_DISTANCE 1.47	// when at home, distances in meters for radar rangers
#define RIGHT_RADAR_HOME_DISTANCE 2.17
#define CENTER_RADAR_HOME_DISTANCE 0.5
//#define MAP 0
#define BOT 1
//#define FINAL_RANGE 2.0  // now defined in just movement node

#define MOTOR_DROP_BAR 0
#define MOTOR_BIN_SHADE 1
#define MOTOR_PICKER_UPPER 2
#define MOTOR_BRAKE_SOLENOID 3
#define PD_SPEED_MAX_VALUE 127
#define PICKER_UPPER_UP -1
#define PICKER_UPPER_DOWN 1
#define PICKER_UPPER_DOWN_PREP 2
#define DROP_BAR_UP 1
#define DROP_BAR_DOWN -1
#define BIN_SHADE_UP 1
#define BIN_SHADE_DOWN -1



#endif // __OUTDOOR_BOT_DEFINES__H__
