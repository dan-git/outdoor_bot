#ifndef __OUTDOOR_BOT_DEFINES__H__
#define __OUTDOOR_BOT_DEFINES__H__

#define SERVO_ANTENNA_PAN 0
#define SERVO_ANTENNA_TILT 1
#define DIGCAM_PAN 2
#define ZOOM_DIGCAM_PAN 3
#define FRONT_WEBCAM_PAN 4
#define FRONT_WEBCAM_TILT 5
//#define REAR_DIGCAM_PAN 6
//#define REAR_DIGCAM_TILT 7
//#define SERVO_PWM_CHANNEL 8
//#define REAR_WEBCAM_PAN 9
//#define REAR_WEBCAM_TILT 10

#define ZOOM_DIGCAM_PAN_CENTER 160 // with a 160 center, valid inputs range from -160 to 94, with 0 being centered
#define DIGCAM_PAN_CENTER 87 		// with a 87 center, valid inputs range from -87 to 167, with 0 being centered
#define FRONT_WEBCAM_PAN_CENTER 107
#define FRONT_WEBCAM_TILT_LEVEL 67 // this is where the camera points level
#define SERVO_MIN 0
#define SERVO_MAX 255
#define SERVO_UNITS_TO_DEGREES_RATIO 1.27 // about 200 degrees coverage with a 0 - 254 range of units

#define DIGCAM 0
#define ZOOM_DIGCAM 1
#define FRONT_WEBCAM 0
#define REAR_WEBCAM 1

#define WEBCAM 1
#define HOMECAM 2
#define LAPTOP_HAS_BUILTIN_WEBCAM 1	// 1 if it has a built in webcam, 0 if not

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
#define FINAL_RANGE 2.0

#define MOTOR_DROP_BAR 0
#define MOTOR_BIN_SHADE 1
#define MOTOR_PICKER_UPPER 2
#define MOTOR_BRAKE_SOLENOID 3
#define PD_SPEED_MAX_VALUE 127



#endif // __OUTDOOR_BOT_DEFINES__H__
