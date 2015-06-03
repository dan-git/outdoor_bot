/* 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**************************************************************************
 * Definition of general configuration settings.
 **************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

boolean robotPause_;
int toggleAmberLightState_;
unsigned long pauseBlinkTime_;

#define OUTDOOR 4
#define ROBOT_NAME OUTDOOR

// Maximum number of allowed instructions.
// Instructions are methods that can be called via a serial connection.
#define MAX_NUM_INSTRUCTIONS 64

// Maximum number of allowed modules.
#define MAX_NUM_MODULES 32

// Maximum size of serial input.
#define MAX_SERIAL_INPUT_SIZE 512

// Startup defaults
#define DEBUG false
// remember to set the DEBUG_SERIAL_PORT parameter as needed for each particular robot

#define TIMEOUT true // Stops the robot if no ping() is received every TIMEOUT_MILLIS
#define TIMEOUT_MILLIS 8000  // Timeout duration if timeout is enabled

#define MIN_SENSOR_REPORT_TIME 20 // min time between sending navData.  don't want to clog up the serial port

#define RIGHT 0
#define LEFT 1
#define FORWARD 1
#define REVERSE -1

#if defined (__AVR_ATmega2560__)
  #define TIMER1_CHANNELA_PIN 11
  #define TIMER1_CHANNELB_PIN 12
  #define TIMER1_CHANNELA_SPEED_REGISTER OCR1A
  #define TIMER1_CHANNELB_SPEED_REGISTER OCR1B
  #define TIMER1_PWM_TOP_VALUE_REGISTER ICR1
  
  /*
  // note that timer2 is only an 8 bit timer
  #define TIMER2_CHANNELA_PIN 10
  #define TIMER2_CHANNELB_PIN 9
  #define TIMER2_CHANNELA_SPEED_REGISTER OCR2A 
  #define TIMER2_CHANNELB_SPEED_REGISTER OCR2B 
  
  #define TIMER3_CHANNELA_PIN 5
  #define TIMER3_CHANNELB_PIN 2  // can't use this pin because it is also interrupt 0, used for radio control
  #define TIMER3_CHANNELC_PIN 3  // can't use this pin because it is also interrupt 1, used for radio control
  #define TIMER3_CHANNELA_SPEED_REGISTER OCR3A 
  #define TIMER3_CHANNELB_SPEED_REGISTER OCR3B
  #define TIMER3_PWM_TOP_VALUE_REGISTER ICR3
  
  #define TIMER5_CHANNELA_PIN 46  // note that there is a webpage reference saying channel A is pin 44, that is not correct.
  #define TIMER5_CHANNELB_PIN 45  
  #define TIMER5_CHANNELC_PIN 44
  #define TIMER5_CHANNELA_SPEED_REGISTER OCR5A 
  #define TIMER5_CHANNELB_SPEED_REGISTER OCR5B
  #define TIMER5_PWM_TOP_VALUE_REGISTER ICR5
  */
   

// The radio uses pins 2, 3, and 19 (interrupts 0,1,4)
// pin 18 (interrupt 5) is available
// I2C uses pins 20 and 21 (interrupts 3 and 2)

  // Returns the interrupt number for the specified pin.
  // Returns -1 if the pin does not support interrupts.
  // This is configured for an Arduino MEGA 2560. Different boards may have different mappings.
  static int interruptNumber(int pin) {
    switch (pin) {
      case 2: return 0;  // used for RC commands, exposed on the board, can use for quadrature encoder
      case 3: return 1;  // used for RC commands, exposed on the board, can use for quadrature encoder
      case 18: return 5; // same pin as Serial1 tx, exposed on the board
      case 19: return 4; // same pin as Serial1 rx, exposed on the board
      case 20: return 3;
      case 21: return 2;
      default: return -1;
    }
  }
 #endif
 
#define DEFAULT_SERIAL_PORT Serial
#define DEFAULT_SERIAL_PORT_BAUD_RATE  115200  // pins: tx 1, rx 0, connects to laptop and server

#define USE_I2C true // we do have an I2C device, the gyro, but comm with it uses its own library
#define MAX_NUM_I2C_READINGS 12

  // general defines:
  #define NUM_SERIAL_EVENT_HANDLERS 2
  #define MOTOR_SERIAL_PORT Serial3 // Serial3 pins: tx 14, rx 15
  #define MOTOR_SERIAL_PORT_BAUD_RATE 115200
  #define MOTOR_SERIAL_PORT_TIMEOUT 2000
  #define AUTONOMOUS_SERIAL_PORT Serial2 // Serial2 pins: tx 16, rx 17, FTDI udev: /dev/ttyUSB_FTDI_A703BXBL
  #define AUTONOMOUS_SERIAL_PORT_NUMBER 2  // this is for the serialEvent handler
  #define AUTONOMOUS_SERIAL_PORT_BAUD_RATE 115200  
  #define AUTONOMOUS_COMMANDING true
  #define DEBUG_SERIAL_PORT Serial  // same as default port
  //#define BAT_MON_SERIAL_PORT Serial1
  //#define BAT_MON_SERIAL_PORT_BAUD_RATE 2400
  //#define BAT_MON_SERIAL_PORT_TIMEOUT 2000

   
  #define MIN_UPDATE_PERIOD 20 // if we go too fast, the gyro and serial ports get called too often 
  #define ENABLE_RADIO_ON_STARTUP true  // Enable radio on startup

  #define BAT_MON_BUFF_SIZE 10

                                  
  // motorPD  config
  // using pololu smc 24v23 drivers
  
  #define NUM_MOTORS_POLOLU_DRIVER 4
  #define MOTOR_RESET_PIN 48 // pin 48 connects to SMC nRST
  #define MOTOR_ERROR_PIN 49   // pin 49 connects to SMC ERR
  #define PD_SPEED_MAX_VALUE 127
  #define PD_SPEED_MIN_VALUE -127 
  #define MOTOR0_PREFERRED_SPEED PD_SPEED_MAX_VALUE //dropbar
  #define MOTOR1_PREFERRED_SPEED PD_SPEED_MAX_VALUE / 4  // bin shade
  #define MOTOR2_PREFERRED_SPEED PD_SPEED_MAX_VALUE  // picker upper
  #define BRAKES_MAX_VALUE 95 // these are 12V motors, so only drive at 75% of our 16V battery
  #define BRAKES_MIN_VALUE -95   
  #define BRAKES_SPEED 85
  #define BRAKES_APPLY_TIMER 2500
  #define BRAKES_RELEASE_TIMER 2000
  #define BRAKES_CURRENT_PIN A3
  #define BRAKES_CURRENT_THRESHOLD 450
  
  #define PICKER_UPPER_DOWN_CONTACT_PIN 8
  #define PICKER_UPPER_UP_CONTACT_PIN 9
  #define SCOOPER_TIMEOUT 20000
  
  #define MOTOR_DROP_BAR_INDEX 0
  #define MOTOR_BIN_SHADE_INDEX 1
  #define MOTOR_PICKER_UPPER_INDEX 2
  #define MOTOR_BRAKE_INDEX 3
  #define PICKER_UPPER_UP -1
  #define PICKER_UPPER_DOWN 1
  //#define PICKER_UPPER_DOWN_PREP 2
  #define DROP_BAR_UP 1
  #define DROP_BAR_DOWN -1
  #define BIN_SHADE_UP 1
  #define BIN_SHADE_DOWN -1
  
  #define DROP_BAR_UP_POSITION -100
  #define DROP_BAR_DOWN_POSITION -3000
  #define DROP_BAR_TIMEOUT  15000
  #define BIN_SHADE_UP_POSITION 0
  #define BIN_SHADE_DOWN_POSITION1 580
  #define BIN_SHADE_DOWN_POSITION2 1549
  #define BIN_SHADE_DOWN_POSITION3 2433
  #define BIN_SHADE_TIMEOUT 10000
  
  // motorPD driver device numbers (writtenonto the driver board EEPROM)
  // /dev/ttyACM_PololuMotorDriver_53FF-7506-4987-4953-3711-1387
  // /dev/ttyACM_PololuMotorDriver_53FF-7106-4987-4953-1911-1387
  // /dev/ttyACM_PololuMotorDriver_53FF-6A06-4987-4953-3025-1387 
  // /dev/ttyACM_PololuMotorDriver_53FF-7306-4987-4953-4911-1387
  
   
  #define MOTOR_DROP_BAR 14
  #define MOTOR_PICKER_UPPER 15  
  #define MOTOR_BIN_SHADE 16   // note that polarities are reversed for 16 and 17 because it was meant to be for diff drive base
  #define MOTOR_BRAKE 17  
  
  
  #define REVERSE_LEFT_PIN 40 // blue
  #define REVERSE_RIGHT_PIN 41 // orange
  #define REVERSE_ENCODER_LEFT_PIN 42  // the reverse pins are noisy, so use an additional pin to send info to the encoder
  #define REVERSE_ENCODER_RIGHT_PIN 43
  //#define REVERSE_RIGHT_PIN 42 // green and purple
  //#define REVERSE_LEFT_PIN 43// blue and orange
  
  #define NUM_DACS 2
  //#define DAC_I2C_REAR_LEFT 0x62
  //#define DAC_I2C_FRONT_LEFT 0x63
  #define DAC_I2C_RIGHT 0x60
  #define DAC_I2C_LEFT 0x61
  #define DAC_START_VALUE 720  // 720 corresponds to 0.87V
  #define DAC_MIN_VALUE 720
  #define DAC_LOWER_VALUE 900  // 1000 corresponds to about 1.2V, the working range of the controller seems to be about 1.2 to 2.5V
  #define GOOSE_SPEED 2000
  #define DAC_MAX_VALUE 2000 //2867  // corresponds to 3.5V
  #define RIGHT_MOTOR_SPEED_BIAS 40
  
  // dac_motor defines: 
  #define STOPPED_MOVING_COUNT_THRESHOLD 10
  #define NO_COMMAND_BRAKES_THRESHOLD 500
  #define USE_NO_COMMAND_MOVE_CHECK false
  #define USE_RUNAWAY_TRAIN_CHECK false
  #define RUNAWAY_SPEED_THRESHOLD 2000
  #define MAX_WHEEL_VELOCITY 4000.0f  // mm/sec, how fast can we make the wheel spin
  //#define MIN_ABSVAL_LINEAR_ROBOT_VELOCITY 500
  #define MAX_LINEAR_ROBOT_VELOCITY 2000.0f   // in mm/sec
  //#define MIN_STANDING_ABSVAL_ANGULAR_ROBOT_VELOCITY 20.0f  // in deg/sec
  #define MAX_ANGULAR_ROBOT_VELOCITY 60.0f  // in deg/sec
  #define MAX_RC_ANGULAR_VEL 20.0f  // in deg/sec
  #define MAX_RC_LINEAR_VEL 2000.0f  // in mm/sec 
  #define MOTION_BUFFER_SIZE 0
  #define WHEEL_SEPARATION 1067.f  // mm distance between powered wheels
  #define STANDING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO 100.0  // 9.3 // convert from angular velocity in degs/sec to wheel velocity in mm/sec,
                                                    // factor is (pi/180)*(WHEEL_SEPARATION/2)
  #define MOVING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO 50.
  #define MOTOR_DAC_SPEED_VELOCITY_RATIO 0.2 //0.7// 1.03  // ratio of motor speed (900 - 2867) to wheel velocity in mm/sec  Obviously depends on the surface
                                    // as hard floor will be very different from carpet
                                    // for our 12" wheels, 37.7" circumference, top speed of 120 rpm (4522 in/min = 1914 mm/sec)
                                    // is the difference between lower and max DAC values:  2867 - 900 = 1967 and 1967 / 1914 = 1.03 
  
  #define USE_ENCODERS true
  #define NUM_SPI_ENCODERS 6 
  #define LEFT_ENCODER_SELECT_PIN 35   // top board #2
  #define RIGHT_ENCODER_SELECT_PIN 34  // top board #1
  #define PICKER_UPPER_ENCODER_SELECT_PIN 36   // orange, bottom board #1  pickig up is negative, range from bottom to full pickup = 100
  #define BIN_SHADE_ENCODER_SELECT_PIN 39 // brown -> purple, middle board #1
  #define DROP_BAR_ENCODER_SELECT_PIN 37  // blue, bottom board, #2
  #define EXTRA_ENCODER_SELECT_PIN 38 
  #define REVERSE_ENCODER_LEFT_PIN 42
  #define REVERSE_ENCODER_RIGHT_PIN 43  
  #define MM_PER_TICK_EBIKE 55.
  #define MM_PER_TICK US_DIGITAL 1
  // eBike: 10 revs = 396 inches = 10000 mm = 200 ticks (20 ticks per rev), so 50 mm/tick
  // with max speed = 6 rev/sec, max ticks/ sec = 144 or 3 per arduino cycle.
  // at 10 rev/sec, max ticks / sec = 240 or about 5 per cycle.
  #define UNSTICK_SCOOPER_ATTEMPTS_ALLOWED 1
  #define SCOOPER_STUCK_ENCODER_THRESHOLD 95
  #define SCOOPER_STUCK_TIME 3000
  
  #define NUM_SERVOS 12  // make this the size of the servo board slots, because we use the slot numbers as indices, which could otherwise cause a BAD seg fault
  #define SERVO_DRIVER_DEV_NUM 20 
  #define SERVO_ERROR_PIN 44
  //#define MINI_SSC_OFFSET 0  // this is the first servo number that the servo driver uses
  #define SERVO_MAX_USEC 2400   // used to translate the get servo position call to the range 0 - 254, not used for setting positions
  #define SERVO_RANGE_USEC 1900 //1760  // min is 500  //624 useconds, so range is 2384 - 624 = 1760
  #define SERVO_CENTER_USEC 1500 //1504
  
  // note that none of these values can be > NUM_SERVOS because we create ServoDriver[NUM_SERVOS], so that will cause a seg fault
  // when you initialize the values with these numbers used as indicies
  #define SERVO_ANTENNA_PAN 0
  #define SERVO_ANTENNA_TILT 1
  #define ZOOM_DIGCAM_PAN 2
  #define DIGCAM_PAN 3
  #define FRONT_WEBCAM_PAN 7
  #define FRONT_WEBCAM_TILT 6
  //#define DIGCAM_PAN 6
  //#define DIGCAM_TILT 7
  #define SERVO_PWM_CHANNEL 8  // channel 8 is the PWM channel on the 12 channel pololu servo driver board
  //#define REAR_WEBCAM_PAN 9
  //#define REAR_WEBCAM_TILT 10
  

  
  #define MOVE_ANTENNA 0
  #define PAN_SWEEP 1
//  #define TILT_SWEEP 2
//  #define LINE_UP 3
  #define NUM_PAN_STEPS 10
  #define DEGREES_PER_PAN_STEP 0.64
  #define DEGREES_PER_TILT_STEP 0.64
  #define PAN_STEPS_PER_DEGREE 1.6
  #define TILT_STEPS_PER_DEGREE 1.6
  #define PAN_CENTER 127

  // remember if you add more servo channels, you have to enable them in the servo driver EEPROM
  
  #define DIRECTIONAL_ANTENNA_PIN A2
  #define DIRANT_PAN_CENTER 127
  #define DIRANT_PAN_MAX 254
  #define DIRANT_PAN_MIN 0
  //#define DIRANT_TILT_LEVEL 115  // this is where the antenna points level
  //#define DIRANT_TILT_CENTER 138  // halfway between min and max
  //#define DIRANT_TILT_MAX 200  // there is a mechanical stop corresponding to 1888 useconds and 254 - ((2384 - 1188) / 1760) *254) = 200
  //#define DIRANT_TILT_MIN 76   // there is a mechanical stop corresponding to 1152 useconds and 0 + ((1152 - 624) / 1760) * 254 = 76
    
  #define FRONT_WEBCAM_PAN_CENTER 127 // with a 107 center, valid inputs range from -107 to 147
  #define FRONT_WEBCAM_PAN_MAX 254
  #define FRONT_WEBCAM_PAN_MIN 0
  #define FRONT_WEBCAM_TILT_LEVEL 127 // this is where the camera points level
  #define FRONT_WEBCAM_TILT_CENTER 127 // halfway between min and max
  #define FRONT_WEBCAM_TILT_MAX 254  
  #define FRONT_WEBCAM_TILT_MIN 0 
 
  #define ZOOM_DIGCAM_PAN_CENTER 127  // with an 87 center, valid inputs range from -87 to 167
  #define DIGCAM_PAN_MAX 254
  #define ZOOM_DIGCAM_PAN_MAX 254
  #define ZOOM_DIGCAM_PAN_MIN 0
  //#define ZOOM_DIGCAM_TILT_LEVEL 60 // this is where the camera points level
  //#define ZOOM_DIGCAM_TILT_CENTER 127  // halfway between min and max
  //#define ZOOM_DIGCAM_TILT_MAX 254 
  //#define ZOOM_DIGCAM_TILT_MIN 0
 
  #define DIGCAM_PAN_CENTER 127  // with a 160 center, valid inputs range from -160 to 94, with 0 being centered
  #define DIGCAM_PAN_MIN 0
  //#define DIGCAM_TILT_LEVEL 72 // this is where the camera points level
  //#define DIGCAM_TILT_CENTER 127  // halfway between min and max
  //#define DIGCAM_TILT_MAX 200  
  //#define DIGCAM_TILT_MIN 76 
  
  //#define REAR_WEBCAM_PAN_CENTER 127
  //#define REAR_WEBCAM_PAN_MAX 254
  //#define REAR_WEBCAM_PAN_MIN 0
  //#define REAR_WEBCAM_TILT_LEVEL 55 // this is where the camera points level
  //#define REAR_WEBCAM_TILT_CENTER 127  // halfway between min and max
  //#define REAR_WEBCAM_TILT_MAX 254 
  //#define REAR_WEBCAM_TILT_MIN 0 
  
  // PID defines:
  #define KP_LINEAR 0.6f
  #define KI_LINEAR 2.0f
  #define KD_LINEAR 0.0f
  #define KP_GYRO 0.7f 
  #define KI_GYRO 10.0f //5.0f  // no point in going much greater than 10.0, as the max change per cycle limits how fast the output can change.
  #define KD_GYRO 0.0f 
  
  // gyro defines:
  #define USE_GYRO true
    // gyro gain + or - sign depends on gyro orientation.  Positive gyro readings should indicate a turn to the left (CCW) to conform to ROS right hand rules.
  //#define GYRO_GAIN_YAW 0.07 // when full scale sensitivity is 2000 dps = 70 mdps/digit
  //#define GYRO_GAIN_YAW 0.01825 //0.0175 //  when full scale sensitivity is 500 dps = 17.5 mdps/digit
  
  // NOTE!! gyro gain seems to have drifted over time, had to increase the gain to make it stay accurate!
  
  #define GYRO_GAIN_YAW 0.00875 // when full scale sensitivity is 250 dps = 8.75 mdps/digit
  #define ACCEL_CONVERSION_TO_M_PER_SEC_SQUARED 0.00231  //0.000568 //0.0374
  
// The radio uses pins 2, 3, and 19 (interrupts 0,1,4)
// pin 18 (interrupt 5) is available
// I2C uses pins 20 and 21 (interrupts 3 and 2)
#define RADIO_CONNECTED_PIN 4
#define RADIO_PAUSE_PIN 5                    // switch throw for pause function
#define RADIO_JOYSTICK_X_PIN 3               // movement of joystick in x direction
#define RADIO_JOYSTICK_Y_PIN 2               // movement of joystick in y direction
#define RADIO_MIN_HIGH_DURATION_MICROS 1000  // minimum high signal duration (radio is off below this value)
#define RADIO_NEGATIVE_START_MICROS 1500     // signals below this value are interpreted as negative
#define RADIO_NEGATIVE_RANGE_MICROS 350.0f   // difference between highest and lowest value in negative range
#define RADIO_POSITIVE_START_MICROS 1530     // signals above this value are interpreted as positive
#define RADIO_POSITIVE_RANGE_MICROS 350.0f   // difference between highest and lowest value in positive range
#define RADIO_MIN 0.21f                       // < RADIO_MIN is interpreted as zero input
#define RC_LINEAR_LIMITER 0.25                       // reduces the forward and reverse speeds that max rc input can reach.  does not limit turning speeds

#define AMBER_LIGHT_PIN 6  // solid on when paused, flashing at approx 1 Hz when enabled.
#define AMBER_LIGHT_BLINK_TIME 1000

#define BATTERY_MONITOR_PIN A4  // note that this refers to arduino pin A4, since it is an analog read
#define ZERO_PERCENT_BATTERY_VOLTAGE 10.5
#define FULL_BATTERY_VOLTAGE 13.0
#define VOLTAGE_DIVIDER_RATIO 3.2  // R1 / R1 + R2 in theory, determined in practice by 
  
#endif  CONFIG_H_

