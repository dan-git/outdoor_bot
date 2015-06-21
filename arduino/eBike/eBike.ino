/*
 *
 * The code is divided into modules. This file defines the main setup()
 * and loop() functions and functions that handle serial control.
 *
 * Specific behavior should be defined in an existing or new module.
 * Modules are automatically included by setup() and loop() when they
 * define their respective ModuleSetup and ModuleLooper classes.
 *
 * To have a module specific setup() called, define a subclass of
 * ModuleSetup in the module and override its run() method. It will
 * automatically get called by setup().
 *
 * To have a module specific method be called by loop(), define a subclass
 * of ModuleLoop in the module and override its run() method. It will
 * automatically get called with each iteration of loop().
 *
 * The MAX_NUM_MODULES flag in config.h limits the maximum number of
 * permissible modules.
 *************************************************************************
 */
 
#include <Arduino.h>

#include <Wire.h>
#include <I2C.h>
#include <SPI.h>
//#include <L3G.h>  // gyro    // same deal, cannot put these in gyro.h for some reason
#include <L3G_newI2C.h>
#include <LSM303.h> // for compass and accelerometer

#include "config.h"
#include "core.h"
#include "batMon.h"
#include "motorPD.h"
#include "dac.h"
#include "motorDAC.h"
#include "pidControl.h"
#include "encoderSPI.h"
#include "gyro.h"
#include "servosP.h"
#include "motionPD.h"
#include "motionServos.h"
#include "motionDAC.h"
#include "radio.h"
#include "sensors.h"


// serial_queue[i] represents the current unprocessed input from the respective serial port.
// it gets filled by serial event handlers.
String serial_queue[NUM_SERIAL_EVENT_HANDLERS];


void startSerialPorts()
{
  #ifdef  DEFAULT_SERIAL_PORT
    DEFAULT_SERIAL_PORT.begin(DEFAULT_SERIAL_PORT_BAUD_RATE);
    while (!DEFAULT_SERIAL_PORT);
    delay(50);
    DEFAULT_SERIAL_PORT.println("Default serial port working");
  #endif

  #ifdef AUTONOMOUS_SERIAL_PORT 
    AUTONOMOUS_SERIAL_PORT.begin(AUTONOMOUS_SERIAL_PORT_BAUD_RATE);
    while (!AUTONOMOUS_SERIAL_PORT);
    delay(50);
    DEFAULT_SERIAL_PORT.println("autonomous serial port working");
  #endif
  
  #ifdef MOTOR_SERIAL_PORT
    MOTOR_SERIAL_PORT.begin(MOTOR_SERIAL_PORT_BAUD_RATE);
    while (!MOTOR_SERIAL_PORT);
    delay(50);
    MOTOR_SERIAL_PORT.setTimeout(MOTOR_SERIAL_PORT_TIMEOUT);
    DEFAULT_SERIAL_PORT.println("motor serial port working");
  #endif 
 /* 
  #ifdef BAT_MON_SERIAL_PORT
    BAT_MON_SERIAL_PORT.begin(BAT_MON_SERIAL_PORT_BAUD_RATE);
    while (!BAT_MON_SERIAL_PORT);
    delay(50);
    BAT_MON_SERIAL_PORT.setTimeout(BAT_MON_SERIAL_PORT_TIMEOUT);
    DEBUG_SERIAL_PORT.println("Default serial port working");
  #endif
  */  
  
}

// Main setup function. Module setup function are automatically called and
// should not be added to this method.

void setup() {
    for (int i = 0; i < NUM_SERIAL_EVENT_HANDLERS; ++i) {
    serial_queue[i] = "";
  }
 
  robotPause_ = false;
  newPauseState_ = true;
  toggleAmberLightState_ = 0;
  long pauseBlinkTime_ = 1000;
  

  Wire.begin();
  startSerialPorts();
  delay(200);
  if (DEBUG) DEBUG_SERIAL_PORT.println("Serial ports are setup");
  
  
  rcs::ModuleSetup::runAll();
  rcs::robot_base.stop();
  
  encoder_spi[getEncoderNumber(LEFT_ENCODER_SELECT_PIN)].setEncoderMode_SPI(false); // put encoder in non-quadrature mode
  encoder_spi[getEncoderNumber(RIGHT_ENCODER_SELECT_PIN)].setEncoderMode_SPI(false); // put encoder in non-quadrature mode
  encoder_spi[getEncoderNumber(LEFT_ENCODER_SELECT_PIN)].clearEncoderCount(); // set encoders to 0
  encoder_spi[getEncoderNumber(RIGHT_ENCODER_SELECT_PIN)].clearEncoderCount();
   
  encoder_spi[getEncoderNumber(PICKER_UPPER_ENCODER_SELECT_PIN)].setEncoderMode_SPI(true); // put encoder in quadrature mode
  encoder_spi[getEncoderNumber(BIN_SHADE_ENCODER_SELECT_PIN)].setEncoderMode_SPI(true); // put encoder in quadrature mode
  encoder_spi[getEncoderNumber(DROP_BAR_ENCODER_SELECT_PIN)].setEncoderMode_SPI(true); // put encoder in quadrature mode 
  encoder_spi[getEncoderNumber(PICKER_UPPER_ENCODER_SELECT_PIN)].clearEncoderCount(); // set encoders to 0
  encoder_spi[getEncoderNumber(BIN_SHADE_ENCODER_SELECT_PIN)].clearEncoderCount();
  encoder_spi[getEncoderNumber(DROP_BAR_ENCODER_SELECT_PIN)].clearEncoderCount();
   
   digitalWrite(REVERSE_ENCODER_RIGHT_PIN, HIGH);
   digitalWrite(REVERSE_ENCODER_LEFT_PIN, HIGH);
  
  for (int i = 0; i < NUM_MOTORS_POLOLU_DRIVER; i++)
  {
    motor_pololu_driver[i].setRegenBrakes(); // stop accessory motors, note it does not set the robot's brakes!
    previousPDmotorSpeed_[i] = 0;  // this is used to store motor speeds during a pause state
  }
  
  for (int i = 0; i < NUM_DACS; i++)
  {
    motor_dac[i].stop();
    previousDACmotorSpeed_[i] = DAC_MIN_VALUE;
  }
  
  servoDriver.setPosition(SERVO_ANTENNA_PAN, DIRANT_PAN_CENTER);
  //servoDriver.setPosition(SERVO_ANTENNA_TILT, DIRANT_TILT_LEVEL);
  servoDriver.setPosition(DIGCAM_PAN, DIGCAM_PAN_CENTER);
  servoDriver.setPosition(ZOOM_DIGCAM_PAN, ZOOM_DIGCAM_PAN_CENTER);
  servoDriver.setPosition(FRONT_WEBCAM_PAN, FRONT_WEBCAM_PAN_CENTER);
  servoDriver.setPosition(FRONT_WEBCAM_TILT, FRONT_WEBCAM_TILT_LEVEL);
  

  I2c.timeOut(1000);
}

void loop()
{
  //DEFAULT_SERIAL_PORT.println("in loop");
 
 /*for (int i=0; i < 4000; i++)
 {
   dac[0].setVoltage(DAC_START_VALUE + i);
   dac[1].setVoltage(DAC_START_VALUE + i);
   delay(10);
 }
 delay(1000);
 rcs::ping.resetTimeout();
 */
 
  #if TIMEOUT
  if (rcs::ping.timeout()) {
     if (DEBUG) DEFAULT_SERIAL_PORT.println("timeout");
 //    motor[0].setRegenBrakes(); 
 //    for (int i=0; i < NUM_MOTORS; i++) motor[i].setSpeed_ValueOnly(0);  // updates the speed variable (does not change the actual motor speed) 
     //rcs::stop();  // this runs the instruction stop which calls robot_base.stop() which stops the robot and resets all the parameters
     rcs::ping.resetTimeout();  // only call the timeout once for each timeout period
  }
  #endif
  for (int i = 0; i < NUM_SERIAL_EVENT_HANDLERS; ++i) {
    if (serial_queue[i].length() >= MAX_SERIAL_INPUT_SIZE) serial_queue[i] = ""; // serial queues are being filled in the serial event handlers
      // Pop next instruction call.
      int index = serial_queue[i].indexOf(';');
      if (index >= 0) {
        if (DEBUG) DEBUG_SERIAL_PORT.println(serial_queue[i]);
        // Parse and execute instruction.
        String instruction_command = serial_queue[i].substring(0, index);
        index++;
        if (serial_queue[i].length() > index) {
          serial_queue[i] = serial_queue[i].substring(index);
        } else {
          serial_queue[i] = "";
        }
        if (instruction_command.length() >= 3 &&
            instruction_command[instruction_command.length() - 1] == ')') {
          index = instruction_command.indexOf('(');
          if (index > 1) {
            String instruction_name = instruction_command.substring(0, index);
            String args = instruction_command.substring(index + 1, instruction_command.length() - 1);
            boolean success = rcs::Instruction::run(instruction_name, &args);
            if (DEBUG && !success) {
              DEBUG_SERIAL_PORT.println("error in running an instruction");
            }
          } // closes if (index > 1).....
        } // closes if (instruction_..)
      }  // closes if (index...)
  } // closes for(...)
  rcs::ModuleLoop::runAll(); 
}

// Serial event handlers, during each loop cycle this runs and will load up serial_queue

void serialEvent()
{
  while (Serial.available() && serial_queue[0].length() < MAX_SERIAL_INPUT_SIZE)
  {
    char ch = (char) DEFAULT_SERIAL_PORT.read();
    serial_queue[0] += ch;
  }
}

#ifdef AUTONOMOUS_SERIAL_PORT
  #if AUTONOMOUS_SERIAL_PORT_NUMBER == 2
  void serialEvent2()
  {
    while (AUTONOMOUS_SERIAL_PORT.available() && serial_queue[1].length() < MAX_SERIAL_INPUT_SIZE)
    {
      char ch = (char) AUTONOMOUS_SERIAL_PORT.read();
      serial_queue[1] += ch;
    }
  }
  
  #elif AUTONOMOUS_SERIAL_PORT_NUMBER == 3
  void serialEvent3()
  {
    while (AUTONOMOUS_SERIAL_PORT.available() && serial_queue[1].length() < MAX_SERIAL_INPUT_SIZE)
    {
      char ch = (char) AUTONOMOUS_SERIAL_PORT.read();
      serial_queue[1] += ch;
    }
  }
  #endif
#endif
