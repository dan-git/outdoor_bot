/* 
 *
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
 * Definition of hardware specific motor settings.
 **************************************************************************/
 
//  To use addressed commands, transmit 0xAA as the first (command) byte, followed by a Device Number data byte. 
//  Any controller on the line whose device number matches the specified device number accepts the command that follows.
//  all other Pololu devices ignore the command.

//  To command all devices, use the "compact command" which simply does not have the above stuff.  Note that
//  this format doesn't work for commands that require a response, as all the devices would respond at once
//  and interfere with each other.
//  Also note that it doesn't work if there are other types of devices chained to this serial port
//  Specifically, we have a servo driver also using the port, so we cannot use compact commands or the different devices will get confused.

// A quirky feature of the interface is that command bytes always have their most significant bit (not byte) set,
// while data bytes almost always have their most significant bit cleared.

// For example, when using the compact command format, the command might be 0x85 (move forward)
// and the data bytes might be 0 and 50 (at 50% speed).  Like this: 0x85 0 50
// However, when using the addressed command, the 0x85 is
// not used, instead 0x05 is used for move forward.  That's because it is considered a data byte of the 0xAA command.
// So to get device 14 to go at 50% speed, the bytes would be : 0xAA 14 0x05 0 50
// The above is a poor example, because it suggests that the entire most significant BYTE be set to 0 for data
// but really, it is just the most significant BIT

// Finally, there is the mini-SSC protocol that we will actually use to set motor speeds.
// It reduces the number of bytes needed for speed commands.
// Send 0xFF followed by the device number followed by the speed value where 
// 0 = full reverse, 127 = stop, 254 = full forward
// if you use a device number = 254, then all the devices will respond to the speed command.
// we will use that for standard forward and reverse moves.

// moving:
//   forward:
//       compact format: 0x85, speed byte 1, speed byte 2
//       addressed format: 0x05, device number, speed byte 1, speed byte 2
//   reverse:
//       compact format: 0x86, speed byte 1, speed byte 2
//       addressed format: 0x06, device number, speed byte 1, speed byte 2 
//   but we won't use the above two, since we will use mini-SSC for setting speeds
//   mini-SSC format: 0xFF, device number (254 for all devices), speed byte (0 full reverse, 127 stop, 254 full forward)

// stop (will use decel limits to slow and stop, will need safe-start conditions set before moving again)
//   compact format: 0xE0
//   addressed format: 0xAA device number, 0x60

// braking:
//   compact format, 0x92 brake amount (from 0 (coast) to 32 (full brake)
//   addressed format:  0x92 device number 0x12 brake amount 

// get Variable:
//   compact: 0xA1 variable ID
//   addressed: 0xAA device number 0x21 variable ID
//   response byte 1 is the low byte, response byte 2 is the high byte
//   Variable IDs:
//    board temperature (variable ID 24), return in units of 0.1 C  T = byte1 + (256 * byte2)
//    error status (ID 0), errors occurred (ID 1, reading clears it), serial errors occured (ID 2)
//    motor limit status (ID 3), source of reset (ID 127)

// get Firmware Version:
//   0xAA devNum, 0x42
//   responds with 4 bytes:  product ID low byte, product ID high byte, minor Firmware version, major Firmware version
//  
// Set temporary motor limit 0xAA, devNum, 0x22, limit ID, limit byte 1, limit byte 2
//   returns a response code
//   see the docs for limit IDs
//   response codes (basically, 0 if set OK, other number if not)
//   0: OK.
//   1: Unable to set forward limit to the specified value because of Hard Motor Limit settings.
//   2: Unable to set reverse limit to the specified value because of Hard Motor Limit settings.
//   3: Unable to set forward and reverse limits to the specified value because of Hard Motor Limit settings.


#ifndef MOTOR_POLOLU_DRIVER_H_
#define MOTOR_POLOLU_DRIVER_H_


// some variable IDs
#define ERROR_STATUS_ID 0
/*
#define ERROR_HISTORY_ID 1
#define SERIAL_ERRORS_ID 2
#define LIMIT_STATUS_ID 3
#define SOURCE_OF_RESET_ID 127
#define TARGET_SPEED_ID 20
#define INPUT_VOLTAGE_ID 23
#define TEMPERATURE_ID 24

// some motor limit IDs (not values, just the variable's ID)
  
// these ID's cover both forward and reverse
#define MAX_SPEED_ID 0  // 0 - 3200
#define MAX_ACCELERATION_ID 1 // 0 - 3200 with 0 = no limit, units are delta speed per update
#define MAX_DECELERATION_ID 2 // 0 - 3200 with 0 = no limit, units are delta speed per update

#define MIN_BRAKE_DURATION_WHEN_SWITCHING_DIRECTIONS_ID 3  // 0 - 16384  NOTE:  units are 4 ms, not 1 ms !!!

// these ID's are for just one direction
#define MAX_FORWARD_SPEED_ID 4
#define MAX_FORWARD_ACCELERATION_ID 5
#define MAX_FORWARD_DECELERATION_ID 6
#define MIN_FORWARD_BRAKE_DURATION_WHEN_SWITCHING_DIRECTIONS_ID 7
#define MAX_REVERSE_SPEED_ID 8
#define MAX_REVERSE_ACCELERATION_ID 9
#define MAX_REVERSE_DECELERATION_ID 10
#define MIN_REVERSE_BRAKE_DURATION_WHEN_SWITCHING_DIRECTIONS_ID 11
*/


// Represents a motor.
class motor_pololu_driver {
  public:
    motor_pololu_driver(int devNum) {    
    devNum_ = devNum; // identifies which motor this is
    exitSafeStart();    
    setRegenBrakes();
    } 
    
    // required to allow motors to move
    // this command must be issued when the controller is first powered
    // up, after any reset, and after any error stops the motor. This command has no serial response.    
    void exitSafeStart(boolean setAll = false)
    {
      if (setAll) MOTOR_SERIAL_PORT.write(0x83);  // enables all motors at once
      else
      {
        MOTOR_SERIAL_PORT.write(0xAA);
        MOTOR_SERIAL_PORT.write(devNum_);
        MOTOR_SERIAL_PORT.write(0x03);
      }  
    }
    
    // This command sets the motor speed 
    // speed should be a number from PD_SPEED_MIN_VALUE to PD_SPEED_MAX_VALUE (-127 to 127)
    void setMotorSpeed(int speed) 
    {
      //if (robotPause_ && newPauseState_) return; // we need to control the motors until we stop them in sensors.h, then don't respond to move commands.
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("motorPD speed received = ");
        DEBUG_SERIAL_PORT.println(speed);
      }
      if (speed < PD_SPEED_MIN_VALUE || speed > PD_SPEED_MAX_VALUE) 
      {
        if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print("error, in setMotorSpeed, out of bounds speed value = ");
          DEBUG_SERIAL_PORT.println(speed);
        }
        speed = 0;  // if we get a weird speed, just stop  
      }
      previousSpeed_ = speed_;
      speed_ = speed;
      
      speed += PD_SPEED_MAX_VALUE; // with mini-SSc format, values go from 0 (full reverse) to 254 (full forward), with PD_SPEED_MAX_VALUE = stop
      MOTOR_SERIAL_PORT.write(0xFF);
      MOTOR_SERIAL_PORT.write(devNum_);
      MOTOR_SERIAL_PORT.write(speed);
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("motor speed set = ");
        DEBUG_SERIAL_PORT.println(speed);
      }
    }
    
   
    //This command causes the motor to immediately brake by the specified amount (configured
    //deceleration limits are ignored). The Brake Amount byte can have a value from 0 to 32, with 0 resulting in
    //maximum coasting (the motor leads are floating almost 100% of the time) and 32 resulting in full braking (the
    //motor leads are shorted together 100% of the time). Requesting a brake amount greater than 32 results in a Serial
    //Format Error. This command has no serial response.
    
    //NOTE: THIS DOES NOT ACTUATE THE ROBOT'S BRAKES!  IT JUST SETS MOTOR BRAKING FOR AN INDIVIDUAL MOTOR
    
    void setRegenBrakes(int brakingValue = 32, boolean setAll = false)
    {
        if (brakingValue < 0 || brakingValue > 32) brakingValue = 32; 
  
        if (setAll) MOTOR_SERIAL_PORT.write(0x92);  // sends to all devices, have to update the speed variable separately
        else
        {
          MOTOR_SERIAL_PORT.write(0xAA);
          MOTOR_SERIAL_PORT.write(devNum_);
          MOTOR_SERIAL_PORT.write(0x12);
        }
        MOTOR_SERIAL_PORT.write(brakingValue);
        speed_ = 0;
    }
    
    // set motor limits
    int setMotorLimit(unsigned char limitID, unsigned int limitValue)
    {
       //if (setAll) MOTOR_SERIAL_PORT.write(0xA2);  // sends to all devices
       //else
      //{
        MOTOR_SERIAL_PORT.write(0xAA);
        MOTOR_SERIAL_PORT.write(devNum_);
        MOTOR_SERIAL_PORT.write(0x22);
      //}
      
        MOTOR_SERIAL_PORT.write(limitID);
        MOTOR_SERIAL_PORT.write(limitValue & 0x7F);  // low seven bits of limitValue (we are taking modulus 128) for limit byte 1
        MOTOR_SERIAL_PORT.write(limitValue >> 7);    // high seven bits (we are dividing by 128) for limit byte 2
      
      //if (!setAll)  // if just setting a single motor, we can get its response
      //{
        char c;
        if(MOTOR_SERIAL_PORT.readBytes(&c, 1) == 0){ return -1; }
        return (byte)c;
      //}
      //delay(1000); // wait for all the replies to finish
      //while (MOTOR_SERIAL_PORT.available()) MOTOR_SERIAL_PORT.read();  // multiple responses will be garbled, so just flush them
      //return 0;
    }
    
    
    // This command sets the motor target speed to zero and makes the controller susceptible to a safe-start
    // violation error if Safe Start is enabled. Put another way, this command will stop the motor (configured deceleration
    // limits will be respected) and not allow the motor to start again until the Safe-Start conditions required by the Input
    // Mode are satisfied. This command has no serial response.
    
    void stopAndDisable()
    {
      //if (setAll) MOTOR_SERIAL_PORT.write(0xE0); // sends to all devices, have to update the speed variable separately
      //else
      //{
        MOTOR_SERIAL_PORT.write(0xAA);
        MOTOR_SERIAL_PORT.write(devNum_);
        MOTOR_SERIAL_PORT.write(0x60);
      //}
    }
    
    //This command lets you read the Simple Motor Controller product number and firmware version
    //number. The first two bytes of the response are the low and high bytes of the product ID (each Simple Motor
    //Controller version has a unique product ID), and the last two bytes of the response are the firmware minor and
    //major version numbers in binary-coded decimal (BCD) format [http://en.wikipedia.org/wiki/Binary-coded_decimal].
    //BCD format means that the version number is the value you get when you write it in hex and then read it as if it
    //were in decimal. For example, a minor version byte of 0x15 (21) means a the minor version number is 15, not 21.
    
    boolean showFirmwareVersion()
    {
      char buf[10];
      unsigned char out[5];
      MOTOR_SERIAL_PORT.write(0xAA);
      MOTOR_SERIAL_PORT.write(devNum_);
      MOTOR_SERIAL_PORT.write(0x42);
      
      // responds with 4 bytes:  product ID low byte, product ID high byte, minor Firmware version, major Firmware version
    
      if (MOTOR_SERIAL_PORT.readBytes(buf,4) == 4) // read four bytes
      {    
        // reaadBytes will only take a signed char array, but the values being
        // sent are unsigned, so if a value is > 127, it is a negative number in buf
        // so we have to convert to unsigned or the numbers are wrong
        for (int i=0; i < 2; i++) out[i] = (buf[i] < 0)?(buf[i] + 256):buf[i];
        
        DEBUG_SERIAL_PORT.print("For device number ");
        DEBUG_SERIAL_PORT.println(devNum_);
        DEBUG_SERIAL_PORT.print("motor_pololu_driver product ID is ");
        DEBUG_SERIAL_PORT.println(out[0] + (out[1] * 256));
        DEBUG_SERIAL_PORT.print("Firmware version is: ");
        DEBUG_SERIAL_PORT.print(buf[3], HEX);
        DEBUG_SERIAL_PORT.print(".");
        DEBUG_SERIAL_PORT.println(buf[2], HEX);
        return true;
      }
      return false;
    }
      
     
    // puts the specified variable into value as an unsigned integer.
    // if the requested variable is signed, the value should be typecast as an int.
    // Requesting variable IDs between 41 and 127 results in a Serial Format Error, and the controller does not transmit a response.
    
    boolean getVariable(unsigned char variableID, unsigned int *value)
    {
      char buf[10];
      unsigned char out[3];
      if (variableID > 40 && variableID < 128)
      {
        DEBUG_SERIAL_PORT.print("Variable ID out of range");
        return false;
      }
      MOTOR_SERIAL_PORT.write(0xAA);
      MOTOR_SERIAL_PORT.write(devNum_);
      MOTOR_SERIAL_PORT.write(0x21);
      MOTOR_SERIAL_PORT.write(variableID);
      if (MOTOR_SERIAL_PORT.readBytes(buf,9) == 2) // try to read two bytes, could put up to 9 into buffer, that would mean there is garbage in the port
      {
        // readBytes will only take a signed char array, but the values being
        // sent are unsigned, so if a value is > 127, it is a negative number in buf
        // so we have to convert to unsigned or the numbers are wrong
        for (int i=0; i < 2; i++) out[i] = (buf[i] < 0)?(buf[i] + 256):buf[i];
        
        *value = out[0] + (out[1] * 256);
        return true;
      }
      // possibly there is garbage in the port, flush it out
      while (MOTOR_SERIAL_PORT.available()) MOTOR_SERIAL_PORT.readBytes(buf,9);
      return false;
    }
    
    //    error status (ID 0), errors occurred (ID 1, reading clears it), serial errors occured (ID 2)
    //    motor limit status (ID 3), source of reset (ID 127)
    
    
    boolean getErrors( unsigned int *errorValue) 
    {
      unsigned int value;
      boolean errorFound = false;     
        /*
        ERROR_STATUS
        Bit 0: Safe Start Violation
        Bit 1: Required Channel Invalid
        Bit 2: Serial Error
        Bit 3: Command Timeout
        Bit 4: Limit/Kill Switch
        Bit 5: Low VIN
        Bit 6: High VIN
        Bit 7: Over Temperature
        Bit 8: motor_pololu_driver error
        Bit 9: ERR Line High
        */
        *errorValue = 0;
        if (getVariable(ERROR_STATUS_ID, &value))
        {
          DEBUG_SERIAL_PORT.print("motor_pololu_driver ");
          DEBUG_SERIAL_PORT.print(devNum_);
          if (value == 0) DEBUG_SERIAL_PORT.println(" currently reports no errors");
          else 
          {
            DEBUG_SERIAL_PORT.print(" reports errors.  Error code = ");
            DEBUG_SERIAL_PORT.println(value);
            *errorValue = value;
            errorFound = true;
          }
        }
      return errorFound;
    }
     
    int getSpeed()
    {
      return speed_;
    }
    
    void setSpeedValue(int value)
    {
      previousSpeed_ = speed_;
      speed_ = value;
    }
    
    int getPreviousSpeed()
    {
      return previousSpeed_;
    }  
    
    int getDevNum()
    {
      return devNum_;
    }
    
    void setDevNum(int value)
    {
      devNum_ = value;
    }
    
     
  private:
    int devNum_;
    int speed_;  // current speed
    int previousSpeed_;
}

// if you change the order here, change outdoor_bot_defines.h in the ROS code and also in config.h in arduino code
// for the values of MOTOR_DROP_BAR_INDEX, MOTOR_BIN_SHADE_INDEX, MOTOR_PICKER_UPPER_INDEX, MOTOR_BRAKE_INDEX
motor_pololu_driver[NUM_MOTORS_POLOLU_DRIVER] =  {MOTOR_DROP_BAR, MOTOR_BIN_SHADE, MOTOR_PICKER_UPPER, MOTOR_BRAKE}; 
int previousPDmotorSpeed_[NUM_MOTORS_POLOLU_DRIVER];
int newPauseState_;

// pull shade down is direction -1, for example:  autoPP(0,-1);
// let dropbar down is direction -1
// let scoop down up is direction -1
// set brakes is either direction


int getMotorPDNumber(int driverNumber)
{
  if (driverNumber == MOTOR_DROP_BAR) return 0;
  if (driverNumber == MOTOR_BIN_SHADE) return 1;
  if (driverNumber == MOTOR_PICKER_UPPER) return 2;
  if (driverNumber == MOTOR_BRAKE) return 3;
  Serial.println("ERROR:  unknown PD driver number sent to getMotorPDNumber!");
  return -1;
}

/*

int getMotorPDDriverNumber(int motorNumber)
{
  if (motorNumber == 0) return MOTOR_DROP_BAR;
  if (motorNumber == 1) return MOTOR_BIN_SHADE;
  if (motorNumber == 2) return MOTOR_PICKER_UPPER;
  if (motorNumber == 3) return MOTOR_BRAKE;
  Serial.println("ERROR:  unknown PD motor number sent to getMotorPDDriverNumber!");
  return -1; 
} 

int getMotorPDEncoderNumber(int driverNumber)
{
  if (driverNumber == MOTOR_DROP_BAR) return DROP_BAR_ENCODER_SELECT_PIN;  
  if (driverNumber == MOTOR_BIN_SHADE) return BIN_SHADE_ENCODER_SELECT_PIN;
  if (driverNumber == MOTOR_PICKER_UPPER) return PICKER_UPPER_ENCODER_SELECT_PIN;
  if (driverNumber == MOTOR_BRAKE) return -1; // no encoder
  Serial.println("ERROR:  unknown PD driver number sent to getMotorPDNumber!");
  return -2;
}


int getMotorPD_EncoderNumber(int motorNumber)
{
  if (motorNumber == 0) return DROP_BAR_ENCODER_SELECT_PIN;
  if (motorNumber == 1) return BIN_SHADE_ENCODER_SELECT_PIN;
  if (motorNumber == 2) return PICKER_UPPER_ENCODER_SELECT_PIN;
  //Serial.println("ERROR:  unknown PD motor number sent to getMotorPDDriverNumber!");
  return -1; 
}
*/

#endif  // MOTOR_H_

