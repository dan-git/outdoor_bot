// pololu maestro servo control

// VERY IMPORTANT-- set the mini SSC Offset in the driver's EEPROM to allow all 12 of the servos
// to have numbers that do not conflict with the other devices on this serial port
// Specifically, they cannot have a value that is the same as the device number of one of the motor drivers
// The motor driver device numbers are 14,15,16,17 and the servo driver device number is 20
// So for now the default offset of 0 is fine, since the servos will have addresses = 0 to 11
// Since we can use a value of 0, we do not bother to include it in the code.  If it changes
// then you can assign the offset value in config.h (MINI_SSC_OFFSET) and add that to servoNum in the code below

// VERY IMPORTANT-- the channels need to be enabled in the EEPROM to be active.
// I have currently enabled servo channels 0 to 5 and PWM on its channel (8)

//  To use addressed commands, transmit 0xAA as the first (command) byte, followed by a Device Number data byte. 
//  Any controller on the line whose device number matches the specified device number accepts the command that follows.
//  all other Pololu devices ignore the command.

// A quirky feature of the interface is that command bytes always have their most significant bit (not byte) set,
// while data bytes almost always have their most significant bit cleared.


#ifndef SERVOS_P_H
#define SERVOS_P_H

class servoDriver {
  public:
    // Interrupts must be supported on the specified pins.
    // See attach() method for supported pins.
    servoDriver(int devNum)
    {
      servoDevNum_ = devNum;
      setMaxMinCenterLevelValues();
    }
  
    void setPosition(int servoNum, int value)  // full range values are 0 to 254 with 127 being center of the range (not necessarily camera facing the center of the field)
    {
        if (robotPause_) return;
        if (servoNum < 0 || servoNum > NUM_SERVOS - 1 || servoNum == SERVO_PWM_CHANNEL) return;
        //int setServo = value + servoCenter_[servoNum];     // we add the center value, so if the center is not 127, the range is not full on one side.
        int setServo = value;
        if (setServo > servoMax_[servoNum]) setServo = servoMax_[servoNum];
        else if (setServo < servoMin_[servoNum]) setServo = servoMin_[servoNum];        
        MOTOR_SERIAL_PORT.write(0xFF);
        MOTOR_SERIAL_PORT.write(servoNum);
        MOTOR_SERIAL_PORT.write(setServo); // command to set position, with the MSB set to 0
        if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print("Servo number, position set = ");
          DEBUG_SERIAL_PORT.print(servoNum);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(setServo);
          //long servoPos;
          //getPosition(servoNum, &servoPos);
          //DEBUG_SERIAL_PORT.print("Servo position reads as = ");
          //DEBUG_SERIAL_PORT.println(servoPos);
          
        }
    }
    
    // sets the PWM output to the specified on time and period, in units of 1/48 μs
    // This is on channel 8 for the 12 channel device and also, that channel must be set to OUTPUT in the device EEPROM for this to work.
    void setPWM(int onTime, int period)
    {
      if (robotPause_) return;
      MOTOR_SERIAL_PORT.write(0xAA);
      MOTOR_SERIAL_PORT.write(servoDevNum_);
      MOTOR_SERIAL_PORT.write(0x0A);
      MOTOR_SERIAL_PORT.write(onTime & 0x7F); // the lower 7 bits of onTime.
      MOTOR_SERIAL_PORT.write((onTime >> 7) & 0x7F); // bits 7-13 of onTime
      MOTOR_SERIAL_PORT.write(period & 0x7F); // the lower 7 bits of period.
      MOTOR_SERIAL_PORT.write((period >> 7) & 0x7F); // bits 7-13 of period.
    }
      
    
    boolean getPosition(int servoNum, long *value)  // not working, get numbers for useconds that are too large by about a factor of 4
    {
      char buf[10];
      unsigned char out[3];
      MOTOR_SERIAL_PORT.write(0xAA);
      MOTOR_SERIAL_PORT.write(servoDevNum_);
      MOTOR_SERIAL_PORT.write(0x10);
      MOTOR_SERIAL_PORT.write(servoNum);
      if (MOTOR_SERIAL_PORT.readBytes(buf,9) == 2) // try to read two bytes, could put up to 9 into buffer, that would mean there is garbage in the port
      {
        // reaadBytes will only take a signed char array, but the values being
        // sent are unsigned, so if a value is > 127, it is a negative number in buf
        // so we have to convert to unsigned or the numbers are wrong
        for (int i=0; i < 2; i++) out[i] = (buf[i] < 0)?(buf[i] + 256):buf[i];
        
        long useconds = out[0] + (out[1] * 256);
        *value = useconds;
        //*value = ((SERVO_MAX_USEC - useconds) * 254) / SERVO_RANGE_USEC; // return in the range 0-254, different from our Position input range
        return true;
      }
      // possibly there is garbage in the port, flush it out
      while (MOTOR_SERIAL_PORT.available()) MOTOR_SERIAL_PORT.readBytes(buf,9);
      return false;
    }
    
    boolean getMovingState(int *value) // puts 0 into value if no servos are moving, 1 otherwise
    {
      char buf[10];
      MOTOR_SERIAL_PORT.write(0xAA);
      MOTOR_SERIAL_PORT.write(servoDevNum_);
      MOTOR_SERIAL_PORT.write(0x13);
      if (MOTOR_SERIAL_PORT.readBytes(buf,9) == 1) // try to read one byte, could put up to 9 into buffer, that would mean there is garbage in the port
      {
        *value = buf[0];
        return true;
      }
      // possibly there is garbage in the port, flush it out
      while (MOTOR_SERIAL_PORT.available()) MOTOR_SERIAL_PORT.readBytes(buf,9);
      return false;
    }
    
    boolean getErrors(int servoNum, unsigned int *value)  // calling this clears the error buffer and reports the error values
    {
      char buf[10];
      unsigned char out[3];
      MOTOR_SERIAL_PORT.write(0xAA);
      MOTOR_SERIAL_PORT.write(servoDevNum_);
      MOTOR_SERIAL_PORT.write(0x21);
      if (MOTOR_SERIAL_PORT.readBytes(buf,9) == 2) // try to read two bytes, could put up to 9 into buffer, that would mean there is garbage in the port
      {
        // reaadBytes will only take a signed char array, but the values being
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
    
    int setMaxMinCenterLevelValues()
    {
      servoCenter_[SERVO_ANTENNA_PAN] = DIRANT_PAN_CENTER;
      servoMax_[SERVO_ANTENNA_PAN] = DIRANT_PAN_MAX;
      servoMin_[SERVO_ANTENNA_PAN] = DIRANT_PAN_MIN;
      //servoLevel_[SERVO_ANTENNA_TILT] = DIRANT_TILT_LEVEL; 
      //servoCenter_[SERVO_ANTENNA_TILT] = DIRANT_TILT_LEVEL;  // we are going to put center at the level spot so it will = 0 position in ros code
      //servoMax_[SERVO_ANTENNA_TILT] = DIRANT_TILT_MAX;
      //servoMin_[SERVO_ANTENNA_TILT] = DIRANT_TILT_MIN;
      
      servoCenter_[ZOOM_DIGCAM_PAN] = ZOOM_DIGCAM_PAN_CENTER;
      servoMax_[ZOOM_DIGCAM_PAN] = ZOOM_DIGCAM_PAN_MAX;
      servoMin_[ZOOM_DIGCAM_PAN] = ZOOM_DIGCAM_PAN_MIN;
      //servoLevel_[FRONT_DIGCAM_TILT] = FRONT_DIGCAM_TILT_LEVEL; 
      //servoCenter_[FRONT_DIGCAM_TILT] = FRONT_DIGCAM_TILT_LEVEL;  // we are going to put center at the level spot so it will = 0 position in ros code
      //servoMax_[FRONT_DIGCAM_TILT] = FRONT_DIGCAM_TILT_MAX;
      //servoMin_[FRONT_DIGCAM_TILT] = FRONT_DIGCAM_TILT_MIN;
      
      servoCenter_[FRONT_WEBCAM_PAN] = FRONT_WEBCAM_PAN_CENTER;
      servoMax_[FRONT_WEBCAM_PAN] = FRONT_WEBCAM_PAN_MAX;
      servoMin_[FRONT_WEBCAM_PAN] = FRONT_WEBCAM_PAN_MIN;
      servoLevel_[FRONT_WEBCAM_TILT] = FRONT_WEBCAM_TILT_LEVEL; 
      servoCenter_[FRONT_WEBCAM_TILT] = FRONT_WEBCAM_TILT_LEVEL;  // we are going to put center at the level spot so it will = 0 position in ros code
      servoMax_[FRONT_WEBCAM_TILT] = FRONT_WEBCAM_TILT_MAX;
      servoMin_[FRONT_WEBCAM_TILT] = FRONT_WEBCAM_TILT_MIN;
      
      servoCenter_[DIGCAM_PAN] = DIGCAM_PAN_CENTER;
      servoMax_[DIGCAM_PAN] = DIGCAM_PAN_MAX;
      servoMin_[DIGCAM_PAN] = DIGCAM_PAN_MIN;
      //servoLevel_[DIGCAM_TILT] = DIGCAM_TILT_LEVEL; 
      //servoCenter_[DIGCAM_TILT] = DIGCAM_TILT_LEVEL;  // we are going to put center at the level spot so it will = 0 position in ros code
      //servoMax_[DIGCAM_TILT] = DIGCAM_TILT_MAX;
      //servoMin_[DIGCAM_TILT] = DIGCAM_TILT_MIN;
      /*
      servoCenter_[REAR_WEBCAM_PAN] = REAR_WEBCAM_PAN_CENTER;
      servoMax_[REAR_WEBCAM_PAN] = REAR_WEBCAM_PAN_MAX;
      servoMin_[REAR_WEBCAM_PAN] = REAR_WEBCAM_PAN_MIN;
      servoLevel_[REAR_WEBCAM_TILT] = REAR_WEBCAM_TILT_LEVEL; 
      servoCenter_[REAR_WEBCAM_TILT] = REAR_WEBCAM_TILT_LEVEL;  // we are going to put center at the level spot so it will = 0 position in ros code
      servoMax_[REAR_WEBCAM_TILT] = REAR_WEBCAM_TILT_MAX;
      servoMin_[REAR_WEBCAM_TILT] = REAR_WEBCAM_TILT_MIN;
      */
    }
        
      
   private:
    int servoDevNum_;
    String servoName_;
    int servoCenter_[NUM_SERVOS + 1], servoLevel_[NUM_SERVOS + 1], servoMax_[NUM_SERVOS + 1], servoMin_[NUM_SERVOS + 1]; // +1 for PWM channel 8
}

servoDriver(SERVO_DRIVER_DEV_NUM);

      //servoDriver.setName("test");


#endif  // SERVOS_P_H
