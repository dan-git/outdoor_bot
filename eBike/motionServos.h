// servos and also directional antenna
// read RSSI value and rotate antenna to find what direction is max

#ifndef MOTION_SERVOS_H
#define MOTION_SERVOS_H

namespace rcs {
  
class dirAnt {
  public:
    dirAnt()
    {
      direction_ = DIRANT_PAN_CENTER;
      maxDir_ = -1;
      maxAngle_ = -500;
      sweepNumber_ = 0;
      //tilt_ = 127;
      //maxTilt_ = -1;
    }
  
    boolean panSweep()
    {
      int level, maxDir_ = DIRANT_PAN_MIN;
      if (direction_ != DIRANT_PAN_MIN)
      {
        servoDriver.setPosition(SERVO_ANTENNA_PAN, DIRANT_PAN_MIN);
        delay(1000);
      }
      int maxLvl = analogRead(DIRECTIONAL_ANTENNA_PIN);
      
      int increment = (DIRANT_PAN_MAX - DIRANT_PAN_MIN) / NUM_PAN_STEPS;
      DEBUG_SERIAL_PORT.print("directional antenna sweep values = ");
      DEBUG_SERIAL_PORT.print(maxLvl);
      for(int i = DIRANT_PAN_MIN + increment; i <= DIRANT_PAN_MAX; i += increment)
      {
        servoDriver.setPosition(SERVO_ANTENNA_PAN, i);
        delay(1000); //  time to get there
        level = analogRead(DIRECTIONAL_ANTENNA_PIN);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.print(level);
        if (level > maxLvl)
        {
          maxLvl = level;
          maxDir_ = i;
         // DEBUG_SERIAL_PORT.print("new max direction = ");
         // DEBUG_SERIAL_PORT.println(i);
        }
      }
      DEBUG_SERIAL_PORT.println();
      DEBUG_SERIAL_PORT.print("max direction = ");
      DEBUG_SERIAL_PORT.println(maxDir_);
      servoDriver.setPosition(SERVO_ANTENNA_PAN, maxDir_);  // point at max direction
      maxAngle_ = (DIRANT_PAN_CENTER - maxDir_) * DEGREES_PER_PAN_STEP;
      sweepNumber_++;
      DEBUG_SERIAL_PORT.print("maximun angle, level = ");
      DEBUG_SERIAL_PORT.print(maxAngle_);
      DEBUG_SERIAL_PORT.print(", ");
      DEBUG_SERIAL_PORT.println(maxLvl);
      return true;
    }
/*    
    boolean tiltSweep()
    {
      int level, maxTilt_ = DIRANT_TILT_MIN;
      if (tilt_ != DIRANT_TILT_MIN)
      {
        servoDriver.setPosition(SERVO_ANTENNA_TILT, DIRANT_TILT_MIN);
        delay(500);
      }
      int maxLvl = analogRead(DIRECTIONAL_ANTENNA_PIN);
      
      int increment = (DIRANT_TILT_MAX - DIRANT_TILT_MIN) / NUM_TILT_STEPS;
      for(int i = DIRANT_TILT_MIN; i <= DIRANT_TILT_MAX; i += increment)
      {
        servoDriver.setPosition(SERVO_ANTENNA_TILT, i);
        delay(100); //  time to get there
        level = analogRead(DIRECTIONAL_ANTENNA_PIN);
        if (level > maxLvl)
        {
          maxLvl = level;
          maxTilt_ = i;
        }
      }
      servoDriver.setPosition(SERVO_ANTENNA_TILT, maxTilt_);  // point at max tilt
      tilt_ = maxTilt_;
      return true;
    }
 */ 

/* 
    boolean lineUp()
    {
      if (panSweep())
      {
        double panMove = (maxDir_ - DIRANT_PAN_CENTER) * DEGREES_PER_PAN_STEP;
        gyro_stabilizer.setYawTotal(0);
        // start turning
        if (motion_pd.getBrakesState()) motion_pd.releaseRobotBrakes();
        motor_dac[RIGHT].setMotorSpeed(200);
        motor_dac[LEFT].setMotorSpeed(-200);
        
        while (abs(gyro_stabilizer.getYawTotal()) < abs(panMove))
        {
          Serial.print("turning, now up to ");
          Serial.print(gyro_stabilizer.getYawTotal());
          delay(200);
        }
        motor_dac[RIGHT].stop();
        motor_dac[LEFT].stop();
        Serial.print("turned ");
        Serial.println(panMove);
      } 
      return true;      
    }
 */       

    int getLevel()
    {
      return analogRead(DIRECTIONAL_ANTENNA_PIN);
    }
    
    int getMaxAngle()
    {
      return maxAngle_;
    }
    
    int getSweepNumber()
    {
      return sweepNumber_;
    }
/*   
    int getMaxTilt()
    {
      return maxTilt_;
    }
*/    
  private:
    int direction_, maxDir_, maxAngle_, sweepNumber_; //, tilt_, maxTilt_;
}

dirAnt;

// class to implement directional antenna instructions for autonomous navigation
// form is dirAnt(command, pan, tilt);
 
class antCommand : 
  public Instruction {
public:
    antCommand() : 
    Instruction("dirAnt") {
    }

    virtual boolean run(String* args) {
      int antennaCommand;
      int pan;
      //int tilt;
      popInteger(args, &antennaCommand);
      popInteger(args, &pan);
      //popInteger(args, &tilt);
      return (*this)(antennaCommand, pan); //, tilt);
    }

    boolean operator()(int antennaCmd, int panValue) //, int tiltValue) 
    { 
      if (robotPause_) return false;
      int panSet = panValue; 
      //int tiltSet = tiltValue;
      if (antennaCmd == MOVE_ANTENNA)
      {
         if (panSet > DIRANT_PAN_MAX) panSet = DIRANT_PAN_MAX;
         else if (panSet < DIRANT_PAN_MIN) panSet = DIRANT_PAN_MIN;
         //if (tiltSet > DIRANT_TILT_MAX) tiltSet = DIRANT_TILT_MAX;
         //else if (tiltSet < DIRANT_TILT_MIN) tiltSet = DIRANT_TILT_MIN;
         servoDriver.setPosition(SERVO_ANTENNA_PAN, panSet);
         //servoDriver.setPosition(SERVO_ANTENNA_TILT, tiltSet); 
         unsigned int value = 99;
         if (DEBUG)
         {
           servoDriver.getErrors(SERVO_ANTENNA_PAN, &value);
           DEBUG_SERIAL_PORT.print("servo error check result, pan, tilt = ");
           DEBUG_SERIAL_PORT.println(value);
           //DEBUG_SERIAL_PORT.print(", ");
           //servoDriver.getErrors(SERVO_ANTENNA_TILT, &value);
           //DEBUG_SERIAL_PORT.println(value);
         }
         
      }
      else if (antennaCmd == PAN_SWEEP) dirAnt.panSweep();
      //else if (antennaCmd == TILT_SWEEP) dirAnt.tiltSweep();
      //else if (antennaCmd == LINE_UP) dirAnt.lineUp();
      ping.resetTimeout();
      if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print("Antenna command received: ");
          DEBUG_SERIAL_PORT.print(antennaCmd);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(panValue);
          //DEBUG_SERIAL_PORT.print(", ");
          //DEBUG_SERIAL_PORT.println(tiltValue);  
        }
      return true;
    }
  } 
  antCommand;  

class servoCommand : 
  public Instruction {
public:
    servoCommand() : 
    Instruction("autoS") {    // specify the pan servo and the tilt servo number is done automatically (next servo index).  Range is 0 - 255 with 127 being center
    }
    virtual boolean run(String* args) {
      int servoNumber;
      int servoPosition;
      //int tilt;
      popInteger(args, &servoNumber);
      popInteger(args, &servoPosition);
      //popInteger(args, &tilt);
      //DEBUG_SERIAL_PORT.println("servo instruction received");
      return (*this)(servoNumber, servoPosition);
    }

    boolean operator()(int servoNumber, int servoPositionValue) 
    { 
      if (robotPause_) return false;
      // input is in degrees offset from the center, we need to convert to a servo position from 0 - 254, with 127 as the center
      // also, ROS convention is counterclockwise for + Yaw, so due to the servos orientation, we have to  invert the sign
      int servoPositionSet =  (int) ((((double) -servoPositionValue) * PAN_STEPS_PER_DEGREE) + PAN_CENTER);
      //int tiltSet = tiltValue;

      servoDriver.setPosition(servoNumber, servoPositionSet);
      //servoDriver.setPosition(servoNumber + 1, tiltSet); 
      unsigned int value = 99;
      if (DEBUG)
      {
        servoDriver.getErrors(SERVO_ANTENNA_PAN, &value);
        DEBUG_SERIAL_PORT.print("servo error check result, servoPosition = ");
        DEBUG_SERIAL_PORT.print(value);
        DEBUG_SERIAL_PORT.print(", ");
        servoDriver.getErrors(servoNumber, &value);
        DEBUG_SERIAL_PORT.println(value);
      }
      ping.resetTimeout();
      //if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("Servo command received for servo number: ");
        DEBUG_SERIAL_PORT.print(servoNumber);
        DEBUG_SERIAL_PORT.print(": servoPosition = ");
        DEBUG_SERIAL_PORT.println(servoPositionValue);
        //DEBUG_SERIAL_PORT.print(", ");
        //DEBUG_SERIAL_PORT.println(tiltValue);  
      }
      return true;
    }
  } 
  servoCommand;  

}  // ending namespace rcs
#endif  // MOTION_SERVOS_H
