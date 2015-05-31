#ifndef MOTION_PD_H_
#define MOTION_PD_H_

// implements commands to multiple accessory motors driven with Pololu 23v24 driver boards

namespace rcs {

class MotionPD {
private:
    int motorSpeed_[NUM_MOTORS_POLOLU_DRIVER];
    bool pickingUp_, puttingDown_, motorPickerUpperCommand_;
    bool dropbarUp_, dropbarDown_, motorDropbarCommand_;
    bool binshadeUp_, binshadeDown_, motorBinshadeCommand_ ;
    long scooperStartTime_, dropbarStartTime_, binshadeStartTime_;
    bool brakesOn_;
    
public:
    MotionPD()
    {
      motor_pololu_driver[0].exitSafeStart(true);  // enables all pololu driver motors     
      //motor_pololu_driver[0].setBrakes(32, true); // sets max brakes on all pololu driver motors
      // since we used the "all motors" form of setBrakes, we have to set the speed values in a separate statement
      //for (int i=0; i < NUM_MOTORS_POLOLU_DRIVER; i++) motor_pololu_driver[i].setSpeed(0);  // updates the speed variable (does not change the actual motor speed)     

      pinMode(PICKER_UPPER_UP_CONTACT_PIN, INPUT_PULLUP);
      pinMode(PICKER_UPPER_DOWN_CONTACT_PIN, INPUT_PULLUP);
      motorSpeed_[0] = MOTOR0_PREFERRED_SPEED;
      motorSpeed_[1] = MOTOR1_PREFERRED_SPEED;
      motorSpeed_[2] = MOTOR2_PREFERRED_SPEED;
      motorSpeed_[3] = BRAKES_SPEED;
      pickingUp_ = false;
      puttingDown_ = false;
      dropbarUp_ = false;
      dropbarDown_ = false;
      binshadeUp_ = false;
      binshadeDown_ = false;
      motorPickerUpperCommand_ = false;
      motorDropbarCommand_ = false;
      motorBinshadeCommand_ = false;
      brakesOn_ = false;
      scooperStartTime_ = millis();
      dropbarStartTime_ = millis();
      binshadeStartTime_ = millis();
      
      //for (int i = 0; i < NUM_MOTORS_POLOLU_DRIVER; i++) encoderNumber_[i] = getMotorPD_EncoderNumber(i); // get the encoder numbers associated with motor numbers
    }
    
    ~MotionPD()
    {
    }

    // Stop motor
    void stop(int motorNumber) 
    {
      motor_pololu_driver[motorNumber].setRegenBrakes(32); // sets max regen -- does not cause the robot to brake
    }

    void pauseViaReset() // sends to all pololu devices, have to update the speed variable separately
    {
      digitalWrite(MOTOR_RESET_PIN, LOW);
      for (int i=0; i < NUM_MOTORS_POLOLU_DRIVER; i++) motor_pololu_driver[i].setSpeedValue(0);  // updates the speed variable (does not change the actual motor speed)
    }
       
    void resumeViaReset()
    {
      digitalWrite(MOTOR_RESET_PIN, HIGH);
      delay(2); // need at least 1 msec delay after releasing reset
      motor_pololu_driver[0].exitSafeStart(true);  // enables all pololu driver motors 
    } 
    
    
    boolean checkForErrors()
    {
     unsigned int errorValue;
     boolean errorCheck;
     if (!MOTOR_ERROR_PIN) return false;  // no errors
     for (int i=0; i < NUM_MOTORS_POLOLU_DRIVER; i++)
     {
       errorCheck = motor_pololu_driver[i].getErrors(&errorValue);
       if (errorCheck)
       {
         DEBUG_SERIAL_PORT.print("Errors reported from motor ");
         DEBUG_SERIAL_PORT.print(i);
         DEBUG_SERIAL_PORT.print(", code = ");
         DEBUG_SERIAL_PORT.println(errorValue);
       }
       else
       {
         DEBUG_SERIAL_PORT.print("Motor  ");
         DEBUG_SERIAL_PORT.print(i); 
         DEBUG_SERIAL_PORT.println(" reported no errors");
       }            
     }
     return true;  // error found
    }
    
    void applyRobotBrakes()
    {
      if (brakesOn_)
      {
        DEBUG_SERIAL_PORT.println("apply brakes command received, but brakes are already applied");
        return;
      }
      brakesOn_ = true;
      motor_pololu_driver[MOTOR_BRAKE_INDEX].setMotorSpeed(BRAKES_SPEED);
      DEBUG_SERIAL_PORT.println("applying brakes");
      bool allDone = false;
      long startTime = millis();
      while (!allDone) 
      {
        int motorCurrent = analogRead(BRAKES_CURRENT_PIN);
        long timer = millis() - startTime ;
        if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print("timer, motor current = ");
          DEBUG_SERIAL_PORT.print(timer);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(motorCurrent);
        }
        if (timer > BRAKES_APPLY_TIMER || (motorCurrent < BRAKES_CURRENT_THRESHOLD && timer > 50)) allDone = true; // waiting for brakes to apply
      }
      motor_pololu_driver[MOTOR_BRAKE_INDEX].setMotorSpeed(0);
    }
      
    void releaseRobotBrakes()
    {
      if (robotPause_ || (!brakesOn_) ) return;
      brakesOn_ = false;
      DEBUG_SERIAL_PORT.println("releasing brakes");
      motor_pololu_driver[MOTOR_BRAKE_INDEX].setMotorSpeed(-BRAKES_SPEED);
      //delay(BRAKES_RELEASE_TIMER);
      bool allDone = false;
      long startTime = millis();
      while (!allDone) 
      {
        int motorCurrent = analogRead(BRAKES_CURRENT_PIN);
        long timer = millis() - startTime ;
        if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print("timer, motor current = ");
          DEBUG_SERIAL_PORT.print(timer);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(motorCurrent);
        }
        if (timer > BRAKES_RELEASE_TIMER ) allDone = true; // waiting for brakes to apply
      }
      motor_pololu_driver[MOTOR_BRAKE_INDEX].setMotorSpeed(0); 
    }    
    void start(int motorNumber, int motorDirection) 
    {
      if (robotPause_) return; // we stop the motors in the pause section of sensors.h, here we just need to not start them when paused.
      if (motorDirection == 0)  
      {
        stop(motorNumber);
        if (motorNumber == MOTOR_PICKER_UPPER_INDEX && motorPickerUpperCommand_)
        {
           DEBUG_SERIAL_PORT.println("finished auto scooping");
           motorPickerUpperCommand_ = false;
           pickingUp_ = false;
           puttingDown_ = false;
        }    
  
        if (motorNumber == MOTOR_DROP_BAR_INDEX && motorDropbarCommand_)
        {
           DEBUG_SERIAL_PORT.println("finished dropbar move");
           motorDropbarCommand_ = false;
           dropbarUp_ = false;
           dropbarDown_ = false;
        }    
    
        if (motorNumber == MOTOR_BIN_SHADE_INDEX && motorBinshadeCommand_)
        {
           DEBUG_SERIAL_PORT.println("finished binshade move");
           motorBinshadeCommand_ = false;
           binshadeUp_ = false;
           binshadeDown_ = false;
        }      
        return;
      }
      
      if (motorNumber == MOTOR_BRAKE && motorDirection == BRAKES_SPEED + 10)  // this is the code from ROS to go into pause mode (presumably we are all done)
      {
        robotPause_ = true;
      } 
      
      else if (motorNumber == MOTOR_PICKER_UPPER_INDEX)
      {
        if (motorDirection == PICKER_UPPER_UP || motorDirection == PICKER_UPPER_DOWN)
        {
          scooperStartTime_ = millis();
          motorPickerUpperCommand_ = true;
          if (motorDirection == PICKER_UPPER_UP)
          {
             pickingUp_ = true;
             puttingDown_ = false;
          }
          else 
          {
            pickingUp_ = false;
            puttingDown_ = true;
          }
          DEBUG_SERIAL_PORT.println("auto moving scooper");
        }
        else
        {
           if (motorPickerUpperCommand_) DEBUG_SERIAL_PORT.println("finished auto moving scooper");
           motorPickerUpperCommand_ = false;
        }
      }
      else if (motorNumber == MOTOR_DROP_BAR_INDEX)
      {
        if (motorDirection == DROP_BAR_UP || motorDirection == DROP_BAR_DOWN)
        {
          dropbarStartTime_ = millis();
          motorDropbarCommand_ = true;
          if (motorDirection == DROP_BAR_UP)
          {
             dropbarUp_ = true;
             dropbarDown_ = false;
          }
          else 
          {
            dropbarUp_ = false;
            dropbarDown_ = true;
          }
          DEBUG_SERIAL_PORT.println("auto moving dropbar");
        }
        else
        {
           if (motorDropbarCommand_) DEBUG_SERIAL_PORT.println("finished auto moving dropbar");
           motorDropbarCommand_ = false;
        }
      }
        
      else if (motorNumber == MOTOR_BIN_SHADE_INDEX)
      {
        if (motorDirection == BIN_SHADE_UP || motorDirection == BIN_SHADE_DOWN)
        {
          binshadeStartTime_ = millis();
          motorBinshadeCommand_ = true;
          if (motorDirection == DROP_BAR_UP)
          {
             binshadeUp_ = true;
             binshadeDown_ = false;
          }
          else 
          {
            binshadeUp_ = false;
            binshadeDown_ = true;
          }
          DEBUG_SERIAL_PORT.println("auto moving binshade");
        }
        else
        {
           if (motorBinshadeCommand_) DEBUG_SERIAL_PORT.println("finished auto moving binshade");
           motorBinshadeCommand_ = false;
        }
      }
      
      if (motorDirection > 0) motor_pololu_driver[motorNumber].setMotorSpeed(motorSpeed_[motorNumber]);
      else motor_pololu_driver[motorNumber].setMotorSpeed(-motorSpeed_[motorNumber]);
      
      
      //checkForErrors();
  
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("motor and distance commanded: ");
        DEBUG_SERIAL_PORT.print(motorNumber);
        DEBUG_SERIAL_PORT.print(", ");
        if (motorDirection > 0) DEBUG_SERIAL_PORT.println(" forward");
        else DEBUG_SERIAL_PORT.println(" reverse");     
      }
    }
    
    bool getPickerUpperCommand() { return motorPickerUpperCommand_; }
    void setPickerUpperCommand(boolean value) { motorPickerUpperCommand_  = value; }
    bool getPickingUp() { return pickingUp_; }
    bool getPuttingDown() { return puttingDown_; }
    long getScooperStartTime() { return scooperStartTime_; }
    void setScooperStartTime(long value) { scooperStartTime_ = value; }
    bool getBrakesState() { return brakesOn_; }
    
    bool getDropbarCommand() { return motorDropbarCommand_; }
    void setDropbarCommand(boolean value) { motorDropbarCommand_  = value; }
    bool getDropbarUp() { return dropbarUp_; }
    bool getDropbarDown() { return dropbarDown_; }
    long getDropbarStartTime() { return dropbarStartTime_; }
    void setDropbarStartTime(long value) { dropbarStartTime_ = value; }
    
    bool getBinshadeCommand() { return motorBinshadeCommand_; }
    void setBinshadeCommand(boolean value) { motorBinshadeCommand_  = value; }
    bool getBinshadeUp() { return binshadeUp_; }
    bool getBinshadeDown() { return binshadeDown_; }
    long getBinshadeStartTime() { return binshadeStartTime_; }
    void setBinshadeStartTime(long value) { binshadeStartTime_ = value; }
} 
motion_pd;

  // class to implement direct move instructions for polol motors
  // form is directPP(motorNumber, distance);  note that if distance = 0, motor will stop
  // Also, distance is currently used just to determine the direction of the motor command
class directCommand_motionPD : 
  public Instruction {
public:
    directCommand_motionPD() : 
    Instruction("directPP") {
    }

    virtual boolean run(String* args) {
      int motorNumber;
      int motorDirection;
      popInteger(args, &motorNumber);
      popInteger(args, &motorDirection);
      return (*this)(motorNumber, motorDirection);
    }

    boolean operator()(int motorNumber, int motorDirection) 
    { 
      motion_pd.setPickerUpperCommand(false);
      motion_pd.setDropbarCommand(false);
      motion_pd.setBinshadeCommand(false);
      motion_pd.start(motorNumber, motorDirection);
      motion_pd.setPickerUpperCommand(false);
      motion_pd.setDropbarCommand(false);
      motion_pd.setBinshadeCommand(false);
      ping.resetTimeout();
      #ifdef DEBUG_SERIAL_PORT
      if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print("direct pololu motor command received: ");
          DEBUG_SERIAL_PORT.print("motorNumber, motorDirection = ");
          DEBUG_SERIAL_PORT.print(motorNumber);
          DEBUG_SERIAL_PORT.print(", ");         
          DEBUG_SERIAL_PORT.println(motorDirection);  
        }
      #endif
      return true;
    }
  } 
  direct_command_motionPD;

  // class to implement move instructions for autonomous navigation
  // form is autoPP(motorNumber, distance);  note that if distance = 0, motor will stop
  // Also, distance is currently used just to determine the direction of the motor command
class autonomousCommand_motionPD : 
  public Instruction {
public:
    autonomousCommand_motionPD() : 
    Instruction("autoPP") {
    }

    virtual boolean run(String* args) {
      int motorNumber;
      int motorDirection;
      popInteger(args, &motorNumber);
      popInteger(args, &motorDirection);
      return (*this)(motorNumber, motorDirection);
    }

    boolean operator()(int motorNumber, int motorDirection) 
    { 
      motion_pd.start(motorNumber, motorDirection);
      ping.resetTimeout();
      #ifdef DEBUG_SERIAL_PORT
      if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print("Autonomous pololu motor command received: ");
          DEBUG_SERIAL_PORT.print("motorNumber, motorDirection = ");
          DEBUG_SERIAL_PORT.print(motorNumber);
          DEBUG_SERIAL_PORT.print(", ");         
          DEBUG_SERIAL_PORT.println(motorDirection);  
        }
      #endif
      return true;
    }
  } 
  autonomous_command_motionPD;
  
class MotionPDLoop : 
  public ModuleLoop {
  
public:
    virtual void loop()
      {           
         if (motion_pd.getPickerUpperCommand())
         {
           if (robotPause_)
           {
             motion_pd.setScooperStartTime(millis());  // if we are paused in the middle of a scoop, we want to finish it and not timeout when unpaused
             motion_pd.setDropbarStartTime(millis());  // if we are paused in the middle of a scoop, we want to finish it and not timeout when unpaused
             motion_pd.setBinshadeStartTime(millis());  // if we are paused in the middle of a scoop, we want to finish it and not timeout when unpaused
           }
             
           long scooperTimeout = millis() - motion_pd.getScooperStartTime();
           long scooperEncoder = encoder_spi[getEncoderNumber(PICKER_UPPER_ENCODER_SELECT_PIN)].readEncoder();
           if (scooperTimeout > SCOOPER_TIMEOUT - 1000)
           {
              DEBUG_SERIAL_PORT.println("scooper close to timing out: time, encoder = ");
              DEBUG_SERIAL_PORT.print(scooperTimeout);
              DEBUG_SERIAL_PORT.print(", ");
              DEBUG_SERIAL_PORT.println(scooperEncoder);
           }
           if (motion_pd.getPickingUp()) 
           {
             if (!digitalRead(PICKER_UPPER_UP_CONTACT_PIN) || scooperTimeout > SCOOPER_TIMEOUT)
             {
                motion_pd.start(MOTOR_PICKER_UPPER_INDEX, 0);
                DEBUG_SERIAL_PORT.print("finished scooping up, time, encoder = ");
                DEBUG_SERIAL_PORT.print(scooperTimeout);
                DEBUG_SERIAL_PORT.print(", ");
                DEBUG_SERIAL_PORT.println(scooperEncoder);
             }
           }
           else if (motion_pd.getPuttingDown())
           {
             if (!digitalRead(PICKER_UPPER_DOWN_CONTACT_PIN) || scooperTimeout > SCOOPER_TIMEOUT)
             {
               motion_pd.start(MOTOR_PICKER_UPPER_INDEX, 0);
               DEBUG_SERIAL_PORT.print("finished putting scooper down, time, encoder = ");
               DEBUG_SERIAL_PORT.print(scooperTimeout);
               DEBUG_SERIAL_PORT.print(", ");
               DEBUG_SERIAL_PORT.println(scooperEncoder);
             }
           }
           if (DEBUG)
           {
             if (!digitalRead(PICKER_UPPER_UP_CONTACT_PIN)) DEBUG_SERIAL_PORT.println("top contact closed");
             if (!digitalRead(PICKER_UPPER_DOWN_CONTACT_PIN)) DEBUG_SERIAL_PORT.println("lower contact closed");
           } 
         }
         
         if (motion_pd.getDropbarCommand())
         {
           if (robotPause_)
           {
             motion_pd.setDropbarStartTime(millis());  // if we are paused in the middle of a scoop, we want to finish it and not timeout when unpaused
           }
             
           long dropbarTimeout = millis() - motion_pd.getDropbarStartTime();
           long dropbarEncoder = encoder_spi[getEncoderNumber(DROP_BAR_ENCODER_SELECT_PIN)].readEncoder();
           if (dropbarTimeout > DROP_BAR_TIMEOUT - 1000)
           {
              DEBUG_SERIAL_PORT.print("dropbar close to timing out: time, encoder = ");
              DEBUG_SERIAL_PORT.print(dropbarTimeout);
              DEBUG_SERIAL_PORT.print(" ms, ");
              DEBUG_SERIAL_PORT.println(dropbarEncoder);
           }
           
           if (motion_pd.getDropbarUp()) 
           {
               if (dropbarEncoder < DROP_BAR_UP_POSITION)
               {
                motion_pd.start(MOTOR_DROP_BAR_INDEX, 0);
                DEBUG_SERIAL_PORT.print("finished pulling dropbar up: time, encoder = ");
                DEBUG_SERIAL_PORT.print(dropbarTimeout);
                DEBUG_SERIAL_PORT.print(",");
                DEBUG_SERIAL_PORT.println(dropbarEncoder);
               }
            }
            else if (motion_pd.getDropbarDown()) 
            {
              if (dropbarEncoder > DROP_BAR_DOWN_POSITION)
              {
                motion_pd.start(MOTOR_DROP_BAR_INDEX, 0);
                DEBUG_SERIAL_PORT.print("finshed putting dropbar down: time, encoder = ");
                DEBUG_SERIAL_PORT.print(dropbarTimeout);
                DEBUG_SERIAL_PORT.print(",");
                DEBUG_SERIAL_PORT.println(dropbarEncoder);
               }   
            }         
                 
         }
        if (motion_pd.getBinshadeCommand())
         {
           if (robotPause_)
           {
             motion_pd.setBinshadeStartTime(millis());  // if we are paused in the middle of a scoop, we want to finish it and not timeout when unpaused
           }
             
           long binshadeTimeout = millis() - motion_pd.getBinshadeStartTime();
           long binshadeEncoder = encoder_spi[getEncoderNumber(BIN_SHADE_ENCODER_SELECT_PIN)].readEncoder();
           if (binshadeTimeout > BIN_SHADE_TIMEOUT - 1000)
           {
              DEBUG_SERIAL_PORT.println("binshade close to timing out: time, encoder = ");
              DEBUG_SERIAL_PORT.print(motion_pd.getBinshadeStartTime());
              DEBUG_SERIAL_PORT.print(", ");
              DEBUG_SERIAL_PORT.println(binshadeEncoder);
           }
           
           if (motion_pd.getBinshadeUp()) 
           {
               if (binshadeEncoder < BIN_SHADE_UP_POSITION)
               {
                motion_pd.start(MOTOR_DROP_BAR_INDEX, 0);
                DEBUG_SERIAL_PORT.print("finshed pulling binshade up: time, encoder = ");
                DEBUG_SERIAL_PORT.print(binshadeTimeout);
                DEBUG_SERIAL_PORT.print(",");
                DEBUG_SERIAL_PORT.println(binshadeEncoder);
               }
            }
            else if (motion_pd.getBinshadeDown()) 
            {
              if (binshadeEncoder > BIN_SHADE_DOWN_POSITION)
              {
                motion_pd.start(MOTOR_BIN_SHADE_INDEX, 0);
                DEBUG_SERIAL_PORT.print("finshed putting binshade down: time, encoder = ");
                DEBUG_SERIAL_PORT.print(binshadeTimeout);
                DEBUG_SERIAL_PORT.print(",");
                DEBUG_SERIAL_PORT.println(binshadeEncoder);
               }   
            }         
                 
         }                   
      }   
  } 
  motionPD_loop;

 

}  // namespace rcs
#endif  MOTION_PD_H_

