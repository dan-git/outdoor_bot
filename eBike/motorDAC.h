#ifndef MOTOR_DAC_H_
#define MOTOR_DAC_H_

// Represents a brushless ebike motor with speed controlled by a dac 

class motor_dac {
  public:
    motor_dac(int devNum) {    
    devNum_ = devNum; // identifies which motor this is and which DAC it is associated with 
    speed_ = DAC_MIN_VALUE;
    previousSpeed_ = speed_;
    motorDirection_ = 0;
    } 
    
    // This command sets the motor speed 
    // speed should be a number from DAC_MIN_VALUE to DAC_MAX_VALUE
    // sign is defined as positive if it will move the robot forward
    // The routine takes into account that the right side motors are facing backwards
    void setMotorSpeed(int motorSpeed) 
    {     
      //if (robotPause_ && newPauseState_) return; // we need to control the motors until we stop them in sensors.h, then don't respond to move commands.
      int newSpeed;
      if (abs(motorSpeed) <= DAC_MIN_VALUE || abs(motorSpeed) > DAC_MAX_VALUE) 
      {
        if (DEBUG && (abs(motorSpeed) != DAC_MIN_VALUE))
        {
          DEBUG_SERIAL_PORT.print("error, in ");
          if (devNum_ == RIGHT) DEBUG_SERIAL_PORT.print("right");
          else DEBUG_SERIAL_PORT.print("left");
          DEBUG_SERIAL_PORT.print(" motor_dac setMotorSpeed, out of bounds speed value = ");
          DEBUG_SERIAL_PORT.println(motorSpeed);
        }
        newSpeed = DAC_MIN_VALUE;  // if we get a weird speed, just stop, this number has to have a positive value
        previousSpeed_ = speed_;
        speed_ = DAC_MIN_VALUE * rcs::sign(speed_);  // we want to preserve the direction, for the encoder to stay correct.       
      }
      else
      {      
        newSpeed = motorSpeed;
        setMotorDirection(&newSpeed);  // sets direction pins and, if speed is negative, makes it positive
        previousSpeed_ = speed_;  // setMotorDirection uses previousSpeed_, so it has to be set after that call
        speed_ = motorSpeed;
      }
            
      
     if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("motorSpeed sent to ");
        if (devNum_ == RIGHT) DEBUG_SERIAL_PORT.print("right dac = ");
        else DEBUG_SERIAL_PORT.print("left dac = ");
        DEBUG_SERIAL_PORT.print(newSpeed);
        DEBUG_SERIAL_PORT.print(" in ");
        if (speed_ > 0) DEBUG_SERIAL_PORT.print("forward");
        else if (speed_ < 0) DEBUG_SERIAL_PORT.print("backward");
        else ("no direction");
        DEBUG_SERIAL_PORT.println(" direction");
        DEBUG_SERIAL_PORT.print("motorDirection_ = ");
        DEBUG_SERIAL_PORT.println(motorDirection_);
      }
      int sendSpeed = newSpeed;
      if (devNum_ == RIGHT) sendSpeed += RIGHT_MOTOR_SPEED_BIAS * rcs::sign(newSpeed);      
      dac[devNum_].setVoltage(sendSpeed);  // unsigned value
    }
    
    void stop()
    {
      setMotorSpeed(DAC_MIN_VALUE);
      //motorDirection_ = FORWARD;
      //if (devNum_ == RIGHT) digitalWrite(REVERSE_RIGHT_PIN, LOW);
      //else digitalWrite(REVERSE_LEFT_PIN, HIGH);
      //digitalWrite(REVERSE_ENCODER_LEFT_PIN, HIGH);
      //digitalWrite(REVERSE_ENCODER_RIGHT_PIN, HIGH);
    }
    
    void setMotorDirection(int *motorDirection) 
    {
      // setting reverse pin low drives in reverse, but right side motors go backwards, so set
      // the right side high and the left side low for the robot to go in reverse.
      boolean changedDirection = false, goingForward = true;
      int dirChange;
      //if (devNum_ == RIGHT) dirChange = rcs::sign(motorDirection_) * ((*motorDirection) - (DAC_MIN_VALUE + RIGHT_MOTOR_SPEED_BIAS));
      //else 
      dirChange = rcs::sign(motorDirection_) * ((*motorDirection) - DAC_MIN_VALUE);
      if ( dirChange < 0) changedDirection = true;  // use this to decide if we need to stop before sending move command
      motorDirection_ = *motorDirection;
      if (motorDirection == 0) return;
      if (motorDirection_ < 0)
      {
        if (devNum_ == RIGHT)
        {
          digitalWrite(REVERSE_RIGHT_PIN, HIGH); 
          digitalWrite(REVERSE_ENCODER_RIGHT_PIN, LOW); 
        }
        else
        {
          digitalWrite(REVERSE_LEFT_PIN, LOW);
          digitalWrite(REVERSE_ENCODER_LEFT_PIN, LOW);
        }
        *motorDirection = -*motorDirection;
        goingForward = false;
      }
      else 
      {
        if (devNum_ == RIGHT) 
        {
          digitalWrite(REVERSE_RIGHT_PIN, LOW);
          digitalWrite(REVERSE_ENCODER_RIGHT_PIN, HIGH);
        }
        else
        {
          digitalWrite(REVERSE_LEFT_PIN, HIGH);
          digitalWrite(REVERSE_ENCODER_LEFT_PIN, HIGH);
        }
      } 
      if (changedDirection)
      {
        if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print("Changed direction on ");
          if (devNum_ == RIGHT) DEBUG_SERIAL_PORT.print("right, now going ");
          else DEBUG_SERIAL_PORT.print("left, now going ");
          if (goingForward) DEBUG_SERIAL_PORT.println("forward");
          else DEBUG_SERIAL_PORT.println("backward");
          
        }
        
        // don't need the lines below if we goose the motors
        //if (abs(previousSpeed_) > DAC_MIN_VALUE) dac[devNum_].setVoltage(DAC_MIN_VALUE);  // stop before reversing direction
        //delay(50); // changed directions, so have to give some time for the pin to switch and be seen by controller
      }
    }      

    int getSpeed()
    {
      return speed_;
    }
    
    void setSpeed_ValueOnly(int value)
    {
      previousSpeed_ = speed_;
      speed_ = value;
    }
    
    int getPreviousSpeed()
    {
      return previousSpeed_;
    }  
    
    int getMotorDirection()
    {
      return motorDirection_;
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
    int motorDirection_;
}

motor_dac[NUM_DACS] =  {RIGHT, LEFT};
int previousDACmotorSpeed_[NUM_DACS];

#endif  // MOTOR_DAC_H_

