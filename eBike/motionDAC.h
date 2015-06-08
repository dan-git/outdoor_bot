#ifndef MOTION_DAC_H_
#define MOTION_DAC_H_

namespace rcs {
  
class RobotBase {
public:
    RobotBase()
    {
      leftPreviousTicks_ = 0;
      rightPreviousTicks_ = 0;
      kpLinearV_ = KP_LINEAR;
      kiLinearV_ = KI_LINEAR;
      kdLinearV_ = KD_LINEAR;
      speedVelocityRatio_ = MOTOR_DAC_SPEED_VELOCITY_RATIO;
      //pinMode(REVERSE_REAR_LEFT_PIN, OUTPUT);
      //pinMode(REVERSE_FRONT_LEFT_PIN, OUTPUT);
      pinMode(REVERSE_RIGHT_PIN, OUTPUT);
      pinMode(REVERSE_LEFT_PIN, OUTPUT);
      
      //digitalWrite(REVERSE_REAR_LEFT_PIN, HIGH);  // start with going forward
      //digitalWrite(REVERSE_FRONT_LEFT_PIN, HIGH);
      digitalWrite(REVERSE_RIGHT_PIN, LOW);  // these wheels are mounted in the opposite direction, so for the
                                             // robot to move forward, these have to turn in reverse, so the pin is set LOW
      digitalWrite(REVERSE_LEFT_PIN, HIGH);
      
     
              
      pidLinear_ = new pidControl(KP_LINEAR, KI_LINEAR, KD_LINEAR, 10.0f, MAX_LINEAR_ROBOT_VELOCITY);  // limit change to 10 mm/sec per cycle     
    }
    
    ~RobotBase()
    {
      delete pidLinear_;
    }

    // Stop the robot and re-initializes parameters
    void stop() {
      for (int i=0; i < NUM_DACS; i++) motor_dac[i].setMotorSpeed(DAC_MIN_VALUE);   // sets all dac motor speeds to 0 
      linearCommandedVelocity_ = 0;
      angCommandedVelocity_ = 0;
      linearCurrentVelocity_ = 0;
      angCurrentVelocity_ = 0;
      angOnly_ = false;
      rightSpeed_ = DAC_MIN_VALUE; 
      leftSpeed_ = DAC_MIN_VALUE;
      previousRightSpeed_ = DAC_MIN_VALUE;
      previousLeftSpeed_ = DAC_MIN_VALUE;
      previousLeftDirection_ = 1; // only used in gooseMotors()
      previousRightDirection_ = 1;
      totalDistance_ = 0;
      totalLinearVelocityCorrection_ = 0.0;
      for (int i=0; i < MOTION_BUFFER_SIZE; i++)
      {
        bufferDistance_[i] = 0.0;
        motionBufferTime_[i] = 0;
      }
      autonomousCommandMode_ = false;
      autoMoveMode_ = false;
      autoMoveStopped_ = false;
      stoppedCounter_ = 0;
      newCommandMode_ = true;
      messageLimit_ = false;
      motionPreviousTime_ = millis();      
      linearPreviousVelocity_ = 0.0;
      linearPreviousTargetVelocity_ = 0.0;
      angPreviousVelocity_ = 0.0;
      angPreviousTargetVelocity_ = 0.0;
    }

  void gooseMotors() // ebike motor controllers take a while to set the direction, so we need to give time for that.
                     //  Also, goose them to be sure that they move, as sometimes they won't respond to small speed inputs
  {
    /*
    if  ( ((rightSpeed_ * previousRightDirection_ < 0 ) && abs(rightSpeed_) > DAC_LOWER_VALUE) 
       ||  ((leftSpeed_ * previousLeftDirection_ < 0 ) && abs(leftSpeed_) > DAC_LOWER_VALUE) )
    {
      int direction = rcs::sign(rightSpeed_); // we need extra variable because setting direction can change the sign of the paramter sent
      motor_dac[RIGHT].setMotorDirection(&direction);
      direction = rcs::sign(leftSpeed_);
      motor_dac[LEFT].setMotorDirection(&direction);
      previousRightDirection_ = rcs::sign(rightSpeed_);  // the ebike motor controller remembers the last non-zero direction, so we have to too
      previousLeftDirection_ = rcs::sign(leftSpeed_); 
      if (motion_pd.getBrakesState() || motion_pd.getApplyingBrakes()) motion_pd.releaseRobotBrakes();
      else delay(100);

      motor_dac[RIGHT].setMotorSpeed(GOOSE_SPEED * rcs::sign(rightSpeed_)); 
      motor_dac[LEFT].setMotorSpeed(GOOSE_SPEED * rcs::sign(leftSpeed_)); 
      delay(50);
    }   
  
    if (motion_pd.getBrakesState() || motion_pd.getApplyingBrakes()) motion_pd.releaseRobotBrakes();
    if (abs(rightSpeed_) > DAC_LOWER_VALUE) motor_dac[RIGHT].setMotorSpeed(GOOSE_SPEED * rcs::sign(rightSpeed_)); // send eike controllers a large speed to get going
    if (abs(leftSpeed_) > DAC_LOWER_VALUE) motor_dac[LEFT].setMotorSpeed(GOOSE_SPEED * rcs::sign(leftSpeed_)); // and then reduce it to the commanded speed
    delay(100);
    DEBUG_SERIAL_PORT.println("goosing");
    */
    if  ( ((rightSpeed_ * previousRightDirection_ < 0 ) && abs(rightSpeed_) > DAC_LOWER_VALUE) 
       ||  ((leftSpeed_ * previousLeftDirection_ < 0 ) && abs(leftSpeed_) > DAC_LOWER_VALUE) )
    {
      int direction = rcs::sign(rightSpeed_); // we need extra variable because setting direction can change the sign of the paramter sent
      motor_dac[RIGHT].setMotorDirection(&direction);
      direction = rcs::sign(leftSpeed_);
      motor_dac[LEFT].setMotorDirection(&direction);
      previousRightDirection_ = rcs::sign(rightSpeed_);  // the ebike motor controller remembers the last non-zero direction, so we have to too
      previousLeftDirection_ = rcs::sign(leftSpeed_); 
      if (motion_pd.getBrakesState() || motion_pd.getApplyingBrakes()) motion_pd.releaseRobotBrakes();
      delay(100);

      motor_dac[RIGHT].setMotorSpeed(GOOSE_SPEED * rcs::sign(rightSpeed_)); 
      motor_dac[LEFT].setMotorSpeed(GOOSE_SPEED * rcs::sign(leftSpeed_)); 
      delay(10);
    }   
  
    //if (motion_pd.getBrakesState() || motion_pd.getApplyingBrakes()) motion_pd.releaseRobotBrakes();
    //if (abs(rightSpeed_) > DAC_LOWER_VALUE) motor_dac[RIGHT].setMotorSpeed(GOOSE_SPEED * rcs::sign(rightSpeed_)); // send eike controllers a large speed to get going
    //if (abs(leftSpeed_) > DAC_LOWER_VALUE) motor_dac[LEFT].setMotorSpeed(GOOSE_SPEED * rcs::sign(leftSpeed_)); // and then reduce it to the commanded speed
    //delay(100);
    //DEBUG_SERIAL_PORT.println("goosing");    
    
  }   
    void startAutoMove(float distance, float angle, float suggestedSpeed)  // distance in mm, angle in degrees, speed in mm/sec or deg/sec
    {
      rcs::ping.resetTimeout();
      if (robotPause_) return; // we stop the motors in the pause section of sensors.h, here we just need to not start them when paused. 
      if (fabs(distance) + fabs(angle) + fabs(suggestedSpeed) < 1.) // command from ROS to set the brakes
      {
        stop();
        motion_pd.applyRobotBrakes();
        DEBUG_SERIAL_PORT.println("user commanded autoMove brakes on");
        return;
      }
      if (fabs(suggestedSpeed) < 1.)
      {
         stop();
         DEBUG_SERIAL_PORT.println("user commanded stop");
         return;
      }
      autoMoveStopped_ = false;
      stoppedCounter_ = 0;
      leftPreviousTicks_ = encoder_spi[getEncoderNumber(LEFT_ENCODER_SELECT_PIN)].readEncoder(); 
      rightPreviousTicks_ = encoder_spi[getEncoderNumber(RIGHT_ENCODER_SELECT_PIN)].readEncoder();
      autoInitialYaw_ = gyro_stabilizer.getYawTotal(); 
  
      if (fabs(angle) < 1.)  // this is a linear move 
      {
        angCommandedVelocity_ = 0;
        linearCommandedVelocity_ = suggestedSpeed;
        autoLinearMoving_ = true;
      } 
      else // this is an angular move
      {
        angCommandedVelocity_ = suggestedSpeed;
        linearCommandedVelocity_ = 0;
        autoLinearMoving_ = false;
      }
      
      transformVelocityToMotorSpeed(linearCommandedVelocity_, angCommandedVelocity_, &leftSpeed_, &rightSpeed_);
      
      motion_pd.releaseRobotBrakes();
      gooseMotors();
      
      motor_dac[RIGHT].setMotorSpeed(rightSpeed_);
      motor_dac[LEFT].setMotorSpeed(leftSpeed_); 
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("linear and angular velocities commanded: ");
        DEBUG_SERIAL_PORT.print(linearCommandedVelocity_);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(angCommandedVelocity_); 
        DEBUG_SERIAL_PORT.print("speeds, L,R: ");
        DEBUG_SERIAL_PORT.print(leftSpeed_);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(rightSpeed_);     
      }
      
      autoDistance_ = distance;
      autoInitialLeftTicks_ = leftPreviousTicks_;
      autoInitialRightTicks_ = rightPreviousTicks_;
      autoDegrees_ = angle;      
      angPreviousTargetVelocity_ = angCommandedVelocity_;
      linearPreviousTargetVelocity_ = linearCommandedVelocity_;
      motionPreviousTime_ = millis();   
    }
    
       
    /*void startMove(float linearVelocity, float angVelocity) // commanded velocities in mm/sec and deg/sec
    {
      rcs::ping.resetTimeout();
      if (robotPause_) return; // we stop the motors in the pause section of sensors.h, here we just need to not start them when paused.

      linearCommandedVelocity_ = linearVelocity;
      angCommandedVelocity_ = angVelocity;
      
      leftPreviousTicks_ = encoder_spi[getEncoderNumber(LEFT_ENCODER_SELECT_PIN)].readEncoder(); 
      rightPreviousTicks_ = encoder_spi[getEncoderNumber(RIGHT_ENCODER_SELECT_PIN)].readEncoder(); 


//      if (!newCommandMode_) return; // this is an updated move command, not a new one, so we don't want to reset motor speeds to initial values
      
      transformVelocityToMotorSpeed(linearCommandedVelocity_, angCommandedVelocity_, &leftSpeed_, &rightSpeed_);

      if (motion_pd.getBrakesState()) motion_pd.releaseRobotBrakes();

      gooseMotors();
      

      motor_dac[RIGHT].setMotorSpeed(rightSpeed_);
      motor_dac[LEFT].setMotorSpeed(leftSpeed_);       
      
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("linear and angular velocities commanded: ");
        DEBUG_SERIAL_PORT.print(linearCommandedVelocity_);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(angCommandedVelocity_); 
        DEBUG_SERIAL_PORT.print("speeds, L,R: ");
        DEBUG_SERIAL_PORT.print(leftSpeed_);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(rightSpeed_);     
      }
      
      
      angPreviousTargetVelocity_ = angCommandedVelocity_;
      linearPreviousTargetVelocity_ = linearCommandedVelocity_;
      motionPreviousTime_ = millis();
    }
    */
    void startMove(float linearVelocity, float angVelocity) // commanded velocities in mm/sec and deg/sec  
    {
      rcs::ping.resetTimeout();
      if (robotPause_) return; // we stop the motors in the pause section of sensors.h, here we just need to not start them when paused. 
      if (fabs(linearVelocity) + fabs(angVelocity) < 1)
      {
        stop();
        return;
      }
  
      if (fabs(angVelocity) < 1.)  // this is just a linear move 
      {
        angCommandedVelocity_ = 0;
        linearCommandedVelocity_ = linearVelocity;
      } 
      else if (fabs(linearVelocity) < 1.) // this is just an angular move
      {
        angCommandedVelocity_ = angVelocity;
        linearCommandedVelocity_ = 0;
      }
      else
      {
        angCommandedVelocity_ = angVelocity;
        linearCommandedVelocity_ = linearVelocity;
      }
      
      if (autonomousCommandMode_) return; // this is just an updated command, not a new move
      
      leftPreviousTicks_ = encoder_spi[getEncoderNumber(LEFT_ENCODER_SELECT_PIN)].readEncoder(); 
      rightPreviousTicks_ = encoder_spi[getEncoderNumber(RIGHT_ENCODER_SELECT_PIN)].readEncoder();
      autoInitialYaw_ = gyro_stabilizer.getYawTotal(); 
        
      transformVelocityToMotorSpeed(linearCommandedVelocity_, angCommandedVelocity_, &leftSpeed_, &rightSpeed_);
      
      //motion_pd.releaseRobotBrakes();  goose does this
      gooseMotors();
      
      motor_dac[RIGHT].setMotorSpeed(rightSpeed_);
      motor_dac[LEFT].setMotorSpeed(leftSpeed_); 
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("linear and angular velocities commanded: ");
        DEBUG_SERIAL_PORT.print(linearCommandedVelocity_);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(angCommandedVelocity_); 
        DEBUG_SERIAL_PORT.print("speeds, L,R: ");
        DEBUG_SERIAL_PORT.print(leftSpeed_);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(rightSpeed_);     
      }
         
      angPreviousTargetVelocity_ = angCommandedVelocity_;
      linearPreviousTargetVelocity_ = linearCommandedVelocity_;
      motionPreviousTime_ = millis(); 
  }

    void checkSpeed()
    {      
      if (robotPause_)
      {
        for (int i=0; i < NUM_DACS; i++) motor_dac[i].setMotorSpeed(DAC_MIN_VALUE);   // sets all dac motor speeds to 0
        return;
      }
      
      // no feedback case:  NOTE: this will make autoMoves go forever
      /*
      transformVelocityToMotorSpeed(linearCommandedVelocity_, angCommandedVelocity_, &leftSpeed_, &rightSpeed_);
      
      if (motion_pd.getBrakesState()) motion_pd.releaseRobotBrakes();

      motor_dac[RIGHT].setMotorSpeed(rightSpeed_);
      motor_dac[LEFT].setMotorSpeed(leftSpeed_);
      motionPreviousTime_ = millis();
      return;
      */
      
      double deltaDistance, angTargetV, linearTargetVelocity;
      long leftTotalTicks = 0;
      long rightTotalTicks = 0;
      long leftDeltaTicks = 0;
      long rightDeltaTicks = 0;
      
      unsigned long currentTime = millis();
      unsigned long deltaT = currentTime - motionPreviousTime_;
      motionPreviousTime_ = currentTime;
      
      // gyro section.
      angTargetV = angPreviousTargetVelocity_;       
      gyro_stabilizer.stabilize(&angTargetV, &angCurrentVelocity_, angCommandedVelocity_, deltaT, getSpeedVelocityRatio(), startUpMode()); 
      angPreviousTargetVelocity_ = angTargetV;
      //end of gyro section
       
      leftTotalTicks = encoder_spi[getEncoderNumber(LEFT_ENCODER_SELECT_PIN)].readEncoder(); 
      rightTotalTicks = encoder_spi[getEncoderNumber(RIGHT_ENCODER_SELECT_PIN)].readEncoder(); 
  
      leftDeltaTicks = leftTotalTicks - leftPreviousTicks_;
      rightDeltaTicks = rightTotalTicks - rightPreviousTicks_;
         
      /*
      if (DEBUG && (!TIMEOUT))
      {   
        // for testing, stop after going 1 meter
        if ((double)((abs(leftTotalTicks) + abs(rightTotalTicks)) * MM_PER_TICK_EBIKE) > 10000)  // adding sides, so we go half of 2000 mm = 1 meter
        {
          stop();
          #ifdef DEBUG_SERIAL_PORT
            DEBUG_SERIAL_PORT.print("total distance in mm = ");
            DEBUG_SERIAL_PORT.println( ( (double) ( (abs(leftTotalTicks) + abs(rightTotalTicks)) * MM_PER_TICK_EBIKE) ) / 2);
          #endif
          return;
        }
       }
      */
        // note that vels are in mm/sec, angV in degs/sec 
        deltaDistance = ((double) (leftDeltaTicks + rightDeltaTicks)) * ((double) MM_PER_TICK_EBIKE) / 2.0;    // distance in mm
        //double deltaWheelMovement = ((double) (abs(leftDeltaTicks) + abs(rightDeltaTicks))) * ((double) MM_PER_TICK_EBIKE) / 2.0; 
        totalDistance_ += deltaDistance;
        leftPreviousTicks_ = leftTotalTicks;
        rightPreviousTicks_ = rightTotalTicks;
 
        double currentBufferDistance = 0;
        unsigned long currentBufferTime = 0;
        if (MOTION_BUFFER_SIZE > 0)
        {
          for (int i=1; i < MOTION_BUFFER_SIZE; i++)
          {
            bufferDistance_[i-1] = bufferDistance_[i]; 
            motionBufferTime_[i-1] = motionBufferTime_[i];
          }
          bufferDistance_[MOTION_BUFFER_SIZE - 1] = deltaDistance;
          motionBufferTime_[MOTION_BUFFER_SIZE - 1] = deltaT;

          for (int i=0; i < MOTION_BUFFER_SIZE; i++)
          {
            currentBufferDistance += bufferDistance_[i]; 
            currentBufferTime += motionBufferTime_[i];
          }
          // no need to divide out since we use both these two numbers
          // to calc velocity
          //currentBufferDistance /= MOTION_BUFFER_SIZE;
          //currentBufferTime /= MOTION_BUFFER_SIZE;
        } 
        else
        {
          currentBufferDistance = deltaDistance;
          currentBufferTime = deltaT;         
        }

        linearCurrentVelocity_ = currentBufferDistance * 1000.0f / ((double) currentBufferTime); // velocity in mm/sec 
        
        

        
        if (autoMoveMode_)
        {
          double distanceMoved = ((abs(leftTotalTicks - autoInitialLeftTicks_) + abs(rightTotalTicks - autoInitialRightTicks_)) *  MM_PER_TICK_EBIKE) / 2.;
          if (autoMoveStopped_) // just waiting to stop rolling
          {
            if ( abs(leftDeltaTicks) + abs(rightDeltaTicks) == 0) stoppedCounter_++;  //abs(deltaWheelMovement) < STOPPED_MOVING_TICKS_THRESHOLD * ((double) MM_PER_TICK_EBIKE) / 2.0 )
            else stoppedCounter_ = 0; // we want a sequential set of 0 movements
            if (stoppedCounter_ > STOPPED_MOVING_COUNT_THRESHOLD)
            { 
              stop();
              autoMoveMode_ = false;
              autoMoveStopped_ = false;
              //DEBUG_SERIAL_PORT.print("last value for deltaWheelMovement = ");
              //DEBUG_SERIAL_PORT.println(deltaWheelMovement);
              if (autoLinearMoving_)
              {
                DEBUG_SERIAL_PORT.print("autoMove completed, initial distance, final distance, total move = ");
                DEBUG_SERIAL_PORT.print((abs(autoInitialLeftTicks_) + abs(autoInitialRightTicks_)) *  MM_PER_TICK_EBIKE / 2.);
                DEBUG_SERIAL_PORT.print(", ");
                DEBUG_SERIAL_PORT.print((abs(leftTotalTicks) + abs(rightTotalTicks)) *  MM_PER_TICK_EBIKE / 2.);
                DEBUG_SERIAL_PORT.print(", ");
                DEBUG_SERIAL_PORT.println(distanceMoved); 
                DEBUG_SERIAL_PORT.print("during move, initial yaw, final yaw, total turn = ");
                DEBUG_SERIAL_PORT.print(autoInitialYaw_);
                DEBUG_SERIAL_PORT.print(", ");
                DEBUG_SERIAL_PORT.print(gyro_stabilizer.getYawTotal());
                DEBUG_SERIAL_PORT.print(", ");
                DEBUG_SERIAL_PORT.println(gyro_stabilizer.getYawTotal() - autoInitialYaw_);
               }
               else
               {                  
                DEBUG_SERIAL_PORT.print("autoMove turn done: initial yaw, final yaw, total turn = ");
                DEBUG_SERIAL_PORT.print(autoInitialYaw_);
                DEBUG_SERIAL_PORT.print(", ");
                DEBUG_SERIAL_PORT.print(gyro_stabilizer.getYawTotal());
                DEBUG_SERIAL_PORT.print(", ");
                DEBUG_SERIAL_PORT.println(gyro_stabilizer.getYawTotal() - autoInitialYaw_);
                DEBUG_SERIAL_PORT.print("during turn, initial distance, final distance, total move = ");
                DEBUG_SERIAL_PORT.print((abs(autoInitialLeftTicks_) + abs(autoInitialRightTicks_)) *  MM_PER_TICK_EBIKE / 2.);
                DEBUG_SERIAL_PORT.print(", ");
                DEBUG_SERIAL_PORT.print((abs(leftTotalTicks) + abs(rightTotalTicks)) *  MM_PER_TICK_EBIKE / 2.);
                DEBUG_SERIAL_PORT.print(", ");
                DEBUG_SERIAL_PORT.println(distanceMoved); 
               }     
            }
            /*
            else
            {
              DEBUG_SERIAL_PORT.print("stopped counter = ");
              DEBUG_SERIAL_PORT.print(stoppedCounter_); 
              DEBUG_SERIAL_PORT.println(", waiting for wheels to stop");
            }
            */
            return;
           }
           
          if (autoLinearMoving_)
          {
            //DEBUG_SERIAL_PORT.print("automove distance moved so far = ");
            //DEBUG_SERIAL_PORT.println(distanceMoved);
            if (fabs(distanceMoved) >= fabs(autoDistance_))
            {
              for (int i=0; i < NUM_DACS; i++) motor_dac[i].setMotorSpeed(DAC_MIN_VALUE);   // sets all dac motor speeds to 0 
              linearCommandedVelocity_ = 0;
              angCommandedVelocity_ = 0;
              linearCurrentVelocity_ = 0;
              angCurrentVelocity_ = 0;
              rightSpeed_ = DAC_MIN_VALUE; 
              leftSpeed_ = DAC_MIN_VALUE;
              previousRightSpeed_ = DAC_MIN_VALUE;
              previousLeftSpeed_ = DAC_MIN_VALUE;
              autoMoveStopped_ = true;
              return;
            }
          }
          else  // we are turning
          {
             if ( abs(gyro_stabilizer.getYawTotal() - autoInitialYaw_) >= abs(autoDegrees_) )
             {
                for (int i=0; i < NUM_DACS; i++) motor_dac[i].setMotorSpeed(DAC_MIN_VALUE);   // sets all dac motor speeds to 0 
                linearCommandedVelocity_ = 0;
                angCommandedVelocity_ = 0;
                linearCurrentVelocity_ = 0;
                angCurrentVelocity_ = 0;
                rightSpeed_ = DAC_MIN_VALUE; 
                leftSpeed_ = DAC_MIN_VALUE;
                previousRightSpeed_ = DAC_MIN_VALUE;
                previousLeftSpeed_ = DAC_MIN_VALUE;
                autoMoveStopped_ = true;
                return;
             }
          }               
        }
        
 
   
        linearTargetVelocity = linearPreviousTargetVelocity_;
        pidLinear_->calc(&linearTargetVelocity, linearCommandedVelocity_, linearCurrentVelocity_, deltaT);

        
        #ifdef DEBUG_SERIAL_PORT
        if (DEBUG)
          {         
            // console print for testing
            DEBUG_SERIAL_PORT.print("total ticks, L, R = ");
            DEBUG_SERIAL_PORT.print(leftTotalTicks);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.println(rightTotalTicks);
            DEBUG_SERIAL_PORT.print("delta ticks, L, R = ");
            DEBUG_SERIAL_PORT.print(leftDeltaTicks);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.println(rightDeltaTicks);
            DEBUG_SERIAL_PORT.print("total distance = ");
            DEBUG_SERIAL_PORT.println(totalDistance_);
            DEBUG_SERIAL_PORT.print("delta distance = ");
            DEBUG_SERIAL_PORT.println(deltaDistance);
            DEBUG_SERIAL_PORT.print("buffer distance, buffer time = ");
            DEBUG_SERIAL_PORT.print(currentBufferDistance);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.println(currentBufferTime);
            DEBUG_SERIAL_PORT.print("linearCurrentVelocity = ");
            DEBUG_SERIAL_PORT.println(linearCurrentVelocity_);
            DEBUG_SERIAL_PORT.print("pidLinearV = ");
            DEBUG_SERIAL_PORT.println(linearTargetVelocity);
          }
        #endif  // ends #ifdef DEBUG_SERIAL_PORT     
        // don't let pid controller swap our linear direction
        // unless we are mostly just turning
        /*
        if (rcs::sign(linearCommandedVelocity_) > 0)
        {
          if (rcs::sign(linearTargetVelocity) < 0)
          {
             DEBUG_SERIAL_PORT.println("pid wants to change our linear direction to reverse, but we won't let it do that");
             linearTargetVelocity = MIN_ABSVAL_LINEAR_ROBOT_VELOCITY;
          }
        }
        else if (rcs::sign(linearCommandedVelocity_) < 0)
        {
          if (rcs::sign(linearTargetVelocity) > 0)
          {
            DEBUG_SERIAL_PORT.println("pid wants to change our linear direction to forward, but we won't let it do that");
            linearTargetVelocity = -MIN_ABSVAL_LINEAR_ROBOT_VELOCITY;
          }
        }
        */ 
        linearPreviousTargetVelocity_ = linearTargetVelocity;
      
      transformVelocityToMotorSpeed(linearTargetVelocity, angTargetV, &leftSpeed_, &rightSpeed_);
      
      // sanity check on controller, need the deadband because edge cases will cross over with pid control, for example, autoC(100,37);
      double angCommandedVelocityMMperSec;
      if ( fabs(linearCommandedVelocity_) < 0.1) angCommandedVelocityMMperSec = angCommandedVelocity_ * STANDING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO;
      else angCommandedVelocityMMperSec = angCommandedVelocity_ * MOVING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO;
      
      
      if ( ((linearCommandedVelocity_ - angCommandedVelocityMMperSec > 50) && leftSpeed_ < -DAC_LOWER_VALUE )
        || ((linearCommandedVelocity_ - angCommandedVelocityMMperSec < -50) && leftSpeed_ > DAC_LOWER_VALUE )
        || ((linearCommandedVelocity_ + angCommandedVelocityMMperSec > 50) && rightSpeed_ < -DAC_LOWER_VALUE)
        || ((linearCommandedVelocity_ + angCommandedVelocityMMperSec < -50) && rightSpeed_ > DAC_LOWER_VALUE) )
      {
          //if (DEBUG) 
          {
            DEBUG_SERIAL_PORT.print("speeds reversed compared to commands, probably trying to correct yaw drift, left, right speed = ");
            DEBUG_SERIAL_PORT.print(leftSpeed_);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.println(rightSpeed_);
            DEBUG_SERIAL_PORT.print("commanded linear (mm/sec), angular (deg/sec), angular (mm/sec) velocities = ");
            DEBUG_SERIAL_PORT.print(linearCommandedVelocity_);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.print(angCommandedVelocity_);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.println(angCommandedVelocityMMperSec);
            DEBUG_SERIAL_PORT.print("the pid generated targets are: linearTargetVelocity, angTargetV =  ");
            DEBUG_SERIAL_PORT.print(linearTargetVelocity);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.println(angTargetV);
            DEBUG_SERIAL_PORT.print("current linear, angular (deg/sec), angular (mm/sec) velocities = ");          
            DEBUG_SERIAL_PORT.print(linearCurrentVelocity_);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.print(angCurrentVelocity_);
          }
        if ( ((linearCommandedVelocity_ - angCommandedVelocityMMperSec > 50) && leftSpeed_ < -DAC_LOWER_VALUE )
        || ((linearCommandedVelocity_ - angCommandedVelocityMMperSec < -50) && leftSpeed_ > DAC_LOWER_VALUE )) 
        {
          leftSpeed_ = 0;
          rightSpeed_ = (linearCommandedVelocity_ * speedVelocityRatio_) + (sign(linearCommandedVelocity_) * DAC_LOWER_VALUE);
          DEBUG_SERIAL_PORT.print("setting left speed = 0 and right speed = ");
          DEBUG_SERIAL_PORT.println(rightSpeed_);
        }
        else
        { 
           rightSpeed_ = 0;
           leftSpeed_ = (linearCommandedVelocity_ * speedVelocityRatio_) + (sign(linearCommandedVelocity_) * DAC_LOWER_VALUE);
           DEBUG_SERIAL_PORT.print("setting right speed = 0 and left speed = ");
           DEBUG_SERIAL_PORT.println(leftSpeed_);
        }
      }
      
      if (DEBUG) 
        {    
          DEBUG_SERIAL_PORT.print("commanded linear (mm/sec), angular (deg/sec), angular (mm/sec) velocities = ");
          DEBUG_SERIAL_PORT.print(linearCommandedVelocity_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.print(angCommandedVelocity_); 
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.print(angCommandedVelocityMMperSec);
          //if (fabs(linearCommandedVelocity_) < 0.1 ) DEBUG_SERIAL_PORT.println(angCommandedVelocity_ * STANDING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO);
          //else DEBUG_SERIAL_PORT.println(angCommandedVelocity_ * MOVING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO);           
          DEBUG_SERIAL_PORT.print("current linear, angular (deg/sec), angular (mm/sec) velocities = ");          
          DEBUG_SERIAL_PORT.print(linearCurrentVelocity_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.print(angCurrentVelocity_);
          DEBUG_SERIAL_PORT.print(", ");
          if (fabs(linearCommandedVelocity_) < 0.1 ) DEBUG_SERIAL_PORT.println(angCurrentVelocity_ * STANDING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO); // based on commanded, not current velocity
          else DEBUG_SERIAL_PORT.println(angCurrentVelocity_ * MOVING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO); 
          DEBUG_SERIAL_PORT.print("pid target linear, angular velocities = ");
          DEBUG_SERIAL_PORT.print(linearTargetVelocity);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(angTargetV);
          DEBUG_SERIAL_PORT.print("previous L, R speeds = ");
          DEBUG_SERIAL_PORT.print(previousLeftSpeed_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(previousRightSpeed_); 
          DEBUG_SERIAL_PORT.print("current L, R speeds = ");
          DEBUG_SERIAL_PORT.print(leftSpeed_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(rightSpeed_);     
          DEBUG_SERIAL_PORT.print("delta time = ");
          DEBUG_SERIAL_PORT.println(deltaT);
          DEBUG_SERIAL_PORT.println();
          DEBUG_SERIAL_PORT.println(); 
        }
        if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print(leftSpeed_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.print(rightSpeed_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.print(linearCurrentVelocity_); 
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.print(angCurrentVelocity_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(deltaT);
        }     
       
        motor_dac[RIGHT].setMotorSpeed(rightSpeed_);
        motor_dac[LEFT].setMotorSpeed(leftSpeed_);  
     
        previousRightSpeed_ = rightSpeed_;
        previousLeftSpeed_ = leftSpeed_ ;  
        
        if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print("linear and angular velocities commanded: ");
          DEBUG_SERIAL_PORT.print(linearCommandedVelocity_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(angCommandedVelocity_); 
          DEBUG_SERIAL_PORT.print("speeds, L,R: ");
          DEBUG_SERIAL_PORT.print(leftSpeed_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(rightSpeed_);     
        }
        
        
        angPreviousTargetVelocity_ = angCommandedVelocity_;
        linearPreviousTargetVelocity_ = linearCommandedVelocity_;     
    }
 
    void transformVelocityToMotorSpeed(double linearVelocity, double angVelocity, int *leftSpeed, int *rightSpeed)
    {
      /*
      if(DEBUG)
      {  
          DEBUG_SERIAL_PORT.print("transform linear, angular velocities received = ");
          DEBUG_SERIAL_PORT.print(linearVelocity);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(angVelocity);
      }
      */
          
      if (abs(linearVelocity) + abs(angVelocity) < 0.1f)
      {
        *leftSpeed = DAC_MIN_VALUE;
        *rightSpeed = DAC_MIN_VALUE;
        return;
      }
      
      // check if we will be commanding speeds beyond what the robot can do
      if (fabs(linearVelocity) > MAX_LINEAR_ROBOT_VELOCITY) linearVelocity = MAX_LINEAR_ROBOT_VELOCITY * sign(linearVelocity);
      if (fabs(angVelocity) > MAX_ANGULAR_ROBOT_VELOCITY) angVelocity = MAX_ANGULAR_ROBOT_VELOCITY * sign(angVelocity);
     // else if (fabs(angVelocity) > 1. && fabs(angVelocity) <  MIN_STANDING_ABSVAL_ANGULAR_ROBOT_VELOCITY && fabs(linearVelocity) < 1.) angVelocity = MIN_STANDING_ABSVAL_ANGULAR_ROBOT_VELOCITY * sign(angVelocity);
      
      if (fabs(linearCommandedVelocity_) < 0.1f) angVelocity *= STANDING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO; // express angular velocity in wheel speed
      else angVelocity *= MOVING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO; // when we are moving, we have less frictional force to overcome, so this ratio is lower than when standing
      
      if (fabs(linearVelocity) + fabs(angVelocity) > MAX_WHEEL_VELOCITY)     
      {
        // keep the turn, limit the forward and reverse
        //double ratio = ((double)MAX_LINEAR_ROBOT_VELOCITY) / (abs(linearVelocity) + abs(angVelocity * MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO));
        if ( fabs(angVelocity) > MAX_WHEEL_VELOCITY) angVelocity = MAX_WHEEL_VELOCITY * sign(angVelocity);
        linearVelocity = (MAX_WHEEL_VELOCITY - fabs(angVelocity)) * sign(linearVelocity);
        //linearVelocity *= ratio;
        //angVelocity *= ratio;
        //if (DEBUG && 
        if (!messageLimit_)
        {
          DEBUG_SERIAL_PORT.print("velocities exceeded max, had to be throttled.  linear vel, ang vel =  ");
          DEBUG_SERIAL_PORT.print(linearVelocity);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(angVelocity);
         // DEBUG_SERIAL_PORT.println(ratio);
          messageLimit_ = true;  // don't want to flood the print screen with tons of these messages
        }
      }
      else messageLimit_ = false;
      
      double leftTargetSpeed = (linearVelocity - angVelocity) * speedVelocityRatio_;
      double rightTargetSpeed = (linearVelocity + angVelocity) * speedVelocityRatio_; 

//      if (linearCommandedVelocity_ > 0.1)  // with outdoor bot, the ebike motor controllers have a hard time with reversing directions, so when we are moving forward,
//      {                                    // we will limit the turn rate so that we do not command wheels to go in reverse.  we also do not want to trigger code that looks for speeds = 0, so we will set it = 1
//        if (leftTargetSpeed < 0) leftTargetSpeed = 1;
//        if (rightTargetSpeed < 0) rightTargetSpeed = 1;
//      }   

      *leftSpeed =  ((int) leftTargetSpeed) + (sign(leftTargetSpeed) * DAC_LOWER_VALUE); // speeds are integers, so we can only use the integer part
      *rightSpeed =  ((int) rightTargetSpeed) + (sign(rightTargetSpeed) * DAC_LOWER_VALUE);  // motors start moving when dac is above DAC_LOWER_VALUE
      
      /*
      if (DEBUG)
      {
          DEBUG_SERIAL_PORT.print("linear, angular velocities = ");
          DEBUG_SERIAL_PORT.print(linearVelocity);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(angVelocity);
          DEBUG_SERIAL_PORT.print("transform L, R speeds sent = ");
          DEBUG_SERIAL_PORT.print(*leftSpeed);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(*rightSpeed);
      }
      */
    } 
    
    void setSpeedVelocityRatios()
    {
      if (robotPause_) return;
      double angVelocity;
      if (fabs(linearCommandedVelocity_) < 0,1) angVelocity = STANDING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO * angCurrentVelocity_;
      else angVelocity = STANDING_MM_PER_SEC_CONVERSION_TO_DEGREES_PER_SECOND_RATIO * angCurrentVelocity_;
      double leftWheelVelocity = linearCurrentVelocity_ - angVelocity;
      double rightWheelVelocity = linearCurrentVelocity_ + angVelocity;
      
      if (fabs(leftWheelVelocity) < 50 || fabs(rightWheelVelocity) < 50)
      {
        //if (DEBUG) 
        DEBUG_SERIAL_PORT.println("Speed-Velocity ratios not set because velocities were too slow");        
        return; // don't divide by tiny numbers
      }
      double localLeftSpeed = (double) (leftSpeed_ - (DAC_LOWER_VALUE * rcs::sign(leftSpeed_)));
      double localRightSpeed = (double) (rightSpeed_ - (DAC_LOWER_VALUE * rcs::sign(rightSpeed_)));
      double LocalSpeedVelocityRatio = ((localLeftSpeed / leftWheelVelocity ) + (localRightSpeed / rightWheelVelocity)) / 2.0f;
     
      //if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("Speed-Velocity ratio = ");
        DEBUG_SERIAL_PORT.println(LocalSpeedVelocityRatio);
      }
    }
    
    boolean startUpMode() // when motors are just getting going, we need to use larger pid parameters than in steady state
    {
      if ( abs(linearCurrentVelocity_) + abs(angCurrentVelocity_) < (abs(linearCommandedVelocity_) + abs(angCommandedVelocity_)) / 2.0) return true;
      return false;
    }

    void setAutonomousCommandMode(boolean value) { 
      autonomousCommandMode_ = value; 
    } 
    boolean getAutonomousCommandMode() { 
      return autonomousCommandMode_;  
    }
    
    void setAutoMoveMode(boolean value) { 
      autoMoveMode_ = value; 
    } 
    boolean getAutoMoveMode() { 
      return autoMoveMode_;  
    }
    void setNewCommandMode(boolean value) { 
      newCommandMode_ = value;  
    }  
    boolean getNewCommandMode() { 
      return newCommandMode_; 
    }
    
    double getLinearCommandedVelocity() { return linearCommandedVelocity_; }
    double getAngCommandedVelocity() { return angCommandedVelocity_; }
    
    void setAngOnly(boolean value) { angOnly_ = value; }
    boolean getAngOnly() { return angOnly_; }
    
    unsigned long getMotionPreviousTime() { return motionPreviousTime_; }    
    double getSpeedVelocityRatio() { return speedVelocityRatio_; }

private:
    boolean remoteCommandMode_, autonomousCommandMode_, newCommandMode_, messageLimit_, autoMoveMode_, autoMoveStopped_, stoppedCounter_, autoLinearMoving_, angOnly_;
    long rightPreviousTicks_,leftPreviousTicks_;
    int rightSpeed_, leftSpeed_, previousRightSpeed_, previousLeftSpeed_, previousLeftDirection_, previousRightDirection_;
    int rightPreviousDirection_, leftPreviousDirection_; // used to goose motors with ebike controllers
    double linearCommandedVelocity_, angCommandedVelocity_;
    double linearCurrentVelocity_, angCurrentVelocity_;
    double totalDistance_;
    double totalLinearVelocityCorrection_;
    unsigned long motionBufferTime_[MOTION_BUFFER_SIZE + 2]; // covers for MOTION_BUFFER_SIZE = 0
    double bufferDistance_[MOTION_BUFFER_SIZE + 2], motionPreviousTime_;
    double linearPreviousVelocity_, linearVelIntegralError_, angPreviousVelocity_, angVelIntegralError_;
    double linearPreviousTargetVelocity_, angPreviousTargetVelocity_;
    double speedVelocityRatio_;
    double kpLinearV_, kiLinearV_, kdLinearV_, kpAngularV_, kiAngularV_, kdAngularV_;
    double autoDistance_, autoDegrees_, autoInitialYaw_;
    long autoInitialLeftTicks_, autoInitialRightTicks_;
    pidControl *pidLinear_;
  } 
  robot_base;

/*
  // init();
  // Initializes the board for robot control.
class Init : 
  public Instruction {
public:
    Init() : 
    Instruction("init") {
    }

    virtual boolean run(String* args) {
      return (*this)();
    }
    boolean operator() () {             
      robot_base.stop(); // initialize all parameters and also make sure the pins are not commanding movement.
      return true;  
    }  
  } 
  init;
*/
  // class to implement move instructions for autonomous navigation
  // form is autoC(linear vel in mm/sec, angular vel in degs/sec);
class autonomousCommand : 
  public Instruction {
public:
    autonomousCommand() : 
    Instruction("autoC") {
    }

    virtual boolean run(String* args) {
      float lin_velocity;
      float ang_velocity;
      popFloat(args, &lin_velocity);
      popFloat(args, &ang_velocity);
      return (*this)(lin_velocity, ang_velocity);
    }

    boolean operator()(float linear_velocity, float angular_velocity) 
    { 
      if (!robot_base.getAutonomousCommandMode())
      {
        robot_base.setAutonomousCommandMode(true);
        robot_base.setNewCommandMode(true);
      }
      else robot_base.setNewCommandMode(false); 
      
      // only allow turning or moving,don't combine them
      
      if (fabs(angular_velocity) > 1.) 
      {
        linear_velocity = 0;
        robot_base.setAngOnly(true);
      }
      else 
      {
        angular_velocity = 0;
        robot_base.setAngOnly(false);
      }
      
      robot_base.startMove(linear_velocity, angular_velocity);
      ping.resetTimeout();
      
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("Autonomous command received: ");
        DEBUG_SERIAL_PORT.print(linear_velocity);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(angular_velocity);
        DEBUG_SERIAL_PORT.print("newCommandMode = ");
        DEBUG_SERIAL_PORT.println(robot_base.getNewCommandMode());

      }
      
      return true;
    }
  } 
  autonomous_command;     
  
  
  class autoMoveCommand : 
  public Instruction {
public:
    autoMoveCommand() : 
    Instruction("autoMove") {
    }

    virtual boolean run(String* args) {  // distance in mm, angle in degrees, speed in mm/sec or deg/sec
      float autoDistance;
      float autoAngle;
      float suggestedSpeed;
      popFloat(args, &autoDistance);
      popFloat(args, &autoAngle);
      popFloat(args, &suggestedSpeed);

      return (*this)(autoDistance, autoAngle, suggestedSpeed);
    }

    boolean operator()(float autoDistance, float autoAngle, float suggestedSpeed) 
    {         
      // only turn or go straight, not both
      if (fabs(autoAngle) > 1.) 
      {
        autoDistance = 0;
        robot_base.setAngOnly(true);
      }
      else 
      {
        autoAngle = 0;
        robot_base.setAngOnly(false);
      }
      
      robot_base.startAutoMove(autoDistance, autoAngle, suggestedSpeed);
      robot_base.setAutoMoveMode(true);
      robot_base.setAutonomousCommandMode(false);
      ping.resetTimeout();
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("autoMove command received: ");
        DEBUG_SERIAL_PORT.print(autoDistance);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.print(autoAngle);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(suggestedSpeed);
      }
      return true;
    }
  } 
  autoMove_command;     


/*
  // Initializes the motion module.
  // Automatically called by setup().
class MotionModuleSetup : 
  public ModuleSetup {
public:
    virtual void setup() {
      init();
    }
  } 
  motion_setup;
*/

class MotionModuleLoop : 
  public ModuleLoop {
  
public:
    virtual void loop()
      {           
         if (millis() - robot_base.getMotionPreviousTime() >= MIN_UPDATE_PERIOD) // don't want to update too fast, otherwise can swamp the gyro and the motor controllers
         {
           if (robot_base.getAutonomousCommandMode() || robot_base.getAutoMoveMode() )
           {
              robot_base.checkSpeed(); 
              //robot_base.setSpeedVelocityRatios();
           }
           
           //if (robot_base.
          
         }
      }    
  } 
  motion_loop;


}  // namespace rcs

#endif  MOTION_DAC_H_


