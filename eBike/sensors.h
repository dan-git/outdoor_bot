/**************************************************************************
 *                          Sensors Module
 *
 * The sensors module reads the varios sensors, other than the gyro and the encoders, attached to the robot
 *
 *
 **************************************************************************
 */

#ifndef SENSORS_H
#define SENSORS_H


namespace rcs
{
double zero_percent_battery_voltage_default,full_battery_voltage_default,voltage_divider_ratio_default;
uint8_t battery_monitor_pin_default;
 

class RobotSensor
{
  public:
    RobotSensor()
    {
      previousSentDataTime_ = millis();
      dataCounter_ = 0;
      pinMode(AMBER_LIGHT_PIN, OUTPUT);
      digitalWrite(AMBER_LIGHT_PIN, HIGH);  // continuous on for paused state, blinking at approx 1 Hz for enabled
      pinMode(DIRECTIONAL_ANTENNA_PIN, INPUT);
      pinMode(BRAKES_CURRENT_PIN, INPUT);
    } 
    
    unsigned long getpreviousSentDataTime() { return previousSentDataTime_; }
    void setpreviousSentDataTime(unsigned long value) { previousSentDataTime_ = value; }
   
  protected:
   unsigned long previousSentDataTime_, dataCounter_;
};

class Battery : public RobotSensor
{
  public:
    Battery() :
     batteryPin_(BATTERY_MONITOR_PIN) 
    {}
    
    int checkBattery()
    {
      double voltage =  (double) ((analogRead(BATTERY_MONITOR_PIN) / 1023.) * 5.0 ) * VOLTAGE_DIVIDER_RATIO;
      double batteryRange = FULL_BATTERY_VOLTAGE - ZERO_PERCENT_BATTERY_VOLTAGE ;
      int batteryPercent =  (int) ( 100. * ( ( voltage - ZERO_PERCENT_BATTERY_VOLTAGE ) / batteryRange)); // returns percentage
      if (batteryPercent > 99) batteryPercent = 100;
      if (batteryPercent < 0) batteryPercent = 0;
      
      // ********************** testing mode
      batteryPercent = analogRead(BATTERY_MONITOR_PIN);  // for testing, note, around 765 (3.76 V on the pin, 11.96 V on the battery, the computer is close to shutting down)
      // ************************
      return batteryPercent;
    } 
  private:
    uint8_t batteryPin_;
} battery_monitor;


class SensorData : public RobotSensor
{
  public:
    SensorData()
    {
      leftWheelEncoder_ = 0;
      rightWheelEncoder_ = 0;
    }
    
    void sendNavData(unsigned long loopTime)
    {            
       #ifdef AUTONOMOUS_SERIAL_PORT 
        if (AUTONOMOUS_COMMANDING)
        {
          double xAcc, yAcc, zAcc; //, compassHeading;
          gyro_stabilizer.readAccel(&xAcc, &yAcc, &zAcc); //, &compassHeading);
          
          AUTONOMOUS_SERIAL_PORT.print("navdata");
          AUTONOMOUS_SERIAL_PORT.print(dataCounter_);
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(gyro_stabilizer.readGyro());
          AUTONOMOUS_SERIAL_PORT.print(", ");
          rightWheelEncoder_ = encoder_spi[getEncoderNumber(RIGHT_ENCODER_SELECT_PIN)].readEncoder();
          leftWheelEncoder_ = encoder_spi[getEncoderNumber(LEFT_ENCODER_SELECT_PIN)].readEncoder();
          AUTONOMOUS_SERIAL_PORT.print(rightWheelEncoder_);
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(leftWheelEncoder_);
          AUTONOMOUS_SERIAL_PORT.print(", ");
          
          AUTONOMOUS_SERIAL_PORT.print(encoder_spi[getEncoderNumber(PICKER_UPPER_ENCODER_SELECT_PIN)].readEncoder());
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(encoder_spi[getEncoderNumber(BIN_SHADE_ENCODER_SELECT_PIN)].readEncoder());
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(encoder_spi[getEncoderNumber(DROP_BAR_ENCODER_SELECT_PIN)].readEncoder());
          AUTONOMOUS_SERIAL_PORT.print(", ");
          
          
          AUTONOMOUS_SERIAL_PORT.print(dataCounter_); // we send this 3 times to check the integrity of the data string
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(robotPause_);
          AUTONOMOUS_SERIAL_PORT.print(", ");
          /*
          AUTONOMOUS_SERIAL_PORT.print(dirAnt.getMaxAngle());
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(dirAnt.getSweepNumber());
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(dirAnt.getLevel());
          AUTONOMOUS_SERIAL_PORT.print(", ");
          */
          AUTONOMOUS_SERIAL_PORT.print(robot_base.getAutoMoveMode());          
          AUTONOMOUS_SERIAL_PORT.print(", ");

          AUTONOMOUS_SERIAL_PORT.print("99"); //battery_monitor.checkBattery());
          AUTONOMOUS_SERIAL_PORT.print(", ");          
          AUTONOMOUS_SERIAL_PORT.print( motion_pd.getPickerUpperCommand() || motion_pd.getDropbarCommand() || motion_pd.getBinshadeCommand() );
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(xAcc); 
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(yAcc);
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(zAcc);
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(robot_base.getAngOnly());
          AUTONOMOUS_SERIAL_PORT.print(", ");
          
          AUTONOMOUS_SERIAL_PORT.print(loopTime);
          AUTONOMOUS_SERIAL_PORT.print(", ");
          AUTONOMOUS_SERIAL_PORT.print(dataCounter_); // we send this 3 times to check the integrity of the data string
          AUTONOMOUS_SERIAL_PORT.println(", ");
          dataCounter_++;
          /*
          #ifdef BAT_MON_SERIAL_PORT
          if (!(dataCounter_ % 1500)) // check the battery once every 30 seconds
          {
            float amps, ampHours;
            batt_mon.quickCheckValues(&amps, &ampHours);
            AUTONOMOUS_SERIAL_PORT.print(", ");
            AUTONOMOUS_SERIAL_PORT.print(amps);
            AUTONOMOUS_SERIAL_PORT.print(", ");
            AUTONOMOUS_SERIAL_PORT.print(ampHours);
            AUTONOMOUS_SERIAL_PORT.print(", "); 
            AUTONOMOUS_SERIAL_PORT.print(batt_mon.getMaxAmps());
          }  
          #endif // BAT_MON_SERIAL_PORT
          AUTONOMOUS_SERIAL_PORT.println(", ");         
          //AUTONOMOUS_SERIAL_PORT.print(gyro_stabilizer.getGyroBaseline());
          //AUTONOMOUS_SERIAL_PORT.print(", ");
          //AUTONOMOUS_SERIAL_PORT.print(gyro_stabilizer.getYawTotal());
          //AUTONOMOUS_SERIAL_PORT.println(", ");
          */
          
        }
      #endif // AUTONOMOUS_SERIAL_PORT  
    }
    
    long getLeftWheelEncoder() { return leftWheelEncoder_; }
    long getRightWheelEncoder() { return rightWheelEncoder_; }
    
    private:
    long leftWheelEncoder_, rightWheelEncoder_;

    
} sensor_data;

class SensorsModuleLoop : public ModuleLoop {
  public:
    virtual void loop() {    
      unsigned long deltaT = millis() - sensor_data.getpreviousSentDataTime();
      if (deltaT >= MIN_SENSOR_REPORT_TIME) 
      {
        robotPause_ = digitalRead(RADIO_PAUSE_PIN);
        if (!digitalRead(RADIO_CONNECTED_PIN)) robotPause_ = true; // if we lose radio contact, pause the bot       
        if (robotPause_)
        {
           if (newPauseState_)  // we entered the pause state during this cycle
           {
              digitalWrite(AMBER_LIGHT_PIN, HIGH);
              //if (DEBUG) 
              DEBUG_SERIAL_PORT.println("Entering pause state ");
              for (int i = 0; i < NUM_MOTORS_POLOLU_DRIVER; i++)
              {
                previousPDmotorSpeed_[i] = motor_pololu_driver[i].getSpeed();  // this is used to store motor speeds during a pause state
                motor_pololu_driver[i].setMotorSpeed(0);

                if (DEBUG)
                {
                  DEBUG_SERIAL_PORT.print("PDmotor ");
                  DEBUG_SERIAL_PORT.print(i);
                  DEBUG_SERIAL_PORT.print(" saved speed =  ");
                  DEBUG_SERIAL_PORT.println(previousPDmotorSpeed_[i]);
                }               
              } 
              
              for (int i = 0; i < NUM_DACS; i++)
              {
                previousDACmotorSpeed_[i] = motor_dac[i].getSpeed();
                motor_dac[i].stop();
                //robot_base.stop();
                if (DEBUG)
                {
                  DEBUG_SERIAL_PORT.print("dac_motor ");
                  DEBUG_SERIAL_PORT.print(i);
                  DEBUG_SERIAL_PORT.print(" saved speed =  ");
                  DEBUG_SERIAL_PORT.println(previousDACmotorSpeed_[i]);
                }
              }
              motion_pd.applyRobotBrakes();
              newPauseState_ = false;
           }
        }
            
        else
        {
          if (!newPauseState_) // we exited the pause state during this cycle
          {
             // if (DEBUG)
              DEBUG_SERIAL_PORT.println("Exiting pause state ");
              newPauseState_ = true;
              for (int i = 0; i < NUM_MOTORS_POLOLU_DRIVER; i++)
              {
                motor_pololu_driver[i].setMotorSpeed(previousPDmotorSpeed_[i]); 
                previousPDmotorSpeed_[i] = 0;  // this is used to store motor speeds during a pause state
              } 
 //             for (int i = 0; i < NUM_DACS; i++)
  //            {
 //               motor_dac[i].setMotorSpeed(previousDACmotorSpeed_[i]);  //***************** decided to not resume speed from before pause
 //             } 
               motion_pd.releaseRobotBrakes();
            }          
            pauseBlinkTime_ += deltaT;
            if (pauseBlinkTime_ > AMBER_LIGHT_BLINK_TIME)
            {
              if (toggleAmberLightState_ == 0) toggleAmberLightState_ = 1;
              else toggleAmberLightState_ = 0;
              digitalWrite(AMBER_LIGHT_PIN, toggleAmberLightState_);
              pauseBlinkTime_ = 0;
            }
          } // closes pause section
          
          // runaway train section
          long static previousWheelEncoderValue = 0, netNoCommandMove = 0, previousTime = millis() - 1, goodSpeedCounter = 0, runawaySpeedCounter = 0;
          bool static runawayBrakesApplied = false;
          long newWheelEncoderValue = abs(sensor_data.getLeftWheelEncoder()) + abs(sensor_data.getRightWheelEncoder());
          long deltaTicks = previousWheelEncoderValue - newWheelEncoderValue;
          
          if (USE_NO_COMMAND_MOVE_CHECK)
          {            
            // movement with no command
            if ( fabs(robot_base.getLinearCommandedVelocity()) + fabs(robot_base.getAngCommandedVelocity()) < 0.1
              && (!(radio_joystick_x_handler.getRadioCommandMode()) || radio_joystick_y_handler.getRadioCommandMode()) ) // no commanded move
            {             
               netNoCommandMove += deltaTicks; 
               double distanceMoved = ((double) netNoCommandMove) * MM_PER_TICK_EBIKE / 2.;
               if ( distanceMoved > NO_COMMAND_BRAKES_THRESHOLD) motion_pd.applyRobotBrakes();
            }
            else netNoCommandMove = 0;
          }
          
          if (USE_RUNAWAY_TRAIN_CHECK)
          {  
            double currentSpeed =  (((double) deltaTicks) * MM_PER_TICK_EBIKE * 1000.) / ((double) (millis() - previousTime)); // mm/sec
            if (currentSpeed > RUNAWAY_SPEED_THRESHOLD) runawaySpeedCounter++;
            else goodSpeedCounter++;
            if (goodSpeedCounter > 20)
            {
              goodSpeedCounter = 0;
              runawaySpeedCounter = 0;
            }
            if (runawaySpeedCounter - goodSpeedCounter > 25) 
            {
              motion_pd.applyRobotBrakes();
              runawayBrakesApplied = true;
              goodSpeedCounter = 0;
              runawaySpeedCounter = 0;
            }
            else if (runawayBrakesApplied && (goodSpeedCounter - runawaySpeedCounter > 10))
            {
              motion_pd.releaseRobotBrakes();
              runawayBrakesApplied = false;
              goodSpeedCounter = 0;
              runawaySpeedCounter = 0; 
            }          
            
          }
          
          
          previousWheelEncoderValue = newWheelEncoderValue;
          
          // movement with excessive speed
          
          
          
          sensor_data.setpreviousSentDataTime(millis());
          sensor_data.sendNavData(deltaT);         
       } // closes if (deltaT >= MIN_SENSOR_REPORT_TIME)    
    }
} sensors_loop;



} // ends namespace
#endif // SENSORS_H
