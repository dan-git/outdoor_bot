/**************************************************************************
 *                          Gyro Module
 *
 * The gyro module uses the built-in gyroscope to stabilize the robot.
 * In order to use the gyro module, the controller must have a compatible
 * gyroscope attached or built-in.
 *
 * The gyro module uses the L3G line of gyroscopes and requires the L3G
 * library.
 *
 * In order to use the module, USE_GYRO in config.h must be set to true.
 * If no gyroscope is attached, USE_GYRO in config.h must be set to false.
 *
 * The gyro module automatically registers itself as a motion stabilizer
 * with the motion module. This causes the motion module to make automatic
 * use of the gyro to adjust motion 
 **************************************************************************/



#ifndef GYRO_H_
#define GYRO_H_

namespace rcs {
  
#define ZERO_ANGV_EXTRA_INTEGRAL_PARAMETER 2.0
#define MAX_ZERO_ANGV_EXTRA_INTEGRAL_PARAMETER 20.
  
// Abstact base class for motion stabilizers.
// A motion stabilizer is used to ensure that the robot moves in the intended direction and speed.
class MotionStabilizer {
  public:
    // Computes an updated direction for the robot.
    virtual void stabilize(double *pidResult, double *angV, double commandedAngV, long deltaTime, double speedVelocityRatio, boolean startUpMode);
};

/*
 * The GyroStabilizer is used to stabilize the motion of the robot using
 * the built-in gyro. When registered RobotBase, it is periodically called
 * by the RobotBase to stabilize the robot.
 */
class GyroStabilizer : public MotionStabilizer {
  public:
    GyroStabilizer()
      : direction_adjustment_(0.0f), gyroCorrectionTotal_(0.0f),
      previousAngV_(0.0f), angVelIntegralError_(0.0f),
      kpGyro_(KP_GYRO), kiGyro_(KI_GYRO), kdGyro_(KD_GYRO), yawTotal_(0.0)
      {
          
        pidGyro_ = new pidControl(KP_GYRO, KI_GYRO, KD_GYRO, MAX_PID_ANGVEL_CHANGE_PER_CYCLE, MAX_ANGULAR_ROBOT_VELOCITY);  
        previousTime_ = millis();
        yawZeroAngV_ = 0.;
        firstPassZeroAngV_ = true;
      }
      
     ~GyroStabilizer()
     {
       delete pidGyro_;
     }

    // Initializes the gyroscope. Must be called before the stabilize() is
    // called.
    boolean init() {
      if (!USE_GYRO) return false;
      if (!gyro_.init())
      {
        delay(2000);  // give time for power supplies to come up
        if (!gyro_.init())
        {
          delay(3000);
          if (!gyro_.init())          
          { 
            Serial.println("GYRO FAILED TO INIT");
            return false;
          }
        }
       }
     
 
        
        //gyro_.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
        
        // set the data rate, called ODR.  Possible values are 95, 190, 380, or 760Hz, default is 95
        // we will use 190 Hz
        // and we also set the bandwidth here, we will use a cutoff of 12.5 Hz
        // so we have 0100 for the high byte in register 1 
        
        // we also set the power mode and which axes are enabled via register 1
        // we choose normal power mode and only the Z axis enabled
        // bit 3 = power state, bits 0,1,2 = Y,X,Z axis enables
        // note that bit 3 is useful for powe savings (power on/off), but even with bit 3 = 1
        // you can go into sleep mode by setting bits 0,1,2 all = 0
        // so we have 1100 for the low byte
        // all together that is 0100 1100 = 0x4C
  
        gyro_.writeReg(L3G_CTRL_REG1, 0x4C);
        
        
        // next set high pass filter mode to normal (HPM1 = 1, HPM0 = 0)
        // note that you can also use a normal mode (HPM1 = 0, HPM0 = 0) where the filter is reset with a read to HP_RESET_FILTER
        // and set the high pass filter cutoff frequency.  We will use 13.5 Hz and, with ODR = 190, that is all 0s in HPCF
        // and corresponds to the default setting and also the highest available high pass cutoff frequency for ODR = 190
        // This will cause the high pass to not have much effect, since we set our cutoff frequency to be 12.5 Hz
        gyro_.writeReg(L3G_CTRL_REG2, 0x23);
        
        // register 3 is for interrupts and FIFO, which we are not using,so the entire register defaults to 0
        
        // next we set the sensitivity.  We do this automatically, based on the value for GYRO_GAIN_YAW set in config.h
        // That allows us to use different sensitivities for different robots
        
        if (abs(GYRO_GAIN_YAW) > 0.05) gyro_.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale, GYRO_GAIN_YAW = 0.07 
        else if (abs(GYRO_GAIN_YAW) > 0.01) gyro_.writeReg(L3G_CTRL_REG4, 0x10); // 500 dps full scale, GYRO_GAIN_YAW = 0.0175
        else gyro_.writeReg(L3G_CTRL_REG4, 0x00); // 250 dps full scale, GYRO_GAIN_YAW = 0.00875
        // High byte: (00: 250 dps; 01: 500 dps; 10: 2000 dps; 11: 2000 dps)
      
         
        
       if ( accel_.init())
       {
        if (accel_.getDeviceType() == LSM303::device_DLHC) 
        {
          accel_.writeAccReg(LSM303::CTRL_REG1_A, 0x47); // normal power mode, all axes enabled, 50 Hz
          accel_.writeAccReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution 
        }
        else 
        {
          accel_.writeAccReg(LSM303::CTRL_REG1_A, 0x27); // normal power mode, all axes enabled, 50 Hz
          accel_.writeAccReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11 on DLH, DLM 
        }

        // get accelerometer and heading data to send to ROS
        // tests show that 
        // accelZ avg = -261.96 with std = 0.03 when accelY avg = -1.79 with stddev = 0.03 and accelX avg = 3.21 with stddev 0.03
        // so the total gravity vector is sqrt of the sum of those three squared = 262.0
        // so to convert to m/sec*sec, we need to multipy by 0.0374
        // with that conversion, accelZ avg = , std dev = , variance = 
        // accel Y avg value = -2.05, std dev = 0.03
        // accelX avg value 0.12, 
        
        gyroBaseline_ = 0.0;
        xAccelBaseline_ = 0.0; 
        yAccelBaseline_ = 0.0;
        zAccelBaseline_ = 0.0;
        delay(200);  // gyro needs some time to initialize
        for (int i=0; i < 5; i++)  // throw out first few reads
        {
          gyro_.read();
          accel_.read();
          delay(50);
        }
        int baselineNumPoints = 20;
        for (int i=0; i < baselineNumPoints; i++)
        {
          gyro_.read();
          accel_.read();
          gyroBaseline_ += gyro_.g.z;
          xAccelBaseline_ += accel_.a.x; 
          yAccelBaseline_ += accel_.a.y;
          zAccelBaseline_ += accel_.a.z;
          delay(50);
        }
        gyroBaseline_ /= baselineNumPoints;
        xAccelBaseline_ /= baselineNumPoints; 
        yAccelBaseline_ /= baselineNumPoints;
        zAccelBaseline_ /= baselineNumPoints; // this takes out GRAVITY as well, so no need for an extra entry
        if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print("accel baselines: ");
          DEBUG_SERIAL_PORT.print(gyroBaseline_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.print(xAccelBaseline_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.print(yAccelBaseline_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(zAccelBaseline_);
        }
        return true;
      }
      Serial.println("ACCEL FAILED TO INIT");
      return false;
    }
    
    double readGyro()
    {
      gyro_.read();
      double yawVel = (gyro_.g.z - gyroBaseline_) * GYRO_GAIN_YAW;  // in degrees/sec
      //if (left_motor.speed() > 0 && right_motor.speed() > 0) yaw = gyro_.g.z - gyroBaseline_)
      yawTotal_ += yawVel * ( (double) (millis() - previousTime_) / 1000.);
      previousTime_ = millis();
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("Yaw velocity, totalYaw = ");
        DEBUG_SERIAL_PORT.print(yawVel); 
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(yawTotal_);
      } 
      if (abs(yawVel) < 0.2) gyroBaseline_ +=  (gyro_.g.z - gyroBaseline_) / 100.; // if we are just seeing noise, update the baseline
      //DEBUG_SERIAL_PORT.println((gyro_.g.z - gyroBaseline_) - (-(accel_.a.y - yAccelBaseline_ ))/10.
      return yawVel;
    }
    
    double getYawTotal()  // in degrees
    {
      return yawTotal_;
    }
    
    void setYawTotal(double value)
    {
      yawTotal_ = value;
    }

    double getGyroBaseline()
    {
      return gyroBaseline_;
    } 
    
    // read accelerometer, signs are setup for ROS conventions
    void readAccel(double *xAccel, double *yAccel, double *zAccel) //, double *compassHeading)
    {
      accel_.read();
      // use signs that give correct values for pitch and roll by the ROS conventions
      
        *xAccel = (accel_.a.x - xAccelBaseline_) * ACCEL_CONVERSION_TO_M_PER_SEC_SQUARED;  // accelerate to the left gives positive values
        *yAccel = (accel_.a.y - yAccelBaseline_ )* ACCEL_CONVERSION_TO_M_PER_SEC_SQUARED; // accelerate forward gives positive values
        *zAccel = -(accel_.a.z - zAccelBaseline_) * ACCEL_CONVERSION_TO_M_PER_SEC_SQUARED; // accel upward gives positive values
    }

    virtual void stabilize(double *pidResult, double *angV, double commandedAngV, long deltaTime, double speedVelocityRatio, boolean startUpMode)
    {
      double deltaT = (double) deltaTime;

      double currentAngV = readGyro();
      *angV = currentAngV;
      /*
      if (startUpMode)
      {
        kpGyro_ = KP_GYRO_STARTUP;
        kiGyro_ = KI_GYRO_STARTUP;
        kdGyro_ = KD_GYRO_STARTUP;
      }
      else
      {
        kpGyro_ = KP_GYRO;
        kiGyro_ = KI_GYRO;
        kdGyro_ = KD_GYRO;
      }

          
      

      //pidControl(&previousAngV_, &angVelIntegralError_, pidResult, 
      // currentAngV, commandedAngV, deltaT, kpGyro_, kiGyro_, kdGyro_);
      */
     
      pidGyro_->calc(pidResult, commandedAngV, currentAngV, deltaT);
      
      
      
      if (commandedAngV == 0)
      {
        if (firstPassZeroAngV_)
        {
          firstPassZeroAngV_ = false;
          yawZeroAngV_ = yawTotal_;
        }
        double correction = (yawZeroAngV_ - yawTotal_) *  ZERO_ANGV_EXTRA_INTEGRAL_PARAMETER;
        // avoid windup
        if (fabs(correction) > MAX_ZERO_ANGV_EXTRA_INTEGRAL_PARAMETER)
        {
           DEBUG_SERIAL_PORT.print("exceeded max zero angular velocity correction in gyro.h: yawZeroAngV_, yawTotal_ = ");
           DEBUG_SERIAL_PORT.print(yawZeroAngV_);
           DEBUG_SERIAL_PORT.print(", ");
           DEBUG_SERIAL_PORT.println(yawTotal_);
           DEBUG_SERIAL_PORT.print("calculated and actually used yaw corrections = ");
           DEBUG_SERIAL_PORT.print(correction);
           DEBUG_SERIAL_PORT.print(", ");
           correction = MAX_ZERO_ANGV_EXTRA_INTEGRAL_PARAMETER * rcs::sign(correction);
           DEBUG_SERIAL_PORT.println(correction);
        }
        //else if (DEBUG)
        {
           DEBUG_SERIAL_PORT.print("zero angular velocity correction in gyro.h: yawZeroAngV_, yawTotal_, correction = ");
           DEBUG_SERIAL_PORT.print(yawZeroAngV_);
           DEBUG_SERIAL_PORT.print(", ");
           DEBUG_SERIAL_PORT.print(yawTotal_);
           DEBUG_SERIAL_PORT.print(", ");
           DEBUG_SERIAL_PORT.println(correction);
        }          
        *pidResult += correction;
      }
      else firstPassZeroAngV_ = true; // w got a turn command, so we reset for use next time there is a linear move command
      
      #ifdef DEBUG_SERIAL_PORT
      //if (DEBUG)
        {       
          DEBUG_SERIAL_PORT.print("total yaw (degrees), current angular velocity in degs/sec = " );
          DEBUG_SERIAL_PORT.print(yawTotal_);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(currentAngV);
          DEBUG_SERIAL_PORT.print("commanded angular vel and pidResult = ");
          DEBUG_SERIAL_PORT.print(commandedAngV);
          DEBUG_SERIAL_PORT.print(", ");
          DEBUG_SERIAL_PORT.println(*pidResult);       
        }
      #endif
      
   }
 

  private:
    L3G gyro_;
    LSM303 accel_;
    float direction_adjustment_, gyroCorrectionTotal_;
    double gyroBaseline_;
    double xAccelBaseline_, yAccelBaseline_, zAccelBaseline_ ;
    double previousAngV_, angVelIntegralError_;
    double kpGyro_, kiGyro_, kdGyro_;
    double yawTotal_;
    unsigned long previousTime_;
    double yawZeroAngV_;
    boolean firstPassZeroAngV_;
    pidControl *pidGyro_;
    
} gyro_stabilizer;


// Initializes the gyro module.
// Automatically called by setup().
class GyroModuleSetup : public ModuleSetup {
  public:
    virtual void setup() {
            
      DEBUG_SERIAL_PORT.println("in gyro setup");
      
      
      gyro_stabilizer.init();
    }
} gyro_setup;

}  // namespace rcs

#endif  //GYRO_H_


