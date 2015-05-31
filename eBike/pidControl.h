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
 *                          pid Control
 *
 *  Provides pid control for dynamic control of both base and arm movements
 **************************************************************************/


#ifndef PIDCONTROL_H_
#define PIDCONTROL_H_

namespace rcs {
  
class pidControl {
  public:
   pidControl(double kp, double ki, double kd, double maxChange, double maxOutput)
   {
     kp_ = kp;
     ki_ = ki;
     kd_ = kd;
     maxChange_ = maxChange;
     maxOutput_ = maxOutput;
     previousMeasuredValue_ = 0.0;
     previousPreviousMeasuredValue_ = 0.0;
   }
   
   void calc( double *output, double setPoint, double measuredValue, unsigned long deltaT) 
   {
      double error = setPoint - measuredValue;
      if (deltaT < 5) 
      {
        if ( DEBUG )
        {
          DEFAULT_SERIAL_PORT.print("pidControl not done because it was called with very small deltaT = ");
          DEFAULT_SERIAL_PORT.println(deltaT);
        }
        previousPreviousMeasuredValue_ = previousMeasuredValue_;
        previousMeasuredValue_ = measuredValue;  // update since deltaT will update and we want the next call to be right
        return;
      }
  
      double integralError = ki_ * error * deltaT / 1000.0f; 
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("pid, ki, error, deltaT, intError = ");
        DEBUG_SERIAL_PORT.print(ki_);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.print(error);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.print(deltaT);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(integralError);
      }

      // we use "derivative on measurement to eliminate derivative kick that comes when the setpoint changes
      // In our case it is present in both the proportional and the derivative terms
      // It is present in the proportional turn because we are using velocity control, where the proportional term uses error change instead of just error
      // 
      // recall that dError/dt = dSetpoint/dt - dMeasured/dt
      // so instead of the normal pid term of adding kd * dError,
      // we subtract kd * dMeasured  
      
      double dMeasured = measuredValue - previousMeasuredValue_;
      double dDoubleMeasured = (measuredValue + previousPreviousMeasuredValue_ - (2.0 * previousMeasuredValue_) ) * 1000.0f / deltaT;
      
      // unlike position pid, the proportional term uses the change in error, not the error
      // and here we use change in measurement, to avoid derivative kick in the proportional term
      // remember that it has to be the negative of the measurement difference, hence the
      // subtraction of the proportional term.
      // Of course, this leaves us with just the integral term responding to error directly
      // which is an issue to balance vs derivative kick.
      
      // compute the new output value:
      double outputChange = (integralError - (kp_ * dMeasured)) - (kd_ * dDoubleMeasured);
      
      // check boundaries
      if ( abs(outputChange) > maxChange_) outputChange = maxChange_ * sign(outputChange);
      double newOutput = outputChange + *output; // add to previous output
      if ( abs(newOutput) > maxOutput_) newOutput = maxOutput_ * sign(newOutput);
      

      
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("setPoint, measuredValue = ");
        DEBUG_SERIAL_PORT.print(setPoint);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(measuredValue);
        DEBUG_SERIAL_PORT.print("error, dMeasured, prop error, intError, dDoubleMeasured/dt, dervError: ");
        DEBUG_SERIAL_PORT.print(error);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.print(dMeasured);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.print(-kp_ * dMeasured);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.print(integralError);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.print(dDoubleMeasured);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(-kd_ * dDoubleMeasured);
        DEBUG_SERIAL_PORT.print("output change = ");
        DEBUG_SERIAL_PORT.println(outputChange);
        DEBUG_SERIAL_PORT.print("new output = ");
        DEBUG_SERIAL_PORT.println(newOutput);
      }
      
      *output = newOutput;      
      previousPreviousMeasuredValue_ = previousMeasuredValue_;
      previousMeasuredValue_ = measuredValue;  // update since deltaT will update and we want the next call to be right
    }   
    
    private:
      double kp_, ki_, kd_;
      double maxChange_, maxOutput_;
      double previousMeasuredValue_, previousPreviousMeasuredValue_;
};
} // ends namespace

#endif //PIDCONTROL_H_
