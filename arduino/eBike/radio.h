#ifndef RADIO_H_
#define RADIO_H_



namespace rcs {
  
int previousRightRadioDirection_ = 1, previousLeftRadioDirection_ = 1;
bool previouslyRadioStopped_ = false;

// True if radio control is on.
boolean radio_control_allowed = ENABLE_RADIO_ON_STARTUP;

void radioInterruptHandler0();
void radioInterruptHandler1();
//void radioInterruptHandler4();

// RadioInterruptHandler can be configuered to respond to radio signal interrupts at the specified pin.
class RadioInterruptHandler {
  public:
    // Interrupts must be supported on the specified pins.
    // See attach() method for supported pins.
    explicit RadioInterruptHandler(int pin)
      : pin_(pin),
        state_(0),
        RadioCommandMode_(false),  
        high_micros_(0),
        high_duration_micros_(0) {
    }

    // Tries to attach the interrupt handler and returns true on success.
    // The radio uses pins 2 and 3 (interrupts 0,1) for movement.
    // Interrupt pins 18 (interrupt 5) and 19 (interrupt 4) are available
    // Note that interrupt pins 20 and 21 (interrupts 3 and 2) are on the IC2 pins SDA and SCL.
    boolean attach() {
      int interrupt_number = interruptNumber(pin_);
      switch (interrupt_number) {
        case 0: attachInterrupt(interrupt_number, radioInterruptHandler0, CHANGE); break; // pin 2
        case 1: attachInterrupt(interrupt_number, radioInterruptHandler1, CHANGE); break; // pin 3
        case 2: attachInterrupt(interrupt_number, radioInterruptHandler0, CHANGE); break; // pin 21
        case 3: attachInterrupt(interrupt_number, radioInterruptHandler1, CHANGE); break; // pin 20
        case 4: attachInterrupt(interrupt_number, radioInterruptHandler0, CHANGE); break; // pin 19
        case 5: attachInterrupt(interrupt_number, radioInterruptHandler1, CHANGE); break; // pin 18
        default: return false;
      }
      return true;
    }

    // Tries to detach the interrupt handler and returns true on success.
    boolean detach() {
      int interrupt_number = interruptNumber(pin_);
      if (interrupt_number < 0) return false;
      detachInterrupt(interrupt_number);
      return true;
    }

    // Returns the signal value in the range -1.0 .. +1.0.
    // This corresponds to the displacement of the joystick in the respective direction.
    float getJoystickPosition() {
      if (high_duration_micros_ < RADIO_MIN_HIGH_DURATION_MICROS) return 0.0f;
      float position;
      if (high_duration_micros_ > RADIO_NEGATIVE_START_MICROS && high_duration_micros_ < RADIO_POSITIVE_START_MICROS) {
        position = 0.0f;
      } else if (high_duration_micros_ >= RADIO_POSITIVE_START_MICROS) {
        position = (high_duration_micros_ - RADIO_POSITIVE_START_MICROS) / RADIO_POSITIVE_RANGE_MICROS;
        if (position > 1.0f) position = 1.0f;
      } else {
        // high_duration <= RADIO_NEGATIVE_START
        position = (RADIO_NEGATIVE_START_MICROS - high_duration_micros_) / -RADIO_NEGATIVE_RANGE_MICROS;
        if (position < -1.0f) position = -1.0f;
      }
      return position;
    }

    // Handles speed interrupts.
    void handle() {
      switch (digitalRead(pin_)) {
        case HIGH:
          if (state_ == 0) {
            high_micros_ = micros();
            state_ = 1;
          }
          break;
        case LOW:
          if (state_ == 1) {
            high_duration_micros_ = micros() - high_micros_;
            if (high_duration_micros_ < 0) {
              // clock overflow
              high_duration_micros_ = micros() + (4294967295 - high_micros_);
            }
            state_ = 0;
          }
          break;
      }
    }
    
    void setRadioCommandMode(boolean value) { RadioCommandMode_ = value;  } 
    boolean getRadioCommandMode() { return RadioCommandMode_;  }

  private:
    int pin_;
    int state_;
    boolean RadioCommandMode_;
    unsigned long high_micros_;  // time when the signal goes HIGH in micros
    unsigned long high_duration_micros_;  // duration of high signal
};

RadioInterruptHandler radio_joystick_x_handler(RADIO_JOYSTICK_X_PIN);  // turning
RadioInterruptHandler radio_joystick_y_handler(RADIO_JOYSTICK_Y_PIN);  // forward and back
//RadioInterruptHandler radio_pause_handler(RADIO_PAUSE_PIN);

// Interrupt handler for interrupt 0.
void radioInterruptHandler0() {
  radio_joystick_y_handler.handle();
}

// Interrupt handler for interrupt 1.
void radioInterruptHandler1() {
  radio_joystick_x_handler.handle();
}

// Interrupt handler for interrupt 4.
//void radioInterruptHandler4() {
//  radio_pause_handler.handle();
//}


// radio(on|off);
// Turns radio control on or off.
class Radio : public Instruction {
  public:
    Radio() : Instruction("radio") {}

    virtual boolean run(String* args) {
      String arg;
      popToken(args, &arg);
      return (*this)(arg == "on");
    }

    boolean operator()(boolean on) {
      radio_control_allowed = on;
      return true;
    }
} radio;


// Initializes the radio module.
// Automatically called by setup().
class RadioModuleSetup : public ModuleSetup {
  public:
    virtual void setup() {
      pinMode(RADIO_JOYSTICK_X_PIN, INPUT);
      pinMode(RADIO_JOYSTICK_Y_PIN, INPUT);
      pinMode(RADIO_PAUSE_PIN, INPUT);  // this is connected via a pololu RC switch, so we do not have to use an interrupt to read it
      radio_joystick_x_handler.attach();
      radio_joystick_y_handler.attach();
      //radio_pause_handler.attach();
    }
} radio_setup;

// Runs the radio loop.
// Automatically called by loop().
class RadioModuleLoop : public ModuleLoop {
  public:
    virtual void loop() {
      if (radio_control_allowed) {       
        float x = radio_joystick_x_handler.getJoystickPosition();  // gets a value in the range -1 to 1
        float y = radio_joystick_y_handler.getJoystickPosition();
        
        y *=  RC_LINEAR_LIMITER;  // we limit how fast we can go forward or reverse
        x *=  RC_ANGLULAR_LIMITER; // we limit how fast we can turn
        float total_abs_velocity = fabs(x) + fabs(y);
        if (total_abs_velocity < RADIO_MIN) {    // below the RADIO_MIN threshold is considered zero input for a channel
          if ((radio_joystick_x_handler.getRadioCommandMode() || radio_joystick_y_handler.getRadioCommandMode()) && (!previouslyRadioStopped_))
          {
            robot_base.stop(); // only stop if we are
                                // currently in RadioCommandMode.  Otherwise this stop will interfere with other command modes.
            previouslyRadioStopped_ = true;
            motor_dac[RIGHT].setMotorSpeed(DAC_MIN_VALUE);
            motor_dac[LEFT].setMotorSpeed(DAC_MIN_VALUE);
            #ifdef DEBUG_SERIAL_PORT
            if (DEBUG)
            {             
               DEBUG_SERIAL_PORT.print("Stopped for RC values, x, y = ");
               DEBUG_SERIAL_PORT.print(x);
               DEBUG_SERIAL_PORT.print(", ");
               DEBUG_SERIAL_PORT.println(y);
               DEBUG_SERIAL_PORT.print("Motor speeds = ");
               DEBUG_SERIAL_PORT.print(motor_dac[0].getSpeed());
               DEBUG_SERIAL_PORT.print(", ");
               DEBUG_SERIAL_PORT.println(motor_dac[1].getSpeed());              
            }
            #endif // DEBUG_SERIAL_PORT
            radio_joystick_x_handler.setRadioCommandMode(false);
            radio_joystick_y_handler.setRadioCommandMode(false);
          }
          return;
        }
        // got a commanding value for x or y, so proceed here
        previouslyRadioStopped_ = false;
        radio_joystick_x_handler.setRadioCommandMode(true);
        radio_joystick_y_handler.setRadioCommandMode(true);
        robot_base.setAutonomousCommandMode(false); // if an auto cmd comes in, treat it as a new command and also do not run checkSpeed
        robot_base.setAutoMoveMode(false);
        
        ping.resetTimeout();
        if (fabs(x) < RADIO_MIN/2) x = 0.;
        if (fabs(y) < RADIO_MIN/2) y = 0.;
        //float linear_velocity = y * MAX_RC_LINEAR_VEL; // this is often much faster than the standard max, since it is under direct control
        //float angular_velocity = -x * MAX_RC_ANGULAR_VEL;
              
        
        float yNorm = (y * y) / total_abs_velocity;
        float xNorm = (x * x ) / total_abs_velocity;
        // alternative method: we will fully command turns and, if there is any available extra velocity, use it for forward and reverse.
        //if (total_abs_velocity > 1.0) y = (1.0 - abs(x)) * sign(y);
        if (y < 0) yNorm = -yNorm;
        if (x < 0) xNorm = -xNorm;
        float dacRange = ((float) DAC_MAX_VALUE) - ((float) DAC_LOWER_VALUE);
        float left_speed = ((y + x) * dacRange);
        float right_speed = ((y - x) * dacRange);
        
        if (!robotPause_)
        {        
          // ebike motor controllers remember the last direction moved and start up going that way, so we need to goose them when the direction changes
          if  ( ((right_speed * motor_dac[RIGHT].getMotorDirection() < 0 ) && fabs(right_speed) > 0) 
             ||  ((left_speed * motor_dac[LEFT].getMotorDirection() < 0 ) && fabs(left_speed) > 0 ) )
          {
            if (DEBUG)
            {
              DEBUG_SERIAL_PORT.print("radio changed direction, now going left, right = ");
              if (left_speed > 0) DEBUG_SERIAL_PORT.print("forward");
              else DEBUG_SERIAL_PORT.print("reverse");
              DEBUG_SERIAL_PORT.print(", ");
              if (right_speed > 0) DEBUG_SERIAL_PORT.println("forward");
              else DEBUG_SERIAL_PORT.println("reverse");
            }
            
            int newDirection = rcs::sign(right_speed);
            motor_dac[RIGHT].setMotorDirection(newDirection);
            newDirection = rcs::sign(left_speed);
            motor_dac[LEFT].setMotorDirection(newDirection);
            delay(100);

            motor_dac[RIGHT].setMotorSpeed(GOOSE_SPEED * rcs::sign(right_speed)); 
            motor_dac[LEFT].setMotorSpeed(GOOSE_SPEED * rcs::sign(left_speed)); 
            delay(10);
          }   
  
          //if (abs(right_speed) > 0) motor_dac[RIGHT].setMotorSpeed(GOOSE_SPEED * rcs::sign(right_speed)); // send eike controllers a large speed to get going
          //if (abs(left_speed) > 0) motor_dac[LEFT].setMotorSpeed(GOOSE_SPEED * rcs::sign(left_speed)); // and then reduce it to the commanded speed
          //delay(50);
          
          if ( fabs(right_speed) > 0) previousRightRadioDirection_ = rcs::sign(right_speed);  // the ebike motor controller remembers the last non-zero direction, so we have to too
          if ( fabs(left_speed) > 0) previousLeftRadioDirection_ = rcs::sign(left_speed);  
          
  
            if (fabs(right_speed + (DAC_LOWER_VALUE * rcs::sign(right_speed))) < DAC_MIN_VALUE) motor_dac[RIGHT].setMotorSpeed(DAC_MIN_VALUE);
            else motor_dac[RIGHT].setMotorSpeed(right_speed + (DAC_LOWER_VALUE * rcs::sign(right_speed)));
            if (fabs(left_speed + (DAC_LOWER_VALUE * rcs::sign(left_speed))) < DAC_MIN_VALUE) motor_dac[LEFT].setMotorSpeed(DAC_MIN_VALUE);
            else motor_dac[LEFT].setMotorSpeed(left_speed + (DAC_LOWER_VALUE * rcs::sign(left_speed)));
        }
        
        
        if (DEBUG)
        {
            DEBUG_SERIAL_PORT.print("left and right speed, x, y: ");
            DEBUG_SERIAL_PORT.print(left_speed);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.print(right_speed);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.print(x);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.println(y);     
        } 
            
      }
    }
} radio_loop;

}  // namespace rcs

#endif  // RADIO_H_

