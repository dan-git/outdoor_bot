/*
   Using three LS7366 Breakout boards from adafruit
   
   SPI  pins on arduino mega:
            MOSI   -------------------   SDO (pin 51)
            MISO   -------------------   SDI (pin 50)
            SCK    -------------------   SCK (pin 52)			
*/

#ifndef ENCODER_SPI_H
#define ENCODER_SPI_H


class encoder_spi
{
  public:
    encoder_spi(int selectPin)
    {
      SPI.begin();
      selectPin_ = selectPin;
      encoderCount_ = 0;  // clears our value
      clearEncoderCount();  // clears the values on the encoder board
      prevTicks_ = 0;
      totalCount_ = 0;
      prevTotalCount_ = 0;
      pinMode(selectPin_, OUTPUT);
      digitalWrite(selectPin_, HIGH);  // Set select pins HIGH, dropping to LOW selects this encoder for comm
      pinMode(REVERSE_ENCODER_LEFT_PIN,OUTPUT);
      pinMode(REVERSE_ENCODER_RIGHT_PIN, OUTPUT);
      digitalWrite(REVERSE_ENCODER_LEFT_PIN, LOW);
      digitalWrite(REVERSE_ENCODER_RIGHT_PIN, LOW);
    }
    
    void setEncoderMode_SPI(boolean quadratureMode = true)
    {
      // Initialize encoder with these values:
      //    Clock division factor: 0
      //    Negative index input
      //    free-running count mode
      //    x4 quatrature count mode (four counts per quadrature cycle)
      // NOTE: For more information on commands, see datasheet
      
      digitalWrite(selectPin_, LOW);       // Begin SPI conversation
      SPI.transfer(0x88);                  // Write to MDR0
      if (quadratureMode)  SPI.transfer(0x01);                  // Configure to one count per quadrature cycle, index disabled
      else SPI.transfer(0x00); // non quadrature mode, A = clock, B = direction, index disabled
      digitalWrite(selectPin_,HIGH);       // Terminate SPI conversation
      delayMicroseconds(100); // guarantee a short delay between SPI conversations
    }
    
    long readEncoder() 
    {
      // Initialize temporary variables for SPI read
      unsigned int count_1, count_2, count_3, count_4;
      long count_value;  
      digitalWrite(selectPin_,LOW);      // Begin SPI conversation
      SPI.transfer(0x60);                // Request count
      count_1 = SPI.transfer(0x00);      // Read highest order byte
      count_2 = SPI.transfer(0x00);           
      count_3 = SPI.transfer(0x00);           
      count_4 = SPI.transfer(0x00);      // Read lowest order byte
      digitalWrite(selectPin_,HIGH);     // Terminate SPI conversation 
     
      // Calculate encoder count
      count_value = (count_1 << 8) + count_2;
      count_value = (count_value << 8) + count_3;
      count_value = (count_value << 8) + count_4;
      
      if (count_value > prevTicks_ + 5 || count_value < prevTicks_ - 5)
      {
        DEBUG_SERIAL_PORT.print("noisy encoder on selectPin = ");
        DEBUG_SERIAL_PORT.print(selectPin_);
        DEBUG_SERIAL_PORT.print(", corresponding to the ");
        if (selectPin_ == RIGHT_ENCODER_SELECT_PIN) 
        {
    //      totalCount_ += rcs::sign(motor_dac[RIGHT].getMotorDirection());
          DEBUG_SERIAL_PORT.println("right wheel motor encoder");
        }
        else if (selectPin_ == LEFT_ENCODER_SELECT_PIN)
        {
          DEBUG_SERIAL_PORT.println("left wheel motor encoder");
   //       totalCount_ += rcs::sign(motor_dac[LEFT].getMotorDirection());
        }
        else 
        {
          DEBUG_SERIAL_PORT.println("one of the pololu motor encoders");
  //        totalCount_ += rcs::sign(count_value - prevTicks_);  // when there is noise, just increment by 1
        }
      
        DEBUG_SERIAL_PORT.print("count value, prevTicks, difference = ");
        DEBUG_SERIAL_PORT.print(count_value);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.print(prevTicks_);
        DEBUG_SERIAL_PORT.print(", ");
        DEBUG_SERIAL_PORT.println(count_value - prevTicks_);
       
      }
      //else 
      //{      
      //   totalCount_ += (count_value - prevTicks_);
         //if ( selectPin_ == RIGHT_ENCODER_SELECT_PIN) totalCount_ += (count_value - prevTicks_); // * rcs::sign(motor_dac[RIGHT].getMotorDirection());
        //else if (selectPin_ == LEFT_ENCODER_SELECT_PIN) totalCount_ += (count_value - prevTicks_);//* rcs::sign(motor_dac[LEFT].getMotorDirection());
     // } 
        
          /*
          if (DEBUG)
          {
            DEBUG_SERIAL_PORT.print("encoder = ");
            DEBUG_SERIAL_PORT.print(totalCount_);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.print(count_value);
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.print(prevTicks_);        
            DEBUG_SERIAL_PORT.print(", ");
            DEBUG_SERIAL_PORT.println(motorDirection_);
          }
          */
        else totalCount_ += count_value - prevTicks_;  // quadrature motors seem OK with directions
      
      prevTicks_ = count_value;
      
      return totalCount_;
    }
    
    int deltaTicks()
    {
      long ticks = readEncoder();
      long deltaTicks = ticks - prevTotalCount_;
      prevTotalCount_ = ticks;      
      return deltaTicks; 
    }     
    
    void clearEncoderCount()
    {   
      // Set data register to 0
      digitalWrite(selectPin_,LOW);      // Begin SPI conversation  
      // Write to DTR
      SPI.transfer(0x98);    
      // Load data
      SPI.transfer(0x00);  // Highest order byte
      SPI.transfer(0x00);           
      SPI.transfer(0x00);           
      SPI.transfer(0x00);  // lowest order byte
      digitalWrite(selectPin_,HIGH);     // Terminate SPI conversation   
      delayMicroseconds(100);  // guarantee a short delay between SPI conversations
      
      // Set current data register to center
      digitalWrite(selectPin_,LOW);      // Begin SPI conversation  
      SPI.transfer(0xE0);    
      digitalWrite(selectPin_,HIGH);     // Terminate SPI conversation   
      delayMicroseconds(100);  // guarantee a short delay between SPI conversations
    }  
  
  private:
    int selectPin_;
    long encoderCount_, prevTicks_, totalCount_, prevTotalCount_;
}

encoder_spi[NUM_SPI_ENCODERS] = {RIGHT_ENCODER_SELECT_PIN, LEFT_ENCODER_SELECT_PIN,
                                  PICKER_UPPER_ENCODER_SELECT_PIN, BIN_SHADE_ENCODER_SELECT_PIN,
                                  DROP_BAR_ENCODER_SELECT_PIN, EXTRA_ENCODER_SELECT_PIN};
                            
int getEncoderNumber(int selectPinValue)
{
  if (selectPinValue == RIGHT_ENCODER_SELECT_PIN) return 0;
  if (selectPinValue == LEFT_ENCODER_SELECT_PIN) return 1;
  if (selectPinValue == PICKER_UPPER_ENCODER_SELECT_PIN) return 2;
  if (selectPinValue == BIN_SHADE_ENCODER_SELECT_PIN) return 3;
  if (selectPinValue == DROP_BAR_ENCODER_SELECT_PIN) return 4;
  if (selectPinValue == EXTRA_ENCODER_SELECT_PIN) return 5;
  Serial.println("ERROR:  unknown PD driver number sent to getEncoderNumber!");
  return 0;
}

long distanceTraveled()
{
  long rightTicks = encoder_spi[getEncoderNumber(RIGHT_ENCODER_SELECT_PIN)].readEncoder();
  long leftTicks = encoder_spi[getEncoderNumber(LEFT_ENCODER_SELECT_PIN)].readEncoder();
  return (long) ( ((double) (rightTicks + leftTicks)) * ((double) MM_PER_TICK_EBIKE) / 2.);
}

long deltaTicks()
{
  long rightTicks = encoder_spi[getEncoderNumber(RIGHT_ENCODER_SELECT_PIN)].deltaTicks();
  long leftTicks = encoder_spi[getEncoderNumber(LEFT_ENCODER_SELECT_PIN)].deltaTicks();
  long deltaTicks = (rightTicks + leftTicks) / 2;
  return deltaTicks;
}
    
  
  
      
#endif //ENCODER_SPI_H
