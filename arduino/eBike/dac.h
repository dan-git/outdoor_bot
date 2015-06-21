// transmitting over I2C to the dac
// first send the address, our addresses will be 0x60, 0x61, 0x62, 0x63
// 60 and 61 are for the ones from sparkfun, 62 and 63 for the ones from adafruit
// next send the command
// for high speed transmission, the high byte = 0 and the low byte = the 4 MSB; the next word is the 8 LSB
// for normal speed transmissioon, the high byte = 4, low byte = 0, the next word is the 8 MSB,
//        the next word high byte = 4 LSB, low byte = 0
// for normal speed transmission and writing to the EEPROM to remember this voltage level and go to it when
//        powered on, is the same as regular normal speed transmission except the high byte = 6 instead of 4

// note!  there is a note saying if the address is pulled high then the second byte 
//          of the first word = 1 instead of 0

// can significantly increase the I2C transmission rate by using TWBR to set the I2C frequency = 400 KHz

//#define MCP4726_CMD_WRITEDAC            (0x40)  // Writes data to the DAC
//#define MCP4726_CMD_WRITEDACEEPROM      (0x60)  // Writes data to the DAC and the EEPROM (persisting the assigned value after reset)
//For devices with A0 pulled HIGH, use 0x61

// For Adafruit boards (MCP4725A1) the I2C address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
// For Sparkfun boards (MCP4725A0), the I2Caddress is 0x60 (default) or 0x61 (ADDR jumper tied to VCC)
  
// working range of Smart Pie controller is from around 1.2 to 3.5 V
// it seems to need to see around 0.9V to override the safety cutoff for a disconnected throttle.
// when the throttle is at 0 setting, we see about 0.867 V, at full setting, around 3.63V

#ifndef DAC_H_
#define DAC_H_

// Represents a digital to analog converter.
class dac {
  public:
    dac(int i2c_address) {    
    //i2cDev_ = new I2Cdevice(i2c_address);
    i2caddr_ = i2c_address;     
    } 

  bool setVoltage( int output )  // value range is 0 - 4095 (12 bit resolution)
  {
    if (output < DAC_MIN_VALUE || output > DAC_MAX_VALUE)
    {
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("out of range dac command sent, value =  ");
        DEBUG_SERIAL_PORT.println(output);
      }
      return false;
    }
    
    byte valuesToWrite[2];
    
    // normal mode
    valuesToWrite[0] = output >> 4;  // 8 MSB    (D11.D10.D9.D8.D7.D6.D5.D4) (dividing by 16, 2^4)
    valuesToWrite[1] = (output % 16) << 4; // 4 LSB put into high byte (D3.D2.D1.D0.x.x.x.x)
    // normal mode, write to EEPROM with a device having its A0 address line low
    //if (i2cDev_->writeToDevice(valuesToWrite, 2, 0x60))
    // normal mode, write to EEPROM with a device having its A0 address line high
    //if (i2cDev_->writeToDevice(valuesToWrite, 2, 0x61))
    // normal mode, do not write to EEPEOM
    int result = I2c.write(i2caddr_, 0x40, valuesToWrite, (byte)2);
    if (!result)  // result = 0 means no error
    //if (i2cDev_->writeToDevice(valuesToWrite, 0x02, 0x40))
    
    // fast mode
    //valuesToWrite[0] = output >> 8;  // the 4 most significant bits (dividing by 256, 2^8)
    //valuesToWrite[1] = output % 256;
    //if (i2cDev_->writeFastToDevice(valuesToWrite, 2))
    {
      
      if (DEBUG)
      {
        DEBUG_SERIAL_PORT.print("dac voltage on device ");
        DEBUG_SERIAL_PORT.print(i2caddr_); 
        DEBUG_SERIAL_PORT.print(" was set = ");
        DEBUG_SERIAL_PORT.println(output);
      }
      
      return true;
    }
    else
    {
      if (DEBUG) DEBUG_SERIAL_PORT.println("dac voltage set attempt failed with I2C error");
      return false;
    }
    return true;
  }
  

  private:
    byte i2caddr_;
}

dac[NUM_DACS] =  {DAC_I2C_RIGHT, DAC_I2C_LEFT};

#endif // DAC_H
