// monitor battery with trimetric TM 2030-RV
// values are published at 2400,8,n,1 continuously from the unit's RT-45 jack, pin 4 (4th from the edge of the case)
// values published are:
// Volts 1, Volts 2, Filtered Volts 1, Amps, Filtered Amps, Amp-hr, PercentFull, Watts, Days since charged, days since equalized.
// example output:
// V=14.3,FV=14.3,V2=00.0,A=-0.11,FA=-0.11,PW=0FF,AH=-0.04,%=100,W=-1.57,DSC=0.01,DSE=0.01,PW=0FF,

#ifndef BAT_MON_H_
#define BAT_MON_H_

#ifdef BAT_MON_SERIAL_PORT
{
  
class batMon
{
   public:
      batMon()
      {
         maxAmps_ = 0.0;
         ampHours_ = 0.0;
         for (int i = 0; i < BAT_MON_BUFF_SIZE; i++)
         {
           buffMA_[i] = 0;
           buffAH_[i] = 0;
         }         
      }

      float getMaxAmps() { return maxAmps_; }
      float getAmpHours() { return ampHours_; }
      
      
      // this returns a good result if the values happen to be in the serial buffer
      // which happens most of the time.  The advantage is that it is 10x faster than 
      // getting a result every time
      void quickCheckValues(float *ampsValue, float *ampHoursValue)
      {
        boolean gotAmpsValue = false, gotAmpHoursValue = false;
        char buff[256], buffMA[BAT_MON_BUFF_SIZE], buffAH[BAT_MON_BUFF_SIZE], prevChar = 'x', prevPrevChar = 'x';
        int numChars = BAT_MON_SERIAL_PORT.available();
        if (numChars > 0) for (int i=0; i < numChars; i++) buff[i] = BAT_MON_SERIAL_PORT.read();
        else
        {
          *ampsValue = 0;
          *ampHoursValue = 0;
          return;
        }
        //DEBUG_SERIAL_PORT.println();
        for (int i=0; i < numChars; i++)
        {
          //if (buff[i] != ',') DEBUG_SERIAL_PORT.print(buff[i]);
          //else DEBUG_SERIAL_PORT.println();
          if (prevPrevChar == ',' && prevChar == 'A' && buff[i] == '=')  // found an amps entry
          {
            int numValues = 0;
            i++;
            while ( i < numChars && buff[i] != ',' && numValues < BAT_MON_BUFF_SIZE)
            {
              buffMA[numValues] = buff[i];
              i++;
              numValues++;
            }
            if (i < numChars && numValues < BAT_MON_BUFF_SIZE)
            {
              *ampsValue = atof(buffMA);
              if (abs(*ampsValue) > abs(maxAmps_)) maxAmps_ = *ampsValue;
              gotAmpsValue = true;
            }
          }
          if (prevPrevChar == 'A' && prevChar == 'H' && buff[i] == '=')  // found an amp hours entry
          {
            int numValues = 0;
            i++;
            while ( i < numChars && buff[i] != ',' && numValues < BAT_MON_BUFF_SIZE)
            {
              buffAH[numValues] = buff[i];
              i++;
              numValues++;
            }
            if (i < numChars && numValues < BAT_MON_BUFF_SIZE)
            {
              ampHours_ = atof(buffAH);
              *ampHoursValue = ampHours_;
              gotAmpHoursValue = true;
            }
          }
          prevPrevChar = prevChar;
          prevChar = buff[i];
        }
        if (!gotAmpsValue) *ampsValue = 0;
        if (!gotAmpHoursValue) *ampHoursValue = ampHours_;  // send the previously measured value
      }

      // this always returns a good result but is very slow
      void checkValues(float *ampsValue, float *ampHoursValue)
      {
        char buff[200]; 
        char inChar = 'x', prevInChar = 'x';
        float amps = 0.;
        int numChars = BAT_MON_SERIAL_PORT.available();
        if (numChars > 40) numChars = 40;
        if (numChars > 0) for (int i=0; i < numChars; i++) buff[i] = BAT_MON_SERIAL_PORT.read();
        prevInChar = inChar;
        unsigned long timeout = millis();
        while(prevInChar != '2' || inChar != '=' || millis() - timeout > 2000) // pickup V2, the one before amps
        {
          prevInChar = inChar;
          while (!BAT_MON_SERIAL_PORT.available());
          inChar = BAT_MON_SERIAL_PORT.read();          
        }
  
        // there are 12 parameters, but we dont have time to cycle through them all
        while (!BAT_MON_SERIAL_PORT.available());
        inChar = BAT_MON_SERIAL_PORT.read();
        buffMA_[0] = inChar;
        buffMAIndex_ = 0;
        buffAHIndex_ = 0;
        for (int i = 0; i < 5; i++)  // only read up to amp hours, ignore the rest
        {

          timeout = millis();
          while (inChar != ',' || millis() - timeout > 2000)
          {
            //if (DEBUG) DEBUG_SERIAL_PORT.print(inChar);
            while (!BAT_MON_SERIAL_PORT.available());
            inChar = BAT_MON_SERIAL_PORT.read();
            if (i == 1 && inChar != 'A' && inChar != '=' && inChar != ',')     // pick off the amps
            {
              if (buffMAIndex_ < BAT_MON_BUFF_SIZE) buffMA_[buffMAIndex_] = inChar;
              buffMAIndex_++;
            }
            if (i == 4 && inChar != 'A'  && inChar != 'H' && inChar != '=' && inChar != ',') // pick off the amp hours
            {
              if (buffAHIndex_ < BAT_MON_BUFF_SIZE) buffAH_[buffAHIndex_] = inChar;
              buffAHIndex_++;
            }
          }
          while (!BAT_MON_SERIAL_PORT.available()); // read the next character after ,
          inChar = BAT_MON_SERIAL_PORT.read();
          //if (DEBUG) DEBUG_SERIAL_PORT.println();
        }
        
        amps = atof(buffMA_);
        if (abs(amps) > abs(maxAmps_))  // goes negative when charging
        {
          maxAmps_ = amps;
          //if (DEBUG)
          //{
          //   DEBUG_SERIAL_PORT.print("new max amps = ");
          //   DEBUG_SERIAL_PORT.println(amps);
          //}
        }
        ampHours_ = atof(buffAH_);
        //if (DEBUG)
        //{
        //    DEBUG_SERIAL_PORT.print("max amps = ");
        //    DEBUG_SERIAL_PORT.println(maxAmps_);
        //    DEBUG_SERIAL_PORT.print("amp hours = ");
        //    DEBUG_SERIAL_PORT.println(ampHours_);
        //}
        
        for (int i = 0; i< BAT_MON_BUFF_SIZE; i++)
        {
          buffMA_[i] = 0;
          buffAH_[i] = 0;
        }
        //if (DEBUG) DEBUG_SERIAL_PORT.println();
        *ampsValue = amps;
        *ampHoursValue = ampHours_;
      }  
        
   private:
      char buffMA_[BAT_MON_BUFF_SIZE];
      char buffAH_[BAT_MON_BUFF_SIZE];
      float maxAmps_, ampHours_;
      int buffMAIndex_, buffAHIndex_; 
}
batt_mon;

#endif // ifdef BAT_MON_SERIAL_PORT
#endif // BAT_MON_H_

  
