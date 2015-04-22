//_____________________________________________________________________________
//
// Copyright 2011-2 Time Domain Corporation
//
//
// rcmSampleApp.c
//
//   Sample code showing how to interface to P400 RCM module.
//
//   This code uses the functions in rcm.c to:
//      - make sure the RCM is awake and in the correct mode
//      - get the configuration from the RCM and print it
//      - get the status/info from the RCM and print it
//      - range to another RCM node
//      - broadcast that range in a data packet
//
// This sample can communicate with the RCM over Ethernet, the 3.3V serial port,
// or the USB interface (which acts like a serial port).
//
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// #includes 
//_____________________________________________________________________________

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/rcmRadar/rcmIf.h"
#include "../include/rcmRadar/rcm.h"


//_____________________________________________________________________________
//
// #defines 
//_____________________________________________________________________________

#define DEFAULT_DEST_NODE_ID    101


//_____________________________________________________________________________
//
// typedefs
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// static data
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// local function prototypes
//_____________________________________________________________________________

void usage(void)
{
    printf("usage: rcmSampleApp -i <IP address> | -s <COM port> | -u <USB COM port>\n");
    printf("\nTo connect to radio at IP address 192.168.1.100 via Ethernet:\n");
    printf("\trcmSampleApp -i 192.168.1.100\n");

#ifdef WIN32
    printf("\nTo connect to radio's serial port at COM3:\n");
    printf("\trcmSampleApp -s COM3\n");
    printf("\nTo connect to radio's USB port at COM10:\n");
    printf("\trcmSampleApp -u COM10\n");
#else
    printf("\nTo connect to radio's serial port at /dev/ttyUSB0:\n");
    printf("\trcmSampleApp -s /dev/ttyUSB0\n");
    printf("\nTo connect to radio's USB port at /dev/ttyACM0:\n");
    printf("\trcmSampleApp -u /dev/ttyACM0\n");
#endif

	exit(0);
}

class radarRanger
{
private:
   char str[100];
   int destNodeId_=DEFAULT_DEST_NODE_ID;
   int status_;
   rcmIfType   rcmIf_;
   rcmConfiguration rcmConfig_;
   rcmMsg_GetstatusInfoConfirm statusInfo_;
   rcmMsg_FullRangeInfo rangeInfo_;
   rcmMsg_DataInfo dataInfo_;
   rcmMsg_ScanInfo scanInfo_;
   rcmMsg_FullScanInfo fullscanInfo_;

public:
  radarRanger()
   {
      rcmIf_ = rcmIfUsb;
      // initialize the interface to the RCM
      if (rcmIfInit(rcmIf_, "/dev/ttyACM0") != OK)
      {
        printf("Initialization failed.\n");
        exit(0);
      }

      // Make sure RCM is awake
      if (rcmSleepModeSet(RCM_SLEEP_MODE_ACTIVE) != 0)
      {
        printf("Time out waiting for sleep mode set.\n");
        exit(0);
      }

      // Make sure opmode is RCM
      if (rcmOpModeSet(RCM_OPMODE_RCM) != 0)
      {
        printf("Time out waiting for opmode set.\n");
        exit(0);
      }

      // execute Built-In Test - verify that radio is healthy
      if (rcmBit(&status_) != 0)
      {
        printf("Time out waiting for BIT.\n");
        exit(0);
      }

      if (status_ != OK)
      {
        printf("Built-in test failed - status %d.\n", status_);
        exit(0);
      }
      else
      {
        printf("Radio passes built-in test.\n\n");
      }

      // retrieve config from RCM
      if (rcmConfigGet(&rcmConfig_) != 0)
      {
        printf("Time out waiting for config confirm.\n");
        exit(0);
      }

      // print out configuration
      printf("Configuration:\n");
      printf("\tnodeId: %d\n", rcmConfig_.nodeId);
      printf("\tintegrationIndex: %d\n", rcmConfig_.integrationIndex);
      printf("\tantennaMode: %d\n", rcmConfig_.antennaMode);
      printf("\tcodeChannel: %d\n", rcmConfig_.codeChannel);
      printf("\telectricalDelayPsA: %d\n", rcmConfig_.electricalDelayPsA);
      printf("\telectricalDelayPsB: %d\n", rcmConfig_.electricalDelayPsB);
      printf("\tflags: 0x%X\n", rcmConfig_.flags);
      printf("\ttxGain: %d\n", rcmConfig_.txGain);

      // retrieve status/info from RCM
      if (rcmStatusInfoGet(&statusInfo_) != 0)
      {
        printf("Time out waiting for status info confirm.\n");
        exit(0);
      }

      // print out status/info
      printf("\nStatus/Info:\n");
      printf("\tPackage version: %s\n", statusInfo_.packageVersionStr);
      printf("\tRCM version: %d.%d build %d\n", statusInfo_.appVersionMajor,
            statusInfo_.appVersionMinor, statusInfo_.appVersionBuild);
      printf("\tUWB Kernel version: %d.%d build %d\n", statusInfo_.uwbKernelVersionMajor,
            statusInfo_.uwbKernelVersionMinor, statusInfo_.uwbKernelVersionBuild);
      printf("\tFirmware version: %x/%x/%x ver %X\n", statusInfo_.firmwareMonth,
            statusInfo_.firmwareDay, statusInfo_.firmwareYear,
            statusInfo_.firmwareVersion);
      printf("\tSerial number: %08X\n", statusInfo_.serialNum);
      printf("\tBoard revision: %c\n", statusInfo_.boardRev);
      printf("\tTemperature: %.2f degC\n\n", statusInfo_.temperature/4.0);
   }

   ~radarRanger()
   {
      rcmIfClose();
   }

   // Determine range to a radio. May also get data and scan packets.
   int getRange()
   {
      if (rcmRangeTo(destNodeId_, RCM_ANTENNAMODE_TXA_RXA, 0, NULL,
          &rangeInfo_, &dataInfo_, &scanInfo_, &fullscanInfo_) == 0)
      {
         // we always get a range info packet
         printf("RANGE_INFO: responder %d, msg ID %u, range status %d, "
                 "stopwatch %d ms, channelRiseTime %d, vPeak %d, measurement type %d\n",
                 rangeInfo_.responderId, rangeInfo_.msgId, rangeInfo_.rangeStatus,
                 rangeInfo_.stopwatchTime, rangeInfo_.noise, rangeInfo_.vPeak,
                 rangeInfo_.rangeMeasurementType);

         // The RANGE_INFO can provide multiple types of ranges
         if (rangeInfo_.rangeMeasurementType & RCM_RANGE_TYPE_PRECISION)
         {
             printf("Precision range: %d mm, error estimate %d mm\n",
                     rangeInfo_.precisionRangeMm, rangeInfo_.precisionRangeErrEst);
         }

         if (rangeInfo_.rangeMeasurementType & RCM_RANGE_TYPE_COARSE)
         {
             printf("Coarse range: %d mm, error estimate %d mm\n",
                     rangeInfo_.coarseRangeMm, rangeInfo_.coarseRangeErrEst);
         }

         if (rangeInfo_.rangeMeasurementType & RCM_RANGE_TYPE_FILTERED)
         {
             printf("Filtered range: %d mm, error estimate %d mm\n",
                     rangeInfo_.filteredRangeMm, rangeInfo_.filteredRangeErrEst);
             printf("Filtered velocity: %d mm/s, error estimate %d mm/s\n",
                     rangeInfo_.filteredRangeVel, rangeInfo_.filteredRangeVelErrEst);
         }


         // only get a data info packet if the responder sent data
         // dataSize will be non-zero if we there is data
         if (dataInfo_.dataSize)
             printf("DATA_INFO from node %d: msg ID %u, channelRiseTime %d, vPeak %d, %d bytes\ndata: %*s\n",
                     dataInfo_.sourceId, dataInfo_.msgId, dataInfo_.noise, dataInfo_.vPeak,
                     dataInfo_.dataSize, dataInfo_.dataSize, dataInfo_.data);

         // only get a scan info packet if the SEND_SCAN bit is set in the config
         // numSamples will be non-zero if there is scan data
         // we don't do anything with the scan data itself here
         if (scanInfo_.numSamples)
             printf("SCAN_INFO from node %d: msg ID %u, %d samples, channelRiseTime %d, vPeak %d\n",
                     scanInfo_.sourceId, scanInfo_.msgId, scanInfo_.numSamples,
                     scanInfo_.noise, scanInfo_.vPeak);

         // only get a full scan info packet if the FULL_SCAN bit is set in the config
         // numSamplesInMessage will be non-zero if there is scan data
         // we don't do anything with the scan data itself here
         if (fullscanInfo_.numSamplesInMessage)
             printf("FULL_SCAN_INFO from node %d: msg ID %u, %d samples, channelRiseTime %d, vPeak %d\n",
                     fullscanInfo_.sourceId, fullscanInfo_.msgId, fullscanInfo_.numSamplesInMessage,
                     fullscanInfo_.noise, fullscanInfo_.vPeak);

         // a rangeStatus of 0 means the range was calculated successfully
         if (rangeInfo_.rangeStatus == 0)
         {
             // now broadcast the range in a data packet
             sprintf(str, "The range from %d to %d is %d mm.",
                     rcmConfig_.nodeId, destNodeId_,
                     rangeInfo_.precisionRangeMm);
             rcmDataSend(RCM_ANTENNAMODE_TXA_RXA, strlen(str), str);
         }
      }
   }
};


/*
int main(int argc, char *argv[])
{
   //Initialize ROS
   ros::init(argc, argv, "radarRanger");
   ros::NodeHandle nh;
   ROS_INFO("radarRanger starting");
   radarRanger rR(nh);
   cout << "range = " << rR.getRange() << endl;
   // perform cleanup
   return 0;
}
*/

