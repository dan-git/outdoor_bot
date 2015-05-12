#ifndef RADAR_RANGER_H
#define RADAR_RANGER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../include/rcmRadar/rcmIf.h"
#include "../include/rcmRadar/rcm.h"

class radarRanger
{
   private:
      char str[100];
      int destNodeId_;
      int status_;
      bool initStatus_;
      rcmIfType   rcmIf_;
      rcmConfiguration rcmConfig_;
      rcmMsg_GetStatusInfoConfirm statusInfo_;
      rcmMsg_FullRangeInfo rangeInfo_;
      rcmMsg_DataInfo dataInfo_;
      rcmMsg_ScanInfo scanInfo_;
      rcmMsg_FullScanInfo fullscanInfo_;
      RCMInfo rcmInfo_;

   public:
      radarRanger(const char* radarPortString);
      ~radarRanger();
      int getRange(int destNode);
      bool getInitStatus();
};

#endif // RADAR_RANGER_H
