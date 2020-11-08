#ifndef COMMANDARBITRATOR
#define COMMANDARBITRATOR

//#include "customdatatypes.h"
#include <rosrccar_messages/VehicleCommand.h>

class CommandArbitrator {
  protected:
    bool emergency_off;
  public:
    rosrccar_messages::VehicleCommand arbitrate(rosrccar_messages::VehicleCommand &rccommand, rosrccar_messages::VehicleCommand &roscommand) {
      // rccommand contains RC throttle and steering as well as previous command modes
      // manual, automated, off --> "limited" to be done
      rosrccar_messages::VehicleCommand output;

      // emergency off
      if((roscommand.operationmode_lon!=manual)&&(rccommand.operationmode_lon!=manual)&&(rccommand.target_lon<-500)) {
        emergency_off = true;
      }
      
      if(emergency_off) {
        if(roscommand.operationmode_lon==manual&&roscommand.operationmode_lat==manual) { // reset emergency off
          emergency_off = false;
        }
        output.target_lon = 0;
        output.target_lat = 0;
        output.operationmode_lat = off;
        output.operationmode_lon = off;
      }
      else { // otherwise ROS command overrides RC
        output.operationmode_lon = roscommand.operationmode_lon;
        output.operationmode_lat = roscommand.operationmode_lat;
        if(output.operationmode_lon==manual) {
          output.target_lon = rccommand.target_lon;
        } else if(output.operationmode_lon==automated) {
          output.target_lon = roscommand.target_lon;
        }
        if(output.operationmode_lat==manual) {
          output.target_lat = rccommand.target_lat;
        } else if(output.operationmode_lat==automated) {
          output.target_lat = roscommand.target_lat;
        }
      }
    }
};

#endif
