#ifndef CUSTOMDATATYPES
#define CUSTOMDATATYPES

struct VehicleState {
  float speed_mps;
  float wheelspeed_mps;
  float yawrate_radps;
  float accelerationx_mps2;
  float accelerationy_mps2;
  float slipangle_rad;
  
  float accelerator;
  float steering;
};

struct VehiclePosition {
  float x;
  float y;
  float yaw;
};

struct VehicleMeasurement {
  float opticalvelocityx_mps;
  float opticalvelocityy_mps;
  float driveshaftspeed_radps;
  float accelerationx_mps2;
  float accelerationy_mps2;
  float yaw_rad;
  float batteryvoltage_volt;
  unsigned int looptime_usec;
  float rcaccelerator;
  float rcsteering;
};

enum VehicleOperationMode {
  off,
  manual,
  manual_limitedspeed,
  manual_limitedslip,
  automated,
  pi_command_start_recording,
  pi_command_stop_recording,
  pi_command_shutdown
};

#endif
