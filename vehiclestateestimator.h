#ifndef VEHICLESTATEESTIMATOR
#define VEHICLESTATEESTIMATOR

#include <rosrccar_messages/VehicleState.h>
#define ACCELERATION_SCALE (9.81/8196)

// wheel circumference approximately 21 cm
// roughly 24 ticks per wheel revolution
#define RATIO_SHAFT_SPEED 0.206*11/34
#define RATIO_SHAFT_ENGINE 58/24
#define usec_TO_sec 1e-6

// States to estimate:
//int16 yaw_rad_e3        # yaw                       in radians         times 1000 
//int16 slipangle_rad_e3  # slip angle                 in radians         times 1000
//int16 wheelspeed_mps_e3 # wheelspeed                in meters/second   times 1000
//int16 yawrate_radps_e3  # yawrate                   in radians/second  times 1000 
//int16 acc_x_mps2_e3     # longitudinal acceleration in meters/second^2 times 1000
//int16 acc_y_mps2_e3     # lateral acceleration      in meters/second^2 times 1000
//int16 acc_command_e3    # accelerator command       between -1 and 1   times 1000
//int16 steer_command_e3  # steering command          between -1 and 1   times 1000

static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

class VehicleStateEstimator {
  public:
    VehicleStateEstimator(){}
    void update(VehicleMeasurement& measurements, unsigned int delta_time_microseconds);
    rosrccar_messages::VehicleState& getstate(){
      return state;
    }
    rosrccar_messages::VehicleState state;
  private:
    int driving_direction;
};

void VehicleStateEstimator::update(VehicleMeasurement& measurements, unsigned int delta_time_microseconds) {
  state.yawrate_radps_e3 = (measurements.yaw_rad*1e3-state.yaw_rad_e3)*50;//needs some investigation
  state.yaw_rad_e3 = int(measurements.yaw_rad*1e3);
  state.slipangle_rad_e3 = 0;
  state.wheelspeed_mps_e3 = int(RATIO_SHAFT_SPEED*1e3*measurements.driveshaftspeed_radps);
  state.acc_x_mps2_e3 = int(measurements.accelerationx_mps2*1e3);
  state.acc_y_mps2_e3 = int(measurements.accelerationy_mps2*1e3);
//  state.acc_command_e3 = int(measurements.rcaccelerator*1e3);
//  state.steer_command_e3 = int(measurements.rcsteering*1e3);
}
#endif
