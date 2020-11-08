#ifndef VEHICLESTATEESTIMATOR
#define VEHICLESTATEESTIMATOR

#include <rosrccar_messages/VehicleState.h>
#define ACCELERATION_SCALE (9.81/8196)
#define EXPONENTIALAVERAGE 0.995

// wheel circumference approximately 21 cm
// roughly 24 ticks per wheel revolution
#define RATIO_SHAFT_SPEED 0.206*11/34/TWO_PI
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

enum ESCModes {
  accelerate_forward=1,
  brake_forward,
  idle_forward,
  idle_reverse,
  brake_reverse,
  accelerate_reverse
};

int esc_transitiontable[6][3] = {
  2,3,1,
  2,4,1,
  2,3,1,
  6,4,1,
  5,3,5,
  6,4,5
};

int esc_statemachine(int throttle_e3, int speed_mps_e3, int esc_state) {
  int throttle_sign = 0;
  if(!((abs(speed_mps_e3)<100)||(abs(throttle_e3)<100))) {
    throttle_sign = throttle_e3>0 ? 1 : -1;
  }
  return esc_transitiontable[esc_state-1][throttle_sign+1];
}

int drivingdirection(int esc_state) {
  return esc_state>3? -1 : 1;
}

class VehicleStateEstimator {
  public:
    VehicleStateEstimator(): offset_ax(0), offset_ay(0), esc_state(idle_forward){}
    void update(VehicleMeasurement& measurements, unsigned int delta_time_microseconds);
    rosrccar_messages::VehicleState& getstate(){
      return state;
    }
    rosrccar_messages::VehicleState state;
  private:
    float offset_ax;
    float offset_ay;
    int esc_state;
};

void VehicleStateEstimator::update(VehicleMeasurement& measurements, unsigned int delta_time_microseconds) {
  state.yawrate_radps_e3 = (measurements.yaw_rad*1e3-state.yaw_rad_e3)*50;//needs some investigation
  state.yaw_rad_e3 = int(measurements.yaw_rad*1e3);
  state.slipangle_rad_e3 = 0;
  esc_state = esc_statemachine(state.acc_command_e3, state.wheelspeed_mps_e3, esc_state);
  state.wheelspeed_mps_e3 = int(RATIO_SHAFT_SPEED*1e3*measurements.driveshaftspeed_radps*drivingdirection(esc_state));
  if(state.wheelspeed_mps_e3==0) { // estimate offset when standing
    offset_ax = EXPONENTIALAVERAGE*offset_ax+(1-EXPONENTIALAVERAGE)*measurements.accelerationx_mps2;
    offset_ay = EXPONENTIALAVERAGE*offset_ay+(1-EXPONENTIALAVERAGE)*measurements.accelerationy_mps2;
  }
  state.acc_x_mps2_e3 = int((measurements.accelerationx_mps2-offset_ax)*1e3);
  state.acc_y_mps2_e3 = int((measurements.accelerationy_mps2-offset_ay)*1e3);
}
#endif
