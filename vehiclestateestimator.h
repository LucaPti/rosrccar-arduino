#ifndef VEHICLESTATEESTIMATOR
#define VEHICLESTATEESTIMATOR

#include <rosrccar_messages/VehicleState.h>
#define ACCELERATION_SCALE (9.81/32767)

// wheel circumference approximately 21 cm
// roughly 55 ticks per wheel revolution
#define TICKS_TO_METERS 0.21/55

static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

class VehicleStateEstimator {
  public:
    VehicleStateEstimator(){}
    void update(int acc_x, int acc_y, float yaw, unsigned int delta_encoder_tics, unsigned int delta_time_microseconds, float steering_command, float accelerator_command);
    rosrccar_messages::VehicleState& getstate(){
      return state;
    }
    rosrccar_messages::VehicleState state;
  private:
    int driving_direction;
    float velocity_x;
    float velocity_y;
};

void VehicleStateEstimator::update(int acc_x, int acc_y, float yaw, unsigned int delta_encoder_tics, unsigned int delta_time, float steering_command, float accelerator_command) {
  // Velocity vector
  velocity_x += ACCELERATION_SCALE*acc_x*(delta_time*1e-6);
  velocity_y += ACCELERATION_SCALE*acc_y*(delta_time*1e-6);
  //state.velocity = sqrt((velocity_x*velocity_x)+(velocity_y*velocity_y));
  state.velocity = delta_encoder_tics*TICKS_TO_METERS/(delta_time*1e-6);
  if(abs(state.velocity)>1e-2) {
    if(state.velocity>0) {
      driving_direction = 1;
    }
    else {
      driving_direction = -1;
    }
  }
  else {
    driving_direction = 0;
  }

  // Slip angle, yaw rate and yaw
  if(abs(velocity_x)>1e-2) {
    state.slipangle = velocity_y/velocity_x; // Should be a tangent
  }
  else {
    if(velocity_y>0) {
      state.slipangle = PI/2;
    }
    else {
      state.slipangle = -PI/2;
    }
  }
  state.yawrate = (yaw-state.yaw)/(delta_time*1e-6);
  state.yaw = yaw;

  // Engine speed
  state.enginespeed = 2*PI*delta_encoder_tics/((4*delta_time)*1e-6);

  // Acceleration
  state.acc_x = ACCELERATION_SCALE*acc_x;
  state.acc_y = ACCELERATION_SCALE*acc_y;

  state.acc_command = accelerator_command;
  state.steer_command = steering_command;
  
  state.valid = true;
}
#endif
