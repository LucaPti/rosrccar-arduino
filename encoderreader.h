#ifndef ENCODERREADER
#define ENCODERREADER

#include <limits.h>
#include "pwmreader.h"
#define MICROS_STANDSTILL 100000

class EncoderReader : public PWMReader {
  protected:
    float ticks_per_revolution_times_usec_to_sec;
  public:
    EncoderReader(int attachTo, int ticksperrevolution):PWMReader(attachTo),ticks_per_revolution_times_usec_to_sec(ticksperrevolution*1e-6){}
    float getangularspeed();
};

float EncoderReader::getangularspeed() {
  unsigned long incomplete_pwm_interval = (micros()-phase_start_time);
  if((incomplete_pwm_interval>pwm_interval)||(incomplete_pwm_interval>MICROS_STANDSTILL)){
    if(incomplete_pwm_interval>MICROS_STANDSTILL) {
      return 0;
    }
    else {
      return TWO_PI/(incomplete_pwm_interval*ticks_per_revolution_times_usec_to_sec);
    }
  }
  else {
    return TWO_PI/(pwm_interval*ticks_per_revolution_times_usec_to_sec);
  }
}
#endif
