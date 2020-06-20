#ifndef RCREADER
#define RCREADER

// Calibration
#define MAX_NORM_VAL 1980  // Reference 2200
#define MIN_NORM_VAL 1156  // Reference 700
#define MAX_LIMIT_VAL 2200 // Reference 2500, measured 2116
#define MIN_LIMIT_VAL 1000 // Reference 500, measured 1032
#define ZERO_NORM_VAL 1536 // Reference 1500, measured 1536

#include "pwmreader.h"

class RCReader : public PWMReader {
  public:
    RCReader(int attachTo): PWMReader(attachTo),minlevel(MIN_NORM_VAL),zerolevel(ZERO_NORM_VAL),maxlevel(MAX_NORM_VAL){}
    void setlevels(int minpulsetime,int zeropulsetime,int maxpulsetime);
    float normalizedinput();
    float failsafeinput();
    boolean signalisvalid();
  private:
    int minlevel;
    int zerolevel;
    int maxlevel;
    unsigned long failsafe_lastinterruptdetected;
};

void RCReader::setlevels(int minpulsetime,int zeropulsetime,int maxpulsetime) {
  // This method updates the limits for calibration.
  minlevel = minpulsetime;
  zerolevel = zeropulsetime;
  maxlevel = maxpulsetime;
}

float RCReader::normalizedinput(void) {
  if(pulse_interval>zerolevel) {
    return ((float)(pulse_interval-zerolevel))/(maxlevel-zerolevel);
  }
  else {
    return ((float)(pulse_interval-zerolevel))/(zerolevel-minlevel);
  }
}

boolean RCReader::signalisvalid(void) {
  boolean failsafe_engage;
  // Checks as in https://create.arduino.cc/projecthub/kelvineyeone/read-pwm-decode-rc-receiver-input-and-apply-fail-safe-6b90eb
  failsafe_engage = (pwm_interval>100000)||(pwm_interval<3000)||((micros()-phase_start_time)>100000);
  failsafe_engage = failsafe_engage||((pulse_interval<MIN_LIMIT_VAL)||(pulse_interval>MAX_LIMIT_VAL));
  return !failsafe_engage;
}

float RCReader::failsafeinput(void) {
  // Validates current readings: PWM interval, "HIGH" interval, and last interrupt
  if(!(this->signalisvalid())){
    return 0;
  }
  else{
    return min(max(normalizedinput(),-1),1);
  }
}
#endif
