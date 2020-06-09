#ifndef RCREADER
#define RCREADER

#include "pwmreader.h"

class RCReader : public PWMReader {
  public:
    RCReader(int attachTo): PWMReader(attachTo),minlevel(700),zerolevel(1500),maxlevel(2300){}
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
  failsafe_engage = (pwm_interval>100000)||(pwm_interval<3000);
  failsafe_engage = failsafe_engage||((pulse_interval<500)||(pulse_interval>2500));
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
