#ifndef PWMREADER
#define PWMREADER

class PWMReader
{
  public:
    PWMReader(int attachTo) : pin(attachTo) {}//Pin is required to discern between rising and falling flanks.
    void  processinterrupt();                 //This needs to be called by an interrupt. ATTACH IN MAIN FILE! 
    float getdutycycle();
    int   getpwminterval();
    int   getpulsetime();
  protected:
    int   pin;                                //Watch this pin.
    unsigned long phase_start_time;           //Time at which the last "HIGH" interval began.
    float duty_cycle;                         //Ratio of "HIGH" interval to PWM interval
    unsigned long pwm_interval;               //Duration of PWM interval in uS.
    int   pulse_interval;                     //Duration of "HIGH" interval in uS.
};

void PWMReader::processinterrupt(void) {
  // ATTACH THIS METHOD TO INTERRUPT IN MAIN FILE LIKE THIS (with pwmreader an instance of this class):
  // void handleinterrupt() {
  //    pwmreader.processinterrupt();
  // }
  // void setup() {
  //   attachInterrupt(digitalPinToInterrupt(PWM_PIN), handleinterrupt, CHANGE);
  // }
  // based on https://www.camelsoftware.com/2015/12/25/reading-pwm-signals-from-an-rc-receiver-with-arduino/ 
  // This method notes the time at interrupt and updates the duration of the full pwm interval and 
  // the "HIGH" interval.
  
  unsigned long micros_now{micros()}; // Save time of interrupt in the beginning to avoid delay
  if(digitalRead(pin)==HIGH) {
    if(phase_start_time!=0) {
      pwm_interval = (micros_now-phase_start_time);
    }
    phase_start_time = micros_now;
  }
  else {
    if(phase_start_time!=0) {
      pulse_interval = (int)(micros_now-phase_start_time);
    }
  }
}

float PWMReader::getdutycycle(){
  duty_cycle = ((float)pulse_interval)/((float)pwm_interval);
  return duty_cycle;
}
int PWMReader::getpwminterval(){
  return pwm_interval;
}
int PWMReader::getpulsetime(){
  return pulse_interval;
}
#endif
