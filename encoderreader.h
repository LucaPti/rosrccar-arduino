#ifndef ENCODERREADER
#define ENCODERREADER

#include <limits.h>
#include "pwmreader.h"

class EncoderReader : public PWMReader {
  protected:
    unsigned int ticks_per_revolution;
  public:
    EncoderReader(int attachTo, int ticksperrevolution):PWMReader(attachTo),ticks_per_revolution(ticksperrevolution){}
    float getangularvelocity();
};

float EncoderReader::getangularvelocity() {
  return PI/(pwm_interval*ticks_per_revolution);
}
#endif
