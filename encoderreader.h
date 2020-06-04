#ifndef ENCODERREADER
#define ENCODERREADER

#include <limits.h>

class EncoderReader {
  protected:
    unsigned int current_ticks{0};
    unsigned int last_ticks{0};
    unsigned int overflows{0};
  public:
    EncoderReader& operator++();
    unsigned int totalticks();
    unsigned int deltaticks();
};

EncoderReader& EncoderReader::operator++()
{
  if(current_ticks==UINT_MAX)
  {
    overflows++;
  }
  current_ticks++;
  return *this;
}

unsigned int EncoderReader::totalticks()
{
  return current_ticks;
}

unsigned int EncoderReader::deltaticks()
{
  unsigned int delta{0};
  if(overflows>0)
  {
    delta = (UINT_MAX-last_ticks)+current_ticks;
    overflows--;
  }
  else
  {
    delta = current_ticks-last_ticks;
  }
  last_ticks = current_ticks;
  return delta;
}

#endif
