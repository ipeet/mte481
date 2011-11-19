#include <vector>
#include <utility>
#include <stdint.h>

#include "wheelchair_ros/serial.h"

class Sonar : public Serial {
public:
  struct Reading {
    unsigned channel;
    unsigned value;
    Reading(unsigned ch, unsigned v) : channel(ch), value(v) {}
    Reading(const Reading &other) : 
      channel(other.channel), value(other.value) 
      {}
  };

private:
  uint16_t m_adcMask;

public:
  Sonar(const char* sonarDev, uint16_t adcMask) :
    Serial(sonarDev), m_adcMask(adcMask) 
    {}

  /* Ensure that the ubw32 is waiting for a command */
  void clear();
  /* Make a set of sonar readings */
  std::vector<Reading> readSonar();
};
