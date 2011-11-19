#include <vector>
#include <iostream>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include "wheelchair_ros/sonar.h"

using namespace std;

void Sonar::clear() {
  /* Send newline (Clearing anything in the ubw command input buffer) */
  const char *newline = "\n";
  if (write(m_serialFd, newline, 1) != 1) {
    throw new Serial::Exception(strerror(errno));
  }

  // Wait a bit, to allow the ubw to respond to the command terminator
  usleep(25*1000);

  /* Clear any buffered response data. */
  // First, set non-block
  if (fcntl(m_serialFd, F_SETFL, O_RDWR | O_NOCTTY | O_NONBLOCK)) {
    throw new Serial::Exception(strerror(errno));
  }
  // Now, read all the bytes.
  char rxch;
  while (read(m_serialFd, &rxch, 1) == 1);
  // Make sure that the read failed due to no data
  if (errno != EAGAIN) {
    throw new Serial::Exception(strerror(errno));
  }
  // Finally, restore blocking
  if (fcntl(m_serialFd, F_SETFL, O_RDWR | O_NOCTTY)) {
    throw new Serial::Exception(strerror(errno));
  }
}

vector<Sonar::Reading> Sonar::readSonar() {
  const int bufSize = 128;
  char buf[bufSize];
  size_t count;
 
  /* Construct the adc read command */
  if (snprintf(buf, bufSize, "adc %d\n", m_adcMask) >= bufSize) {
    throw new Serial::Exception("Buffer overflow");
  }
  count = strnlen(buf, bufSize); 

  /* Transmit */
  if (write(m_serialFd, buf, count) != (ssize_t)count) {
     throw new Serial::Exception(strerror(errno));
  }

  /* Clear the command, which is echoed back */
  do {
    if (! read(m_serialFd, buf, 1)) {
      throw new Serial::Exception(strerror(errno));
    }
  } while (buf[0] != '\n');

  /* Read in the actual response text. */
  int i = 0;
  do {
    if (! read(m_serialFd, buf + i, 1)) {
      throw new Serial::Exception(strerror(errno));
    }
    buf[++i] = '\0';
  } while ((i < bufSize) && (buf[i-1] != '\n'));

  /* Parse the response.  Has form:
   * ADC:<timestamp>: <ch0>,<ch1>, ... ,<ch15> */
  // First, pull the timestamp:
  long timestamp;
  if(!sscanf(buf, "ADC:%ld:", &timestamp)) {
    throw new Serial::Exception("Parse failure");
  }
  vector<Reading> ret; // parsed output
  int chan = 0;  // tracks which channel / field we're on
  uint16_t imask = m_adcMask; // records which channels should have input
  char* bufp = strchr(buf, ' '); // track where we are in the buffer
  if (bufp) ++bufp; // If char not found, keep null pointer.
  while (bufp && *bufp && imask) {
    /* iterate as long as we haven't reached the end of the string, and 
     * we're expecting to find more values */
    if (imask & 0x1) {  // Expect a value in current CSV field
      int val;
      if (!sscanf(bufp, "%d", &val)) {
        throw new Serial::Exception("Parse falure");
      }
      ret.push_back(Reading(chan, val));
      // Advance to beginning of next CSV field
      bufp = strchr(bufp, ',');
      if (bufp) ++bufp;
    } else { // Don't expect value in current CSV field
      // Advance to next CSV field
      ++bufp;
    }
    ++chan;
    imask = imask >> 1;
  }
  if (imask) {
    // Did we hit end of buffer without finding all expected inputs?
    throw new Serial::Exception("Parse failure");
  }

  return ret;
}

