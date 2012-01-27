/******************************************************************************
 * protocol.h
 *
 * Common definitions for use in both the ros node and micro.
 ******************************************************************************
 * This program is distributed under the of the GNU Lesser Public License. 
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *****************************************************************************/

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>

static const unsigned MSG_HEADER_LENGTH = 3;

/* Sync bytes, sent before each message */
static const uint8_t SYNC_BYTE_1 = 0xaa;
static const uint8_t SYNC_BYTE_2 = 0x55;

/* Type codes for messages */
enum MessageType {
  PING = 1,
  PING_RESP,
  DIGITAL_READ,
  DIGITAL_DATA,
  DIGITAL_WRITE,
  ADC_READ,
  ADC_DATA
};

#define PACKED __attribute__ (( __packed__ ))

/** x86, MSP430, AtMega, PIC are all little-endian.
 *  Assuming we can get compilers to pack nicely, we can save a
 *  whole bunch of effort on parsing. */
typedef struct {
  uint8_t checksum;  /* XOR of all following bytes */
  uint8_t type;
  uint8_t length; // length of payload:
  union { // payload
    uint8_t raw[0];
    struct { 
      uint8_t input;
    } PACKED digitalRead;
    struct {
      uint8_t input;
      uint8_t value;
    } PACKED digitalData;
    struct {
      uint8_t output;
      uint8_t value;
    } PACKED digitalWrite;
    struct {
      uint8_t input;
    } PACKED adcRead;
    struct {
      uint8_t input;
      uint16_t value;
    } PACKED adcData;
  };
} PACKED SerialMessage;

/* Set reader to correct initial state.  (Can also be used as a reset) */
void pr_init();

/* Possible parser return statuses.  All are non-fatal */
enum ParseStatus {
  OK,  // Still reading a message
  COMPLETE, // Just finished a message.  Retrieve with pr_getmsg
  BAD_SYNC,  // Expected a sync byte, but didn't get one
  BAD_CHECKSUM, // Completed message, but checksum didn't match.
  INVALID // Invalid state.  Should never happen.
};

/* Interpret an additional byte. */
enum ParseStatus pr_push(uint8_t byte);

/* Get a pointer to the current message receive buffer.
   Contents only valid immediately after a COMPLETE return from pr_push. */
const SerialMessage *pr_getmsg();

/* Compute the checksum of a message */
uint8_t pr_checksum(const SerialMessage *msg);

#endif //PROTOCOL_H_

