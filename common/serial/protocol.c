/******************************************************************************
 * protocol_reader.c
 *
 * Extracts valid messages from a byte stream.
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

#include <string.h>
#include <stdio.h>

#include "protocol.h"

static enum {
  SYNC_1,
  SYNC_2,
  CHECKSUM,
  TYPE,
  LENGTH,
  PAYLOAD,
  DONE
} _state;
static SerialMessage _buf;
static int _payloadBytes;

// Set initial parser state
void pr_init() {
  _state = SYNC_1;
  _payloadBytes = 0;
}

// Parse a new byte
enum ParseStatus pr_push(uint8_t byte) {
  if (_state == DONE) {
    // start reading a new message
    pr_init();
  }

  switch (_state) {
    case SYNC_1:
      if (byte == SYNC_BYTE_1) {
        _state = SYNC_2;
        return OK;
      } else {
        return BAD_SYNC;
      }

    case SYNC_2:
      if (byte == SYNC_BYTE_2) {
        _state = CHECKSUM;
        return OK;
      } else {
        pr_init();
        return BAD_SYNC;
      }

    case CHECKSUM:
      _buf.checksum = byte;
      _state = TYPE;
      return OK;

    case TYPE:
      _buf.type = byte;
      _state = LENGTH;
      return OK;

    case LENGTH:
      _buf.length = byte;
      if (byte == 0) {
        _state = DONE;
        break;
      } else {
        _payloadBytes = 0;
        _state = PAYLOAD;
        return OK;
      }

    case PAYLOAD:
      _buf.raw[_payloadBytes] = byte;
      _payloadBytes++;
      if (_payloadBytes < _buf.length) {
        return OK;
      } else {
        _state = DONE;
        break;
      }
    
    default:
      pr_init();
      return INVALID;
  };

  // Should only fall through for checksum on a complete package
  if (_state != DONE) return INVALID;

  /* Validate checksum */
  if (pr_checksum(&_buf) == _buf.checksum) {
    return COMPLETE;
  } else {
    return BAD_CHECKSUM;
  }
}

// Get a pointer to the message buffer
const SerialMessage *pr_getmsg() {
  return &_buf;
}

// Compute message checksum.
uint8_t pr_checksum(const SerialMessage *msg) {
  uint8_t check = 0;
  const uint8_t *bufp = (const uint8_t*)(msg);
  int checklen = MSG_HEADER_LENGTH + msg->length - 1 /* minus checksum*/;
  if (checklen > (sizeof(SerialMessage) - 1)) {
    checklen = sizeof(SerialMessage) - 1;
  }
  int i;
  for (i=0; i < checklen; ++i ) {
    check ^= bufp[i+1];
  }
  return check;
}

