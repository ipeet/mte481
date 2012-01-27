/******************************************************************************
 * protocol_test.c
 *
 * Tester program for the common serial protocol source.
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

#include <stdio.h>
#include <string.h>

#include "protocol.h"

int main() {
  int status = 0;

  /* Check assumptions about memory layout of SerialMessage */
  struct SerialMessage msg;
  if ( &(msg.raw[0]) - &(msg.checksum) != MSG_HEADER_LENGTH ) {
    printf("MSG_HEADER_LENGTH is incorrect\n");
    status = 1;
  }
  memset(&msg, 0, sizeof(struct SerialMessage));
  msg.adcData.input = 0xaa;
  if ( msg.raw[0] != 0xaa ) {
    printf("SerialMessage layout is wrong.\n");
    status = 1;
  }
  msg.adcData.value = 0xddbb;
  if ( msg.raw[1] != 0xbb ) {
    printf("SerialMessage layout is wrong, or arch is not little-endian.\n");
    printf("%02x %02x\n", msg.raw[1], msg.raw[2]);
    status = 1;
  }
  msg.adcData.value = 0xccee;
  if ( msg.raw[2] != 0xcc ) {
    printf("SerialMessage layout is wrong, or arch is not little-endian.\n");
    printf("%02x %02x\n", msg.raw[1], msg.raw[2]);
    status = 1;
  }

  /* Binary test messages */
  uint8_t msg1[] = {0xc1, 42, 3, 0xaf, 0x93, 0xd4};
  uint8_t msg2[] = {17, 17, 0};


  /* Check checksum computation */
  msg.checksum = 0xab; // (wrong)
  msg.type = msg1[1];
  msg.length = msg1[2];
  msg.raw[0] = msg1[3];
  msg.raw[1] = msg1[4];
  msg.raw[2] = msg1[5];
  if (pr_checksum(&msg) != 0xc1) {
    printf("Checksum wrong.  Expected C1, got %02x\n", pr_checksum(&msg));
    status = 1;
  }

  /* Excercise the parser */

  /* Check syncing */
  if (pr_push(0x00) != BAD_SYNC) {
    printf("Expected BAD_SYNC\n");
    status = 1;
  }
  if (pr_push(SYNC_BYTE_1) != OK) {
    printf("Expected OK\n");
    status = 1;
  }
  if (pr_push(0x00) != BAD_SYNC) {
    printf("Expected BAD_SYNC\n");
    status = 1;
  }

  /* Now for a valid message */
  pr_push(SYNC_BYTE_1);
  if (pr_push(SYNC_BYTE_2) != OK) {
    printf("Expected OK\n");
    status = 1;
  }
  int i;
  for (i=0; i<5; ++i) {
    if (pr_push(msg1[i]) != OK) {
      printf("Expected OK\n");
      status = 1;
    }
  }
  if (pr_push(msg1[5]) != COMPLETE) {
    printf("Expected COMPLETE\n");
    status = 1;
  }

  /* Another valid message */
  pr_push(SYNC_BYTE_1);
  pr_push(SYNC_BYTE_2);
  pr_push(msg2[0]);
  pr_push(msg2[1]);
  if (pr_push(msg2[2]) != COMPLETE) {
    printf("Expected COMPLETE\n");
    status = 1;
  }

  /* Finally, a checksum error */
  pr_push(SYNC_BYTE_1);
  pr_push(SYNC_BYTE_2);
  pr_push(0x00);
  for (i=1; i<5; ++i) {
    if (pr_push(msg1[i]) != OK) {
      printf("Expected OK\n");
      status = 1;
    }
  }
  if (pr_push(msg1[5]) != BAD_CHECKSUM) {
    printf("Expected BAD_CHECKSUM\n");
    status = 1;
  }

  printf("Test complete: %s\n", status ? "FAIL": "PASS");
  return status;
}
