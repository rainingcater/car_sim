/****************************************************************/
/*                                                              */
/*          SNAV_BIN Packet Protocol Library                    */
/*          C Language Dynamic, Version 1.0                     */
/*   Copyright 2021, Beijing Nuogeng Technology Ltd             */
/*                                                              */
/****************************************************************/
/*
 * Copyright (C) 2021 Beijing Nuogeng Technology Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "snav_bin_packet_protocol.h"
#include "snav_bin_records.h"

const char* toSystemStatusString(int system_status)
{
  switch (system_status)
  {
  case 0: return "Init";
  case 1: return "CorseAlign";
  case 2: return "FineAlign";
  case 3: return "GnssPosition";
  case 4: return "GnssHeading";
  case 5: return "DGNSS";
  case 6: return "DmiFusion";
  case 7: return "DmiCalibration";
  case 8: return "InertialOnly";
  case 9: return "ZeroVelocityCalibration";
  case 10: return "ModeVG";
  case 11: return "OrientationRTK";
  case 12: return "Align_DyncmicAlign";
  case 15: return "DynamicError";
  default:
	break;
  }
  return "Unknown";
}

const char* toRtkStatusString(int rtk_status)
{
  switch (rtk_status)
  {
  case 0: return "None";
  case 1: return "RtkInteger";
  case 2: return "RtkFloat";
  default:
	break;
  }
  return "Unknown";
}


/*
* Function to decode snav_bin_records from raw data
* Returns 0 with valid snav_bin_record
*/
int snav_bin_record_decode(snav_bin_decoder_t* snav_bin_decoder, snav_bin_record_t* snav_bin_record)
{
  // Check recv_buffer, may contain one or more snav_bin_record
  while (snav_bin_decoder->decode_iterator + SNAV_BIN_PACKET_HEADER_SIZE < snav_bin_decoder->recv_buf_length)
  {
	uint8_t* recv_buf = snav_bin_decoder->recv_buffer + snav_bin_decoder->decode_iterator;
	// Header begin with 0xAA55
	if (recv_buf[0] != 0xAA || recv_buf[1] != 0x55) {
	  ++snav_bin_decoder->decode_iterator;
	  continue;
	}

	// Find valid record length
	uint8_t record_length = 0;
	switch (recv_buf[2])
	{
	case record_type_gpfpd: record_length = SNAV_BIN_PACKET_HEADER_SIZE + 49 + 1; break;
	case record_type_gtimu: record_length = SNAV_BIN_PACKET_HEADER_SIZE + 56 + 1; break;
	case record_type_nvstd: record_length = SNAV_BIN_PACKET_HEADER_SIZE + 36 + 1; break;
	case record_type_time: record_length = SNAV_BIN_PACKET_HEADER_SIZE + 9 + 1; break;
	default: break;
	}

	// Invalid record length, continue to check recv_buffer
	if (record_length == 0) {
	  ++snav_bin_decoder->decode_iterator;
	  continue;
	}

	// Recv_buffer should collected enough data byte 
	if (record_length + snav_bin_decoder->decode_iterator <= snav_bin_decoder->recv_buf_length)
	{
	  //Two check sum should be the same
	  uint8_t checksum = 0;
	  for (uint8_t i = 0; i < record_length - 1; ++i) { // Exclude 1 byte check sum
		checksum += recv_buf[i];
	  }

	  snav_bin_record->record_type = recv_buf[2];
	  snav_bin_record->length = record_length - SNAV_BIN_PACKET_HEADER_SIZE - 1;
	  snav_bin_record->data = recv_buf + SNAV_BIN_PACKET_HEADER_SIZE;
	  snav_bin_decoder->decode_iterator += record_length;

	  // Check sum error, skip this package
	  if (recv_buf[record_length - 1] != checksum) {
		snav_bin_decoder->checksum_errors++;
		continue; // Continue to check recv_buffer
	  }

	  return 0; // return one valid record
	}
	
	// Running here, find correct header, but recv_buffer does not have full record data, need more refresh data
	break;
  }

  // Check recv_buffer whether have uncheck data
  if (snav_bin_decoder->decode_iterator < snav_bin_decoder->recv_buf_length)
  {
	if (snav_bin_decoder->decode_iterator > 0)
	{ // Copy uncheck data to begin position of recv_buffer
	  memmove(snav_bin_decoder->recv_buffer,
		snav_bin_decoder->recv_buffer + snav_bin_decoder->decode_iterator,
		(snav_bin_decoder->recv_buf_length - snav_bin_decoder->decode_iterator) * sizeof(uint8_t));

	  snav_bin_decoder->recv_buf_length -= snav_bin_decoder->decode_iterator;
	}
  }
  else
	snav_bin_decoder->recv_buf_length = 0;

  // Reset decoder position to begin position of recv_buffer
  snav_bin_decoder->decode_iterator = 0;
  return 1;
}
