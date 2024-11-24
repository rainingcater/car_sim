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
 
 #include <stdint.h>
 #include <string.h>
 #include "snav_bin_packet_protocol.h"
 #include "snav_bin_records.h"
 #include "struct_pack_unpack.h"

#define vas_bswap16_(s) ((((s) & 0xff) << 8) | (((s) >> 8) & 0xff))  

#define vas_bswap32_(l) (((l) >> 24) | \
(((l) & 0x00ff0000) >> 8) | \
(((l) & 0x0000ff00) << 8) | \
((l) << 24))

#define vas_bswap64_(ll) (((ll) >> 56) | \
(((ll) & 0x00ff000000000000) >> 40)  | \
(((ll) & 0x0000ff0000000000) >> 24)  | \
(((ll) & 0x000000ff00000000) >> 8)   | \
(((ll) & 0x00000000ff000000) << 8)   | \
(((ll) & 0x0000000000ff0000) << 24)  | \
(((ll) & 0x000000000000ff00) << 40)  | \
(((ll) << 56)))


int decode_gpfpd_record(gpfpd_record_t *gpfpd, snav_bin_record_t *snav_bin_record)
{
	if (snav_bin_record->record_type == record_type_gpfpd && snav_bin_record->length == 49)
	{
	  struct_unpack(snav_bin_record->data, "<HI3f3i4f3B",
		&gpfpd->gps_week,
		&gpfpd->gps_time_ms,
		&gpfpd->heading, &gpfpd->pitch, &gpfpd->roll,
		&gpfpd->latitude, &gpfpd->longitude, &gpfpd->altitude,
		&gpfpd->vel_east, &gpfpd->vel_north, &gpfpd->vel_up, &gpfpd->baseline,
		&gpfpd->nsv1, &gpfpd->nsv2, &gpfpd->status.r_uint8);

	  return 0;
	}
	else return 1;
}

int decode_gtimu_record(gtimu_record_t *gtimu, snav_bin_record_t *snav_bin_record)
{
	if (snav_bin_record->record_type == record_type_gtimu && snav_bin_record->length == 56)
	{
	  struct_unpack(snav_bin_record->data, "<HI6dH",
		&gtimu->gps_week,
		&gtimu->gps_time_ms,
		&gtimu->gyro_x, &gtimu->gyro_y, &gtimu->gyro_z, &gtimu->acc_x, &gtimu->acc_y, &gtimu->acc_z,
		&gtimu->temperature);

	  return 0;
	}
	else return 1;
}

int decode_nvstd_record(nvstd_record_t *nvstd, snav_bin_record_t *snav_bin_record)
{
  if (snav_bin_record->record_type == record_type_nvstd && snav_bin_record->length == 36)
  {
	struct_unpack(snav_bin_record->data, "<9f",
	  &nvstd->heading_std, &nvstd->pitch_std, &nvstd->roll_std,
	  &nvstd->lat_std, &nvstd->lon_std, &nvstd->alt_std,
	  &nvstd->ve_std, &nvstd->vn_std, &nvstd->vu_std);

	return 0;
  }
  else return 1;
}

int decode_time_record(time_record_t *time, snav_bin_record_t *snav_bin_record)
{
  if (snav_bin_record->record_type == record_type_time && snav_bin_record->length == 9)
  {
	struct_unpack(snav_bin_record->data, "<HIhB",
	  &time->gps_week, &time->gps_time_ms, &time->utc_offset, &time->flags);

	return 0;
  }
  else return 1;
}