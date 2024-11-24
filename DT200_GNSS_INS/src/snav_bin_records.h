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

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	record_type_gpfpd = 0x01,
	record_type_gtimu = 0x05,
	record_type_nvstd = 0x06,
	record_type_time = 0x65,
} record_type_e;

//=========================== gpfpd_record_t ===========================//
enum eSnavSystemStatus
{
  Align_Init = 0, //Initialise
  Align_CorseAlign, //Corse align
  Align_FineAlign, //Fine align
  Align_GnssPosition, //GPS position
  Align_GnssHeading, //GPS heading
  Align_DGNSS, //DGNSS
  Align_DmiFusion, //DMI combined
  Align_DmiCalibration, //DMI calibration
  Align_InertialOnly, //Pure inertial navigation
  Align_ZeroVelocityCalibration, //Zero speed update
  Align_ModeVG = 0xA, //VG mode
  Align_OrientationRTK = 0xB, //RTK heading
  Align_DyncmicAlign = 0xC, //Dynamic align
  Align_DynamicError = 0xF, //Dynamic error
};

enum eSnavRtkStatus
{
  RTK_None = 0, //None
  RTK_Integer = 4, //RTK fixed
  RTK_Float, //RTK float
};

typedef struct
{
  uint16_t gps_week; // GPS week count since January 1980
  uint32_t gps_time_ms; // GPS time, in milliseconds, of GPS week
  float heading; //deg
  float pitch; //deg
  float roll; //deg
  int32_t latitude; //deg, scale factor=1E-7
  int32_t longitude; //deg, scale factor=1E-7
  int32_t altitude; //m, scale factor=0.001
  float vel_east; //m/s
  float vel_north; //m/s
  float vel_up; //m/s
  float baseline; //m, base line length
  uint8_t nsv1; //Primary satellite number
  uint8_t nsv2; //Secodary satellite number
  union
  {
	uint8_t r_uint8;
	struct
	{
	  uint8_t system : 4; //see enum eSnavSystemStatus
	  uint8_t rtk : 4; //see enum eSnavRtkStatus
	} val;
  } status;

  //stream_size = 49;
} gpfpd_record_t;

//=========================== gtimu_record_t ===========================//
typedef struct
{
  uint16_t gps_week; // GPS week count since January 1980
  uint32_t gps_time_ms; // GPS time, in milliseconds, of GPS week
  double gyro_x; //deg/s
  double gyro_y; //deg/s
  double gyro_z; //deg/s
  double acc_x; //g, 1g=9.80144145m/s
  double acc_y; //g
  double acc_z; //g
  int16_t temperature; // degC, scale factor=0.001

  //stream_size = 56;
} gtimu_record_t;

//=========================== nvstd_record_t ===========================//
typedef struct
{
  float heading_std; //deg
  float pitch_std; //deg
  float roll_std; //deg
  float lat_std; //m
  float lon_std; //m
  float alt_std; //m
  float ve_std; //m/s
  float vn_std; //m/s
  float vu_std; //m/s

  //stream_size = 36;
} nvstd_record_t;

//=========================== time_record_t ===========================//
typedef struct
{
  uint16_t gps_week; // GPS week count since January 1980
  uint32_t gps_time_ms; // GPS time, in milliseconds, of GPS week
  int16_t utc_offset; // GPS to UTC offset
  union
  {
	uint8_t r_uint8;
	struct
	{
	  uint8_t time_valid : 1; // 1 = week and millisecond of week valid
	  uint8_t offset_valid : 1; // 1 = UTC offset validity
	  uint8_t rev1 : 6;
	} val;
  } flags;

  //stream_size = 9
} time_record_t;


int decode_gpfpd_record(gpfpd_record_t *gpfpd_record, snav_bin_record_t *snav_bin_record);
int decode_gtimu_record(gtimu_record_t *gtimu_record, snav_bin_record_t *snav_bin_record);
int decode_nvstd_record(nvstd_record_t *nvstd_record, snav_bin_record_t *snav_bin_record);
int decode_time_record(time_record_t *time_record, snav_bin_record_t *snav_bin_record);

#ifdef __cplusplus
}
#endif