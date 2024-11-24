
#include <ros/ros.h>
#include "tf/transform_datatypes.h"//转换函数头文件
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>//里程计信息格式
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <ros_serial/serial.h>
#include <std_msgs/Empty.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#define rBUFFERSIZE  115
unsigned char r_buffer[1024] ;
//unsigned char r_buffer[rBUFFERSIZE] ;

#define PI (3.14159265358979)

typedef unsigned long U32 ;
typedef unsigned short U16 ;
typedef signed short S16 ;
typedef unsigned char U8 ;

typedef unsigned int INT32 ;
typedef unsigned short INT16 ;

unsigned char gsofData[105] ;
int gsofDataIndex ;
serial::Serial ser; //声明串口对象
static ros::Publisher Imu_pub;
static ros::Publisher fix_pub;

/**********************************************************************/
unsigned long getINT32( unsigned char * * ppData )
/**********************************************************************/
// Used by the decoding routines to grab 4 bytes and pack them into
// a U32.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  unsigned int retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 3 ;

  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getINT32() */


/**********************************************************************/
unsigned long getU32( unsigned char * * ppData )
/**********************************************************************/
// Used by the decoding routines to grab 4 bytes and pack them into
// a U32.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  unsigned long retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 3 ;

  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getU32() */



/**********************************************************************/
float getFloat( unsigned char * * ppData )
/**********************************************************************/
// Used by the decoding routines to grab 4 bytes and pack them into
// a Float.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  float retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 3 ;


  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getFloat() */



/**********************************************************************/
double getDouble( unsigned char * * ppData )
/**********************************************************************/
// Used by the decoding routines to grab 8 bytes and pack them into
// a Double.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  double retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 7 ;


  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getDouble() */



/**********************************************************************/
unsigned short getU16( unsigned char * * ppData )
/**********************************************************************/
// Used by the decoding routines to grab 2 bytes and pack them into
// a U16.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  unsigned short retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 1 ;

  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getU16() */




/**********************************************************************/
void processPositionTime( int length, unsigned char *pData )
/**********************************************************************/
{
  unsigned long msecs ;
  unsigned short weekNumber ;
  int nSVs ;
  int flags1 ;
  int flags2 ;
  int initNumber ;

  printf( "  GsofType:1 - PositionTime  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  msecs = getU32( &pData ) ;
  weekNumber = getU16( &pData ) ;
  nSVs = *pData++ ;
  flags1 = *pData++ ;
  flags2 = *pData++ ;
  initNumber = *pData++ ;

  printf( "  Milliseconds:%ld  Week:%d  #Svs:%d "
          "flags:%02X:%02X init:%d\n",
          msecs,
          weekNumber,
          nSVs,
          flags1,
          flags2,
          initNumber
        ) ;

} /* end of processPositionTime() */



/**********************************************************************/
void processInsFullNavigation( int length, unsigned char *pData )  //这里把imu坐标系从NED坐标转换为与雷达坐标系一样（z正向向上，y正向向左，x轴正向向前）
/**********************************************************************/
{
  unsigned short weekNumber ;
          flags2,
          initNumber
        ) ;

  unsigned int gpsTime ;
  unsigned char IMUalignmentStatus ;
  unsigned char GPSqualityIndicator ;
  double Latitude ;
  double Longitude ;
  double Altitude ;
  float NorthVelocity ;
  float EastVelocity ;
  float DownVelocity ;
  float TotalSpeed ;
  double Roll ;
  double Pitch ;
  double Heading ;
  double TrackAngle ;
  float AngularRateX ;
  float AngularRateY ;
  float AngularRateZ ;
  float LongitudinalAccel ;
  float TraverseAccel ;
  float DownAccel ;

  printf( "  GsofType:49 - INS Full Navigation  len:%d\n",
          length
        ) ;

  weekNumber = getU16( &pData ) ;
  gpsTime = getINT32( &pData ) ;
  IMUalignmentStatus = *pData++ ;
  GPSqualityIndicator = *pData++ ;
  Latitude = getDouble( &pData ) ;
  Longitude = getDouble( &pData ) ;
  Altitude = getDouble( &pData ) ;
  NorthVelocity = getFloat( &pData ) ;
  EastVelocity = getFloat( &pData ) ;
  DownVelocity = getFloat( &pData ) ;
  TotalSpeed = getFloat( &pData ) ;
  Roll = getDouble( &pData ) ;
  Pitch = getDouble( &pData ) ;
  Heading = getDouble( &pData ) ;
  TrackAngle = getDouble( &pData ) ;
  AngularRateX = getFloat( &pData ) ;
  AngularRateY = getFloat( &pData ) ;
  AngularRateZ = getFloat( &pData ) ;
  LongitudinalAccel = getFloat( &pData ) ; // x即纵向加速度
  TraverseAccel = getFloat( &pData ) ;
  DownAccel = getFloat( &pData ) ;

  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(Roll/180*3.1415926, -Pitch/180*3.1415926, -1.57079632+Heading/180*3.1415926);//返回四元数 *PI/180
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = "imu_link";
  imu_msg.orientation.x = quat.x;
  imu_msg.orientation.y = quat.y;
  imu_msg.orientation.z = quat.z;
  imu_msg.orientation.w = quat.w;
  imu_msg.orientation_covariance[0] = 0.01; //由于传感器并无该项输出，这里设为固定值(仿照west.bag数据集)
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.01;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.01;
  //imu_msg.linear_acceleration.x = -(DownAccel - 9.85 * std::sin(Pitch/180*3.1415926)) ; //加速度算在内
  //imu_msg.linear_acceleration.y = (LongitudinalAccel + 9.85 * std::sin(Roll/180*3.1415926) * std::cos(Pitch/180*3.1415926)); //加速度
  //imu_msg.linear_acceleration.z = (TraverseAccel + 9.85 * std::cos(Pitch/180*3.1415926) * std::cos(Roll/180*3.1415926)); //加速度
  //LongitudinalAccel = DownAccel - 9.85 * std::sin(Pitch) ;    //用于cout
  //TraverseAccel = -(LongitudinalAccel + 9.85 * std::sin(Roll) * std::cos(Pitch));  //用于cout
  //DownAccel = -(TraverseAccel + 9.85 * std::cos(Pitch) * std::cos(Roll));  // 用于cout

  imu_msg.linear_acceleration.x = LongitudinalAccel;
  imu_msg.linear_acceleration.y = -TraverseAccel;
  imu_msg.linear_acceleration.z = -DownAccel;
  imu_msg.linear_acceleration_covariance[0] = 0.01; //由于传感器并无该项输出，这里设为固定值
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.01;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.01;
  imu_msg.angular_velocity.x = AngularRateX/180*3.1415926;
  imu_msg.angular_velocity.y = -AngularRateY/180*3.1415926;
  imu_msg.angular_velocity.z = -AngularRateZ/180*3.1415926;
  imu_msg.angular_velocity_covariance[0] = 0.01; //由于传感器并无该项输出，这里设为固定值
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.01;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.01;
  Imu_pub.publish(imu_msg);

  sensor_msgs::NavSatFix fix;
  fix.header.frame_id = "navsat_link";
  fix.header.stamp = ros::Time::now();
  fix.latitude = Latitude;
  fix.longitude = Longitude;
  fix.altitude = Altitude;
  fix.position_covariance[0] = 1.44; //由于传感器并无该项输出，这里设为固定值
  fix.position_covariance[1] = 0;
  fix.position_covariance[2] = 0;
  fix.position_covariance[3] = 0;
  fix.position_covariance[4] = 1.44;
  fix.position_covariance[5] = 0;
  fix.position_covariance[6] = 0;
  fix.position_covariance[7] = 0;
  fix.position_covariance[8] = 5.76;

  fix.status.status = GPSqualityIndicator;

  fix_pub.publish(fix);


  printf( "  weekNumber:%d"
          "  gpsTime:%ld"
          "  IMUalignmentStatus:%d"
          "  GPSqualityIndicator:%d"
          "\n"
          "  Latitude:%.7f"
          "  Longitude:%.7f"
          "  Altitude:%.3f"
          "\n"
          "  NorthVelocity:%.3f"
          "  EastVelocity:%.3f"
          "  DownVelocity:%.3f"
          "  TotalSpeed:%.3f"
          "\n"
          "  Roll:%.3f"
          "  Pitch:%.3f"
          "  Heading:%.3f"
          "  TrackAngle:%.3f"
          "\n"
          "  AngularRateX:%.3f"
          "  AngularRateY:%.3f"
          "  AngularRateZ:%.3f"
          "\n"
          "  LongitudinalAccel:%.3f"
          "  TraverseAccel:%.3f"
          "  DownAccel:%.3f"
          "\n"
          ,
          weekNumber,
          gpsTime,
          IMUalignmentStatus,
          GPSqualityIndicator,
          Latitude,
          Longitude,
          Altitude,
          NorthVelocity,
          EastVelocity,
          DownVelocity,
          TotalSpeed,
          Roll/180*3.1415926,
          -Pitch/180*3.1415926,
          Heading/180*3.1415926,
          TrackAngle,
          AngularRateX/180*3.1415926,
          -AngularRateY/180*3.1415926,
          -AngularRateZ/180*3.1415926,
          imu_msg.linear_acceleration.x,   //imu_msg.linear_acceleration.x  LongitudinalAccel
          imu_msg.linear_acceleration.y,       //imu_msg.linear_acceleration.y   TraverseAccel
          imu_msg.linear_acceleration.z            //imu_msg.linear_acceleration.z   DownAccel
        ) ;


} /* end of processInsFullNavigation */




/**********************************************************************/
void processLatLonHeight( int length, unsigned char *pData )
/**********************************************************************/
{
  double lat, lon, height ;

  printf( "  GsofType:2 - LatLongHeight   len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif
  lat = getDouble( &pData ) * 180.0 / PI ;
  lon = getDouble( &pData ) * 180.0 / PI ;
  height = getDouble( &pData ) ;

  printf( "  Lat:%.7f Lon:%.7f Height:%.3f\n",
          lat,
          lon,
          height
        ) ;
} /* end of processLatLonHeight() */





/**********************************************************************/
void processECEF( int length, unsigned char *pData )
/**********************************************************************/
{
  double X, Y, Z ;

  printf( "  GsofType:3 - ECEF   len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif
  X = getDouble( &pData ) ;
  Y = getDouble( &pData ) ;
  Z = getDouble( &pData ) ;

  printf( "  X:%.3f Y:%.3f Z:%.3f\n", X, Y, Z ) ;

} /* end of processECEF() */



/**********************************************************************/
void processLocalDatum( int length, unsigned char *pData )
/**********************************************************************/
{
  char id[9] ;
  double lat, lon, height ;

  printf( "  GsofType:4 - Local Datum Position  "
          "!!!!!UNTESTED!!!!!!!  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  memcpy( id, pData, 8 ) ;
  pData += 8 ;
  id[9] = 0 ;

  lat = getDouble( &pData ) * 180.0 / PI ;
  lon = getDouble( &pData ) * 180.0 / PI ;
  height = getDouble( &pData ) ;

  printf( "  Id:%s Lat:%.7f Lon:%.7f Height:%.3f\n",
          id,
          lat,
          lon,
          height
        ) ;
} /* end of processLocalDatum() */



/**********************************************************************/
void processEcefDelta( int length, unsigned char *pData )
/**********************************************************************/
{
  double X, Y, Z ;

  printf( "  GsofType:6 - ECEF Delta  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  X = getDouble( &pData ) ;
  Y = getDouble( &pData ) ;
  Z = getDouble( &pData ) ;

  printf( "  X:%.3f Y:%.3f Z:%.3f\n", X, Y, Z ) ;

} /* end of processEcefDelta() */



/**********************************************************************/
void processTangentPlaneDelta( int length, unsigned char *pData )
/**********************************************************************/
{
  double E, N, U ;

  printf( "  GsofType:7 - Tangent Plane Delta  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  E = getDouble( &pData ) ;
  N = getDouble( &pData ) ;
  U = getDouble( &pData ) ;

  printf( "  East:%.3f North:%.3f Up:%.3f\n", E, N, U ) ;

} /* end of processTangentPlaneDelta() */



/**********************************************************************/
void processVelocityData( int length, unsigned char *pData )
/**********************************************************************/
{
  int flags ;
  float velocity ;
  float heading ;
  float vertical ;

  printf( "  GsofType:8 - Velocity Data  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  flags = *pData++ ;

  velocity = getFloat( &pData ) ;
  heading = getFloat( &pData ) * 180.0 / PI ;
  vertical = getFloat( &pData ) ;

  printf( "  Flags:%02X  velocity:%.3f  heading:%.3f  vertical:%.3f\n",
          flags,
          velocity,
          heading,
          vertical
        ) ;

} /* end of processVelocityData() */



/**********************************************************************/
void processUtcTime( int length, unsigned char *pData )
/**********************************************************************/
{

  printf( "  GsofType:16 - UTC Time Info   len:%d\n",
          length
        ) ;

  U32 msecs = getU32( &pData ) ;
  U16 weekNumber = getU16( &pData ) ;
  S16 utcOffset = getU16( &pData ) ;
  U8 flags = *pData++ ;

  printf( "  ms:%lu  week:%u  utcOff:%d  flags:%02x\n",
          msecs,
          weekNumber,
          utcOffset,
          flags
        ) ;

} /* end of processUtcTime() */



/**********************************************************************/
void processPdopInfo( int length, unsigned char *pData )
/**********************************************************************/
{
  float pdop ;
  float hdop ;
  float vdop ;
  float tdop ;

  printf( "  GsofType:9 - PDOP Info   len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  pdop = getFloat( &pData ) ;
  hdop = getFloat( &pData ) ;
  vdop = getFloat( &pData ) ;
  tdop = getFloat( &pData ) ;

  printf( "  PDOP:%.1f  HDOP:%.1f  VDOP:%.1f  TDOP:%.1f\n",
          pdop,
          hdop,
          vdop,
          tdop
        ) ;

} /* end of processPdopInfo() */



/**********************************************************************/
void processBriefSVInfo( int length, unsigned char *pData )
/**********************************************************************/
{
  int nSVs ;
  int i ;

  printf( "  GsofType:13 - SV Brief Info   len:%d\n",
          length
        ) ;

  nSVs = *pData++ ;
  printf( "  SvCount:%d\n", nSVs ) ;

  for ( i = 0 ; i < nSVs ; ++i )
  {
    int prn ;
    int flags1 ;
    int flags2 ;

    prn = *pData++ ;
    flags1 = *pData++ ;
    flags2 = *pData++ ;

    printf( "  Prn:%-2d  flags:%02X:%02X\n", prn, flags1, flags2 );
  }
} /* end of processBriefSVInfo */



/**********************************************************************/
void processAllBriefSVInfo( int length, unsigned char *pData )
/**********************************************************************/
{
  int nSVs ;
  int i ;

  printf( "  GsofType:33 - All SV Brief Info   len:%d\n",
          length
        ) ;

  nSVs = *pData++ ;
  printf( "  SvCount:%d\n", nSVs ) ;

  for ( i = 0 ; i < nSVs ; ++i )
  {
    int prn ;
    int system ;
    int flags1 ;
    int flags2 ;

    prn = *pData++ ;
    system = *pData++;
    flags1 = *pData++ ;
    flags2 = *pData++ ;

    printf( "  %s SV:%-2d  flags:%02X:%02X\n",
            system == 0 ? "GPS"
            : system == 1 ? "SBAS"
            : system == 2 ? "GLONASS"
            : system == 3 ? "GALILEO"
      : system == 4 ? "QZSS"
            : system == 5 ? "BEIDOU"
            : system == 6 ? "RESERVED" : "RESERVED",
            prn, flags1, flags2 );
  }
} /* end of processAllBriefSVInfo */



/**********************************************************************/
void processAllDetailedSVInfo( int length, unsigned char *pData )
/**********************************************************************/
{
  int nSVs ;
  int i ;

  printf( "  GsofType:34 - All SV Detailed Info   len:%d\n",
          length
        ) ;

  nSVs = *pData++ ;
   printf( "  SvCount:%d\n", nSVs ) ;

  for ( i = 0 ; i < nSVs ; ++i )
  {
    int prn ;
    int system ;
    int flags1 ;
    int flags2 ;
    int elevation ;
    int azimuth ;
    int snr[ 3 ];

    prn = *pData++ ;
    system = *pData++;
    flags1 = *pData++ ;
    flags2 = *pData++ ;
    elevation = *pData++ ;
    azimuth = getU16( &pData ) ;
    snr[ 0 ] = *pData++;
    snr[ 1 ] = *pData++;
    snr[ 2 ] = *pData++;

    #if 0
    {
      int ii ;
      for ( ii = 0 ; ii < length ; ++ii )
      {
          printf( "%02X%c", pData[ii], ii % 16 == 15 ? '\n' : ' ') ;
      }
      printf( "\n" ) ;
    }
    #endif

    printf( "  %s SV:%-2d  flags:%02X:%02X\n"
            "     El:%2d  Az:%3d\n"
            "     SNR %3s %5.2f\n"
            "     SNR %3s %5.2f\n"
            "     SNR %3s %5.2f\n",
            system == 0 ? "GPS"
            : system == 1 ? "SBAS"
            : system == 2 ? "GLONASS"
            : system == 3 ? "GALILEO"
      : system == 4 ? "QZSS"
            : system == 5 ? "BEIDOU"
            : system == 6 ? "RESERVED" : "RESERVED",
            prn, flags1, flags2,
            elevation, azimuth,
            system == 3 ? "E1 " : "L1 ", (float)snr[ 0 ] / 4.0,
            system == 3 ? "N/A " : "L2 ", (float)snr[ 1 ] / 4.0,
            system == 3 ? "E5 "
              : system == 2 ? "G1P" : "L5 ", (float)snr[ 2 ] / 4.0
          );

  }

} /* end of processAllDetailedSVInfo */



/**********************************************************************/
void processSvDetailedInfo( int length, unsigned char *pData )
/**********************************************************************/
{
  int nSVs ;
  int i ;

  printf( "  GsofType:14 - SV Detailed Info   len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  nSVs = *pData++ ;
  printf( "  SvCount:%d\n", nSVs ) ;

  for ( i = 0 ; i < nSVs ; ++i )
  {
    int prn ;
    int flags1 ;
    int flags2 ;
    int elevation ;
    int azimuth ;
    int l1Snr ;
    int l2Snr ;

    prn = *pData++ ;
    flags1 = *pData++ ;
    flags2 = *pData++ ;
    elevation = *pData++ ;
    azimuth = getU16( &pData ) ;
    l1Snr = *pData++ ;
    l2Snr = *pData++ ;

    printf( "   Prn:%-2d  flags:%02X:%02X elv:%-2d azm:%-3d  "
            "L1snr:%-5.2f L2snr:%-5.2f\n",
            prn,
            flags1,
            flags2,
            elevation,
            azimuth,
            ((double)l1Snr) / 4.0 ,
            ((double)l2Snr) / 4.0
          ) ;
  }
} /* end of processSvDetailedInfo() */



/**********************************************************************/
void processAttitudeInfo( int length , unsigned char *pData )
/**********************************************************************/
{
  double gpsTime ;
  unsigned char flags ;
  unsigned char nSVs ;
  unsigned char mode ;
  double pitch ;
  double yaw ;
  double roll ;
  double range ;
  double pdop ;

  printf( "  GsofType:27 - AttitudeInfo  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  gpsTime = (double)getU32( &pData ) / 1000.0 ;
  flags = *pData++ ;
  nSVs = *pData++ ;
  mode = *pData++ ;
  ++pData ; // reserved
  pitch = getDouble( &pData ) / PI * 180.0 ;
  yaw   = getDouble( &pData ) / PI * 180.0 ;
  roll  = getDouble( &pData ) / PI * 180.0 ;
  range = getDouble( &pData ) ;

  pdop  = (double)getU16( &pData ) / 10.0 ;

  printf( "  Time:%.3f"
          " flags:%02X"
          " nSVs:%d"
          " mode:%d\n"
          "  pitch:%.3f"
          " yaw:%.3f"
          " roll:%.3f"
          " range:%.3f"
          " pdop:%.1f"
          "\n",
          gpsTime,
          flags,
          nSVs,
          mode,
          pitch,
          yaw,
          roll,
          range,
          pdop
        ) ;

  // Detect if the extended record information is present
  if ( length > 42 )
  {
    float pitch_var ;
    float yaw_var ;
    float roll_var ;
    float pitch_yaw_covar ;
    float pitch_roll_covar ;
    float yaw_roll_covar ;
    float range_var;

    // The variances are in units of radians^2
    pitch_var = getFloat( &pData ) ;
    yaw_var   = getFloat( &pData ) ;
    roll_var  = getFloat( &pData ) ;

    // The covariances are in units of radians^2
    pitch_yaw_covar  = getFloat( &pData ) ;
    pitch_roll_covar = getFloat( &pData ) ;
    yaw_roll_covar   = getFloat( &pData ) ;

    // The range variance is in units of m^2
    range_var = getFloat( &pData ) ;

    printf( "  variance (radians^2)"
            " pitch:%.4e"
            " yaw:%.4e"
            " roll:%.4e"
            "\n",
            pitch_var,
            yaw_var,
            roll_var ) ;

    printf( "  covariance (radians^2)"
            " pitch-yaw:%.4e"
            " pitch-roll:%.4e"
            " yaw-roll:%.4e"
            "\n",
            pitch_yaw_covar,
            pitch_roll_covar,
            yaw_roll_covar ) ;

    printf( "  variance (m^2)"
            " range: %.4e"
            "\n",
            range_var ) ;
  }

} /* end of processAttitudeInfo() */


/**********************************************************************/
void processLbandStatus( int length , unsigned char *pData )
/**********************************************************************/
{
  unsigned char name[5];
  float freq;
  unsigned short bit_rate;
  float snr;
  unsigned char hp_xp_subscribed_engine;
  unsigned char hp_xp_library_mode;
  unsigned char vbs_library_mode;
  unsigned char beam_mode;
  unsigned char omnistar_motion;
  float horiz_prec_thresh;
  float vert_prec_thresh;
  unsigned char nmea_encryption;
  float iq_ratio;
  float est_ber;
  unsigned long total_uw;
  unsigned long total_bad_uw;
  unsigned long total_bad_uw_bits;
  unsigned long total_viterbi;
  unsigned long total_bad_viterbi;
  unsigned long total_bad_messages;
  unsigned char meas_freq_is_valid = -1;
  double        meas_freq = 0.0;

  printf( "  GsofType:40 - LBAND status  len:%d\n",
          length
        ) ;

  memcpy( name, pData, 5 );
  pData += 5;
  freq = getFloat( &pData );
  bit_rate = getU16( &pData );
  snr = getFloat( &pData );
  hp_xp_subscribed_engine = *pData++;
  hp_xp_library_mode = *pData++;
  vbs_library_mode = *pData++;
  beam_mode = *pData++;
  omnistar_motion = *pData++;
  horiz_prec_thresh = getFloat( &pData );
  vert_prec_thresh = getFloat( &pData );
  nmea_encryption = *pData++;
  iq_ratio = getFloat( &pData );
  est_ber = getFloat( &pData );
  total_uw = getU32( &pData );
  total_bad_uw = getU32( &pData );
  total_bad_uw_bits = getU32( &pData );
  total_viterbi = getU32( &pData );
  total_bad_viterbi = getU32( &pData );
  total_bad_messages = getU32( &pData );
  if( length > 61 )
  {
    meas_freq_is_valid = *pData++;
    meas_freq = getDouble( &pData );
  }

  printf( "  Name:%s"
          "  Freq:%g"
          "  bit rate:%d"
          "  SNR:%g"
          "\n"
          "  HP/XP engine:%d"
          "  HP/XP mode:%d"
          "  VBS mode:%d"
          "\n"
          "  Beam mode:%d"
          "  Omnistar Motion:%d"
          "\n"
          "  Horiz prec. thresh.:%g"
          "  Vert prec. thresh.:%g"
          "\n"
          "  NMEA encryp.:%d"
          "  I/Q ratio:%g"
          "  Estimated BER:%g"
          "\n"
          "  Total unique words(UW):%d"
          "  Bad UW:%d"
          "  Bad UW bits:%d"
          "\n"
          "  Total Viterbi:%d"
          "  Corrected Viterbi:%d"
          "  Bad messages:%d"
          "\n"
          "  Meas freq valid?:%d"
          "  Meas freq:%.3f"
          "\n"
          ,
          name,
          freq,
          bit_rate,
          snr,
          hp_xp_subscribed_engine,
          hp_xp_library_mode,
          vbs_library_mode,
          beam_mode,
          omnistar_motion,
          horiz_prec_thresh,
          vert_prec_thresh,
          nmea_encryption,
          iq_ratio,
          est_ber,
          total_uw,
          total_bad_uw,
          total_bad_uw_bits,
          total_viterbi,
          total_bad_viterbi,
          total_bad_messages,
          meas_freq_is_valid,
          meas_freq
        ) ;

} /* end of processLbandStatus() */


/***********************************************************************
 * End of the GSOF subtype parsers.
 **********************************************************************
 */



/**********************************************************************/
void processGsofData( void )
/**********************************************************************/

{
  int i ;
  int gsofType ;
  int gsofLength ;
  unsigned char * pData ;

  printf( "\nGSOF Records\n" ) ;
  pData = gsofData ;

  while (pData < gsofData + 105 )
  {
    gsofType   = *pData++ ;
    gsofLength = *pData++ ;

    // If the type is one that we know about, then call the specific
    // parser for that type.
    if ( gsofType == 1 )
    {
      processPositionTime( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 2 )
    {
      processLatLonHeight( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 3 )
    {
      processECEF( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 4 )
    {
      processLocalDatum( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 8 )
    {
      processVelocityData( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 9 )
    {
      processPdopInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 13 )
    {
      processBriefSVInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 16 )
    {
      processUtcTime( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 33 )
    {
      processAllBriefSVInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 34 )
    {
      processAllDetailedSVInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 14 )
    {
      processSvDetailedInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 27 )
    {
      processAttitudeInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 6 )
    {
      processEcefDelta( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 7 )
    {
      processTangentPlaneDelta( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 40 )
    {
      processLbandStatus( gsofLength, pData ) ;
      pData += gsofLength ;
    }
     else
    if ( gsofType == 49 )
    {
      processInsFullNavigation( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    {
      // Not a type we know about.  Hex dump the bytes and move on.
      printf( "  GsofType:%d  len:%d\n  ",
              gsofType,
              gsofLength
            ) ;

      for ( i = 0 ; i < gsofLength ; ++i )
      {
        printf( "%02X%s",
                *pData++,
                i % 16 == 15 ? "\n  " : " "
              ) ;
      }
      // Terminate the last line if needed.
      if (gsofLength %16 != 0)
        printf( "\n" ) ;
    }

    printf( "\n" ) ;
  }
  printf( "\n" ) ;

} /* end of processGsofData() */



void postGsofData( unsigned char * pData, int length )
/**********************************************************************/

{

  int i ;

  *pData++ ;
  *pData++ ;
  *pData++ ;
  *pData++ ;
  *pData++ ;
  *pData++ ;
  *pData++ ;
  memset(gsofData, 0, sizeof(gsofData));
  // Transfer the data bytes in this portion to the global buffer.
  for (i = 7 ; i < length-3 ; ++i)
   {

     gsofData[ gsofDataIndex++ ] = *pData++ ;
     if(gsofDataIndex == 105)
      {
         gsofDataIndex = 0 ;
      }

   }
  for(int j=0;j<105;j++)
   {
     ROS_INFO("[0x%02x]",gsofData[j]);
   }
  ROS_INFO_STREAM("gsofData type");
  processGsofData() ;

} /* end of postGsofData() */

/*
//回调函数
void write_callback(const std_msgs::String::ConstPtr& msg)
{
   ROS_INFO_STREAM("Writing to serial port" <<msg->data);
   ser.write(msg->data);   //发送串口数据
}
*/

int main (int argc, char** argv)
{
//初始化节点
   ros::init(argc, argv, "serial_node");
//声明节点句柄
   ros::NodeHandle nh;
//订阅主题，并配置回调函数
//   ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
//发布主题
   //Imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_raw", 1000);  //NDT需要输入的话题为/imu_raw /imu/data
   //Imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 1000);  //lego_loam需要输入的话题为/imu/data
   Imu_pub = nh.advertise<sensor_msgs::Imu>("imu_correct", 1000);  //lego_loam需要输入的话题为/imu/data

   fix_pub = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 1000);

try
 {
//设置串口属性，并打开串口
   ser.setPort("/dev/ttyUSB0");
   ser.setBaudrate(115200);
   serial::Timeout to = serial::Timeout::simpleTimeout(1000);
   ser.setTimeout(to);
   ser.open();
 }
catch (serial::IOException& e)
 {
   ROS_ERROR_STREAM("Unable to open port ");
   return -1;
 }
//检测串口是否已经打开，并给出提示信息
if(ser.isOpen())
 {
   ROS_INFO_STREAM("Serial Port initialized");
 }
else
 {
   return -1;
 }


//指定循环的频率
ros::Rate loop_rate(50);
ros_serial::serial msg;
while(ros::ok())
 {
  if(ser.available())
  {
    ROS_INFO_STREAM("Reading from serial port\n");
          ser.read(r_buffer,rBUFFERSIZE);

          postGsofData( r_buffer, rBUFFERSIZE ) ; //type 49
          //  for(int i=0;i<rBUFFERSIZE;i++)
          //  {
          //      ROS_INFO("[0x%02x]",r_buffer[i]);
          //  }
          //  ROS_INFO_STREAM("End reading from serial port");

    } 

//处理ROS的信息，比如订阅消息,并调用回调函数

  //ros::spinOnce();
  loop_rate.sleep();
  }
 exit(0);
}


