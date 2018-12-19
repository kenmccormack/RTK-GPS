
#include <async_comm/serial.h>

#include <cstdint>
#include <cstdio>

#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <signal.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <iostream>


#define GPS_SERIAL

std::mutex radio_mutex;


#define NUM_BYTES 200

async_comm::Serial gps_serial("/dev/ttyUSB0", 57600);



/**
 * @brief Callback function for the async_comm library
 *
 * Prints the received bytes to stdout.
 *
 * @param buf Received bytes buffer
 * @param len Number of bytes received
 */
void gps_callback(const uint8_t* buf, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    std::printf("%c", buf[i]);
  }
}

void radio_callback(const uint8_t* buf, size_t len)
{

    /*send to the gps input */ 
#ifdef GPS_SERIAL
    gps_serial.send_bytes(buf, len);
#endif
  
    //std::printf("radio received %d bytes",(int)len);
  
}


int main(int argc, char** argv)
{
  ros::init(argc , argv, "rtkrover");



  // open serial port
  
#ifdef GPS_SERIAL
  gps_serial.register_receive_callback(&gps_callback);
  if (!gps_serial.init())
  {
    std::printf("Failed to initialize gps serial port\n");
    return 2;
  }
  else
  {
    std::printf("GPS Serial has been initialized");
  }
#endif 


  async_comm::Serial radio_serial("/dev/ttyAMA0", 19200);
  radio_serial.register_receive_callback(&radio_callback);
  if (!radio_serial.init())
  {
    std::printf("Failed to initialize radio serial port\n");
    return 2;
  }
  else
  {
    std::printf("Radio serial port found and initialized\n");
  }

  while (ros::ok())
  {
      ros::spin();
  } 

  // close serial port
  radio_serial.close();
#ifdef GPS_SERIAL
  gps_serial.close();
#endif
  

  return 0;
}
