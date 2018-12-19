
#include <async_comm/serial.h>

#include <cstdint>
#include <cstdio>

#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <signal.h>
#include <mutex.h>


std::mutex radio_mutex;


#define NUM_BYTES 200


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
    gps.serial.send_bytes(buf, len);

    std::printf("radio received %d bytes",(int)len);
  }
}


int main(int argc, char** argv)
{
  ros::init("rtkrover");

  // open serial port
  async_comm::Serial gps_serial("/dev/ttyUSB0", 57600);
  gps_serial.register_receive_callback(&gps_callback);
  if (!gps_serial.init())
  {
    std::printf("Failed to initialize gps serial port\n");
    return 2;
  }


  async_comm::Serial radio_serial("/dev/ttyAMA0", 19200);
  radio_serial.register_receive_callback(&radio_callback);
  if (!radio_serial.init())
  {
    std::printf("Failed to initialize radio serial port\n");
    return 2;
  }

  while (ros::ok())
  {

      ros::spin();

  } 

  // close serial port
  radio_serial.close();
  gps_serial.close();

  return 0;
}
