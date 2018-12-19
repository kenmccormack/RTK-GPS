
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
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <string>

using namespace boost; 

#define GPS_SERIAL

std::mutex radio_mutex;


#define NUM_BYTES 200

async_comm::Serial gps_serial("/dev/ttyUSB0", 57600);

std::string msg_buffer; 


void process_gps_msg(std::string msg)
{
  std::vector<std::string> msg_elements; 
  //ROS_INFO("GPS message %s", msg.c_str());
  char_separator<char> sep(",");
  tokenizer< char_separator<char> > tokens(msg, sep);
  BOOST_FOREACH (const std::string& t, tokens) 
  {
    msg_elements.push_back(t); 
  }

  ROS_INFO("Token count = %d", (int)msg_elements.size());

  if(msg_elements.size() > 1)
  {
    ROS_INFO("%s",msg_elements[0].c_str());
    if (msg_elements[0] == "$PSTI" && msg_elements[1]=="032")
    {
      if (msg_elements[4]=="A")
      {
          ROS_INFO("Valid Position --> East: %s North: %s Up: %s", 
            msg_elements[6].c_str(),
            msg_elements[7].c_str(),
            msg_elements[8].c_str());
      }
      else
      {
        ROS_INFO("Invalid Position");
      }
    }
  }


}

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
    //std::printf("%c", buf[i]);
    if (buf[i] == '$')
    {
      process_gps_msg(msg_buffer);
      msg_buffer.clear();
    }
    msg_buffer.push_back(buf[i]);

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
