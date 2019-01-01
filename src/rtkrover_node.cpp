
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
ros::NodeHandle * nhp; 
ros::Publisher pub_odm;

std::string msg_buffer; 



class DecodeBinaryStream
{
  public:

  

    char last_byte;
    int state;
    char body_length[2];
    char body_buf[500];
    int body_index; 

    DecodeBinaryStream()
    {
      last_byte=0;
      state = HEADER;
      body_index=0;
    }

    void decode(const unsigned char *buf, int len)
    {
    //parse the protocol*/ 
      for(int ii=0;ii<len;ii++)
      {
        switch(state)
        {
          case HEADER: 
            if (last_byte == 0xA0 && buf[ii]==0xA1)
                state = LENGTH1;
            break;

          case LENGTH1:
            body_length[0] = buf[ii];
            state = LENGTH2;
            break;

          case LENGTH2:
            body_length[1] = buf[ii];
            state = BODY;
            body_index = 0; 
            break;

          case BODY:
            body_buf[body_index] = buf[ii];
            body_index++;
            if(body_index >= ( body_length[0]*256 + body_length[1]) )
              state = CRC;
            break;

          case CRC:
            if(body_buf[0] == 0xdf)
            {
              printf("Message id 0x%x, len=%d, (0x%x,0x%x,0x%x,0x%x)\n",
                    body_buf[0],
                    (body_length[0]*256 + body_length[1]) ,
                    body_buf[1],
                    body_buf[2],
                    body_buf[3],
                    body_buf[4] );
            }
            state = HEADER;

            break;

          default :
            state = HEADER;
            break;
        }

      last_byte = buf[ii];
      
      } 
    }

    private:
      enum
      {
        HEADER,
        LENGTH1,
        LENGTH2,
        BODY,
        CRC
      };

};


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

  if(msg_elements.size() > 1)
  {
    
    if (msg_elements[0] == "$PSTI" && msg_elements[1]=="032")
    {

      ROS_INFO("GPS message %s", msg.c_str());

      /*for (int ii=0 ; ii<msg_elements.size(); ii++)
	ROS_INFO("%d %s",ii,msg_elements[ii].c_str());
      */

      if (msg_elements[4]=="A")
      {
          ROS_INFO("Valid Position --> East: %s North: %s Up: %s", 
            msg_elements[6].c_str(),
            msg_elements[7].c_str(),
            msg_elements[8].c_str());
          
          /*go ahead and publish this message to ROS*/ 
          nav_msgs::Odometry odm_msg;
          odm_msg.header.stamp = ros::Time::now(); 
          odm_msg.header.frame_id = "rover";
          odm_msg.pose.pose.position.x = stof(msg_elements[6]);
          odm_msg.pose.pose.position.y = stof(msg_elements[7]);
          odm_msg.pose.pose.position.z = stof(msg_elements[8]);

           
          pub_odm.publish(odm_msg);

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
     static DecodeBinaryStream dbs;
     static int rx_count;

    /*send to the gps input */ 
#ifdef GPS_SERIAL
    gps_serial.send_bytes(buf, len);
#endif

    //#dbs.decode(buf,len);

    if (rx_count%255 == 0)
    {
	    ROS_INFO("Rx'd rtk bytes.");
    } 
    rx_count++;

}




int main(int argc, char** argv)
{
  ros::init(argc , argv, "rtkrover");

  ros::NodeHandle nh; 
  nhp = &nh;

  pub_odm = nh.advertise<nav_msgs::Odometry>("/rover_odm", 10 );
      

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
    std::printf("GPS Serial has been initialized\n");
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
