#ifndef DIFFDRIVE_CONFIG_H
#define DIFFDRIVE_CONFIG_H

#include <string>


struct Config
{
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float wheel_separation = 0.5;
  float wheel_radius = 0.1;
  
  std::string agv_ip;    // The IP address or hostname of the AGV.
  int agv_port;          // The port number of the AGV.
  std::string local_ip;  // The local IP address to bind the UDP socket.
  int local_port;        // The local port number to bind the UDP socket.
};

#endif // DIFFDRIVE_CONFIG_H