#ifndef DIFFDRIVE_CONFIG_H
#define DIFFDRIVE_CONFIG_H

#include <string>


struct Config
{
//   std::string left_wheel_name = "left_wheel";
//   std::string right_wheel_name = "right_wheel";
//   float loop_rate = 30;
//   std::string device = "/dev/ttyUSB0";
//   int baud_rate = 57600;
//   int timeout = 1000;
//   int enc_counts_per_rev = 1920;
  
  std::string agvHost;  // The IP address or hostname of the AGV.
  int agvPort;          // The port number of the AGV.
  std::string localIp;  // The local IP address to bind the UDP socket.
  int localPort;        // The local port number to bind the UDP socket.
};

#endif // DIFFDRIVE_CONFIG_H