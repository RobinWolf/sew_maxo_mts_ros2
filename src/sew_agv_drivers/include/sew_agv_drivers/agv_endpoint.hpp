#ifndef AGV_ENDPOINT_HPP
#define AGV_ENDPOINT_HPP

#include <array>
#include <vector>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "tx.hpp"
#include "rx.hpp"

// Class representing the AGV endpoint for communication
class AgvEndpoint {
public:
    // Constructor initializes member variables
    AgvEndpoint() : udpTx_(-1), udpRx_(-1), connected_(false) {}

    // Destructor ensures sockets are closed upon object destruction
    ~AgvEndpoint() {
        disconnect();
    }

    // Connect to the AGV with provided IP addresses and ports
    bool connect(const std::string& agv_ip, int agv_port, const std::string& local_ip, int local_port) {
        // std::cout << "(AGVEndpoint) connect()" << std::endl;
        // Disconnect any existing connection
        disconnect();

        // Store the connection details
        this->agv_ip_ = agv_ip;
        this->agv_port_ = agv_port;
        this->local_ip_ = local_ip;
        this->local_port_ = local_port;

        // std::cout << "(AGVEndpoint) AGV IP: " << agv_ip_ << std::endl;
        // std::cout << "(AGVEndpoint) AGV Port: " << agv_port_ << std::endl;
        // std::cout << "(AGVEndpoint) Local IP: " << local_ip_ << std::endl;
        // std::cout << "(AGVEndpoint) Local Port: " << local_port_ << std::endl;

        // Create UDP sockets for transmission and reception
        udpTx_ = socket(AF_INET, SOCK_DGRAM, 0);
        udpRx_ = socket(AF_INET, SOCK_DGRAM, 0);

        // std::cout << "(AGVEndpoint) UDP Tx Socket: " << udpTx_ << std::endl;
        // std::cout << "(AGVEndpoint) UDP Rx Socket: " << udpRx_ << std::endl;

        // Check if socket creation was successful
        if (udpTx_ < 0 || udpRx_ < 0) {
            std::cerr << "(AGVEndpoint) Failed to create sockets" << std::endl;
            return false;
        }

        // add timeout for receiving data to dont block the programm
        struct timeval timeout;
        timeout.tv_sec =0;  // Timeout in Sekunden
        timeout.tv_usec = 200000; // Timeout in Mikrosekunden
        setsockopt(udpRx_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));


        // std::cout << "(AGVEndpoint) Set up the local address structure "<< std::endl;
        // Set up the local address structure
        sockaddr_in localAddr {};
        localAddr.sin_family = AF_INET;
        localAddr.sin_addr.s_addr = inet_addr(local_ip_.c_str());
        localAddr.sin_port = htons(local_port_);
        // std::cout << "(AGVEndpoint) Local Address Data: " << std::endl;
        // std::cout << "Family: " << localAddr.sin_family << std::endl;
        // std::cout << "IP Address: " << inet_ntoa(localAddr.sin_addr) << std::endl;
        // std::cout << "Port: " << ntohs(localAddr.sin_port) << std::endl;


        std::cout << "(AGVEndpoint) Bind the receiving socket to the local address "<< std::endl;
        // Bind the receiving socket to the local address
        if (bind(udpRx_, reinterpret_cast<struct sockaddr*>(&localAddr), sizeof(localAddr)) < 0) {
            //std::cerr << "(AGVEndpoint) Failed to bind the socket" << std::endl;
            perror("(AGVEndpoint) Failed to bind the socket");
            return false;
        }

        // std::cout << "(AGVEndpoint) Create a start message to send to the AGV "<< std::endl;
        // Create a start message to send to the AGV
        StartTxMsg startMsg;
        startMsg.setIP(parseIp(local_ip_));
        startMsg.setPort(local_port_);

        // std::cout << "(AGVEndpoint) Encode the start message "<< std::endl;
        // Encode the start message
        std::vector<uint8_t> buf = startMsg.encode();
        sockaddr_in agvAddr {};
        agvAddr.sin_family = AF_INET;
        agvAddr.sin_addr.s_addr = inet_addr(agv_ip_.c_str());
        agvAddr.sin_port = htons(agv_port);

        // std::cout << "(AGVEndpoint) Send start message until a response is received "<< std::endl;
        // Send start message until a response is received
        while (!connected_) {
            // std::cout << "(AGVEndpoint) Send start message "<< std::endl;
            sendDataToAgv(buf);  // Send start message
            buf.resize(1024);
            sockaddr_in senderAddr {};
            socklen_t addrLen = sizeof(senderAddr);
            
            // std::cout << "(AGVEndpoint) Try to receive a response?? "<< std::endl;
            // Receive a response
            int len = recvfrom(udpRx_, buf.data(), buf.size(), 0, reinterpret_cast<struct sockaddr*>(&senderAddr), &addrLen);
            if (len > 0) {
                connected_ = true;
                handleRx(std::vector<uint8_t>(buf.begin(), buf.begin() + len));
                return true;
            } else {
                handleRx({});
            }
        }

        return false;
    }

    // Disconnect from the AGV and close the sockets
    void disconnect() {
        std::cout << "(AGVEndpoint) disconnect()" << std::endl;
        if (udpTx_ >= 0) {
            close(udpTx_);
            udpTx_ = -1;
        }
        if (udpRx_ >= 0) {
            close(udpRx_);
            udpRx_ = -1;
        }
        connected_ = false;
    }

    bool isConnected() const {
        return connected_;
    }

    // Send a control message to the AGV
    bool sendControlToAGV(float speed, float x, float y, ManualJogTxMsg::SpeedMode speed_mode = ManualJogTxMsg::SpeedMode::RAPID) {
        // std::cout << "\n(AGVEndpoint) sendControlToAGV()" << std::endl;

        // Check if the AGV is connected --> not needed because the function is only called if the AGV is connected
        // if (!connected_) {
        //     std::cerr << "AGV is not connected" << std::endl;
        //     return false;
        // }
      
        // Create and set up the control message
        ManualJogTxMsg msg;
        msg.setSpeedMode(speed_mode);
        msg.setSpeed(speed);
        msg.setDirection(x, y);

        // Parse and set the IP address and port
        msg.setIP(parseIp(agv_ip_));
        msg.setPort(agv_port_);

        // std::cout << "(AGVEndpoint) Message Data:" << std::endl;
        // std::cout << "(AGVEndpoint) Speed Mode: " << static_cast<int>(speed_mode) << std::endl;
        // std::cout << "(AGVEndpoint) Speed: " << speed << std::endl;
        // std::cout << "(AGVEndpoint) Direction X: " << x << std::endl;
        // std::cout << "(AGVEndpoint) Direction Y: " << y << std::endl;
        // std::cout << "(AGVEndpoint) IP Address: " << agv_ip_ << std::endl;
        // std::cout << "(AGVEndpoint) Port: " << agv_port_ << std::endl;

        // Encode the message
        std::vector<uint8_t> encoded_msg = msg.encode();

        // Set up the server address
        struct sockaddr_in server_addr;
        std::memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(agv_port_);
        inet_pton(AF_INET, agv_ip_.c_str(), &server_addr.sin_addr);

        // Send the message
        ssize_t sent_bytes = sendto(udpTx_, encoded_msg.data(), encoded_msg.size(), 0,
                                    (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (sent_bytes < 0) {
            perror("sendto error");
            return false;
        } 
        // else {
        //     std::cout << "(AGVEndpoint) Sent " << sent_bytes << " bytes to " << agv_ip_ << ":" << agv_port_ << std::endl;
        //     return true;
        // }
    }

    // Query and print the status of the AGV
    bool getStatusAGV() {
        // std::cout << "\n(AGVEndpoint) getStatusAGV():" << std::endl;
        if (!connected_) {
            std::cerr << "AGVEndpoint) AGV is not connected" << std::endl;
            return false;
        }

        // Buffer to hold the received data
        std::array<uint8_t, 54> buf;
        sockaddr_in senderAddr {};
        socklen_t addrLen = sizeof(senderAddr);

        std::cout << "(AGVEndpoint) Try to recive data" << std::endl;
        // Receive data from the AGV
        int len = recvfrom(udpRx_, buf.data(), buf.size(), 0, reinterpret_cast<struct sockaddr*>(&senderAddr), &addrLen);
        if (len > 0) {
            std::cerr << "(AGVEndpoint) Received status from AGV" << std::endl;
            handleRx(std::vector<uint8_t>(buf.begin(), buf.begin() + len));
            // Decode and print the header and message data
            AgvRxHeader header;
            //header.decode({buf.begin(), buf.begin() + 14});
            std::array<uint8_t, 14> header_buf;
            std::copy(buf.begin(), buf.begin() + 14, header_buf.begin());
            header.decode(header_buf);

            MonitorRxMsg msg;
            msg.decode(buf);

            // Print AGV status
            // std::cout << "(AGVEndpoint) AGV Status:" << std::endl;
            // std::cout << "(AGVEndpoint) State: " << static_cast<int>(header.state) << std::endl;
            // std::cout << "(AGVEndpoint) Color: " << static_cast<int>(header.color) << std::endl;
            // std::cout << "(AGVEndpoint) Current Page: " << static_cast<int>(header.current_page) << std::endl;
            // std::cout << "(AGVEndpoint) Error: " << header.error << std::endl;
            // std::cout << "(AGVEndpoint) Error Code: " << header.error_code << std::endl;
            // std::cout << "(AGVEndpoint) Part Data: " << msg.part_data << std::endl;
            // std::cout << "(AGVEndpoint) In Station: " << msg.in_station << std::endl;
            // std::cout << "(AGVEndpoint) In Station State: " << msg.in_station_state << std::endl;
            // std::cout << "(AGVEndpoint) Transponder: " << msg.transponder << std::endl;
            // std::cout << "(AGVEndpoint) Transponder Distance: " << msg.transponder_distance << std::endl;
            // std::cout << "(AGVEndpoint) V-Track: " << msg.v_track << std::endl;
            // std::cout << "(AGVEndpoint) V-Track Distance: " << msg.v_track_distance << std::endl;
            // std::cout << "(AGVEndpoint) Actual Speed: " << msg.actual_speed << std::endl;
            // std::cout << "(AGVEndpoint) Target Speed: " << msg.target_speed << std::endl;
            // std::cout << "(AGVEndpoint) Speed Limit: " << msg.speed_limit << std::endl;
            // std::cout << "(AGVEndpoint) Charging State: " << msg.charging_state << std::endl;
            // std::cout << "(AGVEndpoint) Power: " << msg.power << std::endl;

            return true;
        } 
        else {
            std::cerr << "(AGVEndpoint) Failed to receive status from AGV" << std::endl;
            return false;
        }
    }

private:
    // Member variables for AGV connection details
    std::string agv_ip_;  // AGV IP address
    int agv_port_;        // AGV port
    std::string local_ip_; // Local IP address
    int local_port_;      // Local port
    int udpTx_;           // UDP socket for transmitting data
    int udpRx_;           // UDP socket for receiving data
    bool connected_;      // Connection status flag

    // Helper method to send data to the AGV
    void sendDataToAgv(const std::vector<uint8_t>& buf) {
        // Set up the AGV address structure
        struct sockaddr_in agvAddr {};
        agvAddr.sin_family = AF_INET;
        agvAddr.sin_addr.s_addr = inet_addr(agv_ip_.c_str());
        agvAddr.sin_port = htons(agv_port_);

        // Send data using the transmission socket
        ssize_t sent_bytes = sendto(udpTx_, buf.data(), buf.size(), 0,
                                    (struct sockaddr*)&agvAddr, sizeof(agvAddr));
        if (sent_bytes < 0) {
            perror("sendto");
        } else {
            std::cout << "Sent " << sent_bytes << " bytes to " << agv_ip_ << ":" << agv_port_ << std::endl;
        }
    }

    // Helper method to parse an IP address string into an array of bytes
    std::array<uint8_t, 4> parseIp(const std::string& ip) {
        std::array<uint8_t, 4> arr;
        sscanf(ip.c_str(), "%hhu.%hhu.%hhu.%hhu", &arr[0], &arr[1], &arr[2], &arr[3]);
        return arr;
    }

    // Handler for received data from the AGV
    void handleRx(const std::vector<uint8_t>& data) {
        // Process the received data (implementation depends on protocol)
        // For now, just print the size of the received data
        std::cout << "Received data of size: " << data.size() << std::endl;
    }
};

#endif // AGV_ENDPOINT_HPP
