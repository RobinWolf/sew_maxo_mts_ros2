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
        // Disconnect any existing connection
        disconnect();

        // Store the connection details
        this->agv_ip_ = agv_ip;
        this->agv_port_ = agv_port;
        this->local_ip_ = local_ip;
        this->local_port_ = local_port;

        // Create UDP sockets for transmission and reception
        udpTx_ = socket(AF_INET, SOCK_DGRAM, 0);
        udpRx_ = socket(AF_INET, SOCK_DGRAM, 0);

        // Check if socket creation was successful
        if (udpTx_ < 0 || udpRx_ < 0) {
            std::cerr << "Failed to create sockets" << std::endl;
            return false;
        }

        // Set up the local address structure
        sockaddr_in localAddr {};
        localAddr.sin_family = AF_INET;
        localAddr.sin_addr.s_addr = inet_addr(local_ip_.c_str());
        localAddr.sin_port = htons(local_port_);

        // Bind the receiving socket to the local address
        if (bind(udpRx_, reinterpret_cast<struct sockaddr*>(&localAddr), sizeof(localAddr)) < 0) {
            std::cerr << "Failed to bind the socket" << std::endl;
            return false;
        }

        // Create a start message to send to the AGV
        StartTxMsg startMsg;
        startMsg.setIP(parseIp(local_ip_));
        startMsg.setPort(local_port_);

        // Encode the start message
        std::vector<uint8_t> buf = startMsg.encode();
        sockaddr_in agvAddr {};
        agvAddr.sin_family = AF_INET;
        agvAddr.sin_addr.s_addr = inet_addr(agv_ip_.c_str());
        agvAddr.sin_port = htons(agv_port);

        // Send start message until a response is received
        while (!connected_) {
            sendDataToAgv(buf);  // Send start message
            buf.resize(1024);
            sockaddr_in senderAddr {};
            socklen_t addrLen = sizeof(senderAddr);

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
    void sendControlToAGV(float speed, float x, float y, ManualJogTxMsg::SpeedMode speed_mode = ManualJogTxMsg::SpeedMode::CREEP) {
        // Create and set up the control message
        ManualJogTxMsg msg;
        msg.setSpeedMode(speed_mode);
        msg.setSpeed(speed);
        msg.setDirection(x, y);

        // Parse and set the IP address and port
        msg.setIP(parseIp(agv_ip_));
        msg.setPort(agv_port_);

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
            perror("sendto");
        } else {
            std::cout << "Sent " << sent_bytes << " bytes to " << agv_ip_ << ":" << agv_port_ << std::endl;
        }
    }

    // Query and print the status of the AGV
    // void getStatusAGV() {
    //     if (!connected_) {
    //         std::cerr << "AGV is not connected" << std::endl;
    //         return;
    //     }

    //     // Buffer to hold the received data
    //     std::array<uint8_t, 54> buf;
    //     sockaddr_in senderAddr {};
    //     socklen_t addrLen = sizeof(senderAddr);

    //     // Receive data from the AGV
    //     int len = recvfrom(udpRx_, buf.data(), buf.size(), 0, reinterpret_cast<struct sockaddr*>(&senderAddr), &addrLen);
    //     if (len > 0) {
    //         // Decode and print the header and message data
    //         AgvRxHeader header;
    //         header.decode({buf.begin(), buf.begin() + 14});

    //         MonitorRxMsg msg;
    //         msg.decode(buf);

    //         // Print AGV status
    //         std::cout << "AGV Status:" << std::endl;
    //         std::cout << "State: " << static_cast<int>(header.state) << std::endl;
    //         std::cout << "Color: " << static_cast<int>(header.color) << std::endl;
    //         std::cout << "Current Page: " << static_cast<int>(header.current_page) << std::endl;
    //         std::cout << "Error: " << header.error << std::endl;
    //         std::cout << "Error Code: " << header.error_code << std::endl;

    //         std::cout << "Part Data: " << msg.part_data << std::endl;
    //         std::cout << "In Station: " << msg.in_station << std::endl;
    //         std::cout << "In Station State: " << msg.in_station_state << std::endl;
    //         std::cout << "Transponder: " << msg.transponder << std::endl;
    //         std::cout << "Transponder Distance: " << msg.transponder_distance << std::endl;
    //         std::cout << "V-Track: " << msg.v_track << std::endl;
    //         std::cout << "V-Track Distance: " << msg.v_track_distance << std::endl;
    //         std::cout << "Actual Speed: " << msg.actual_speed << std::endl;
    //         std::cout << "Target Speed: " << msg.target_speed << std::endl;
    //         std::cout << "Speed Limit: " << msg.speed_limit << std::endl;
    //         std::cout << "Charging State: " << msg.charging_state << std::endl;
    //         std::cout << "Power: " << msg.power << std::endl;
    //     } else {
    //         std::cerr << "Failed to receive status from AGV" << std::endl;
    //     }
    // }

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
