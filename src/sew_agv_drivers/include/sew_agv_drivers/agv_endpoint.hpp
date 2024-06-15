#ifndef AGV_ENDPOINT_HPP
#define AGV_ENDPOINT_HPP

#include <vector>
#include <iostream>
#include <stdexcept>
#include <arpa/inet.h>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>
#include "tx.hpp"
#include "rx.hpp"

class AgvEndpoint {
public:
    /**
     * Constructor to initialize the Endpoint object.
     */
    AgvEndpoint() : udpTx(-1), udpRx(-1), connected(false) {}

    /**
     * Destructor to close the sockets.
     */
    ~AgvEndpoint() {
        disconnect();
    }

    /**
     * Connect to the AGV.
     * 
     * @param agvHost The IP address or hostname of the AGV.
     * @param agvPort The port number of the AGV.
     * @param localIp The local IP address to bind the UDP socket.
     * @param localPort The local port number to bind the UDP socket.
     * @return True if connection is successful, false otherwise.
     */
    bool connect(const std::string& agvHost, int agvPort, const std::string& localIp, int localPort) {
        // Close any existing connections
        disconnect();

        this->agvHost = agvHost;
        this->agvPort = agvPort;
        this->localIp = localIp;
        this->localPort = localPort;

        // Initialize UDP sockets
        udpTx = socket(AF_INET, SOCK_DGRAM, 0);
        udpRx = socket(AF_INET, SOCK_DGRAM, 0);

        if (udpTx < 0 || udpRx < 0) {
            std::cerr << "Failed to create sockets" << std::endl;
            return false;
        }

        sockaddr_in localAddr {};
        localAddr.sin_family = AF_INET;
        localAddr.sin_addr.s_addr = inet_addr(localIp.c_str());
        localAddr.sin_port = htons(localPort);

        if (bind(udpRx, reinterpret_cast<struct sockaddr*>(&localAddr), sizeof(localAddr)) < 0) {
            std::cerr << "Failed to bind the socket" << std::endl;
            return false;
        }

        StartTxMsg startMsg;
        startMsg.setIP(parseIp(localIp));
        startMsg.setPort(localPort);

        std::vector<uint8_t> buf = startMsg.encode();
        sockaddr_in agvAddr {};
        agvAddr.sin_family = AF_INET;
        agvAddr.sin_addr.s_addr = inet_addr(agvHost.c_str());
        agvAddr.sin_port = htons(agvPort);

        while (!connected) {
            sendToAgv(buf);
            buf.resize(1024);
            sockaddr_in senderAddr {};
            socklen_t addrLen = sizeof(senderAddr);

            int len = recvfrom(udpRx, buf.data(), buf.size(), 0, reinterpret_cast<struct sockaddr*>(&senderAddr), &addrLen);
            if (len > 0) {
                connected = true;
                handleRx(std::vector<uint8_t>(buf.begin(), buf.begin() + len));
                return true;
            } else {
                handleRx({});
            }
        }

        return false;
    }

    /**
     * Disconnect from the AGV and close the sockets.
     */
    void disconnect() {
        if (udpTx >= 0) {
            close(udpTx);
            udpTx = -1;
        }
        if (udpRx >= 0) {
            close(udpRx);
            udpRx = -1;
        }
        connected = false;
    }

    /**
     * Set the message to be sent to the AGV.
     * This message will be sent to the AGV until a new message is set.
     * 
     * @param msg The message to be sent to the AGV.
     */
    void setMsg(const AgvTxMsg& msg) {
        this->msg = msg;
    }

    /**
     * Send the current message to the AGV.
     */
    void sendMsg() {
        if (!connected) {
            throw std::runtime_error("Not connected to AGV");
        }

        std::vector<uint8_t> buf = msg.encode();
        sendToAgv(buf);
    }

    /**
     * Receive a message from the AGV.
     * 
     * @return The received message as a vector of bytes.
     */
    std::vector<uint8_t> receiveMsg() {
        std::vector<uint8_t> buf(1024);
        sockaddr_in senderAddr {};
        socklen_t addrLen = sizeof(senderAddr);

        int len = recvfrom(udpRx, buf.data(), buf.size(), 0, reinterpret_cast<struct sockaddr*>(&senderAddr), &addrLen);
        if (len > 0) {
            return std::vector<uint8_t>(buf.begin(), buf.begin() + len);
        } else {
            return {};
        }
    }

private:
    std::string agvHost;
    int agvPort;
    std::string localIp;
    int localPort;
    int udpTx;
    int udpRx;
    bool connected;
    AgvTxMsg msg;

    void handleRx(const std::vector<uint8_t>& buf) {
        if (buf.empty()) {
            connected = false;
            return;
        }

        AgvRxHeader header;
        header.decode(buf);

        if (header.current_page == AgvRxHeader::Page::STATE) {
            MonitorRxMsg monitor;
            monitor.decode(buf);
            // Process monitor data as needed
        }
    }

    void sendToAgv(const std::vector<uint8_t>& buf) {
        sockaddr_in agvAddr {};
        agvAddr.sin_family = AF_INET;
        agvAddr.sin_addr.s_addr = inet_addr(agvHost.c_str());
        agvAddr.sin_port = htons(agvPort);

        sendto(udpTx, buf.data(), buf.size(), 0, reinterpret_cast<struct sockaddr*>(&agvAddr), sizeof(agvAddr));
    }

    std::array<uint8_t, 4> parseIp(const std::string& ip) {
        std::array<uint8_t, 4> result;
        size_t start = 0;
        size_t end = ip.find('.');
        for (int i = 0; end != std::string::npos; ++i) {
            result[i] = std::stoi(ip.substr(start, end - start));
            start = end + 1;
            end = ip.find('.', start);
        }
        result[3] = std::stoi(ip.substr(start, end));
        return result;
    }
};

#endif // AGV_ENDPOINT_HPP
