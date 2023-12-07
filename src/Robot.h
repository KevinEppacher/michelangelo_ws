
#pragma once

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>

namespace Robot
{
    class MobileRobot
    {
    public:
        MobileRobot();
        ~MobileRobot();

    private:
        double scanMsg;
        double odomMsg;
        double cmdVelMsg;

    };
         // Result of fork(); global for sig handler
  

    };

    class Memories{
        public:
        struct Message           // Format of the messages
            {
            long type;            // message type, required
            int data;             // item is an int, can by anything
            };
                    
        private:
        int msgqid;                // Message queue id
        pid_t child_pid;  




    class Socket : public MobileRobot
    {
    public:
        Socket();
        ~Socket();
        void getScanData();
        void getOdomData();
        void getCmdVelData();

        void plotContinuousScanData();

    private:
        void plotScanData();

    private:
        double scanMsg;
        double odomMsg;
        double cmdVelMsg;
    };

    class TCP_Client
    {
    public:
        TCP_Client(boost::asio::io_context& io_context, const std::string& host, const std::string& port);
        ~TCP_Client();
        std::string read();

    private:
        boost::asio::io_context& io_context_;
        boost::asio::ip::tcp::socket socket_;
    };
}