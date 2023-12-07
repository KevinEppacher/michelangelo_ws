
#pragma once

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <iostream>
#include <ctime>

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
                    
        private:
        int msgqid;                // Message queue id
        pid_t child_pid;  

        struct Message           // Format of the messages
            {
            long type;            // message type, required
            int data;             // item is an int, can by anything
            };

    Memories::Memory(){
        msqid = msget(IPC_PRIVATE, 0660);

    }

    void Memories::sendMessage(int type, int data){
        Message sndMsg;
        sndMsg.type = type;
        sndMsg.data = data;

        msgsnd(msqid, &sndMsg, sizeof(int), 0);
        cout << "The message: " << sndMsg.data << " was sent! \n";
    }


    void Memories::receiveMessage(int type){
        Message rcvMsg;
        msgrcv(msqid, &rcvMsg, sizeof(int), type, 0);
        cout << "The message: " << rcvMsg.data << " was received! \n";
    }

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