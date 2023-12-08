#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <nlohmann/json.hpp>
#include <sstream>


#define RCVBUFSIZE 100000   /* Size of receive buffer */

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


    class TCPClient {
    public:
        TCPClient(const char* serverIP, int port);
        
        ~TCPClient();

        void closeTCPconnection();

        void sendData(const char* data);

        void receiveData(char* buffer, ssize_t size);

    private:
        int client_fd;
        ssize_t valread;
        struct sockaddr_in serv_addr;
    };


    class JsonHandler
    {
        public:
            JsonHandler(std::string rawData);
            ~JsonHandler();

            std::string JsonOutputter(const std::string key);
            std::string StringtoRaw(std::string normalString);

        private:
            nlohmann::json jsonData;

    };

    //COCO//


}

#endif // ROBOT_H
