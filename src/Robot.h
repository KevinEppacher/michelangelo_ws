#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <nlohmann/json.hpp>
<<<<<<< HEAD
=======
#include <sstream>

>>>>>>> origin/master

#define RCVBUFSIZE 100000   /* Size of receive buffer */

namespace Robot
{
    struct Pose
    {
        struct Position
        {
            double x = 0, y = 0, z = 0;
            double vx = 0, vy = 0, vz = 0; // Geschwindigkeiten
        };

        struct Orientation
        {
            double roll = 0, pitch = 0, yaw = 0;
            double vRoll = 0, vPitch = 0, vYaw = 0; // Winkelgeschwindigkeiten
        };

        Position position;
        Orientation orientation;
    };
    

    class MobileRobot
    {
    public:
        MobileRobot();
        ~MobileRobot();

        bool linearController(Robot::Pose goalPose, Robot::Pose currentPose);
        double calculateTotalDistance(Robot::Pose diffPose);

    private:
        double scanMsg;
        double odomMsg;
        double cmdVelMsg;
        Robot::Pose diffPose;
        const double totalDistance = 0;
    };


    class TCPClient: public MobileRobot
    {
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

    //COCO//

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


    class JsonHandler

        /*     How to use:
                sudo apt install nlohmann-json3-dev, in case the library is not installed yet
                Robot::JsonHandler jsonHüdai; Initiate Json Object
                jsonHüdai.extractJson(GIVE RAW DATA AS INPUT TO PARSE THE DATA);
                jsonHüdai.JsonOutputter(GIVE KEY AS INPUT) */


    {
        public:
            JsonHandler();
            //JsonHandler();
            ~JsonHandler();

            nlohmann::json extractJson(std::string rawData);
            std::string JsonOutputter(const std::string key);
            std::string StringtoRaw(std::string normalString);

            nlohmann::json get_jsonData();

        private:
            nlohmann::json jsonData;

    };

    //COCO//


}

#endif // ROBOT_H
