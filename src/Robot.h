#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <Eigen/Geometry> 
#include <vector>
#include <chrono>
#include <SFML/Graphics.hpp>
#include <cmath>
#include <deque>


#define RCVBUFSIZE 100000   /* Size of receive buffer */


namespace Robot
{
    struct Pose
    {
        int index = 0;
        double tolerance = 0;
        struct Position
        {
            double x = 0, y = 0, z = 0;
        };

        struct Orientation
        {
            double x = 0, y = 0, z = 0, w = 0;
        };

        Position position;
        Orientation orientation;
    };

    double convertDegreesToRadiant(double degrees);


    struct Twist
    {
        struct Linear
        {
            double x = 0, y = 0, z = 0; // Geschwindigkeiten
        };

        struct Angular
        {
            double x = 0, y = 0, z = 0, w = 0; // Winkelgeschwindigkeiten
        };

        Linear linear;
        Angular angular;
    };

    struct Parameter
    {
        double P = 0.1, I = 0, D = 0;

        double prevDerivative = 0;
        double prevError = 0;
        double error;

        double proportionalError = 0;
        double integralError = 0;
        double derivativeError = 0;
    };

    struct Circle
    {
        double xOffset = 0;
        double yOffset = 0;
        double radius = 1;
    };

    struct sensor_msgs
    {
        struct scan_msg
        {
            std::vector<float> range;
            std::vector<float> angle;
            std::deque<std::pair<float, float>> scanTupleQue;
        };
    };



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MobileRobot
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     
    

  
    class MobileRobot
    {
    public:
        MobileRobot(char* ip);
        MobileRobot(){};
        ~MobileRobot();
        void publishCmdVel(double* linear_x, double* angular_z);
        bool linearController(Robot::Pose goalPose, Robot::Pose currentPose);
        bool pidController(Twist* cmdVel, Parameter PID, double totalDistance, double alpha, double beta);
        bool limitControllerVariables(Twist* cmdVel, double upperLimit, double lowerLimit);
        bool convertQuaternionsToEuler(Pose* currentAngle);
        int goTo(Pose* goalPose, Pose* currentPose);
        bool run();
        void setIP(char* ipAdress);

    protected:
        double calculateTotalDistance(Robot::Pose diffPose);
        double calculateGamma(Robot::Pose diffPose);
        double calculateAlpha(double gamma, Robot::Pose currentAngle);
        double calculateBeta(Robot::Pose goalPose, double gamma);
        bool calculateRobotVektor();
        bool calculateVektorFromRobotToGoal();
        double calculateYaw(Pose* qA);
        double calculateRoll(Pose* qA);
        double calculatePitch(Pose* qA);
        double angleDiff(double angle1, double angle2);
        bool orientationController(Robot::Pose goalPose, Robot::Pose currentPose);
        Robot::Pose robotPose;
        void arrivedEndgoal();
        Robot::Pose currentOdomPose;
        Robot::sensor_msgs::scan_msg scanData;

    private:
        char* ip;
        Robot::Pose diffPose;
        Robot::Twist cmdVel;
        int sequenceNumber = 1;
        float robotVector[2] = { 0 , 0 };
        float distanceVector[2] = { 0 , 0 };
        double totalDistance = 0;
        double gamma = 0;
        double alpha = 0;
        double beta = 0;
        double dt = 0;
    };


    class TCPClient: public MobileRobot
    {
    public:
        TCPClient(const char* serverIP, int port);
        TCPClient(){};
        
        ~TCPClient();

        void closeTCPconnection();

        void sendData(const char* data);

        std::string receiveData(char* buffer, ssize_t size);

    private:
        int client_fd;
        ssize_t valread;
        struct sockaddr_in serv_addr;
        

    };

    //COCO//


    class JsonHandler
    {
        public:
            JsonHandler();
            ~JsonHandler();

            nlohmann::json extractJson(std::string rawData);
            std::string JsonOutputter(const std::string key);
            std::string StringtoRaw(std::string normalString);

            nlohmann::json get_jsonData();

        private:
            nlohmann::json jsonData;

    };

    //COCO//

    class Visualizer: public MobileRobot
    {
    public:
        Visualizer();
        ~Visualizer();

        void run();

    private:
        int screenWidth = 1200;
        int screenHeight = 1000;
        sf::RenderWindow window;
        Robot::MobileRobot turtle;
        float angle = 0.0f;
        float angleIncrement = 1.0f;
        std::deque<std::pair<float, float>> polarPointQueue;
        sf::CircleShape pointShape;
        const int maxBufferSize = 3;

        sf::RectangleShape background; // Background shape
        sf::VertexArray axes; // Axes lines
        float centerX; // Center X coordinate
        float centerY; // Center Y coordinate

        sf::Vector2f polarToCartesian(float radius, float angleDegrees);
    };
    


}


#endif // ROBOT_H