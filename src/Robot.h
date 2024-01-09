#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <arpa/inet.h> 
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <Eigen/Geometry> 
//#include <SFML/Graphics.hpp>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <vector>
#include "matplotlibcpp.h"

>>>>>>> 5f60fb3f (	modified:   src/Robot.cpp)

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



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MobileRobot
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     
    

  
    class MobileRobot
    {
    public:
        MobileRobot();
        ~MobileRobot();
<<<<<<< HEAD
=======
<<<<<<< Updated upstream
=======
>>>>>>> 5f60fb3f (	modified:   src/Robot.cpp)
        void publishCmdVel(double* linear_x, double* angular_z);
        bool linearController(Robot::Pose goalPose, Robot::Pose currentPose);
        bool pidController(Twist* cmdVel, Parameter PID, double totalDistance, double alpha, double beta);
        bool limitControllerVariables(Twist* cmdVel, double upperLimit, double lowerLimit);
        bool convertQuaternionsToEuler(Pose* currentAngle);
        int goTo(Pose* goalPose, Pose* currentPose);
<<<<<<< HEAD
        bool run(char* ip);
        void setIP(char* ipAdress);
=======
        bool run();

>>>>>>> 5f60fb3f (	modified:   src/Robot.cpp)

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
<<<<<<< HEAD
        void arrivedEndgoal();
=======
>>>>>>> Stashed changes
>>>>>>> 5f60fb3f (	modified:   src/Robot.cpp)

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
        
        ~TCPClient();

        void closeTCPconnection();

        void sendData(const char* data);

        std::string receiveData(char* buffer, ssize_t size);


        //memory stuff
        void producerHandler (int sig);  // Signal handler for the producer
        void consumerHandler (int sig);  // Signal handler for the consumer

        struct Message           // Format of the messages
        {
            int type;            // message type, required
            int data;             // item is an int, can by anything
        };
        enum MessageType { PROD_MSG=1, CONS_MSG };
        // CONS_MSG is not used in this program, since the consumer doesn't
        // send messages to the producer.  Can also have more than 2 types
        // of messages if needed.

        int msgqid;                // Message queue id
        pid_t child_pid;           // Result of fork(); global for sig handler
        

    private:
        int client_fd;
        ssize_t valread;
        struct sockaddr_in serv_addr;

    };

<<<<<<< HEAD
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

    class PCA
    {
        public:
            PCA();
            ~PCA();

            void runPCA(std::vector<double> rawLaserScan);  
            Eigen::MatrixXd PolarToCartesian(Eigen::VectorXd laserScanData);
            void FilterLaserscan(Eigen::MatrixXd laserScanData, int filterTolerance);
            Eigen::VectorXd computePCA(const Eigen::MatrixXd &data);
            void plotData(Eigen::MatrixXd laserScanData, Eigen::VectorXd PCA_vector);
            Eigen::VectorXd getAngleDifference();
            Eigen::MatrixXd getFilteredLeftScanData();
            Eigen::MatrixXd getFilteredRightScanData();
            Eigen::VectorXd getPCA_Left();
            Eigen::VectorXd getPCA_Right();

        private:
            Eigen::MatrixXd filteredLaserScanLeftSide;
            Eigen::MatrixXd filteredLaserScanRightSide;
            Eigen::VectorXd PCA_Left;
            Eigen::VectorXd PCA_Right;

    };

    //COCO//

=======
<<<<<<< Updated upstream
=======
    class SharedMemories
    {
        public:
        void producerHandler (int sig);  // Signal handler for the producer
        void consumerHandler (int sig);  // Signal handler for the consumer

        struct Message           // Format of the messages
        {
            int type;            // message type
            int data;             // content of the message
        };
        enum MessageType { PROD_MSG=1, CONS_MSG };
        //used to differentiate bewteen the sender of the message

        int msgqid;                // identifier of the message queue
        pid_t child_pid;           // identifier of the forked process
        


    }
    //COCO//

>>>>>>> Stashed changes
>>>>>>> 5f60fb3f (	modified:   src/Robot.cpp)

}

#endif // ROBOT_H