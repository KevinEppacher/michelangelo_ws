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
//#include <SFML/Graphics.hpp>
#include <vector>
#include <chrono>
#include <stdlib.h> 
#include <signal.h>
#include <sys/types.h>
#include <stdlib.h>  
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <functional>
#include <chrono>
#include <iomanip>


#define RCVBUFSIZE 100000   /* Size of receive buffer */

// Define the Robot namespace to encapsulate all related classes and functions
namespace Robot
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Struct definitions
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   
   
    // Pose struct represents the position and orientation of the robot
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

    // Convert degrees to radians
    double convertDegreesToRadiant(double degrees);



    // Twist struct represents the velocity of the robot
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



    // Parameter struct for PID controller parameters
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



    // Circle struct for representing circular paths
    struct Circle
    {
        double xOffset = 0;
        double yOffset = 0;
        double radius = 1;
    };



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Struct definitions
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MobileRobot
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     
    

    // MobileRobot class handles the robot's movement and control
    class MobileRobot
    {
    public:
        MobileRobot(char* ip);
        MobileRobot(){};
        ~MobileRobot();

        void publishCmdVel(double* linear_x, double* angular_z);
        bool linearController(Robot::Pose goalPose, Robot::Pose currentPose, bool wall);
        bool pidController(Twist* cmdVel, Parameter PID, double totalDistance, double alpha, double beta, bool wall);
        bool limitControllerVariables(Twist* cmdVel, double upperLimit, double lowerLimit);
        bool convertQuaternionsToEuler(Pose* currentAngle);
        int goTo(Pose* goalPose, Pose* currentPose, bool wall);
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
        void process(std::string odomData, std::string laserscanData);
        std::string receive(int port);

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
        Eigen::VectorXd PCA_Angles;
        std::chrono::high_resolution_clock::time_point time;
        std::chrono::high_resolution_clock::time_point lastTime;
        long long getTimeMS();
    };



    // TCPClient class for handling TCP/IP communication
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



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
TCPServer (for testing without Turtlebot)
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    class TCPServer {
    public:
        TCPServer(int port);
        ~TCPServer();
        
        void acceptConnection();
        void receiveData(char* buffer, ssize_t size);
        void sendData(const char* data);

    private:
        int server_fd, new_socket;
        ssize_t valread;
        struct sockaddr_in address;
        socklen_t addrlen;
        int port;
    };



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
TCPServer
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Json Parser
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    //Json class for parsing Sensor-Data
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



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Json Parser
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
PCA Class
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    //Class, which handles PCA calculation
    class PCA
    {
        public:
            PCA();
            ~PCA();

            void runPCA(std::vector<double> rawLaserScan, int ScanSample);  
            Eigen::MatrixXd PolarToCartesian(Eigen::VectorXd laserScanData);
            void FilterLaserscan(Eigen::MatrixXd laserScanData, int filterTolerance);
            Eigen::VectorXd computePCA(const Eigen::MatrixXd &data);
            void plotData(Eigen::MatrixXd laserScanData, Eigen::VectorXd PCA_vector, Eigen::VectorXd PCA_vectorRight, int filterTolerance);
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



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
PCA Class
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
shared Memory
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    //SHM class for shared memory management
    class SHM {
        public:
            SHM(const std::string& input);
            ~SHM();
            
            std::string returnOutput();
            int processID;
            bool shutdown = false;

        private:
            struct SHM_Message {char information[16000];};
            void checkSignal(int semid);
            void setSignal(int semid);
            static void signalHandler(int sig, SHM* instance);

            int mutexID;
            int shmID;
            SHM_Message* shmptr;
            std::string input;
            std::string output;
    };



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
shared Memory
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



}
#endif // ROBOT_H
