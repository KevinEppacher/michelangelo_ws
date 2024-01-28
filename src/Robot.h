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
        void publishCmdVel(double* linear_x, double* angular_z);
        bool linearController(Robot::Pose goalPose, Robot::Pose currentPose);
        bool pidController(Twist* cmdVel, Parameter PID, double totalDistance, double alpha, double beta);
        bool limitControllerVariables(Twist* cmdVel, double upperLimit, double lowerLimit);
        bool convertQuaternionsToEuler(Pose* currentAngle);
        int goTo(Pose* goalPose, Pose* currentPose);
        bool run(char* ip);
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
        void process(std::string odomData);
        std::string receive();

    private:
        double scanMsg;
        double odomMsg;
        double cmdVelMsg;
        Robot::Pose diffPose;
        double totalDistance = 0;
        double gamma = 0;
<<<<<<< HEAD
=======
        double alpha = 0;
        double beta = 0;
        double dt = 0;
        std::chrono::high_resolution_clock::time_point time;
        std::chrono::high_resolution_clock::time_point lastTime;
        long long getTimeMS();
>>>>>>> a51bf602 (kommentiert und couts bearbeitet)
    };


    class TCPClient: public MobileRobot
    {
    public:
        Socket(const char *serverIP, const char *echoString, unsigned short echoServPort = 7);
        ~Socket();
        void establishConnection();
        void sendAndReceiveData();
        unsigned int getEchoStringLen() const;
        int getSock() const;
        char *getEchoBuffer();
        void sendData();
        void receiveData();
        //const int totalBytesRcvd = 0;
        //int bytesRcvd, totalBytesRcvd; /* Bytes read in single recv() and total bytes read */


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
        
        ~TCPClient();

        void closeTCPconnection();

        void sendData(const char* data);

        std::string receiveData(char* buffer, ssize_t size);

    private:
        int sock;                        /* Socket descriptor */
        struct sockaddr_in echoServAddr; /* Echo server address */
        const char *servIP;               /* Server IP address (dotted quad) */
        const char *echoString;           /* String to send to the echo server */
        unsigned int echoStringLen;      /* Length of the string to echo */
        char echoBuffer[RCVBUFSIZE];     /* Buffer for echo string */
        int bytesRcvd, totalBytesRcvd;   /* Bytes read in single recv() and total bytes read */
        unsigned short echoServPort;     /* Echo server port */
        unsigned short odomPort = 9998;     /* Echo server port */
        unsigned short scanPort = 9997;     /* Echo server port */


<<<<<<< HEAD
=======


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
TCPServer
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
>>>>>>> a51bf602 (kommentiert und couts bearbeitet)

    };



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
shared Memory
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     
class SHM {
public:
    SHM(const std::string& input);
    ~SHM();
    
    std::string returnOutput();
    int processID;

private:
    struct SHM_Message {
        char information[16000];
    };

    void checkSignal(int semid);
    void setSignal(int semid);
    static void signalHandler(int sig, SHM* instance);

    int mutexID;
    int shmID;
    SHM_Message* shmptr;

    std::string input;
    std::string output;
};
}
#endif // ROBOT_H