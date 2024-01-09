#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <stdlib.h>  
#include <arpa/inet.h>
#include <signal.h>
#include <sys/socket.h>
<<<<<<< Updated upstream
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/msg.h>
=======
#include <arpa/inet.h> 
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <Eigen/Geometry> 
#include <SFML/Graphics.hpp>
#include <vector>
#include <chrono>
#include <stdlib.h> 

>>>>>>> Stashed changes

#define RCVBUFSIZE 100000   /* Size of receive buffer */

namespace Robot
{
    class MobileRobot
    {
    public:
        MobileRobot();
        ~MobileRobot();
<<<<<<< Updated upstream
=======
        void publishCmdVel(double* linear_x, double* angular_z);
        bool linearController(Robot::Pose goalPose, Robot::Pose currentPose);
        bool pidController(Twist* cmdVel, Parameter PID, double totalDistance, double alpha, double beta);
        bool limitControllerVariables(Twist* cmdVel, double upperLimit, double lowerLimit);
        bool convertQuaternionsToEuler(Pose* currentAngle);
        int goTo(Pose* goalPose, Pose* currentPose);
        bool run();


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
>>>>>>> Stashed changes

    private:
        double scanMsg;
        double odomMsg;
        double cmdVelMsg;
    };

    class Socket : public MobileRobot
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



    };

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

}

#endif // ROBOT_H
