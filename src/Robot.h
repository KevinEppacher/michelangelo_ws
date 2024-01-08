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
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/msg.h>

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


}

#endif // ROBOT_H
