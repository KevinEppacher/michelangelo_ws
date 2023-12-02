#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

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

    class TCPClient {
    public:
        TCPClient(const char* serverIP, int port) {
            client_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (client_fd < 0) {
                printf("\n Socket creation error \n");
                exit(EXIT_FAILURE);
            }

            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(port);

            if (inet_pton(AF_INET, serverIP, &serv_addr.sin_addr) <= 0) {
                printf("\nInvalid address/ Address not supported \n");
                exit(EXIT_FAILURE);
            }

            if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                printf("\nConnection Failed \n");
                exit(EXIT_FAILURE);
            }
            }

            ~TCPClient() {
                std::cout<<"Client wurde geschlossen111"<<std::endl;
                close(client_fd);
            }

            void closeTCPconnection()
            {
                close(client_fd);
            }

            void sendData(const char* data) {
                send(client_fd, data, strlen(data), 0);
                std::cout << "Message sent: " << data << std::endl;
                close(client_fd);
            }

            void receiveData(char* buffer, ssize_t size) {
                valread = read(client_fd, buffer, size - 1);
                buffer[valread] = '\0'; // Null-terminator hinzufügen
                std::cout << buffer << std::endl;
            }

            private:
                int client_fd;
                ssize_t valread;
                struct sockaddr_in serv_addr;
        };
}

#endif // ROBOT_H
