#include "Robot.h"

namespace Robot
{
    MobileRobot::MobileRobot()
    {
        std::cout << "A Robot is born" << std::endl;
    }

    MobileRobot::~MobileRobot()
    {
    }

    Socket::Socket(const char *serverIP, const char *echoString, unsigned short echoServPort)
        : servIP(serverIP), echoString(echoString), echoServPort(echoServPort)
    {
        establishConnection();
    }

    Socket::~Socket()
    {
        close(sock);
    }

    void Socket::establishConnection()
    {
        /* Create a reliable, stream socket using TCP */
        if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        {
            perror("socket() failed");
            exit(1);
        }

        /* Construct the server address structure */
        memset(&echoServAddr, 0, sizeof(echoServAddr));   /* Zero out structure */
        echoServAddr.sin_family = AF_INET;                /* Internet address family */
        echoServAddr.sin_addr.s_addr = inet_addr(servIP); /* Server IP address */
        echoServAddr.sin_port = htons(echoServPort);      /* Server port */

        /* Establish the connection to the echo server */
        if (connect(sock, reinterpret_cast<struct sockaddr *>(&echoServAddr), sizeof(echoServAddr)) < 0)
        {
            perror("connect() failed");
            exit(1);
        }
    }

    void Socket::sendAndReceiveData()
    {
        sendData();

        echoStringLen = strlen(echoString); /* Determine input length */

        /* Receive up to the buffer size (minus 1 to leave space for
        a null terminator) bytes from the sender */
        bytesRcvd = recv(getSock(), getEchoBuffer(), RCVBUFSIZE - 1, 0);
        if (bytesRcvd <= 0)
        {
            perror("recv() failed or connection closed prematurely");
            exit(1);
        }

        totalBytesRcvd += bytesRcvd;               /* Keep tally of total bytes */
        getEchoBuffer()[bytesRcvd] = '\0'; /* Terminate the string! */
        std::cout << getEchoBuffer()<<std::endl;      /* Print the echo buffer */
    }

    unsigned int Socket::getEchoStringLen() const
    {
        return echoStringLen;
    }

    int Socket::getSock() const
    {
        return sock;
    }

    char *Socket::getEchoBuffer()
    {
        return echoBuffer;
    }

    void Socket::sendData()
    {
        /* Send the string to the server */
        echoString = "1.0,0.0";

        if (send(sock, echoString, echoStringLen, 0) != static_cast<ssize_t>(echoStringLen))
        {
            perror("send() sent a different number of bytes than expected");
            exit(1);
        }
    }
}
