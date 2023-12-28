#include "Robot.h"
#include <iostream>
#include <sstream> 
#include <thread>
#include <SFML/Graphics.hpp>
#include <X11/Xlib.h> // Fügen Sie diese Header-Zeile hinzu
#include <atomic>

int main(int argc, char **argv)
{
    Robot::MobileRobot turtle;

    while(true)
    {
        turtle.run();
    }

    return 0;
}


































/*
#pragma once
#include "tcpHandler.h"
class sendCmd : private TCPHandler
{
private:
public:
    sendCmd(){};
    ~sendCmd(){};
    void sendCmdVel(float linear, float angular)
    {
        std::cout << "sendCmdVel is called!\n";
        echoServPort = 9999;
        std::stringstream ss;
        ss << "---START---{\"linear\":" << linear << ", \"angular\":" << angular << "}__END__";
        std::string echoString = ss.str();

        if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
            DieWithError("socket() failed");
        memset(&echoServAddr, 0, sizeof(echoServAddr));
        echoServAddr.sin_family = AF_INET;
        echoServAddr.sin_addr.s_addr = inet_addr(servIP.c_str());
        echoServAddr.sin_port = htons(echoServPort);
        if (connect(sock, (struct sockaddr *)&echoServAddr, sizeof(echoServAddr)) < 0)
        {
            DieWithError("connect() failed");
        }

        size_t echoStringLen = echoString.length();

        if (send(sock, echoString.c_str(), echoStringLen, 0) != static_cast<ssize_t>(echoStringLen))
            DieWithError("send() sent a different number of bytes than expected");
        std::cout << std::endl;

        close(sock);
    }
};


*/









/*
int main(int argc, char *argv[])
{
    //Robot::Socket robotSocket(argv[1], argv[2], (argc == 4) ? atoi(argv[3]) : 7);

    Robot::Socket robotSocket(argv[1], "hello", 8080);



    // Receive the same string back from the server
    while (true)
    {
        robotSocket.sendAndReceiveData();
        std::cout<<"Testing sending data"<<std::endl;
    }
    return 0;
}

*/