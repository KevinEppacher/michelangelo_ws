#include "Robot.h"
#include <iostream>
#include <sstream> 

int main(int argc, char *argv[]) {

    char buffer[8192] = {};

    //Robot::TCPClient client("127.0.0.1", 8080);
    double i = 0;

    Robot::TCPClient client(argv[1], 9998);
    client.getOdom(client.receiveData(buffer, sizeof(buffer)));

    while(true)
    {
/*         i+=0.005;  
        double linear  = 0 + i;
        double angular  = 0;
        std::stringstream ss;
        ss << "---START---{\"linear\": " << linear << ", \"angular\": " << angular << "}___END___";
        std::string echoString = ss.str();

        std::cout<<i<<std::endl;
        client.receiveData(buffer, sizeof(buffer));   */
        
        
/*         Robot::TCPClient client("192.168.100.55", 9997);
        client.getOdom(client.receiveData(buffer, sizeof(buffer))); */

        //client.sendData(echoString.c_str());
        //client.closeTCPconnection();

        
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
