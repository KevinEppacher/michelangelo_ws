#include <iostream>
#include "Robot.h"

#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

int main()
{
    Robot::MobileRobot mobot;

    // IP-Adresse und Port des Servers (Raspberry Pi)
    const char* serverIP = "192.168.100.52";
    const int serverPort = 9999;

    // Erzeugen eines Socket
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Fehler beim Erzeugen des Sockets" << std::endl;
        return 1;
    }

    
    return 0;
}