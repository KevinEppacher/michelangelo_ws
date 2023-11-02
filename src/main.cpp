#include <iostream>
#include "Robot.h"

#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */


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