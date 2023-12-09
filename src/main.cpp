#include "Robot.h"
#include <iostream>
#include <sstream> 

int main(int argc, char *argv[]) {

    char buffer[8192] = {};
    double i = 0;

    Robot::TCPClient client("127.0.0.1", 8080);


    while(true)
    {
        
        client.getOdom(client.receiveData(buffer, sizeof(buffer)));
        
    }

    return 0;
}
