#include "Robot.h"
#include <iostream>
#include <sstream> 
#include <thread>
#include <atomic>

int main(int argc, char **argv)
{
    Robot::MobileRobot turtle(argv[1]);

    while(true)
    {
        turtle.run();
    }

    return 0;
}





/*
void stopRobot(int argc, char **argv)
{
    Robot::MobileRobot turtle;

    std::stringstream ss;
    ss << "---START---{linear: 0 , angular:   0  }___END___";
    std::string echoString = ss.str();
    Robot::TCPClient client(argv[1], 9999);
    client.sendData(echoString.c_str());
    //client.receiveData(buffer, sizeof(buffer));  
    client.closeTCPconnection(); 

}

*/


















/*


    Robot::MobileRobot turtle;
    
    while(true)
    {
        turtle.run(argv[1]);
    }


    return 0;
}

*/