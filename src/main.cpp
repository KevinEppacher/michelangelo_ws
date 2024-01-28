#include "Robot.h"
#include <iostream>
#include <sstream> 
#include <thread>
#include <atomic>

int main(int argc, char **argv)
{
    Robot::MobileRobot turtle(argv[1]);

    // Receive the same string back from the server
    std::cout << "starting loop: "; /* Setup to print the echoed string */
    while (true)
    {
        turtle.run();
    }

    return 0;
}