#include "Robot.h"
#include <iostream>
#include <sstream> 
#include <thread>
#include <atomic>

int main(int argc, char **argv)
{
    Robot::MobileRobot turtle(argv[1]);
    Robot::Visualizer visualizer;

    // Receive the same string back from the server
    std::cout << "Received: "; /* Setup to print the echoed string */
    while (true)
    {
        //turtle.run();
        visualizer.run();
    }

    return 0;
}