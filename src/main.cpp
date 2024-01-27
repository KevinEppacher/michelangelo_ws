#include "Robot.h"
#include <iostream>
#include <sstream> 
#include <thread>
#include <atomic>

int main(int argc, char **argv)
{
    Robot::MobileRobot turtle(argv[1]);
    Robot::Visualizer visualizer;

    while (true)
    {
        //turtle.run();
        visualizer.run();
    }

    return 0;
}