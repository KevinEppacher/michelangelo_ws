#include <SFML/Graphics.hpp>
#include <cmath>
#include <deque> // Für die Verwendung einer Warteschlange (Queue)
#include "Robot.h"
#include <iostream>
#include <sstream> 
#include <thread>
#include <atomic>

int main(int argc, char **argv)
{
    Robot::MobileRobot turtle(argv[1]);

    while (true)
    {
        turtle.run();       //starts Robot process
    }

    return 0;
}
