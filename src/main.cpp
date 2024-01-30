#include "Robot.h"

int main(int argc, char **argv)
{
    Robot::MobileRobot turtle(argv[1]);

    while (true)
    {
        turtle.run();       //starts Robot process
    }

    return 0;
}
