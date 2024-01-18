#include "Robot.h"
#include <iostream>
#include <sstream> 

int main(int argc, char *argv[]) {


    while(true)
    {
        
        Robot::MobileRobot turtle;

        
        while(true)
        {
            turtle.run(argv[1]);
        }

        return 0;
        
    }

    return 0;
}
