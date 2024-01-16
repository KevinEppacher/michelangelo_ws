#include "Robot.h"
#include <iostream>
#include <sstream> 

int main(int argc, char *argv[]) {


    while(true)
    {
        
        Robot::MobileRobot turtle;

        std::cout << argc << std::endl;
        std::cout << argv[1] << std::endl;

        while(true)
        {
            turtle.run();
        }

        return 0;
        
    }

    return 0;
}
