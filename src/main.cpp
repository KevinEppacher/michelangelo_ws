#include "Robot.h"

int main(int argc, char *argv[])
{
    //Robot::Socket robotSocket(argv[1], argv[2], (argc == 4) ? atoi(argv[3]) : 7);

    //Robot::SharedMemories robotSocket(argv[1], "hello", 8080);
    Robot::SharedMemories robotSocket;

    // Receive the same string back from the server
    std::cout << "Received: "; /* Setup to print the echoed string */
    while (true)
    {

        



        //robotSocket.sendAndReceiveData();
        //std::cout<<"Testing sending data"<<std::endl;
    }
    return 0;
}
