#include "Robot.h"

int main(int argc, char *argv[])
{
    //Robot::Socket robotSocket(argv[1], argv[2], (argc == 4) ? atoi(argv[3]) : 7);

    Robot::Socket robotSocket(argv[1], "hello", 9997);


    robotSocket.sendAndReceiveData();

    // Receive the same string back from the server
    int totalBytesRcvd = 0;
    std::cout << "Received: "; /* Setup to print the echoed string */
    while (totalBytesRcvd < robotSocket.getEchoStringLen())
    {
        /* Receive up to the buffer size (minus 1 to leave space for
           a null terminator) bytes from the sender */
        int bytesRcvd = recv(robotSocket.getSock(), robotSocket.getEchoBuffer(), RCVBUFSIZE - 1, 0);
        if (bytesRcvd <= 0)
        {
            perror("recv() failed or connection closed prematurely");
            exit(1);
        }
        totalBytesRcvd += bytesRcvd;               /* Keep tally of total bytes */
        robotSocket.getEchoBuffer()[bytesRcvd] = '\0'; /* Terminate the string! */
        std::cout << robotSocket.getEchoBuffer();      /* Print the echo buffer */
    }

    std::cout << std::endl; /* Print a final linefeed */

    return 0;
}
