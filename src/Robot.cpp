#include "Robot.h"

namespace Robot
{
    MobileRobot::MobileRobot()
    {
        std::cout << "A Robot is born" << std::endl;
    }

    MobileRobot::~MobileRobot()
    {
    }

    Socket::Socket(const char *serverIP, const char *echoString, unsigned short echoServPort)
        : servIP(serverIP), echoString(echoString), echoServPort(echoServPort)
    {
        establishConnection();
    }

    Socket::~Socket()
    {
        close(sock);
    }

    void Socket::establishConnection()
    {
        /* Create a reliable, stream socket using TCP */
        if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        {
            perror("socket() failed");
            exit(1);
        }

        /* Construct the server address structure */
        memset(&echoServAddr, 0, sizeof(echoServAddr));   /* Zero out structure */
        echoServAddr.sin_family = AF_INET;                /* Internet address family */
        echoServAddr.sin_addr.s_addr = inet_addr(servIP); /* Server IP address */
        echoServAddr.sin_port = htons(echoServPort);      /* Server port */

        /* Establish the connection to the echo server */
        if (connect(sock, reinterpret_cast<struct sockaddr *>(&echoServAddr), sizeof(echoServAddr)) < 0)
        {
            perror("connect() failed");
            exit(1);
        }
        //I KOMM NED WEIDAAAA
        //shared Memory setup comes in here
        Socket::msgqid = msgget(IPC_PRIVATE, 0660);

        if (msgqid == -1) {
            std::cerr << "msgget failed\n";
            exit(EXIT_FAILURE);
   }

        Socket::child_pid = fork();
        if (Socket::child_pid < 0) { /* error occurred */
            std::cerr << "Fork Failed\n";
            exit(-1);
   }

            //maybe woanders hin!
   if (Socket::child_pid == 0) // child process - the consumer
   { 
      signal (SIGINT, consumerHandler);  // catch SIGINT
      std::cout << "I am the child\n";

      // consuming loop
      while (true)
      {
        std::cout << "Consumer attempting to read message\n";
	 	Message consMsg; 

	 // receive message - should test for error
	    msgrcv(Socket::msgqid, &consMsg, sizeof(int), PROD_MSG, 0);
        std::cout << "Consumer read: " << consMsg.data << std::endl;
        sleep(0.1);  // 0.1s
      }  // end consuming loop
   }  // end consumer code
   else  // parent process - the producer
   { 
        signal (SIGINT, Socket::producerHandler);  // catch SIGINT
        std::cout << "I am the producer\n";

        // producing loop
        while (true)
      {
            std::cout << "Producer attempting write\n";
            Message prodMsg;
            prodMsg.type = PROD_MSG;
            prodMsg.data = 0;//INSERT MESSAGE HERE !!!!!!

            // send message - should test for error
            msgsnd(Socket::msgqid, &prodMsg, sizeof(int), 0);
            std::cout << "Producer sent: " << prodMsg.data << std::endl;
            sleep(0.1);  // 0.1s
      }  // end producing loop
   }  // end producer code

//STOP maybe woanders hin




    }

    void Socket::sendAndReceiveData()
    {
        sendData();

        echoStringLen = strlen(echoString); /* Determine input length */

        /* Receive up to the buffer size (minus 1 to leave space for
        a null terminator) bytes from the sender */
        bytesRcvd = recv(getSock(), getEchoBuffer(), RCVBUFSIZE - 1, 0);
        if (bytesRcvd <= 0)
        {
            perror("recv() failed or connection closed prematurely");
            exit(1);
        }

        totalBytesRcvd += bytesRcvd;               /* Keep tally of total bytes */
        getEchoBuffer()[bytesRcvd] = '\0'; /* Terminate the string! */
        std::cout << getEchoBuffer()<<std::endl;      /* Print the echo buffer */
    }

    unsigned int Socket::getEchoStringLen() const
    {
        return echoStringLen;
    }

    int Socket::getSock() const
    {
        return sock;
    }

    char *Socket::getEchoBuffer()
    {
        return echoBuffer;
    }

    void Socket::sendData()
    {
        /* Send the string to the server */
        echoString = "1.0,0.0";

        if (send(sock, echoString, echoStringLen, 0) != static_cast<ssize_t>(echoStringLen))
        {
            perror("send() sent a different number of bytes than expected");
            exit(1);
        }
    }

    //Manages shutdown of consumer process
    void Socket::consumerHandler (int sig)
        {
        std::cout << "Consumer exiting\n";
        exit(EXIT_SUCCESS);
        }  // end consumerHandler

    //Manages shutdown of producer process
    void Socket::producerHandler (int sig)
        {
        // kill child and wait for it
        std::cout << "Producer killing consumer\n";
        kill (Socket::child_pid, SIGINT);
        wait (NULL);

        // remove message queue
        std::cout << "Producer removing message queue\n";
        if (msgctl(Socket::msgqid, IPC_RMID, 0) == -1) {
            std::cerr << "msgctl(IPC_RMID) failed\n";
            exit(EXIT_FAILURE);
        }
        exit(EXIT_SUCCESS);
}  // end producerHandler
}
