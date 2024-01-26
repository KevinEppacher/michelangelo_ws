#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <stdlib.h>  
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <iostream>
#include <thread>
#include <csignal>
#include <atomic>

void signalHandler(int signum);
void producerHandler(int sig);
void consumerHandler(int sig);


std::atomic<bool> shutdown(false);

int mutex;
int shmID;
pid_t childID;

const int BUFFER_SIZE = 1;
struct SharedMemory          // Format of the shared memory
{
   int numItems;             // Count of items in buffer
   int in, out;              // Indexes to back and front
   int buffer[BUFFER_SIZE];  // Items are integers
};
SharedMemory *shmptr;


class SHM{
    public:
        SHM();
        void startTalking();
        void startListening();

        std::thread& getMyPthread();
        std::thread& getMyCthread();
        std::thread myPthread;
        std::thread myCthread;
};

