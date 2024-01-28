#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <stdlib.h>  
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <iostream>
#include <functional>
#include <cstring>

class SHM {
public:
    SHM(const std::string& input);
    ~SHM();
    
    std::string returnOutput();
    int processID;

private:
    struct SHM_Message {
        char information[256];
    };

    void checkSignal(int semid);
    void setSignal(int semid);
    static void signalHandler(int sig, SHM* instance);

    int mutexID;
    int shmID;
    SHM_Message* shmptr;

    std::string input;
    std::string output;
};
