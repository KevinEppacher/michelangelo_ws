#include "test.h"


SHM::SHM(const std::string& input) : input(input), output() {
    //std::cout << "shared memory starting up\n";
    mutexID = semget(IPC_PRIVATE, 1, 0666);
    shmID = shmget(IPC_PRIVATE, sizeof(struct SHM_Message), 0666 | IPC_CREAT);
    processID = fork();
    if (semctl(mutexID, 0, SETVAL, 1) == -1) {
        std::cout << "semctl SETVAL failed!\n shutting down...\n" << std::endl;
        std::exit(EXIT_FAILURE);
}

    if (processID == 0) {  // Consumer process
        //std::cout << "Consumer started \n";
        signal(SIGINT, [](int sig) { SHM::signalHandler(sig, nullptr); });  // Modified this line
        shmptr = (SHM_Message*)shmat(shmID, NULL, 0);
        //std::cout<<"checking signal\n";
        checkSignal(mutexID);
        output = shmptr->information;
        setSignal(mutexID);
        //std::cout << "semaphore successful \n output is: " << output;
    } else {
        //std::cout << "Producer started \n";
        signal(SIGINT, [](int sig) { SHM::signalHandler(sig, nullptr); });  // Modified this line
        shmptr = (SHM_Message*)shmat(shmID, NULL, 0);
        //std::cout<<"checking signal\n";
        checkSignal(mutexID);
        strncpy(shmptr->information, input.c_str(), sizeof(shmptr->information) - 1);        setSignal(mutexID);
        //std::cout << "semaphore successful \n input was: " << input;
        waitpid(processID, 0 , 0);
        kill(processID, SIGTERM);



    }
}
SHM::~SHM() {
    shmdt(shmptr);
    }


void SHM::checkSignal(int semid){
   struct sembuf check = {0, -1, SEM_UNDO};
   //std::cout<< "check done\n";
    if (semop(semid, &check, 1) == -1){
        std::cout << "semaphore check failed!\n shutting down...\n" << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

void SHM::setSignal(int semid){
   struct sembuf set = {0, 1, SEM_UNDO};
    if (semop(semid, &set, 1) == -1){
        std::cout << "semaphore set failed!\n shutting down...\n" << std::endl;
        std::exit(EXIT_FAILURE);
    }
}


void SHM::signalHandler(int sig, SHM* instance) {
    if (sig == SIGINT) {
        if (instance->processID == 0) {  // Consumer process
            std::cout << "Consumer shutting down \n";
        } else {
            std::cout << "Producer shutting down \n";
            std::cout << "Producer killing consumer\n";
            kill(instance->processID, SIGINT);
            wait(NULL);
            std::cout << "Producer removing semaphores\n";
            semctl(instance->mutexID, -1, IPC_RMID);
            std::cout << "Producer detaching and removing shared memory\n";
            if (shmdt(instance->shmptr) == -1) {
                std::cout << "shmdt failed\n";
                exit(EXIT_FAILURE);
            }
            if (shmctl(instance->shmID, IPC_RMID, 0) == -1) {
                std::cout << "shmctl(IPC_RMID) failed\n";
                exit(EXIT_FAILURE);
            }
        }
        exit(EXIT_SUCCESS);
    }
}

std::string SHM::returnOutput() {
    return output;
}



int main(){
    std::cout<<"main\n";

    while(true){
        std::string input = "Coco ist gay x" + std::to_string(rand()%1000);
        const char* charInput = input.c_str();
        std::cout<<"what I say is: " << input << std::endl;
        SHM myBrain(charInput);
        std::string output = myBrain.returnOutput();
        std::cout<< "what I receive is: " << output << std::endl;
        //SHM::~SHM;
        sleep(3);
    }

    return 0;
}

