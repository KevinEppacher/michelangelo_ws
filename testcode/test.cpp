#include "test.h"


void signalHandler(int signum){
    std::cout << "\nshutting down\n";
    shutdown.store(true, std::memory_order_relaxed);
}

SHM::SHM(){
    std::cout<< "shared memory starting up\n";
    std::signal(SIGINT, signalHandler);

    shmID = shmget(IPC_PRIVATE, sizeof(struct SharedMemory), 0666|IPC_CREAT);
    std::cout<<"shared Memory ID: " << shmID << std::endl;

    mutex = semget (IPC_PRIVATE, 1, 0666);
    std::cout << "mutex semaphore is:" << mutex << std::endl;


    myPthread = std::thread(&SHM::startTalking, this);
    myCthread = std::thread(&SHM::startListening, this);


}

void SHM::startListening(){
    while(!shutdown.load(std::memory_order_relaxed)){
        std::cout << "I'm the child\n";
        sleep(1);
    }
}

void SHM::startTalking(){
    while(!shutdown.load(std::memory_order_relaxed)){
        std::cout << "No, I'm your father\n";
        sleep(1);
    }
}

std::thread& SHM::getMyPthread() {
    return myPthread;
}

std::thread& SHM::getMyCthread() {
    return myCthread;
}


int main(){
    std::cout<<"code is running \n";

    
    SHM myBrain;

    while(!shutdown.load(std::memory_order_relaxed)){
        std::cout << "waiting...\n";
        sleep(3);
    }

    myBrain.getMyPthread().join();
    myBrain.getMyCthread().join();

}

