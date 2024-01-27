#include <iostream>
#include <cstring>
#include <netinet/in.h>
#include <unistd.h>

class TCPServer {
public:
    TCPServer(int port) : port(port) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == -1) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        int opt = 1;
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        if (listen(server_fd, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }
    }

    ~TCPServer() {
        close(server_fd);
    }

    void acceptConnection() {
        addrlen = sizeof(address);
        new_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen);
        if (new_socket < 0) {
            perror("accept");
            exit(EXIT_FAILURE);
        }
    }

    void receiveData(char* buffer, ssize_t size) {
        valread = read(new_socket, buffer, size - 1);
        buffer[valread] = '\0'; // Null-terminator hinzufügen
        std::cout << buffer << std::endl;
    }

    void sendData(const char* data) {
        send(new_socket, data, strlen(data), 0);
        std::cout << "Message sent: " << data << std::endl;
    }

private:
    int server_fd, new_socket;
    ssize_t valread;
    struct sockaddr_in address;
    socklen_t addrlen;
    int port;
};




int main() {
    const char* data = R"(---START---{"header": {"seq": 41486, "stamp": {"secs": 1677512013, "nsecs": 49092063}, "frame_id": "odom"}, "child_frame_id": "base_footprint", "pose": {"pose": {"position": {"x": 2.923440933777499e-10, "y": -2.7172184502433083e-08, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.005155239254236221, "w": 0.9999867081642151}}, "covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, "twist": {"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": -0.0022432173136621714}}, "covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}___END___)";
    char buffer[8192] = {};

    TCPServer server(10000);
    server.acceptConnection();
    
    while(true)
    {
        //std::cout<<"test"<<std::endl;
        //server.receiveData(buffer, sizeof(buffer));
        server.sendData(data); 
        std::cout << data << std::endl;
    }

    return 0;
}