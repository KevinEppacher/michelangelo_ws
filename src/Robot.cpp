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


    TCPClient::TCPClient(const char* serverIP, int port)
    {
        client_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (client_fd < 0) {
            printf("\n Socket creation error \n");
            exit(EXIT_FAILURE);
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);

        if (inet_pton(AF_INET, serverIP, &serv_addr.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            exit(EXIT_FAILURE);
        }

        if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            printf("\nConnection Failed \n");
            exit(EXIT_FAILURE);
        }

    }

    TCPClient::~TCPClient() 
    {
        std::cout<<"Client wurde geschlossen111"<<std::endl;
        close(client_fd);
    }

    void TCPClient::closeTCPconnection()
    {
        close(client_fd);
    }

    void TCPClient::sendData(const char* data) 
    {
        send(client_fd, data, strlen(data), 0);
        std::cout << "Message sent: " << data << std::endl;
        close(client_fd);
    }

    void TCPClient::receiveData(char* buffer, ssize_t size) 
    {
        valread = read(client_fd, buffer, size - 1);
        buffer[valread] = '\0'; // Null-terminator hinzufügen
        std::cout << buffer << std::endl;


    }

        //COCO//

    JsonHandler::JsonHandler(std::string rawData)
    {
        std::string startDelimiter = "---START---";
        std::string endDelimiter = "---END---";

        std::size_t startPos = rawData.find(startDelimiter) + startDelimiter.length();
        std::size_t endPos = rawData.find(endDelimiter, startPos);
        std::string jsonStr = rawData.substr(startPos, endPos - startPos);

        try 
        {
            JsonHandler::jsonData = nlohmann::json::parse(jsonStr);
        } 
        
        catch (nlohmann::json::parse_error& e) 
        {
            std::cerr << "JSON parse error: " << e.what() << '\n';
        }
    }

    JsonHandler::~JsonHandler()
    {
        std::cout << "JsonHandler destroyed" << std::endl;
    }


    std::string JsonHandler::JsonOutputter(const std::string key)
    {
        try 
        {
            if (jsonData.contains(key)) {
                return jsonData[key].dump(); 
            } else {
                return "Key not found";
            }
        } 
        
        catch (std::exception& e) {
            std::cerr << "Error: " << e.what() << '\n';
            return "Error occurred";
        }
    }

    std::string JsonHandler::StringtoRaw(std::string normalString)
    {
        std::ostringstream oss;
        oss << "R\"(";
        oss << ")\"";
        return oss.str();
    }


}
