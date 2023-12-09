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

    bool MobileRobot::linearController(Robot::Pose goalPose, Robot::Pose currentPose)
    {
        diffPose.position.x = goalPose.position.x - currentPose.position.x;
        diffPose.position.y = goalPose.position.y - currentPose.position.y;
        std::cout << calculateTotalDistance(diffPose) <<std::endl;
        
        return 1;
    }

    double MobileRobot::calculateTotalDistance(Robot::Pose diffPose)
    {
        return (sqrt(pow(diffPose.position.x,2) + pow(diffPose.position.y, 2)));
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

    JsonHandler::JsonHandler()
    {
        std::cout << "Json handler has risen from the depth of unexistence into existence" << std::endl;
    }

    JsonHandler::~JsonHandler()
    {
        std::cout << "JsonHandler destroyed" << std::endl;
    }

    nlohmann::json JsonHandler::extractJson(std::string rawData)
    {
        std::string startDelimiter = "---START---";
        std::string endDelimiter = "---END---";

        std::size_t startPos = rawData.find(startDelimiter) + startDelimiter.length();
        std::size_t endPos = rawData.find(endDelimiter, startPos);
        std::string jsonStr = rawData.substr(startPos, endPos - startPos);

        try 
        {
            //std::cout << "Here2" << std::endl;
            JsonHandler::jsonData = nlohmann::json::parse(jsonStr);
            std::cout << "Parsing finished correctly" << std::endl;
        } 
        
        catch (nlohmann::json::parse_error& e) 
        {
            std::cerr << "JSON parse error: " << e.what() << '\n';
            std::cout << "Parsing finished uncorrectly" << std::endl;
        }

        return jsonData;
    }

    std::string JsonHandler::JsonOutputter(const std::string key)
    {
        try 
        {
            //std::cout << key << std::endl;
            if (jsonData.contains(key)) 
            {
                return jsonData[key].dump(); 
            } 
            else 
            {
                return "Key not found";
            }
        } 
        
        catch (std::exception& e)
        {
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

    nlohmann::json JsonHandler::get_jsonData()
    {
        return jsonData;
    }


<<<<<<< HEAD
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


    //COCO//
=======
>>>>>>> origin/master
}
