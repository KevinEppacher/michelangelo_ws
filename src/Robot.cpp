#include "Robot.h"

namespace Robot
{
    
    MobileRobot::MobileRobot()
    {
        //std::cout << "A Robot is born" << std::endl;
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
        //std::cout<<"Client wurde geschlossen111"<<std::endl;
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

    std::string TCPClient::receiveData(char* buffer, ssize_t size) 
    {
        valread = read(client_fd, buffer, size - 1);
        buffer[valread] = '\0'; // Null-terminator hinzufügen
        return buffer;
    }

    Robot::Pose TCPClient::getOdom(std::string receivedData)
    {
        Robot::Pose odom_pose;
        Robot::JsonHandler dataHandler;
        nlohmann :: json json;

        dataHandler.extractJson(receivedData);

        json = dataHandler.extractJson(receivedData);

        std::cout << "x-value position: " << std::endl;
        std::cout << json["pose"]["pose"]["position"]["x"] << std::endl;

        std::cout << "y-value position: " << std::endl;
        std::cout << json["pose"]["pose"]["position"]["y"] << std::endl;

        std::cout << "z-value position: " << std::endl;
        std::cout << json["pose"]["pose"]["position"]["z"] << std::endl;


        std::cout << "x-value orientation: " << std::endl;
        std::cout << json["pose"]["pose"]["orientation"]["x"] << std::endl;

        std::cout << "y-value orientation: " << std::endl;
        std::cout << json["pose"]["pose"]["orientation"]["y"] << std::endl;

        std::cout << "z-value orientation: " << std::endl;
        std::cout << json["pose"]["pose"]["orientation"]["z"] << std::endl;

        std::cout << "w-value orientation: " << std::endl;
        std::cout << json["pose"]["pose"]["orientation"]["w"] << std::endl;


        std::cout << "x-value twist: " << std::endl;
        std::cout << json["twist"]["twist"]["linear"]["x"] << std::endl;

        std::cout << "y-value twist: " << std::endl;
        std::cout << json["twist"]["twist"]["linear"]["y"] << std::endl;

        std::cout << "z-value twist: " << std::endl;
        std::cout << json["twist"]["twist"]["linear"]["z"] << std::endl;


        std::cout << "x-value twist: " << std::endl;
        std::cout << json["twist"]["twist"]["angular"]["x"] << std::endl;

        std::cout << "y-value twist: " << std::endl;
        std::cout << json["twist"]["twist"]["angular"]["y"] << std::endl;

        std::cout << "z-value twist: " << std::endl;
        std::cout << json["twist"]["twist"]["angular"]["z"] << std::endl;

        //Loop?

/*         odom_pose.position.x =
        odom_pose.position.y =
        odom_pose.position.z =
        odom_pose.position.vx = 
        odom_pose.position.vy = 
        odom_pose.position.vz =  
        odom_pose.orientation.roll = 
        odom_pose.orientation.pitch = 
        odom_pose.orientation.yaw = 
        odom_pose.orientation.vRoll = 
        odom_pose.orientation.vPitch = 
        odom_pose.orientation.vYaw =  */

        //push to shared memory

        return odom_pose;
    };

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
        std::string jsonStr;
        std::string startDelimiter = "---START---";
        std::string endDelimiter = "___END___";

        if (rawData.find(startDelimiter) != std::string::npos)
        {
            std::size_t startPos = rawData.find(startDelimiter) + startDelimiter.length();
            std::size_t endPos = rawData.find(endDelimiter, startPos);
            jsonStr = rawData.substr(startPos, endPos - startPos);
        }

        try 
        {
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
}