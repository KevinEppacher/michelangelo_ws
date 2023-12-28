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

 
    bool Robot::MobileRobot::linearController(Robot::Pose goalPose, Robot::Pose currentPose)
    {
        Parameter PID;

        convertQuaternionsToEuler(&currentPose);

        robotPose = currentPose;

        diffPose.position.x = goalPose.position.x - currentPose.position.x;
        diffPose.position.y = goalPose.position.y - currentPose.position.y;

        totalDistance = calculateTotalDistance(diffPose);

        gamma = calculateGamma(diffPose);

        alpha = calculateAlpha(gamma, currentPose);

        beta = calculateBeta(goalPose, gamma);
        
        pidController(&cmdVel, PID, totalDistance, alpha, beta);

        limitControllerVariables(&cmdVel, 1, -1);

        publishCmdVel(&cmdVel.linear.x, &cmdVel.angular.z);

        //std::cout << "" <<std::endl;
        //std::cout << "Orientation" << currentPose.orientation.z  * (180 / M_PI) <<std::endl;
        //std::cout << "Diff Pose berechnet: X=" << diffPose.position.x << ", Y=" << diffPose.position.y << std::endl;
        //std::cout << "Gamma berechnet: " << gamma * (180 / M_PI) << std::endl;
        //std::cout << "Gesamtdistanz: " << calculateTotalDistance(diffPose)<< std::endl;
        //std::cout << "Alpha berechnet: " << calculateAlpha(gamma, currentPose) * (180 / M_PI) << std::endl;
        //std::cout << "Beta berechnet: " << calculateBeta(goalPose, gamma)  * (180 / M_PI)<< std::endl;
        //std::cout << "cmdVel.linear.x: " << cmdVel.linear.x << "    ||  cmdVel.angular.z:"<<cmdVel.angular.z<<std::endl;
        //std::cout << "" <<std::endl;
        return 1;
    }

    bool Robot::MobileRobot::orientationController(Robot::Pose goalPose, Robot::Pose currentPose)
    {
        Parameter PID;  
        convertQuaternionsToEuler(&currentPose);    
        diffPose.position.x = goalPose.position.x - currentPose.position.x;
        diffPose.position.y = goalPose.position.y - currentPose.position.y; 
        gamma = calculateGamma(diffPose);   
        alpha = calculateAlpha(gamma, currentPose); 
        beta = calculateBeta(goalPose, gamma);  
        pidController(&cmdVel, PID, totalDistance, alpha, beta);    
        limitControllerVariables(&cmdVel, 1, -1);   
        cmdVel.linear.x = 0;
        publishCmdVel(&cmdVel.linear.x, &cmdVel.angular.z);
        return true;
    }

    void Robot::MobileRobot::publishCmdVel(double* linear_x, double* angular_z) 
    {

    }

    double MobileRobot::calculateTotalDistance(Robot::Pose diffPose)
    {
        return (sqrt(pow(diffPose.position.x,2) + pow(diffPose.position.y, 2)));
    }

    double MobileRobot::calculateGamma(Robot::Pose diffPose)
    {
        return (atan2(diffPose.position.y, diffPose.position.x));
    }

    double MobileRobot::calculateAlpha(double gamma, Robot::Pose currentPose)
    {
        return (angleDiff(gamma , currentPose.orientation.z));
    }

    double MobileRobot::calculateBeta(Robot::Pose goalPose, double gamma)
    {
        return (angleDiff(goalPose.orientation.z , gamma));
    }

    bool MobileRobot::pidController(Twist* cmdVel, Parameter PID, double totalDistance, double alpha, double beta)
    {        
        Parameter Lin, Alpha, Beta;

        Lin.P = 0.2;
        Lin.I = 0;

        Alpha.P = 0.4;
        Alpha.I = 0.01;

        Beta.P = 0.01;
        Beta.I = 0.6;

        Lin.proportionalError = Lin.P * totalDistance;
        Lin.integralError += ( Lin.I / 2 ) * totalDistance;
        Lin.error = Lin.proportionalError + Lin.integralError;
        cmdVel->linear.x = Lin.error;


        Alpha.proportionalError = Alpha.P *  alpha;
        Alpha.integralError += (Alpha.I / 2 ) * alpha;
        Alpha.error = Alpha.proportionalError + Alpha.integralError;

        Beta.proportionalError = Beta.P *  beta;
        Beta.integralError += (Beta.I / 2 ) * beta;
        Beta.error = Beta.proportionalError + Beta.integralError;
        
        cmdVel->angular.z = Alpha.error + Beta.error;

        return true;
    }

    double MobileRobot::angleDiff(double angle1, double angle2) 
    {
        double diff = angle1 - angle2;
        while (diff < -M_PI) diff += 2 * M_PI;
        while (diff > M_PI) diff -= 2 * M_PI;
        return diff;
    }

    bool MobileRobot::limitControllerVariables(Twist* cmdVel, double upperLimit, double lowerLimit)
    {
        if (cmdVel->linear.x > upperLimit)
        {
           cmdVel->linear.x = upperLimit;
        }

        if (cmdVel->linear.x < lowerLimit)
        {
           cmdVel->linear.x = lowerLimit;
        }
        
        if (cmdVel->angular.z > upperLimit)
        {
           cmdVel->angular.z = upperLimit;
        }

        if (cmdVel->angular.z < lowerLimit)
        {
           cmdVel->angular.z = lowerLimit;
        }

        return true;
    }

    bool MobileRobot::convertQuaternionsToEuler(Pose* currentAngle)
    {
        Eigen::Quaterniond q;
        q.x() = 0;
        q.y() = 0;
        q.z() = currentAngle->orientation.z;
        q.w() = currentAngle->orientation.w;

        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

        currentAngle->orientation.x = euler[0];
        currentAngle->orientation.y = euler[1];
        currentAngle->orientation.z = euler[2];

        /*

        if (currentAngle->orientation.x < 0) 
        {
            currentAngle->orientation.x += 2 * M_PI;
        }

        if (currentAngle->orientation.y < 0) 
        {
            currentAngle->orientation.y += 2 * M_PI;
        }

        if (currentAngle->orientation.z < 0) 
        {
            currentAngle->orientation.z += 2 * M_PI;
        }

        */
        return true;
    }

    double MobileRobot::calculateRoll(Pose* qA)
    {
        double numerator = 2 * ( qA->orientation.w * qA->orientation.y + qA->orientation.x * qA->orientation.z );
        double denominator = 1 - 2 * (pow(qA->orientation.y, 2) + pow(qA->orientation.z, 2));
        double rollAngle = atan2(numerator, denominator);
        //std::cout << "Roll: numerator = " << numerator << ", denominator = " << denominator << ", rollAngle = " << rollAngle << std::endl;
        return rollAngle;
    }

    double MobileRobot::calculatePitch(Pose* qA)
    {
        double sinp = 2 * (qA->orientation.w * qA->orientation.y - qA->orientation.z * qA->orientation.x);
        double pitchAngle;

        if (std::abs(sinp) >= 1) {
            pitchAngle = std::copysign(M_PI / 2, sinp);
            //std::cout << ", pitchAngle = " << pitchAngle << " (90 Grad verwendet)" << std::endl;
            return pitchAngle;
        } else {
            pitchAngle = std::asin(sinp);
            //std::cout << ", pitchAngle = " << pitchAngle << std::endl;
            return pitchAngle;
        }
    }

    double MobileRobot::calculateYaw(Pose* qA)
    {
        double numerator = 2 * ((qA->orientation.w * qA->orientation.z) + (qA->orientation.x * qA->orientation.y));
        double denominator = 1 - 2 * (pow(qA->orientation.x, 2) + pow(qA->orientation.y, 2));
        double yawAngle = atan2(numerator, denominator);
        if (yawAngle < 0) 
        {
            yawAngle += 2 * M_PI;
        }

        std::cout << "Yaw: numerator = " << numerator << ", denominator = " << denominator << ", yawAngle = " << yawAngle << std::endl;
        return yawAngle;
    }

    bool MobileRobot::calculateRobotVektor()
    {
        
        robotVector[0] = 0;
        robotVector[1] = 0;

        return true;
    }

    int MobileRobot::goTo(Pose* goalPose, Pose* currentPose)
    {
        Pose diffPose;
        diffPose.position.x = goalPose->position.x - currentPose->position.x;
        diffPose.position.y = goalPose->position.y - currentPose->position.y;

        double totalDistance = calculateTotalDistance(diffPose);

        if (totalDistance < goalPose->tolerance) 
        {
            sequenceNumber += 1;
            std::cout << "Sequence Number: " << sequenceNumber << std::endl;           
        }
        else
        {
            linearController(*goalPose, *currentPose);
        }
    
        return goalPose->index;
    }

    bool MobileRobot::run()
    {

///////////////////////////////////////////////////////////////////////////////////////
//      Cocooooooo, füg dein Shit hier bei den 420is hinzu
///////////////////////////////////////////////////////////////////////////////////////

        Robot::Pose currentPose;
        currentPose.position.x = 420;
        currentPose.position.y = 420;
        currentPose.position.z = 0;
        currentPose.orientation.x = 0;
        currentPose.orientation.y = 0;
        currentPose.orientation.z = 420;
        currentPose.orientation.w = 420;

        Robot::Pose goalPose1, goalPose3, goalPose2, goalPose4;

        goalPose1.index = 1;
        goalPose1.position.x = 1.0;
        goalPose1.position.y = 0.0;
        goalPose1.orientation.z = 0;
        goalPose1.tolerance = 0.2;

        goalPose2.index = 2;
        goalPose2.position.x = 2.0;
        goalPose2.position.y = -1.0;
        goalPose2.orientation.z = 0;
        goalPose2.tolerance = 0.2;

        goalPose3.index = 3;
        goalPose3.position.x = 3;
        goalPose3.position.y = 0;
        goalPose3.orientation.z = M_PI/2;
        goalPose3.tolerance = 0.2;

        goalPose4.index = 4;
        goalPose4.position.x = 2;
        goalPose4.position.y = 1;
        goalPose4.orientation.z = -M_PI/2;
        goalPose4.tolerance = 0.2;

        if(goalPose1.index == sequenceNumber) goTo(&goalPose1, &currentPose);
        if(goalPose2.index == sequenceNumber) goTo(&goalPose2, &currentPose);
        if(goalPose3.index == sequenceNumber) goTo(&goalPose3, &currentPose);
        if(goalPose4.index == sequenceNumber) goTo(&goalPose4, &currentPose);
        if((goalPose1.index + 4) == sequenceNumber) goTo(&goalPose1, &currentPose);


        return true;
    }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
TCP Client
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  

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



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
TCPServer
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
    class TCPServer {
    public:
        TCPServer(int port);
        ~TCPServer();
        
        void acceptConnection();
        void receiveData(char* buffer, ssize_t size);
        void sendData(const char* data);

    private:
        int server_fd, new_socket;
        ssize_t valread;
        struct sockaddr_in address;
        socklen_t addrlen;
        int port;
    };



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
JsonHandler
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 

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


}
