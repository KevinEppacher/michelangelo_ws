#include "Robot.h"

namespace Robot
{
    double convertDegreesToRadiant(double degrees)
    {
        return (degrees * (M_PI/180));
    }
    
    MobileRobot::MobileRobot(char* ip):ip(ip)
    {
        //std::cout << "A Robot is born" << std::endl;
    }

    MobileRobot::~MobileRobot()
    {
        /*
        std::cout<<"Robot was deleted"<<std::endl;
        std::stringstream ss;
        ss << "---START---{linear: 0 , angular:   0  }___END___";
        std::string echoString = ss.str();

        Robot::TCPClient client(this->ip, 9999);
        client.sendData(echoString.c_str());
        //client.receiveData(buffer, sizeof(buffer));  
        client.closeTCPconnection(); 
        */
        
    }

 
    bool Robot::MobileRobot::linearController(Robot::Pose goalPose, Robot::Pose currentOdomPose)
    {
        Parameter PID;

        convertQuaternionsToEuler(&currentOdomPose);

        robotPose = currentOdomPose;

        diffPose.position.x = goalPose.position.x - currentOdomPose.position.x;
        diffPose.position.y = goalPose.position.y - currentOdomPose.position.y;

        totalDistance = calculateTotalDistance(diffPose);

        gamma = calculateGamma(diffPose);

        alpha = calculateAlpha(gamma, currentOdomPose);

        beta = calculateBeta(goalPose, gamma);
        
        pidController(&cmdVel, PID, totalDistance, alpha, beta);

        limitControllerVariables(&cmdVel, 1, -1);

        publishCmdVel(&cmdVel.linear.x, &cmdVel.angular.z);

        //std::cout << "" <<std::endl;
        //std::cout << "Orientation" << currentOdomPose.orientation.z  * (180 / M_PI) <<std::endl;
        //std::cout << "Diff Pose berechnet: X=" << diffPose.position.x << ", Y=" << diffPose.position.y << std::endl;
        //std::cout << "Gamma berechnet: " << gamma * (180 / M_PI) << std::endl;
        //std::cout << "Gesamtdistanz: " << calculateTotalDistance(diffPose)<< std::endl;
        //std::cout << "Alpha berechnet: " << calculateAlpha(gamma, currentOdomPose) * (180 / M_PI) << std::endl;
        //std::cout << "Beta berechnet: " << calculateBeta(goalPose, gamma)  * (180 / M_PI)<< std::endl;
        //std::cout << "cmdVel.linear.x: " << cmdVel.linear.x << "    ||  cmdVel.angular.z:"<<cmdVel.angular.z<<std::endl;
        //std::cout << "" <<std::endl;
        return 1;
    }

    bool Robot::MobileRobot::orientationController(Robot::Pose goalPose, Robot::Pose currentOdomPose)
    {
        Parameter PID;  
        convertQuaternionsToEuler(&currentOdomPose);    
        diffPose.position.x = goalPose.position.x - currentOdomPose.position.x;
        diffPose.position.y = goalPose.position.y - currentOdomPose.position.y; 
        gamma = calculateGamma(diffPose);   
        alpha = calculateAlpha(gamma, currentOdomPose); 
        beta = calculateBeta(goalPose, gamma);  
        pidController(&cmdVel, PID, totalDistance, alpha, beta);    
        limitControllerVariables(&cmdVel, 1, -1);   
        cmdVel.linear.x = 0;
        publishCmdVel(&cmdVel.linear.x, &cmdVel.angular.z);
        return true;
    }

    void Robot::MobileRobot::publishCmdVel(double* linear_x, double* angular_z) 
    {
        std::stringstream ss;
        ss << "---START---{\"linear\": " << *linear_x << ", \"angular\": " << *angular_z << "}___END___";
        std::string echoString = ss.str();

        Robot::TCPClient client(this->ip, 9999);
/*         std::cout << *linear_x << std::endl;
        std::cout << *angular_z << std::endl; */
        client.sendData(echoString.c_str());
        //client.receiveData(buffer, sizeof(buffer));  
        //client.closeTCPconnection();
    }

    double MobileRobot::calculateTotalDistance(Robot::Pose diffPose)
    {
        return (sqrt(pow(diffPose.position.x,2) + pow(diffPose.position.y, 2)));
    }

    double MobileRobot::calculateGamma(Robot::Pose diffPose)
    {
        return (atan2(diffPose.position.y, diffPose.position.x));
    }

    double MobileRobot::calculateAlpha(double gamma, Robot::Pose currentOdomPose)
    {
        return (angleDiff(gamma , currentOdomPose.orientation.z));
    }

    double MobileRobot::calculateBeta(Robot::Pose goalPose, double gamma)
    {
        return (angleDiff(goalPose.orientation.z , gamma));
    }

    bool MobileRobot::pidController(Twist* cmdVel, Parameter PID, double totalDistance, double alpha, double beta)
    {        
        Parameter Lin, Alpha, Beta;

        Lin.P = 0.3;
        Lin.I = 0.01;

        Alpha.P = 1;
        Alpha.I = 0.8;

        Beta.P = -0.3;
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

    void MobileRobot::setIP(char* ipAdress)
    {
        this->ip = ipAdress;
    };

    int MobileRobot::goTo(Pose* goalPose, Pose* currentOdomPose)
    {
        Pose diffPose;
        diffPose.position.x = goalPose->position.x - currentOdomPose->position.x;
        diffPose.position.y = goalPose->position.y - currentOdomPose->position.y;

        double totalDistance = calculateTotalDistance(diffPose);
        std::cout << "totalDistance: " << totalDistance << std::endl;

        convertQuaternionsToEuler(currentOdomPose);
        double totalOrientation = goalPose->orientation.z - currentOdomPose->orientation.z;

        if (totalDistance < goalPose->tolerance ) 
        {
            sequenceNumber += 1;
            std::cout << "Sequence Number: " << sequenceNumber << std::endl;
            arrivedEndgoal();           
        }
        else
        {
            linearController(*goalPose, *currentOdomPose);
        }
    
        return goalPose->index;
    }

    void MobileRobot::arrivedEndgoal()
    {
        double zero=0;
        publishCmdVel(&zero, &zero);
    }

    bool MobileRobot::run()
    {
        //Setup for message
        //setIP(ip);
        char buffer[16000] = {};

        //Receiving odom-data 
        Robot::Pose currentOdomPose;
        Robot::TCPClient odomClient(ip, 9998); 
        std::string odomData = odomClient.receiveData(buffer, sizeof(buffer));
        Robot::JsonHandler OdomdataHandler;
        nlohmann :: json jsonOdom;

        jsonOdom = OdomdataHandler.extractJson(odomData);

        //Overwriting current odometry position with Sensor Odometry Position

        currentOdomPose.position.x = jsonOdom["pose"]["pose"]["position"]["x"];
        currentOdomPose.position.y = jsonOdom["pose"]["pose"]["position"]["y"];
        currentOdomPose.position.z = jsonOdom["pose"]["pose"]["position"]["z"];
        currentOdomPose.orientation.x = jsonOdom["pose"]["pose"]["orientation"]["x"];
        currentOdomPose.orientation.y = jsonOdom["pose"]["pose"]["orientation"]["y"];
        currentOdomPose.orientation.z = jsonOdom["pose"]["pose"]["orientation"]["z"];
        currentOdomPose.orientation.w = jsonOdom["pose"]["pose"]["orientation"]["w"]; 

        std::cout<<" sequence:    " << sequenceNumber<< "   || position.x:   "<<currentOdomPose.position.x<<"    || position.y:   "<<currentOdomPose.position.y<<"  ||  orientation.z:   "<< currentOdomPose.orientation.z<<std::endl;


        //Receiving laserscan-data
        Robot::Pose currentLaserscanPose;
        Robot::TCPClient laserClient(ip, 9997); 
        std::string laserscanData = laserClient.receiveData(buffer, sizeof(buffer));
        Robot::JsonHandler LaserdataHandler;
        nlohmann :: json jsonScan;

        jsonScan = LaserdataHandler.extractJson(laserscanData);
/*
        double poseResolution = 9;
        std::vector<Robot::Pose> circlePaths;

        for (int i = 1; i <= poseResolution; i++)
        {
            Robot::Pose goalPose;
            goalPose.index = i;
        }

        for (const Robot::Pose& pose : circlePaths)
        {
            std::cout<<"Poses: "<<pose.index<<std::endl;
        }
*/
        
        

        //Overwriting current laserscan position with Sensor Laserscan Position
        Robot::Pose goalPose1, goalPose3, goalPose2, goalPose4, goalPose5, goalPose6, goalPose7, goalPose8, goalPose9, goalPose10;
        Robot::Circle circle;
        circle.xOffset = 1.5;
        circle.radius = 0.5;        

        goalPose1.index = 1;
        goalPose1.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-180));
        goalPose1.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-180));
        goalPose1.orientation.z = convertDegreesToRadiant(0);
        goalPose1.tolerance = 0.15;

        goalPose2.index = 2;
        goalPose2.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-135));
        goalPose2.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-135));
        goalPose2.orientation.z = convertDegreesToRadiant(-45);
        goalPose2.tolerance = 0.15;
  

        goalPose3.index = 3;
        goalPose3.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-90));
        goalPose3.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-90));
        goalPose3.orientation.z = convertDegreesToRadiant(0);
        goalPose3.tolerance = 0.15;

        goalPose4.index = 4;
        goalPose4.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-45));
        goalPose4.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-45));
        goalPose4.orientation.z = convertDegreesToRadiant(45);
        goalPose4.tolerance = 0.15;

        goalPose5.index = 5;
        goalPose5.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(0));
        goalPose5.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(0));
        goalPose5.orientation.z = convertDegreesToRadiant(90);
        goalPose5.tolerance = 0.15;

        goalPose6.index = 6;
        goalPose6.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(45));
        goalPose6.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(45));
        goalPose6.orientation.z = convertDegreesToRadiant(135);
        goalPose6.tolerance = 0.15;

        goalPose7.index = 7;
        goalPose7.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(90));
        goalPose7.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(90));
        goalPose7.orientation.z = convertDegreesToRadiant(180);
        goalPose7.tolerance = 0.15;

        goalPose8.index = 8;
        goalPose8.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(135));
        goalPose8.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(135));
        goalPose8.orientation.z = convertDegreesToRadiant(180);
        goalPose8.tolerance = 0.15;

        goalPose9.index = 9;
        goalPose8.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(180));
        goalPose8.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(180));
        goalPose8.orientation.z = convertDegreesToRadiant(180);
        goalPose8.tolerance = 0.15;

        goalPose10.index = 10;
        goalPose10.position.x = 0;
        goalPose10.position.y = 0;
        goalPose10.orientation.z = convertDegreesToRadiant(180);
        goalPose10.tolerance = 0.15;


        if(goalPose1.index == sequenceNumber) goTo(&goalPose1, &currentOdomPose);
        if(goalPose2.index == sequenceNumber) goTo(&goalPose2, &currentOdomPose);
        if(goalPose3.index == sequenceNumber) goTo(&goalPose3, &currentOdomPose);
        if(goalPose4.index == sequenceNumber) goTo(&goalPose4, &currentOdomPose); 
        if(goalPose5.index == sequenceNumber) goTo(&goalPose5, &currentOdomPose); 
        if(goalPose6.index == sequenceNumber) goTo(&goalPose6, &currentOdomPose);
        if(goalPose7.index == sequenceNumber) goTo(&goalPose7, &currentOdomPose); 
        if(goalPose8.index == sequenceNumber) goTo(&goalPose8, &currentOdomPose);
        if(goalPose9.index == sequenceNumber) goTo(&goalPose9, &currentOdomPose);
        if(goalPose10.index == sequenceNumber) goTo(&goalPose10, &currentOdomPose);

  
        //if((goalPose1.index + 4) == sequenceNumber) goTo(&goalPose1, &currentOdomPose);


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
        //std::cout << "Message sent: " << data << std::endl;
        close(client_fd);


        /* fork a child process */
        child_pid = fork();
        if (child_pid < 0) { /* error occurred */
            cerr << "Fork Failed\n";
            exit(-1);
        }

        if(SharedMemories::child_pid != 0)  // child pid not 0 -> producer
        { 
            signal (SIGINT, SharedMemories::producerHandler);  // shutdown if strg+C
            std::cout << "I am the producer\n";

            // producing loop
            while (true)
            {
                std::cout << "Producer attempting write\n";
                Message prodMsg;
                prodMsg.type = PROD_MSG;
                prodMsg.data = 0;//INSERT MESSAGE HERE !!!!!!

                // send message - should test for error
                msgsnd(SharedMemories::msgqid, &prodMsg, sizeof(int), 0);
                std::cout << "Producer sent: " << prodMsg.data << std::endl;
                sleep(0.1);  // 0.1s
            }  // end producing loop
        }  // end producer code
    }

    std::string TCPClient::receiveData(char* buffer, ssize_t size) 
    {
        std::string result;
        bool startFound = false;

        while (true) {
            ssize_t bytesRead = read(client_fd, buffer, sizeof(buffer) - 1);
            if (bytesRead < 0) {
                // Handle error
                return "Error"; // or throw an exception
            }

            buffer[bytesRead] = '\0'; // Null-terminate the buffer
            result.append(buffer);

            // Check if the start marker is found if not already found
            if (!startFound) {
                if (result.find("---START---") != std::string::npos) {
                    startFound = true;
                    result = result.substr(result.find("---START---"));
                    //std::cout << "START FOUND" << std::endl;
                } 
            }

            // Check for the end marker
            if (startFound && result.find("___END___") != std::string::npos) {
                //std::cout << "you did it" << std::endl;
                break;
            }
        }
        return result;
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
 

    JsonHandler::JsonHandler()
        {
            //std::cout << "Json handler has risen from the depth of unexistence into existence" << std::endl;
        }

    JsonHandler::~JsonHandler()
        {
            //std::cout << "JsonHandler destroyed" << std::endl;
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
                //std::cout << "Parsing finished correctly" << std::endl;
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
sharedMemory
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
    SharedMemories::SharedMemories(){
        //Constructor
        startupMemories(); //startet die msq init
    }

    SharedMemories::startupMemories()
    { //msq init
        SharedMemories::msgqid = msgget(IPC_PRIVATE, 0660);

        if (msgqid == -1) {     //wenn msqid < 0 ist -> Fehler
            std::cerr << "msgget failed\n";
            exit(EXIT_FAILURE);
        }
    }

          
   

//STOP maybe woanders hin

    
    //Manages shutdown of consumer process
    void SharedMemories::consumerHandler (int sig)
        {
        std::cout << "Consumer exiting\n";
        exit(EXIT_SUCCESS);
        }  // end consumerHandler

    //Manages shutdown of producer process
    void SharedMemories::producerHandler (int sig)
    {
        // kill child and wait for it
        std::cout << "Producer killing consumer\n";
        kill (SharedMemories::child_pid, SIGINT);
        wait (NULL);

        // remove message queue
        std::cout << "Producer removing message queue\n";
        if (msgctl(SharedMemories::msgqid, IPC_RMID, 0) == -1) {
            std::cerr << "msgctl(IPC_RMID) failed\n";
            exit(EXIT_FAILURE);
        }
        exit(EXIT_SUCCESS);
    }  // end producerHandler
    

    
};