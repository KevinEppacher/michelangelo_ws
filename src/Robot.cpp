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


        Lin.P = 0.1;
        Lin.I = 0.01;

        Alpha.P = 0.8;
        Alpha.I = 0.01;

        Beta.P = 0.05;
        //Beta.I = 0.6;

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
        //I KOMM NED WEIDAAAA
        //shared Memory setup comes in here
        Socket::msgqid = msgget(IPC_PRIVATE, 0660);

        if (msgqid == -1) {
            std::cerr << "msgget failed\n";
            exit(EXIT_FAILURE);
   }

        Socket::child_pid = fork();
        if (Socket::child_pid < 0) { /* error occurred */
            std::cerr << "Fork Failed\n";
            exit(-1);
   }

            //maybe woanders hin!
   if (Socket::child_pid == 0) // child process - the consumer
   { 
      signal (SIGINT, consumerHandler);  // catch SIGINT
      std::cout << "I am the child\n";

      // consuming loop
      while (true)
      {
        std::cout << "Consumer attempting to read message\n";
	 	Message consMsg; 

	 // receive message - should test for error
	    msgrcv(Socket::msgqid, &consMsg, sizeof(int), PROD_MSG, 0);
        std::cout << "Consumer read: " << consMsg.data << std::endl;
        sleep(0.1);  // 0.1s
      }  // end consuming loop
   }  // end consumer code
   else  // parent process - the producer
   { 
        signal (SIGINT, Socket::producerHandler);  // catch SIGINT
        std::cout << "I am the producer\n";

        // producing loop
        while (true)
      {
            std::cout << "Producer attempting write\n";
            Message prodMsg;
            prodMsg.type = PROD_MSG;
            prodMsg.data = 0;//INSERT MESSAGE HERE !!!!!!

            // send message - should test for error
            msgsnd(Socket::msgqid, &prodMsg, sizeof(int), 0);
            std::cout << "Producer sent: " << prodMsg.data << std::endl;
            sleep(0.1);  // 0.1s
      }  // end producing loop
   }  // end producer code

//STOP maybe woanders hin




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

        convertQuaternionsToEuler(currentOdomPose);
        double totalOrientation = goalPose->orientation.z - currentOdomPose->orientation.z;

        if (totalDistance < goalPose->tolerance && (totalOrientation) < 0.1) 
        {
            sequenceNumber += 1;
            //std::cout << "Sequence Number: " << sequenceNumber << std::endl;
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

    std::string MobileRobot::receive()
    {
        char buffer[16000] = {};
        //Receiving odom-data 
        Robot::TCPClient odomClient(ip, 9998); 
        std::string odomData = odomClient.receiveData(buffer, sizeof(buffer));
        return odomData;
    }


    void MobileRobot::process(std::string odomData)
    {
        Robot::Pose currentOdomPose;
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

        // std::cout<<" sequence:    " << sequenceNumber<< "         || currentOdomPose.position.x:   "<<currentOdomPose.position.x<<"         || currentOdomPose.position.y"<<currentOdomPose.position.y<<"         ||  currentOdomPose.orientation.z"<< currentOdomPose.orientation.z<<std::endl;


        //Receiving laserscan-data
        /*
        Robot::Pose currentLaserscanPose;
        Robot::TCPClient laserClient(ip, 9997); 
        std::string laserscanData = laserClient.receiveData(buffer, sizeof(buffer));

        Robot::JsonHandler LaserdataHandler;
        nlohmann :: json jsonScan;

        jsonScan = LaserdataHandler.extractJson(laserscanData);

        Robot::PCA pcaObject;

        pcaObject.runPCA(jsonScan["ranges"]);

        Eigen::VectorXd AngleDiffs = pcaObject.getAngleDifference();

        std::cout << "Thetas are: " << AngleDiffs << std::endl;


/* 

        
        //Overwriting current laserscan position with Sensor Laserscan Position

        currentLaserscanPose.position.x = jsonScan["pose"]["pose"]["position"]["x"];
        currentLaserscanPose.position.y = jsonScan["pose"]["pose"]["position"]["y"];
        currentLaserscanPose.position.z = jsonScan["pose"]["pose"]["position"]["z"];
        currentLaserscanPose.orientation.x = jsonScan["pose"]["pose"]["orientation"]["x"];
        currentLaserscanPose.orientation.y = jsonScan["pose"]["pose"]["orientation"]["y"];
        currentLaserscanPose.orientation.z = jsonScan["pose"]["pose"]["orientation"]["z"];
        currentLaserscanPose.orientation.w = jsonScan["pose"]["pose"]["orientation"]["w"]; 

        std::cout<<" sequence:    " << sequenceNumber<< "         || currentOdomPose.position.x:   "<<currentLaserscanPose.position.x<<"         || currentOdomPose.position.y"<<currentLaserscanPose.position.y<<"         ||  currentOdomPose.orientation.z"<< currentLaserscanPose.orientation.z<<std::endl; */

        Robot::Pose goalPose1, goalPose3, goalPose2, goalPose4;

        goalPose1.index = 1;
        goalPose1.position.x = 1.0;
        goalPose1.position.y = 0.0;
        goalPose1.orientation.z = 0;
        goalPose1.tolerance = 0.2;

        goalPose2.index = 2;
        goalPose2.position.x = 1.0;
        goalPose2.position.y = 1.0;
        goalPose2.orientation.z = 0;
        goalPose2.tolerance = 0.2;

        goalPose3.index = 3;
        goalPose3.position.x = 0;
        goalPose3.position.y = 1.0;
        goalPose3.orientation.z = 0;
        goalPose3.orientation.z = M_PI/2;
        goalPose3.tolerance = 0.2;

        goalPose4.index = 4;
        goalPose4.position.x = 0;
        goalPose4.position.y = 0;
        goalPose3.orientation.z = 0;
        goalPose4.orientation.z = -M_PI/2; 
        goalPose4.tolerance = 0.2;

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


        if(goalPose1.index == sequenceNumber) {
            goTo(&goalPose1, &currentOdomPose);
            //std::cout << "current position is: " << &currentOdomPose << std::endl;
            //std::cout << "current goal is: " << &goalPose1 << std::endl;
            }
        if(goalPose2.index == sequenceNumber) {
            goTo(&goalPose2, &currentOdomPose);
            //std::cout << "current position is: " << &currentOdomPose << std::endl;
            //std::cout << "current goal is: " << &goalPose2 << std::endl;
            }
        if(goalPose3.index == sequenceNumber) {
            goTo(&goalPose3, &currentOdomPose);
            //std::cout << "current position is: " << &currentOdomPose << std::endl;
            //std::cout << "current goal is: " << &goalPose3 << std::endl;
            }
        if(goalPose4.index == sequenceNumber) {
            goTo(&goalPose4, &currentOdomPose);
            //std::cout << "current position is: " << &currentOdomPose << std::endl;
            //std::cout << "current goal is: " << &goalPose4 << std::endl;
            }
        if(goalPose5.index == sequenceNumber) {
            goTo(&goalPose5, &currentOdomPose);
            //std::cout << "current position is: " << &currentOdomPose << std::endl;
            //std::cout << "current goal is: " << &goalPose5 << std::endl;
            } 
        if(goalPose6.index == sequenceNumber) {
            goTo(&goalPose6, &currentOdomPose);
            //std::cout << "current position is: " << &currentOdomPose << std::endl;
            //std::cout << "current goal is: " << &goalPose6 << std::endl;
            }
        if(goalPose7.index == sequenceNumber) {
            goTo(&goalPose7, &currentOdomPose);
            //std::cout << "current position is: " << &currentOdomPose << std::endl;
            //std::cout << "current goal is: " << &goalPose7 << std::endl;
            }
        if(goalPose8.index == sequenceNumber) {
            goTo(&goalPose8, &currentOdomPose);
            //std::cout << "current position is: " << &currentOdomPose << std::endl;
            //std::cout << "current goal is: " << &goalPose8 << std::endl;
            }
        if(goalPose9.index == sequenceNumber) {
            goTo(&goalPose9, &currentOdomPose);
            //std::cout << "current position is: " << &currentOdomPose << std::endl;
            //std::cout << "current goal is: " << &goalPose9 << std::endl;
            }
        if(goalPose10.index == sequenceNumber) {
            goTo(&goalPose10, &currentOdomPose);
            //std::cout << "current position is: " << &currentOdomPose << std::endl;
            //std::cout << "current goal is: " << &goalPose10 << std::endl;
            }

  
        //if((goalPose1.index + 4) == sequenceNumber) goTo(&goalPose1, &currentOdomPose);

    }
    

    bool MobileRobot::run()
    {
        std::string input = MobileRobot::receive();     //calling receive function and saving data as input
        const char* charInput = input.c_str();          //structuring input
        SHM myBrain(charInput);                         //creating SHM class object and calling constructor -> starting shared Memory
        std::string output = myBrain.returnOutput();    //calling result of shared memory via getter
        //std::cout << "Output: " << output << std::endl; //outputting Output data
        MobileRobot::process(output);                   //calling process function responsible for calculating movements and commanding the robot with the output as parameter
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


    PCA::PCA(){};
    PCA::~PCA(){};

    void PCA::runPCA(std::vector<double> rawLaserScan)
    {
        //Map std::vector to eigen::vector
        Eigen::VectorXd laser = Eigen::VectorXd::Map(&rawLaserScan[0], rawLaserScan.size());

        //convert polar coordinates to cartesian coordinates
        Eigen::MatrixXd data = this->PolarToCartesian(laser);

        //Compute PCA
        Eigen::VectorXd principal_component = this->computePCA(data);
        //std::cout << "Principal Component of all: \n" << principal_component << std::endl;
        //pca.plotData(data, principal_component); 

        //Filter Right and Left side and calculate PCA seperately
        this->FilterLaserscan(data, 30);
        Eigen::MatrixXd left = this->getFilteredLeftScanData();
        Eigen::MatrixXd right = this->getFilteredRightScanData();

        Eigen::VectorXd principal_componentLeft = this->computePCA(left);
        this->PCA_Left = principal_componentLeft;
        //std::cout << "Principal Component Left: \n" << principal_componentLeft << std::endl;
        this->plotData(data, principal_componentLeft);  

        Eigen::VectorXd principal_componentRight = this->computePCA(right);
        this->PCA_Right = principal_componentRight;
        //std::cout << "Principal Component Right: \n" << principal_componentRight << std::endl;
        //this->plotData(data, principal_componentRight);
    }

    
    Eigen::MatrixXd PCA::PolarToCartesian(Eigen::VectorXd laserScanData)
    {
        Eigen::MatrixXd cartesianLaserScanData(360,2);
        //int theta = 1;

        for (int theta = 0; theta < 360; theta++)
        {
            double currentRange = laserScanData[theta];
            double thetaRadians = theta * M_PI / 180.0;
            double x = currentRange * cos(thetaRadians);
            double y = currentRange * sin(thetaRadians);
            cartesianLaserScanData(theta,0) = x;
            cartesianLaserScanData(theta,1) = y; 
        }

        //std::cout << "Cartesian laser scan: " << cartesianLaserScanData << std::endl;
        return cartesianLaserScanData;
        
    };

    void PCA::FilterLaserscan(Eigen::MatrixXd laserScanData, int filterTolerance)
    {
        int rangeSize = 2 * filterTolerance + 1; // Calculate the actual size of the range
        Eigen::MatrixXd tempLeft(rangeSize, 2);  // Initialize tempLeft with the correct size

        int leftCounter = 0;  // Counter for indexing tempLeft
        for (int theta = 90 - filterTolerance; theta <= 90 + filterTolerance; theta++, leftCounter++)
        {
            if (theta >= 0 && theta < laserScanData.rows()) // Check bounds of laserScanData
            {
                tempLeft(leftCounter, 0) = laserScanData(theta, 0);
                tempLeft(leftCounter, 1) = laserScanData(theta, 1);
            }
        }

        this->filteredLaserScanLeftSide = tempLeft;

        //int rangeSize = 2 * filterTolerance + 1; // Calculate the actual size of the range
        Eigen::MatrixXd tempRight(rangeSize, 2);  // Initialize tempLeft with the correct size

        int rightCounter = 0;  // Counter for indexing tempLeft
        for (int theta = 270 - filterTolerance; theta <= 270 + filterTolerance; theta++, rightCounter++)
        {
            if (theta >= 0 && theta < laserScanData.rows()) // Check bounds of laserScanData
            {
                tempRight(rightCounter, 0) = laserScanData(theta, 0);
                tempRight(rightCounter, 1) = laserScanData(theta, 1);
            }
        }

        this->filteredLaserScanRightSide = tempRight;
    };

    Eigen::VectorXd PCA::computePCA(const Eigen::MatrixXd &data)
    {
        // Centering the data

        //std::cout << "Data: " << data << std::endl;

        Eigen::VectorXd mean = data.colwise().mean();

        //std::cout << "Mean: " << mean << std::endl;

        Eigen::MatrixXd centered = data.rowwise() - mean.transpose();

        //std::cout << "Centered: " << centered << std::endl;

        // Computing the covariance matrix
        Eigen::MatrixXd cov = centered.adjoint() * centered;

        //std::cout << "Covariance: " << cov << std::endl;

        // Performing eigen decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov);

        //std::cout << "Eigenvalues: " << eigen_solver.eigenvalues() << std::endl;


        Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();

        //std::cout << "Eigenvectors: " << eigenvectors << std::endl;

        // Extracting the principal component
        Eigen::VectorXd principal_component = eigenvectors.rightCols(1);

        return principal_component;
    };

    void PCA::plotData(Eigen::MatrixXd laserScanData, Eigen::VectorXd PCA_vector)
    {

        matplotlibcpp::ion(); // Turn on interactive mode

        std::vector<double> x_data(laserScanData.rows()), y_data(laserScanData.rows());
        for (int i = 0; i < laserScanData.rows(); ++i) {
            x_data[i] = laserScanData(i, 0);
            y_data[i] = laserScanData(i, 1);
        }

        matplotlibcpp::clf();

        // Plotting the laser scan data
        matplotlibcpp::scatter(x_data, y_data);
        matplotlibcpp::title("PCA vector");

        // Calculate end points for the PCA vector for visualization
        double scale_factor = 10.0;  // Adjust this factor to scale the PCA vector for better visualization
        std::vector<double> pca_x = {0, scale_factor * PCA_vector(0)};
        std::vector<double> pca_y = {0, scale_factor * PCA_vector(1)};

        // Explicitly defining the start points for the quiver plot
        std::vector<double> start_x = {0,0};
        std::vector<double> start_y = {0,0};

        // Plotting the PCA vector
        matplotlibcpp::quiver(start_x, start_y, pca_x, pca_y);

        // Show the plot and update frequently
        //matplotlibcpp::show();
        matplotlibcpp::draw();
        matplotlibcpp::pause(0.01);
    };


    Eigen::VectorXd PCA::getAngleDifference()
    {
        Eigen::VectorXd Thetas(2);

        Eigen::VectorXd directionVector(2);
        directionVector(0) = 1;
        directionVector(1) = 0;

        double dotProductLeft = (directionVector.dot(this->PCA_Left));

        double magnitudeVec1Left = directionVector.norm();
        double magnitudeVec2Left = this->PCA_Left.norm();

        // Calculate the cosine of the angle
        double cosAngleLeft = dotProductLeft / (magnitudeVec1Left * magnitudeVec2Left);

        // Ensure the cosine value is within [-1, 1] to avoid NaN due to floating point errors
        cosAngleLeft = std::max(-1.0, std::min(1.0, cosAngleLeft));

        // Calculate the angle in radians
        Thetas(0) = std::acos(cosAngleLeft);

        double dotProductRight = (directionVector.dot(this->PCA_Right));

        double magnitudeVec1Right = directionVector.norm();
        double magnitudeVec2Right = this->PCA_Right.norm();

        // Calculate the cosine of the angle
        double cosAngleRight = dotProductRight / (magnitudeVec1Right * magnitudeVec2Right);

        // Ensure the cosine value is within [-1, 1] to avoid NaN due to floating point errors
        cosAngleRight = std::max(-1.0, std::min(1.0, cosAngleRight));

        // Calculate the angle in radians
        Thetas(1) = std::acos(cosAngleRight);


        std::cout << "Angles: " << Thetas << std::endl;

        return Thetas;
    }

    Eigen::MatrixXd PCA::getFilteredLeftScanData()
    {
        return this->filteredLaserScanLeftSide;
    };

    Eigen::MatrixXd PCA::getFilteredRightScanData()
    {
        return this->filteredLaserScanRightSide;
    };

    Eigen::VectorXd PCA::getPCA_Left()
    {
        return this->PCA_Left;
    };

    Eigen::VectorXd PCA::getPCA_Right()
    {
        return this->PCA_Right;
    };

        //COCO//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
sharedMemory
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
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


void SHM::signalHandler(int sig, SHM* instance) {                                               //enables clean termination of processes
    if (sig == SIGINT) { 
       /* Robot::Twist cmdVel;                                                                       //if strg+c is pressed
        cmdVel.linear.x = 0;
        cmdVel.angular.z = 0;
        Robot::MobileRobot turtle;
        turtle.publishCmdVel(&cmdVel.linear.x, &cmdVel.angular.z);*/
        
        if (instance->processID == 0) {  // Consumer process                                    //which process has to be killed
            std::cout << "Consumer shutting down \n";                                           //Consumer/ Child doesnt has to be shut down since Producer/Parent kills Child
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


    

    
}
