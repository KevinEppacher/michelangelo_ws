#include "Robot.h"

namespace Robot
{
    double convertDegreesToRadiant(double degrees)
    {
        return (degrees * (M_PI/180));
    }
    
    MobileRobot::MobileRobot(char* ip):ip(ip)
    {
        plt::figure_size(800, 600); // Größe des Fensters in Pixel
        start = std::chrono::system_clock::now();
    }

    MobileRobot::~MobileRobot()
    {
        
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

        limitControllerVariables(&cmdVel, 3, -3);

        publishCmdVel(&cmdVel.linear.x, &cmdVel.angular.z);

        return 1;
    }

    void Robot::MobileRobot::publishCmdVel(double* linear_x, double* angular_z) 
    {
        std::stringstream ss;
        ss << "---START---{\"linear\": " << *linear_x << ", \"angular\": " << *angular_z << "}___END___";
        std::string echoString = ss.str();

        Robot::TCPClient client(this->ip, 9999);

        client.sendData(echoString.c_str());

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
        Lin.P = 0.15;
        //Lin.I = 0.01;

        Alpha.P = 0.6;
        //Alpha.I = 0.04;

        Beta.P = -0.2;
        //Beta.I = -0.06;

        Lin.proportionalError = Lin.P * totalDistance;
        Lin.integralError += ( Lin.I / 2 ) * totalDistance;
        Lin.error = Lin.proportionalError + Lin.integralError;


        Alpha.proportionalError = Alpha.P *  alpha;
        Alpha.integralError += (Alpha.I / 2 ) * alpha;
        Alpha.error = Alpha.proportionalError + Alpha.integralError;

        Beta.proportionalError = Beta.P *  beta;
        Beta.integralError += (Beta.I / 2 ) * beta;
        Beta.error = Beta.proportionalError + Beta.integralError;
        
        cmdVel->linear.x = Lin.error;
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
        //std::cout << "totalDistance: " << totalDistance << std::endl;

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



    void MobileRobot::storePositions()
    {
        Robot::Circle circle;
        circle.xOffset = 1;
        circle.radius = 0.25;

                        goalPose1.index = 1;
        goalPose1.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-180));
        goalPose1.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-180));
        goalPose1.orientation.z = convertDegreesToRadiant(0);
        goalPose1.tolerance = 0.15;
        storedPositions.push_back(goalPose1);

        goalPose2.index = 2;
        goalPose2.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-135));
        goalPose2.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-135));
        goalPose2.orientation.z = convertDegreesToRadiant(-45);
        goalPose2.tolerance = 0.15;
        storedPositions.push_back(goalPose2);

  

        goalPose3.index = 3;
        goalPose3.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-90));
        goalPose3.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-90));
        goalPose3.orientation.z = convertDegreesToRadiant(0);
        goalPose3.tolerance = 0.15;
        storedPositions.push_back(goalPose3);


        goalPose4.index = 4;
        goalPose4.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-45));
        goalPose4.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-45));
        goalPose4.orientation.z = convertDegreesToRadiant(45);
        goalPose4.tolerance = 0.15;
        storedPositions.push_back(goalPose4);


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
        storedPositions.push_back(goalPose6);


        goalPose7.index = 7;
        goalPose7.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(90));
        goalPose7.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(90));
        goalPose7.orientation.z = convertDegreesToRadiant(179);
        goalPose7.tolerance = 0.15;
        storedPositions.push_back(goalPose7);


        goalPose8.index = 8;
        goalPose8.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(135));
        goalPose8.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(135));
        goalPose8.orientation.z = convertDegreesToRadiant(179);
        goalPose8.tolerance = 0.15;
        storedPositions.push_back(goalPose8);


        goalPose9.index = 9;
        goalPose9.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(180));
        goalPose9.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(180));
        goalPose9.orientation.z = convertDegreesToRadiant(180);
        goalPose9.tolerance = 0.15;
        storedPositions.push_back(goalPose9);


        goalPose10.index = 10;
        goalPose10.position.x = 0;
        goalPose10.position.y = 0;
        goalPose10.orientation.z = convertDegreesToRadiant(179);
        goalPose10.tolerance = 0.15;
        storedPositions.push_back(goalPose9);

    }

    void MobileRobot::spinOnce()
    {
        //Setup for message
        //setIP(ip);
        char buffer[16000] = {};

        //Receiving odom-data 
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
        //std::cout << "Laserscan data: " << jsonScan << std::endl;
        scanData.range = jsonScan["ranges"].get<std::vector<float>>();

        // Reservieren oder Größe anpassen, um 360 Winkel zu speichern
        scanData.angle.resize(scanData.range.size());
        // Iterieren über den Bereich und füllen die Winkel

    }


    void MobileRobot::plotErrors()
    {
        auto now = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = now - start;

        // Zeit und Wert aktualisieren
        zeit.push_back(elapsed_seconds.count());
        errorBeta.push_back(Beta.error);
        errorAlpha.push_back(Alpha.error);
        errorLinear.push_back(Lin.error);

        Position currentPosition, targetPosition;
        currentPosition.x.push_back(currentOdomPose.position.x);
        currentPosition.y.push_back(currentOdomPose.position.y);

        plt::clf(); // Vorherigen Plot löschen
        // Jeden Plot mit einem eindeutigen Label versehen
        plt::named_plot("Beta Error", zeit, errorBeta, "r"); // "r" steht für die Farbe Rot
        plt::named_plot("Alpha Error", zeit, errorAlpha, "g"); // "g" steht für die Farbe Grün
        plt::named_plot("Linear Error", zeit, errorLinear, "b"); // "b" steht für die Farbe Blau

        plt::scatter(currentPosition.x, currentPosition.y, 40, {{"color", "red"}}); // Große rote Punkte        

        plt::xlabel("Time [s]");
        plt::ylabel("Error Value");
        plt::title("Error");
        plt::legend();
        plt::pause(0.0001);   
    }




    void MobileRobot::run()
    {

        spinOnce();
 
        storePositions();
      
        for(Robot::Pose& goalPose:storedPositions)
        {
            if(goalPose.index == sequenceNumber) goTo(&goalPose, &currentOdomPose);
        }
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

        //COCO//

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Visualizer
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
    Visualizer::Visualizer()
        : background(sf::Vector2f(screenWidth, screenHeight)),
        axes(sf::Lines, 4),
        centerX(static_cast<float>(screenWidth) / 2.0f),
        centerY(static_cast<float>(screenHeight) / 2.0f),
        window(sf::VideoMode(800, 600), "SFML Window")
    {
        window.create(sf::VideoMode(screenWidth, screenHeight), "Punkte in Polar-Koordinaten");
        background.setFillColor(sf::Color::Black);

        axes[0].position = sf::Vector2f(0, centerY);
        axes[1].position = sf::Vector2f(screenWidth, centerY);
        axes[2].position = sf::Vector2f(centerX, 0);
        axes[3].position = sf::Vector2f(centerX, screenHeight);
        for (int i = 0; i < 4; ++i) {
            axes[i].color = sf::Color::White;
        }



        pointShape.setRadius(3);
        pointShape.setFillColor(sf::Color::Red);

        for (int i = 0; i < maxBufferSize; ++i) {
            polarPointQueue.push_back({ 100.0f + i, angle });
            angle += angleIncrement;
        }
    }

    Robot::Visualizer::~Visualizer() 
    {

    }


    void Visualizer::run()
    {
        sf::Event event;
        while (window.pollEvent(event)) 
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(background);
        window.draw(axes);

        angle += angleIncrement * 30;
        polarPointQueue.push_back({ 150.0f + maxBufferSize, angle });

        if (polarPointQueue.size() > maxBufferSize) {
            polarPointQueue.pop_front();
        }

        for (const auto& polarPoint : polarPointQueue) {
            sf::Vector2f cartesianPoint = polarToCartesian(polarPoint.first, polarPoint.second);
            cartesianPoint += sf::Vector2f(centerX, centerY);
            pointShape.setPosition(cartesianPoint);
            window.draw(pointShape);
        }
        sf::sleep(sf::milliseconds(500));
        window.display();
    }

    sf::Vector2f Visualizer::polarToCartesian(float radius, float angleDegrees)
    {
        float angleRadians = angleDegrees * (M_PI / 180.0f);
        float x = radius * std::cos(angleRadians);
        float y = radius * std::sin(angleRadians);
        return sf::Vector2f(x, y);
    }




}