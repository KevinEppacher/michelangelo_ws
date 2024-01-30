#include "Robot.h"

namespace Robot
{


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MobileRobot
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //Degree to Radiant
    double convertDegreesToRadiant(double degrees)
    {
        return (degrees * (M_PI/180));
    }


    MobileRobot::MobileRobot(char* ip):ip(ip){

    }


    MobileRobot::~MobileRobot()
    {
        
    }

    //Get current time in Unix-Format
    long long Robot::MobileRobot::getTimeMS(){
        auto currentTimePoint = std::chrono::high_resolution_clock::now();
        auto timeSinceEpoch = std::chrono::time_point_cast<std::chrono::milliseconds>(currentTimePoint);
        return timeSinceEpoch.time_since_epoch().count();
    }


    ////Controls the robot's movement towards a specified goal position and orientation
    //// Arguments:
    ////   - goalPose: Target pose of the robot, including position and orientation.
    ////   - currentOdomPose: Current odometry pose of the robot, including position and orientation.
    ////   - wall: Boolean flag indicating the presence of a wall or obstacle (true if present).
    //// Returns: Boolean indicating successful execution of the function.
    bool Robot::MobileRobot::linearController(Robot::Pose goalPose, Robot::Pose currentOdomPose, bool wall)
    {
        // Initialize PID parameters 
        Parameter PID;

        convertQuaternionsToEuler(&currentOdomPose);

        robotPose = currentOdomPose;

        //Calculate distance from the current position to the goal position
        diffPose.position.x = goalPose.position.x - currentOdomPose.position.x;
        diffPose.position.y = goalPose.position.y - currentOdomPose.position.y;
        totalDistance = calculateTotalDistance(diffPose);

        //Calculate gamma, beta and alpha error
        gamma = calculateGamma(diffPose);
        alpha = calculateAlpha(gamma, currentOdomPose);
        beta = calculateBeta(goalPose, gamma);
        
        // Apply PID controller 
        pidController(&cmdVel, PID, totalDistance, alpha, beta, wall);

        // Ensure the velocities are within specified limits to prevent the robot from moving too fast
        limitControllerVariables(&cmdVel, 2, -2);

        // Send the calculated linear and angular velocities to the robot's movement system
        publishCmdVel(&cmdVel.linear.x, &cmdVel.angular.z);

        return 1;
    }


    //TCP/IP Publisher method
    void Robot::MobileRobot::publishCmdVel(double* linear_x, double* angular_z) 
    {
        std::stringstream ss;
        ss << "---START---{\"linear\": " << *linear_x << ", \"angular\": " << *angular_z << "}___END___";
        std::string echoString = ss.str();

        Robot::TCPClient client(this->ip, 9999);

        client.sendData(echoString.c_str());
    }


    //Methods to calculate distance
    double MobileRobot::calculateTotalDistance(Robot::Pose diffPose)
    {
        return (sqrt(pow(diffPose.position.x,2) + pow(diffPose.position.y, 2)));
    }


    //Methods to calculate Gamma-Error
    double MobileRobot::calculateGamma(Robot::Pose diffPose)
    {
        return (atan2(diffPose.position.y, diffPose.position.x));
    }


    //Methods to calculate Alpha-Error
    double MobileRobot::calculateAlpha(double gamma, Robot::Pose currentOdomPose)
    {
        return (angleDiff(gamma , currentOdomPose.orientation.z));
    }


    //Methods to calculate Beta-Error
    double MobileRobot::calculateBeta(Robot::Pose goalPose, double gamma)
    {
        return (angleDiff(goalPose.orientation.z , gamma));
    }


    //Computes the shortest angular difference between two angles
    double MobileRobot::angleDiff(double angle1, double angle2) 
    {
        double diff = angle1 - angle2;
        while (diff < -M_PI) diff += 2 * M_PI;
        while (diff > M_PI) diff -= 2 * M_PI;
        return diff;
    }

    //// PID_Controller method
    //// Arguments:
    ////   - cmdVel: Pointer to a Twist structure for storing computed velocities
    ////   - PID - A Parameter structure containing PID parameters (not used in this implementation)
    ////   - totalDistance: The distance between the robot and its goal
    ////   - alpha: The angular difference between the robot's orientation and the direction to the goal
    ////   - beta: The angular difference between the goal orientation and the direction to the goal
    ////   - wall: A boolean flag indicating whether wall following behavior should be considered
    bool MobileRobot::pidController(Twist* cmdVel, Parameter PID, double totalDistance, double alpha, double beta, bool wall = false)
    {
        // Initialization of PID parameters for linear, alpha, and beta components
        Parameter Lin, Alpha, Beta;
        Lin.P = 0.15;
        //Lin.I = 0.01;
        Alpha.P = 0.4;
        //Alpha.I = 0.01;
        Beta.P = -0.4;
        //Beta.I = -0.01;

        // Calculate proportional, integral, and total errors for linear motion
        Lin.proportionalError = Lin.P * totalDistance;
        Lin.integralError += ( Lin.I / 2 ) * totalDistance;
        Lin.error = Lin.proportionalError + Lin.integralError;
        cmdVel->linear.x = Lin.error;

        // Calculate proportional, integral, and total errors for alpha
        Alpha.proportionalError = Alpha.P *  alpha;
        Alpha.integralError += (Alpha.I / 2 ) * alpha;
        Alpha.error = Alpha.proportionalError + Alpha.integralError;

        // Calculate proportional, integral, and total errors for beta
        Beta.proportionalError = Beta.P *  beta;
        Beta.integralError += (Beta.I / 2 ) * beta;
        Beta.error = Beta.proportionalError + Beta.integralError;

        // Adjust angular velocity based on wall-following flag
        if (wall == true)
        {
            // Incorporate error angles from PCA when wall following is true
            cmdVel->angular.z = (Alpha.error + Beta.error)*0.9 + this->PCA_Angles(0)*0.05 + this->PCA_Angles(1)*0.05;    
        }

        // Set angular velocity based on alpha and beta errors when wall following is false
        else {
            cmdVel->angular.z = Alpha.error + Beta.error;
        }

        return true;
    }

    //Check and limit linear and angular velocity
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

    //Convert Quaternions to Euler Angles
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


/*     double MobileRobot::calculateRoll(Pose* qA)
    {
        double numerator = 2 * ( qA->orientation.w * qA->orientation.y + qA->orientation.x * qA->orientation.z );
        double denominator = 1 - 2 * (pow(qA->orientation.y, 2) + pow(qA->orientation.z, 2));
        double rollAngle = atan2(numerator, denominator);
        return rollAngle;
    }


    double MobileRobot::calculatePitch(Pose* qA)
    {
        double sinp = 2 * (qA->orientation.w * qA->orientation.y - qA->orientation.z * qA->orientation.x);
        double pitchAngle;

        if (std::abs(sinp) >= 1) {
            pitchAngle = std::copysign(M_PI / 2, sinp);
            return pitchAngle;
        } else {
            pitchAngle = std::asin(sinp);
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

        //std::cout << "Yaw: numerator = " << numerator << ", denominator = " << denominator << ", yawAngle = " << yawAngle << std::endl;
        return yawAngle;
    } */


/*     bool MobileRobot::calculateRobotVektor()
    {
        
        robotVector[0] = 0;
        robotVector[1] = 0;

        return true;
    } */


    void MobileRobot::setIP(char* ipAdress)
    {
        this->ip = ipAdress;
    };


    ////Function to initialize Robot movement to the goal
    //// Arguments:
    ////   - goalPose - Pointer to a Pose struct representing the target position and orientation.
    ////   - currentOdomPose - Pointer to a Pose struct representing the robot's current position and orientation from odometry.
    ////   - wall - A boolean flag indicating whether wall detection is enabled (true) or not (false).
    int MobileRobot::goTo(Pose* goalPose, Pose* currentOdomPose, bool wall = false)
    {
        Pose diffPose;

        // Calculate the positional difference between the current position and the goal
        diffPose.position.x = goalPose->position.x - currentOdomPose->position.x;
        diffPose.position.y = goalPose->position.y - currentOdomPose->position.y;

        // Calculate the total distance to the goal
        double totalDistance = calculateTotalDistance(diffPose);

        convertQuaternionsToEuler(currentOdomPose);
        double totalOrientation = goalPose->orientation.z - currentOdomPose->orientation.z;

        // Check if the robot is within the tolerance distance of the goal
        if (totalDistance < goalPose->tolerance) 
        {
            sequenceNumber += 1;        // Increment the sequence number for the next goal
            arrivedEndgoal();           // Call the function to handle arriving at the goal       
        }
        else if (wall == true)          // If wall detection is enabled
        {
            linearController(*goalPose, *currentOdomPose, wall);
        }
        {
            linearController(*goalPose, *currentOdomPose, wall);
        }
    
        return goalPose->index;
    }

    //Set velocities to 0, if goal is reached
    void MobileRobot::arrivedEndgoal()
    {
        double zero=0;
        publishCmdVel(&zero, &zero);
    }


    //Sensor-Data receiver function
    std::string MobileRobot::receive(int port)      //responsible for receiving data from sensor and robot
    {
        char buffer[16000] = {};            //predefined max storage space
        //Receiving odom-data 
        Robot::TCPClient Client(ip, port);  
        std::string SensorData = Client.receiveData(buffer, sizeof(buffer));      //receiving data from the robot and saving it
        return SensorData;                                                            //returning data to run function
    }


    //Sensor-Data processing function
    void MobileRobot::process(std::string odomData, std::string laserscanData)     //responsible for processing data and giving move commands
    {
        //Setup for parsing odom-data

        Robot::Pose currentOdomPose;
        Robot::JsonHandler OdomdataHandler;
        nlohmann :: json jsonOdom;                      //initialising json handler

        jsonOdom = OdomdataHandler.extractJson(odomData);   //extracting string into json

        //Overwriting current odometry position with Sensor Odometry Position
        currentOdomPose.position.x = jsonOdom["pose"]["pose"]["position"]["x"];
        currentOdomPose.position.y = jsonOdom["pose"]["pose"]["position"]["y"];
        currentOdomPose.position.z = jsonOdom["pose"]["pose"]["position"]["z"];
        currentOdomPose.orientation.x = jsonOdom["pose"]["pose"]["orientation"]["x"];
        currentOdomPose.orientation.y = jsonOdom["pose"]["pose"]["orientation"]["y"];
        currentOdomPose.orientation.z = jsonOdom["pose"]["pose"]["orientation"]["z"];
        currentOdomPose.orientation.w = jsonOdom["pose"]["pose"]["orientation"]["w"]; 


        //Setup for parsing laser-data
        Robot::JsonHandler LaserdataHandler;
        nlohmann :: json jsonScan;
        jsonScan = LaserdataHandler.extractJson(laserscanData);

        //Calculating PCA with sample of 60 Angles
        Robot::PCA pca;
        pca.runPCA(jsonScan["ranges"], 30);
        this->PCA_Angles = pca.getAngleDifference();
        
        //Initialising and defining Goal-Positions
        Robot::Pose goalPose1, goalPose3, goalPose2, goalPose4, goalPose5, goalPose6, goalPose7, goalPose8, goalPose9, goalPose10;
        Robot::Circle circle;
        circle.xOffset = 1.5;
        circle.radius = 0.5;        

        goalPose1.index = 1;
        goalPose1.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-180));
        goalPose1.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-180));
        goalPose1.orientation.z = convertDegreesToRadiant(0);
        goalPose1.tolerance = 0.20;

        goalPose2.index = 2;
        goalPose2.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-135));
        goalPose2.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-135));
        goalPose2.orientation.z = convertDegreesToRadiant(-45);
        goalPose2.tolerance = 0.20;

        goalPose3.index = 3;
        goalPose3.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-90));
        goalPose3.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-90));
        goalPose3.orientation.z = convertDegreesToRadiant(0);
        goalPose3.tolerance = 0.20;

        goalPose4.index = 4;
        goalPose4.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(-45));
        goalPose4.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(-45));
        goalPose4.orientation.z = convertDegreesToRadiant(45);
        goalPose4.tolerance = 0.20;

        goalPose5.index = 5;
        goalPose5.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(0));
        goalPose5.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(0));
        goalPose5.orientation.z = convertDegreesToRadiant(90);
        goalPose5.tolerance = 0.20;

        goalPose6.index = 6;
        goalPose6.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(45));
        goalPose6.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(45));
        goalPose6.orientation.z = convertDegreesToRadiant(135);
        goalPose6.tolerance = 0.20;

        goalPose7.index = 7;
        goalPose7.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(90));
        goalPose7.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(90));
        goalPose7.orientation.z = convertDegreesToRadiant(180);
        goalPose7.tolerance = 0.20;

        goalPose8.index = 8;
        goalPose8.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(135));
        goalPose8.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(135));
        goalPose8.orientation.z = convertDegreesToRadiant(180);
        goalPose8.tolerance = 0.20;

        goalPose9.index = 9;
        goalPose9.position.x = circle.xOffset + circle.radius * cos(convertDegreesToRadiant(180));
        goalPose9.position.y = circle.yOffset + circle.radius * sin(convertDegreesToRadiant(180));
        goalPose9.orientation.z = convertDegreesToRadiant(180);
        goalPose9.tolerance = 0.20;

        goalPose10.index = 10;
        goalPose10.position.x = 0;
        goalPose10.position.y = 0;
        goalPose10.orientation.z = convertDegreesToRadiant(180);
        goalPose10.tolerance = 0.20;

        //Sending Robot to Goal-Position
        if(goalPose1.index == sequenceNumber) {
            goTo(&goalPose1, &currentOdomPose, true);
            }
        if(goalPose2.index == sequenceNumber) {
            goTo(&goalPose2, &currentOdomPose);
            }
        if(goalPose3.index == sequenceNumber) {
            goTo(&goalPose3, &currentOdomPose);
            }
        if(goalPose4.index == sequenceNumber) {
            goTo(&goalPose4, &currentOdomPose);
            }
        if(goalPose5.index == sequenceNumber) {
            goTo(&goalPose5, &currentOdomPose);
            } 
        if(goalPose6.index == sequenceNumber) {
            goTo(&goalPose6, &currentOdomPose);
            }
        if(goalPose7.index == sequenceNumber) {
            goTo(&goalPose7, &currentOdomPose);
            }
        if(goalPose8.index == sequenceNumber) {
            goTo(&goalPose8, &currentOdomPose);
            }
        if(goalPose9.index == sequenceNumber) {
            goTo(&goalPose9, &currentOdomPose);
            }
        if(goalPose10.index == sequenceNumber) {
            goTo(&goalPose10, &currentOdomPose);
            }
    }
    

    
    bool MobileRobot::run()
    {
        //receive and process sensor data
        std::string input = MobileRobot::receive(9998);         	//calling receive function and saving data as input
        std::string laserScan = MobileRobot::receive(9997);
        const char* charInput = input.c_str();                      //structuring input
        SHM myBrain(charInput);                                     //creating SHM class object and calling constructor -> starting shared Memory
        std::string odomOutput = myBrain.returnOutput();            //calling result of shared memory via getter
        MobileRobot::process(input, laserScan);                     //calling process function responsible for calculating movements and commanding the robot with the output as parameter
        return true;
    }



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MobileRobot
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
TCP Client
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    //// Constructor for the TCPClient class
    //// Arguments:
    ////   - serverIP - The IP address of the server to connect to as a C-style string
    ////   - port - The port number to connect to on the server
    TCPClient::TCPClient(const char* serverIP, int port)
    {
        // Create a new socket. AF_INET indicates IPv4, SOCK_STREAM indicates TCP
        client_fd = socket(AF_INET, SOCK_STREAM, 0);

        // Check if socket creation was successful
        if (client_fd < 0) {
            printf("\n Socket creation error \n");
            exit(EXIT_FAILURE);                                                             // Exit if socket creation failed
        }

        // Set the family type of the IP address (IPv4)
        serv_addr.sin_family = AF_INET;
        // Convert the port number from host byte order to network byte order
        serv_addr.sin_port = htons(port);

        // Convert the server IP address from text to binary form
        if (inet_pton(AF_INET, serverIP, &serv_addr.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            exit(EXIT_FAILURE);                                                             // Exit if IP address is invalid
        }

        // Establish a connection to the server
        if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            printf("\nConnection Failed \n");
            exit(EXIT_FAILURE);                                                             // Exit if connection to server fails
        }
    }


    TCPClient::~TCPClient() 
    {
        close(client_fd);
    }


/*     void TCPClient::closeTCPconnection()
    {
        close(client_fd);
    } */


    //Send data ia TCP/IP
    void TCPClient::sendData(const char* data) 
    {
        send(client_fd, data, strlen(data), 0);
        close(client_fd);
    }

    //Receive data via TCP/IP
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
                } 
            }

            // Check for the end marker
            if (startFound && result.find("___END___") != std::string::npos) {
                break;
            }
        }
        return result;
    }



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
TCP Client
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Json-Class
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 


    JsonHandler::JsonHandler(){}


    JsonHandler::~JsonHandler(){}


    // Extracts a JSON object from a raw string of data
    nlohmann::json JsonHandler::extractJson(std::string rawData)
        {
            std::string jsonStr;

            // Delimiters to identify the start and end of the JSON data in the raw string
            std::string startDelimiter = "---START---";
            std::string endDelimiter = "___END___";

            // Check if the start delimiter is found in rawData
            if (rawData.find(startDelimiter) != std::string::npos)
            {
                // Calculate the starting position of JSON data after the start delimiter
                std::size_t startPos = rawData.find(startDelimiter) + startDelimiter.length();
                // Find the ending position of the JSON data using the end delimiter
                std::size_t endPos = rawData.find(endDelimiter, startPos);
                // Extract the JSON string from rawData
                jsonStr = rawData.substr(startPos, endPos - startPos);
            }

            try 
            {
                // Parse the extracted JSON string into a json object
                JsonHandler::jsonData = nlohmann::json::parse(jsonStr);
            } 
            
            // Catch block to handle any parsing errors
            catch (nlohmann::json::parse_error& e) 
            {
                std::cerr << "JSON parse error: " << e.what() << '\n';
                std::cout << "Parsing finished uncorrectly" << std::endl;
            }

            return jsonData;
        }


    //Output Value to the corresponding key, if existing
    std::string JsonHandler::JsonOutputter(const std::string key)
        {
            try 
            {
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

    //Convert string to raw string
    std::string JsonHandler::StringtoRaw(std::string normalString)
        {
            std::ostringstream oss;
            oss << "R\"(";
            oss << ")\"";
            return oss.str();
        }


/*     nlohmann::json JsonHandler::get_jsonData()
        {
            return jsonData;
        } */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Json-Class
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
PCA
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    PCA::PCA(){};
    PCA::~PCA(){};


    //Run PCA calculation and plotting
    void PCA::runPCA(std::vector<double> rawLaserScan, int ScanSample)
    {
        //Map std::vector to eigen::vector
        Eigen::VectorXd laser = Eigen::VectorXd::Map(&rawLaserScan[0], rawLaserScan.size());

        //convert polar coordinates to cartesian coordinates
        Eigen::MatrixXd data = this->PolarToCartesian(laser);

/*         //Compute PCA
        Eigen::VectorXd principal_component = this->computePCA(data);
 */
        //Filter Right and Left side and calculate PCA seperately
        this->FilterLaserscan(data, ScanSample);
        Eigen::MatrixXd left = this->getFilteredLeftScanData();
        Eigen::MatrixXd right = this->getFilteredRightScanData();

        Eigen::VectorXd principal_componentLeft = this->computePCA(left);
        this->PCA_Left = principal_componentLeft;

        Eigen::VectorXd principal_componentRight = this->computePCA(right);
        this->PCA_Right = principal_componentRight;
        this->plotData(data, principal_componentLeft, principal_componentRight, ScanSample);
    }

    //Polar-Coordinates to Cartesian-Coordinates Converter
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

        return cartesianLaserScanData;
    };

    //Method to filter Laserscan-Data
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

    //PCA calculation
    Eigen::VectorXd PCA::computePCA(const Eigen::MatrixXd &data)
    {
        // Centering the data

        Eigen::VectorXd mean = data.colwise().mean();
        Eigen::MatrixXd centered = data.rowwise() - mean.transpose();

        // Computing the covariance matrix
        Eigen::MatrixXd cov = centered.adjoint() * centered;

        // Performing eigen decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov);
        Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();

        // Extracting the principal component
        Eigen::VectorXd principal_component = eigenvectors.rightCols(1);

        return principal_component;
    };

    //Plot-Function to visualize Laserscan-Data and PCA-Vectors
    void PCA::plotData(Eigen::MatrixXd laserScanData, Eigen::VectorXd PCA_vectorLeft, Eigen::VectorXd PCA_vectorRight, int filterTolerance )
    {
        matplotlibcpp::ion(); // Turn on interactive mode

        //Convert Eigen::VectorXd to std::vector
        std::vector<double> x_data(laserScanData.rows()), y_data(laserScanData.rows());
        for (int i = 0; i < laserScanData.rows(); ++i) {
            x_data[i] = laserScanData(i, 0);
            y_data[i] = laserScanData(i, 1);
        }

        //Get offset to correct PCA_Vector position
        double offsetRechts = this->filteredLaserScanLeftSide.mean()*2;
        double offsetLinks = this->filteredLaserScanRightSide.mean()*2;

        //clear plot window
        matplotlibcpp::clf();

        // Plotting the laser scan data
        matplotlibcpp::scatter(x_data, y_data);
        matplotlibcpp::title("PCA vector");

        // Calculate end points for the PCA vector for visualization
        double scale_factor = 10.0;                                                            // Adjust this factor to scale the PCA vector for better visualization


        std::vector<double> pca_xLeft = {0, scale_factor * PCA_vectorLeft(0)};
        std::vector<double> pca_yLeft = {0, scale_factor * PCA_vectorLeft(1)};

        std::vector<double> pca_xRight = {0, scale_factor * PCA_vectorRight(0)};
        std::vector<double> pca_yRight = {0, scale_factor * PCA_vectorRight(1)};

        std::vector<double> directionVectorX = {scale_factor, 0};
        std::vector<double> directionVectorY = {0, 0};

        // Explicitly defining the start points for the quiver plot
        std::vector<double> start_x = {0,0};
        std::vector<double> start_y = {0,0};

        std::vector<double> start_xPCALeft = {0,0};
        std::vector<double> start_yPCALeft = {0,offsetLinks};

        std::vector<double> start_xPCARight = {0,0};
        std::vector<double> start_yPCARight = {0,offsetRechts};

        // Plotting the PCA vector
        std::map<std::string, std::string> keywordDirection;
        keywordDirection["color"] = "r";

        std::map<std::string, std::string> keywordPCA;
        keywordPCA["color"] = "b";

        matplotlibcpp::quiver(start_xPCALeft, start_yPCALeft, pca_xLeft, pca_yLeft, keywordPCA);
        matplotlibcpp::quiver(start_xPCARight, start_yPCARight, pca_xRight, pca_yRight, keywordPCA);
        matplotlibcpp::quiver(start_x, start_y, directionVectorX, directionVectorY, keywordDirection);

        // Show the plot and update frequently
        matplotlibcpp::draw();
        matplotlibcpp::pause(0.01);
    };

    //Calculate Angle between Direction of the Robot and PCA
    Eigen::VectorXd PCA::getAngleDifference()
    {
        Eigen::VectorXd Thetas(2);

        //Define direction vector
        Eigen::VectorXd directionVector(2);
        directionVector(0) = 1;
        directionVector(1) = 0;

        //Multiply both vectors together
        double dotProductLeft = (directionVector.dot(this->PCA_Left));

        //Calculate magnitude of both vectors
        double magnitudeVec1Left = directionVector.norm();
        double magnitudeVec2Left = this->PCA_Left.norm();

        // Calculate the cosine of the angle
        double cosAngleLeft = dotProductLeft / (magnitudeVec1Left * magnitudeVec2Left);

        // Ensure the cosine value is within [-1, 1] to avoid NaN due to floating point errors
        cosAngleLeft = std::max(-1.0, std::min(1.0, cosAngleLeft));

        // Calculate the angle in radians
        Thetas(0) = std::acos(cosAngleLeft);

        //Same procedure for the right side
        double dotProductRight = (directionVector.dot(this->PCA_Right));

        double magnitudeVec1Right = directionVector.norm();
        double magnitudeVec2Right = this->PCA_Right.norm();

        double cosAngleRight = dotProductRight / (magnitudeVec1Right * magnitudeVec2Right);

        cosAngleRight = std::max(-1.0, std::min(1.0, cosAngleRight));

        Thetas(1) = std::acos(cosAngleRight);

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



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
PCA
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
sharedMemory
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    SHM::SHM(const std::string& input) : input(input), output() {
        mutexID = semget(IPC_PRIVATE, 1, 0666);                                                                         //creating mutex semaphore
        shmID = shmget(IPC_PRIVATE, sizeof(struct SHM_Message), 0666 | IPC_CREAT);                                      //saving semapgore ID
        processID = fork();                                                                                             //splitting process and saving ID of child as processID
        if (semctl(mutexID, 0, SETVAL, 1) == -1) {                                                                      //initialising semaphore as active
            std::cout << "semctl SETVAL failed!\n shutting down...\n" << std::endl;                     
            std::exit(EXIT_FAILURE);                        
        }                       

        if (processID == 0) {                                                                                           //if the process is the child of the original ->It's the consumer                       
            signal(SIGINT, [](int sig) { SHM::signalHandler(sig, nullptr); });                                          //enabling structured shutdown
            shmptr = (SHM_Message*)shmat(shmID, NULL, 0);                                                               //defining location of saved data                       
            checkSignal(mutexID);                                                                                       //only continue if semaphore is active (meaning no one accessing the data)
            output = shmptr->information;                                                                               //save the data in the specified location by the pointer as output
            setSignal(mutexID);                                                                                         //signaling via semaphore that the data can be accessed again
        }
        else {                                                                                                          //otherwise this has to be the parent/producer
            signal(SIGINT, [](int sig) { SHM::signalHandler(sig, nullptr); });                                          //enabling structered shutdown
            shmptr = (SHM_Message*)shmat(shmID, NULL, 0);                                                               //attach shared memory at pointer location                       
            checkSignal(mutexID);                                                                                       //only continue if semaphore is active (meaning no one accessing the data)
            strncpy(shmptr->information, input.c_str(), sizeof(shmptr->information) - 1);        setSignal(mutexID);    //saving input vriable as string into the shared memory storage
            waitpid(processID, 0 , 0);                                                                                  //wait until the child is completed until continue
            kill(processID, SIGTERM);                                                                                   //stop the child process
        }
    }


    SHM::~SHM() {                                                                                   
        shmdt(shmptr);                                                                                                  //shared memory storage is deleted
    }                   
                    
                    
    void SHM::checkSignal(int semid){                                                                                   //defining checkSignal function
    struct sembuf check = {0, -1, SEM_UNDO};                                                                            //needed for checking semaphores                       
        if (semop(semid, &check, 1) == -1){                                                                             //in case the semaphore is invalid shutdown programm
            std::cout << "semaphore check failed!\n shutting down...\n" << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }


    void SHM::setSignal(int semid){                                                                                     //defining setSignal function
    struct sembuf set = {0, 1, SEM_UNDO};                                                                               //needed for setting semaphores
        if (semop(semid, &set, 1) == -1){                                                                               //in case the semaphore is invalid shutdown the programm
            std::cout << "semaphore set failed!\n shutting down...\n" << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }


    void SHM::signalHandler(int sig, SHM* instance) {                                                                   //enables clean termination of processes
        if (sig == SIGINT) {                
            if (instance->processID == 0) {  // Consumer process                                                        //which process has to be killed
                std::cout << "Consumer shutting down \n";                                                               //Consumer/ Child doesnt has to be shut down since Producer/Parent kills Child
            } else {                
                std::cout << "Producer shutting down \n";                                                           
                std::cout << "Producer killing consumer\n";             
                kill(instance->processID, SIGINT);                                                                      //Producer kills child process
                wait(NULL);             
                std::cout << "Producer removing semaphores\n";                                                          //deleting semaphores
                semctl(instance->mutexID, -1, IPC_RMID);                
                std::cout << "Producer detaching and removing shared memory\n";             
                if (shmdt(instance->shmptr) == -1) {                                                                    //deleting shared memory 
                    std::cout << "shmdt failed\n";              
                    exit(EXIT_FAILURE);                                         
                }               
                if (shmctl(instance->shmID, IPC_RMID, 0) == -1) {                                                       //reseting shared memory
                    std::cout << "shmctl(IPC_RMID) failed\n";
                    exit(EXIT_FAILURE);
                }
            }
            exit(EXIT_SUCCESS);
        }
    }


    std::string SHM::returnOutput() {                                                                                   //getter for output of shared Memory
        return output;
    }



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
sharedMemory
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
};