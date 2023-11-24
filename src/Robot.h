#include <iostream>


namespace Robot
{
    class MobileRobot
    {
    public:
        MobileRobot();
        ~MobileRobot();

    private:
        double scanMsg;
        double odomMsg;
        double cmdVelMsg;

    };
    
    class Socket: public MobileRobot
    {
    public:
        Socket();
        ~Socket();
        void getScanData();
        void getOdomData();
        void getCmdVelData();

    private:

    };

    
}