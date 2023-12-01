#include "Robot.h"

Robot::TCP_Client
{
    tcp::resolver resolver(msg_content_);
    auto endpoints = resolver.resolve(host, port);
    boost::asio::connect(socket_, endpoints);

}

Robot::TCP_Client::read()
{
    boost::asio::streambuf buf;
    boost::asio::read_until(socket_, buf, "\n");
    std::istream is(&buf);
    std::string data;
    std::getline(is, data);
    return data;
}

Robot::MobileRobot::MobileRobot()
{
    std::cout<<"A Robot is born"<<std::endl;
}

Robot::MobileRobot::~MobileRobot()
{

}

void Robot::Socket::getCmdVelData()
{

}

void Robot::Socket::getOdomData()
{

}

void Robot::Socket::getScanData()
{

}
