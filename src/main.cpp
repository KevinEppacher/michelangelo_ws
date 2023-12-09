#include "Robot.h"
#include <iostream>
#include <sstream> 

int main(int argc, char *argv[]) {

    char buffer[1024] = {0};

    //Robot::TCPClient client("127.0.0.1", 8080);
    double i = 0;

    std::string data = R"(---START---{"header": {"seq": 27725, "stamp": {"secs": 1701970462, "nsecs": 195060962}, "frame_id": "base_scan"}, "angle_min": 0.0, "angle_max": 6.2657318115234375, "angle_increment": 0.01745329238474369, "time_increment": 0.0005592841189354658, "scan_time": 0.20134228467941284, "range_min": 0.11999999731779099, "range_max": 3.5, "ranges": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.1610000133514404, 2.9639999866485596, 2.8310000896453857, 2.7209999561309814, 2.6489999294281006, 0.6809999942779541, 0.6809999942779541, 0.6710000038146973, 0.6740000247955322, 0.6759999990463257, 0.6800000071525574, 0.8299999833106995, 1.7020000219345093, 0.7850000262260437, 0.6880000233650208, 0.6930000185966492, 0.6980000138282776, 0.6909999847412109, 0.6880000233650208, 0.6579999923706055, 0.6240000128746033, 0.597000002861023, 0.5860000252723694, 0.5720000267028809, 0.5630000233650208, 0.5590000152587891, 0.5540000200271606, 0.550000011920929, 0.5440000295639038, 0.5460000038146973, 0.5460000038146973, 0.5529999732971191, 0.5600000023841858, 0.5669999718666077, 0.5759999752044678, 0.5860000252723694, 0.6119999885559082, 1.2769999504089355, 0.7919999957084656, 0.7919999957084656, 0.8019999861717224, 0.8059999942779541, 0.7960000038146973, 0.8029999732971191, 0.7990000247955322, 0.7900000214576721, 0.7820000052452087, 0.7950000166893005, 0.7760000228881836, 0.7590000033378601, 0.7590000033378601, 0.7440000176429749, 0.734000027179718, 0.7260000109672546, 0.7269999980926514, 0.718999981880188, 0.7229999899864197, 0.7289999723434448, 0.7360000014305115, 0.7250000238418579, 0.7070000171661377, 0.6959999799728394, 0.6909999847412109, 0.675000011920929, 0.6470000147819519, 0.6809999942779541, 0.6809999942779541, 0.6809999942779541, 0.6759999990463257, 0.6620000004768372, 0.6499999761581421, 0.6389999985694885, 0.6290000081062317, 0.6190000176429749, 0.6129999756813049, 0.6129999756813049, 0.6069999933242798, 0.6000000238418579, 0.5929999947547913, 0.5870000123977661, 0.5809999704360962, 0.574999988079071, 0.5720000267028809, 0.5690000057220459, 0.5720000267028809, 0.531000018119812, 0.5649999976158142, 0.5609999895095825, 0.5580000281333923, 0.5550000071525574, 0.5519999861717224, 0.5519999861717224, 0.5550000071525574, 0.5509999990463257, 0.5529999732971191, 0.550000011920929, 0.5479999780654907, 0.546999990940094, 0.5460000038146973, 0.5450000166893005, 0.5460000038146973, 0.546999990940094, 0.550000011920929, 0.5550000071525574, 0.5609999895095825, 0.5680000185966492, 0.5759999752044678, 0.5849999785423279, 0.597000002861023, 0.6069999933242798, 0.6179999709129333, 0.621999979019165, 0.625, 0.0, 1.3760000467300415, 1.4190000295639038, 1.4290000200271606, 1.4490000009536743, 1.472000002861023, 1.5089999437332153, 1.5460000038146973, 1.5779999494552612, 1.6009999513626099, 1.6360000371932983, 1.6460000276565552, 1.7059999704360962, 1.75, 1.7869999408721924, 1.8070000410079956, 0.9470000267028809, 0.8859999775886536, 0.8669999837875366, 0.8489999771118164, 0.8349999785423279, 0.8429999947547913, 0.8090000152587891, 0.7720000147819519, 0.7379999756813049, 0.7139999866485596, 0.7089999914169312, 0.7009999752044678, 0.6899999976158142, 0.6880000233650208, 0.7289999723434448, 0.7459999918937683, 0.7630000114440918, 0.8820000290870667, 0.0, 0.7080000042915344, 0.6990000009536743, 0.6909999847412109, 0.6899999976158142, 0.6970000267028809, 0.7020000219345093, 0.7680000066757202, 2.13100004196167, 2.188999891281128, 2.184999942779541, 2.180000066757202, 2.177000045776367, 1.9819999933242798, 1.9490000009536743, 1.9600000381469727, 1.9459999799728394, 1.9490000009536743, 1.965000033378601, 1.9639999866485596, 1.9509999752044678, 1.965000033378601, 2.190000057220459, 2.2049999237060547, 2.2200000286102295, 2.2149999141693115, 1.128000020980835, 0.0, 0.0, 0.0, 0.0, 0.6990000009536743, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.22499999403953552, 0.22499999403953552, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.24199999868869781, 0.24199999868869781, 0.23899999260902405, 0.23899999260902405, 0.23800000548362732, 0.2370000034570694, 0.2370000034570694, 0.23600000143051147, 0.23600000143051147, 0.23600000143051147, 0.23600000143051147, 0.23600000143051147, 0.23600000143051147, 0.23600000143051147, 0.2370000034570694, 0.23800000548362732, 0.23800000548362732, 0.23800000548362732, 0.23800000548362732, 0.23800000548362732, 0.23899999260902405, 0.23999999463558197, 0.2409999966621399, 0.24199999868869781, 0.24400000274181366, 0.24500000476837158, 0.24799999594688416, 0.25099998712539673, 0.2529999911785126, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2529999911785126, 0.0, 0.0, 0.8460000157356262, 0.7940000295639038, 0.7519999742507935, 0.7379999756813049, 0.7250000238418579, 0.7319999933242798, 0.7329999804496765, 0.7279999852180481, 0.8090000152587891, 0.8109999895095825, 0.8130000233650208, 0.9110000133514404, 0.9649999737739563, 1.9869999885559082, 2.0179998874664307, 1.9359999895095825, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "intensities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 141.0, 45.0, 90.0, 0.0, 99.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 83.0, 80.0, 66.0, 33.0, 0.0, 33.0, 66.0, 74.0, 83.0, 113.0, 131.0, 103.0, 108.0, 109.0, 117.0, 117.0, 1616.0, 3317.0, 3226.0, 3369.0, 3904.0, 2133.0, 210.0, 383.0, 1261.0, 2767.0, 1875.0, 2638.0, 1720.0, 2529.0, 2434.0, 2825.0, 2954.0, 3555.0, 3818.0, 2767.0, 4069.0, 2640.0, 3231.0, 4179.0, 3863.0, 4335.0, 3784.0, 4267.0, 3661.0, 4941.0, 2789.0, 2234.0, 158.0, 1063.0, 2325.0, 2602.0, 3136.0, 3043.0, 2822.0, 3764.0, 2934.0, 2222.0, 3588.0, 3061.0, 3554.0, 2332.0, 2724.0, 2435.0, 2324.0, 3074.0, 2472.0, 2836.0, 3399.0, 3730.0, 1979.0, 2838.0, 3331.0, 2049.0, 3234.0, 2967.0, 2279.0, 3863.0, 3433.0, 2633.0, 3253.0, 3919.0, 3502.0, 3323.0, 2964.0, 3985.0, 2799.0, 3872.0, 3114.0, 3198.0, 3644.0, 3116.0, 3912.0, 3384.0, 4608.0, 3890.0, 1979.0, 3776.0, 4233.0, 3609.0, 3626.0, 4423.0, 4066.0, 3436.0, 3499.0, 3815.0, 3492.0, 3908.0, 3520.0, 3529.0, 4108.0, 3784.0, 3806.0, 3553.0, 4138.0, 3796.0, 4340.0, 3342.0, 3337.0, 3658.0, 3831.0, 3196.0, 3330.0, 3350.0, 748.0, 1401.0, 1414.0, 1348.0, 1294.0, 1219.0, 1099.0, 1081.0, 978.0, 887.0, 825.0, 745.0, 672.0, 514.0, 480.0, 384.0, 1877.0, 1857.0, 2611.0, 1947.0, 3030.0, 2400.0, 1923.0, 2725.0, 2419.0, 3002.0, 2828.0, 2622.0, 2858.0, 2770.0, 2167.0, 3086.0, 2867.0, 81.0, 499.0, 1896.0, 1789.0, 1880.0, 2041.0, 2189.0, 3080.0, 1772.0, 150.0, 556.0, 585.0, 645.0, 587.0, 935.0, 901.0, 898.0, 914.0, 887.0, 873.0, 894.0, 874.0, 853.0, 472.0, 532.0, 496.0, 499.0, 140.0, 105.0, 91.0, 0.0, 0.0, 6772.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1586.0, 105.0, 0.0, 0.0, 46.0, 0.0, 0.0, 0.0, 59.0, 46.0, 0.0, 52.0, 78.0, 71.0, 97.0, 104.0, 233.0, 703.0, 843.0, 1097.0, 1452.0, 2066.0, 2649.0, 3404.0, 4329.0, 5159.0, 5966.0, 5738.0, 6474.0, 6651.0, 6242.0, 7818.0, 8111.0, 6536.0, 5101.0, 3955.0, 2951.0, 2367.0, 1623.0, 1359.0, 943.0, 666.0, 419.0, 356.0, 216.0, 95.0, 75.0, 55.0, 61.0, 0.0, 32.0, 39.0, 43.0, 39.0, 117.0, 39.0, 0.0, 1595.0, 4558.0, 2339.0, 3978.0, 2521.0, 3152.0, 2868.0, 3955.0, 2723.0, 3029.0, 545.0, 412.0, 781.0, 274.0, 439.0, 303.0, 0.0, 0.0, 0.0, 0.0, 0.0, 68.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}---END---)";
    Robot::JsonHandler jsonHüdai;
    jsonHüdai.extractJson(data);

    //std::cout << jsonHüdai.get_jsonData() << std::endl;

    while(true)
    {
        i+=0.005;  
        double linear  = 0 + i;
        double angular  = 0;
        std::stringstream ss;
        ss << "---START---{\"linear\": " << linear << ", \"angular\": " << angular << "}___END___";
        std::string echoString = ss.str();

        std::cout<<i<<std::endl;

        Robot::TCPClient client("192.168.100.51", 9999);
        client.sendData(echoString.c_str());
        client.receiveData(buffer, sizeof(buffer));  
        //client.closeTCPconnection();


        std::cout << jsonHüdai.JsonOutputter("angle_min") << std::endl;

        
    }

    

    return 0;
}

/*
#pragma once
#include "tcpHandler.h"
class sendCmd : private TCPHandler
{
private:
public:
    sendCmd(){};
    ~sendCmd(){};
    void sendCmdVel(float linear, float angular)
    {
        std::cout << "sendCmdVel is called!\n";
        echoServPort = 9999;
        std::stringstream ss;
        ss << "---START---{\"linear\":" << linear << ", \"angular\":" << angular << "}__END__";
        std::string echoString = ss.str();

        if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
            DieWithError("socket() failed");
        memset(&echoServAddr, 0, sizeof(echoServAddr));
        echoServAddr.sin_family = AF_INET;
        echoServAddr.sin_addr.s_addr = inet_addr(servIP.c_str());
        echoServAddr.sin_port = htons(echoServPort);
        if (connect(sock, (struct sockaddr *)&echoServAddr, sizeof(echoServAddr)) < 0)
        {
            DieWithError("connect() failed");
        }

        size_t echoStringLen = echoString.length();

        if (send(sock, echoString.c_str(), echoStringLen, 0) != static_cast<ssize_t>(echoStringLen))
            DieWithError("send() sent a different number of bytes than expected");
        std::cout << std::endl;

        close(sock);
    }
};


*/









/*
int main(int argc, char *argv[])
{
    //Robot::Socket robotSocket(argv[1], argv[2], (argc == 4) ? atoi(argv[3]) : 7);

    Robot::Socket robotSocket(argv[1], "hello", 8080);



    // Receive the same string back from the server
    while (true)
    {
        robotSocket.sendAndReceiveData();
        std::cout<<"Testing sending data"<<std::endl;
    }
    return 0;
}

*/