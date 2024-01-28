#include <iostream>
#include <cstring>
#include <netinet/in.h>
#include <unistd.h>

class TCPServer {
public:
    TCPServer(int port) : port(port) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == -1) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        int opt = 1;
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        if (listen(server_fd, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }
    }

    ~TCPServer() {
        close(server_fd);
    }

    void acceptConnection() {
        addrlen = sizeof(address);
        new_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen);
        if (new_socket < 0) {
            perror("accept");
            exit(EXIT_FAILURE);
        }
    }

    void receiveData(char* buffer, ssize_t size) {
        valread = read(new_socket, buffer, size - 1);
        buffer[valread] = '\0'; // Null-terminator hinzufügen
        std::cout << buffer << std::endl;
    }

    void sendData(const char* data) {
        send(new_socket, data, strlen(data), 0);
        std::cout << "Message sent: " << data << std::endl;
    }

private:
    int server_fd, new_socket;
    ssize_t valread;
    struct sockaddr_in address;
    socklen_t addrlen;
    int port;
};




int main() {
    const char* data = R"(---START---{"header": {"seq": 70, "stamp": {"secs": 1706289885, "nsecs": 457732796}, "frame_id": "base_scan"}, "angle_min": 0.0, "angle_max": 6.2657318115234375, "angle_increment": 0.01745329238474369, "time_increment": 0.0005592841189354658, "scan_time": 0.20134228467941284, "range_min": 0.11999999731779099, "range_max": 3.5, "ranges": [0.0, 0.0, 0.0, 0.0, 3.380000114440918, 3.36299991607666, 3.371999979019165, 3.365999937057495, 3.390000104904175, 3.436000108718872, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9599999785423279, 0.0, 0.8820000290870667, 0.8859999775886536, 0.890999972820282, 0.906000018119812, 0.0, 0.0, 0.0, 0.0, 3.9200000762939453, 3.9670000076293945, 0.0, 0.0, 0.0, 1.625, 1.6390000581741333, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5839999914169312, 0.0, 1.7489999532699585, 2.9590001106262207, 0.0, 0.0, 0.0, 2.944999933242798, 2.9670000076293945, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.115999937057495, 1.6540000438690186, 1.6009999513626099, 1.5989999771118164, 1.6319999694824219, 1.6540000438690186, 0.0, 3.3369998931884766, 0.0, 1.9199999570846558, 1.9819999933242798, 1.9529999494552612, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.7940000295639038, 1.809999942779541, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.184999942779541, 4.144999980926514, 3.999000072479248, 3.937999963760376, 3.756999969482422, 3.8469998836517334, 3.7170000076293945, 3.6459999084472656, 0.0, 1.190999984741211, 1.187000036239624, 1.2369999885559082, 3.4130001068115234, 3.319000005722046, 3.302999973297119, 3.255000114440918, 3.194000005722046, 3.188999891281128, 3.122999906539917, 3.1050000190734863, 3.0820000171661377, 3.0299999713897705, 2.9560000896453857, 2.947000026702881, 2.9159998893737793, 2.8949999809265137, 2.8550000190734863, 2.8410000801086426, 2.8269999027252197, 2.7829999923706055, 2.763000011444092, 2.753999948501587, 2.7139999866485596, 2.7309999465942383, 2.703000068664551, 2.681999921798706, 2.6510000228881836, 2.6440000534057617, 2.23799991607666, 2.2160000801086426, 2.2070000171661377, 2.188999891281128, 2.184000015258789, 2.184999942779541, 2.2100000381469727, 1.100000023841858, 1.0499999523162842, 1.0740000009536743, 1.0709999799728394, 1.0640000104904175, 1.0570000410079956, 1.0540000200271606, 1.065999984741211, 1.069000005722046, 1.069000005722046, 1.0700000524520874, 1.0570000410079956, 1.0889999866485596, 1.0750000476837158, 1.0820000171661377, 1.0789999961853027, 1.0750000476837158, 1.0700000524520874, 1.0959999561309814, 1.069000005722046, 1.1059999465942383, 1.1169999837875366, 1.093999981880188, 1.11899995803833, 1.121999979019165, 1.1169999837875366, 1.1330000162124634, 1.1490000486373901, 1.125, 1.1339999437332153, 1.152999997138977, 1.1679999828338623, 1.159999966621399, 1.187000036239624, 1.190999984741211, 1.1990000009536743, 1.2070000171661377, 1.2259999513626099, 1.2330000400543213, 1.2519999742507935, 1.2690000534057617, 1.2569999694824219, 1.2949999570846558, 1.3070000410079956, 1.3029999732971191, 1.3489999771118164, 1.3580000400543213, 1.3760000467300415, 1.3969999551773071, 1.4170000553131104, 1.4559999704360962, 1.4759999513626099, 1.4429999589920044, 1.4249999523162842, 1.3980000019073486, 1.3600000143051147, 1.343000054359436, 1.3250000476837158, 1.3029999732971191, 1.2640000581741333, 0.7910000085830688, 0.0, 0.7360000014305115, 1.1929999589920044, 1.2000000476837158, 1.1699999570846558, 1.1690000295639038, 1.121000051498413, 1.1299999952316284, 1.1510000228881836, 1.1239999532699585, 1.1119999885559082, 1.1130000352859497, 1.100000023841858, 1.0980000495910645, 1.090000033378601, 1.0850000381469727, 1.0609999895095825, 1.0750000476837158, 1.034999966621399, 1.034000039100647, 1.0529999732971191, 1.0199999809265137, 1.0080000162124634, 1.0210000276565552, 1.0160000324249268, 1.0160000324249268, 1.0010000467300415, 1.0010000467300415, 1.0019999742507935, 1.0099999904632568, 0.9850000143051147, 0.9850000143051147, 0.9789999723434448, 0.9869999885559082, 0.9810000061988831, 0.9850000143051147, 0.9779999852180481, 0.9769999980926514, 0.9760000109672546, 0.9670000076293945, 0.9670000076293945, 0.968999981880188, 0.9710000157356262, 0.9789999723434448, 0.9789999723434448, 0.9700000286102295, 0.9800000190734863, 0.9750000238418579, 0.9860000014305115, 0.9990000128746033, 1.0010000467300415, 1.0010000467300415, 1.0110000371932983, 1.0110000371932983, 1.0140000581741333, 1.0290000438690186, 1.0329999923706055, 1.0390000343322754, 1.059999942779541, 1.0449999570846558, 1.059999942779541, 1.0490000247955322, 1.0870000123977661, 1.0809999704360962, 1.1169999837875366, 1.1030000448226929, 1.1130000352859497, 1.1239999532699585, 1.1230000257492065, 1.121000051498413, 1.1619999408721924, 1.1540000438690186, 1.1740000247955322, 1.2020000219345093, 1.2050000429153442, 1.225000023841858, 1.253999948501587, 1.2489999532699585, 1.281000018119812, 1.309999942779541, 1.309000015258789, 1.3420000076293945, 1.375, 1.3869999647140503, 1.409999966621399, 1.434999942779541, 1.4470000267028809, 1.496999979019165, 1.5169999599456787, 1.5520000457763672, 1.5950000286102295, 1.6339999437332153, 1.6649999618530273, 1.7050000429153442, 1.7289999723434448, 1.8009999990463257, 1.8300000429153442, 1.9179999828338623, 1.9609999656677246, 2.01200008392334, 2.072000026702881, 2.1610000133514404, 2.200000047683716, 2.2950000762939453, 2.4079999923706055, 2.3970000743865967, 2.367000102996826, 2.3389999866485596, 2.325000047683716, 0.0, 3.0959999561309814, 3.322999954223633, 3.453000068664551, 3.7269999980926514, 3.9260001182556152, 4.1539998054504395, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.739000082015991], "intensities": [301.0, 304.0, 302.0, 49.0, 228.0, 373.0, 369.0, 382.0, 271.0, 176.0, 49.0, 0.0, 39.0, 37.0, 0.0, 0.0, 0.0, 0.0, 0.0, 74.0, 0.0, 0.0, 0.0, 0.0, 4029.0, 0.0, 115.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 71.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 105.0, 0.0, 0.0, 0.0, 69.0, 44.0, 0.0, 47.0, 0.0, 0.0, 59.0, 531.0, 0.0, 2598.0, 2831.0, 5113.0, 2290.0, 0.0, 0.0, 553.0, 0.0, 129.0, 157.0, 113.0, 80.0, 68.0, 1860.0, 263.0, 90.0, 112.0, 116.0, 110.0, 123.0, 130.0, 125.0, 119.0, 441.0, 67.0, 475.0, 120.0, 67.0, 0.0, 54.0, 195.0, 233.0, 0.0, 145.0, 96.0, 0.0, 0.0, 75.0, 111.0, 57.0, 194.0, 441.0, 711.0, 752.0, 695.0, 239.0, 0.0, 271.0, 157.0, 120.0, 403.0, 510.0, 0.0, 0.0, 0.0, 57.0, 69.0, 78.0, 1380.0, 1092.0, 133.0, 147.0, 164.0, 180.0, 197.0, 214.0, 230.0, 240.0, 257.0, 273.0, 149.0, 188.0, 327.0, 347.0, 632.0, 1900.0, 2292.0, 698.0, 415.0, 416.0, 445.0, 454.0, 449.0, 487.0, 483.0, 487.0, 489.0, 504.0, 617.0, 642.0, 645.0, 663.0, 670.0, 693.0, 691.0, 713.0, 717.0, 726.0, 746.0, 757.0, 760.0, 770.0, 773.0, 718.0, 350.0, 518.0, 596.0, 761.0, 881.0, 601.0, 502.0, 1956.0, 2256.0, 2161.0, 2427.0, 2303.0, 2354.0, 2237.0, 2380.0, 2277.0, 2607.0, 2333.0, 2495.0, 2398.0, 2500.0, 2335.0, 2601.0, 2162.0, 2291.0, 2310.0, 2038.0, 1953.0, 2240.0, 2128.0, 2193.0, 2413.0, 2076.0, 2137.0, 2056.0, 2079.0, 2102.0, 2176.0, 2257.0, 2308.0, 2186.0, 2373.0, 1977.0, 2220.0, 2068.0, 2255.0, 2468.0, 2418.0, 2339.0, 2523.0, 2426.0, 2068.0, 2582.0, 2552.0, 2046.0, 2315.0, 2071.0, 1837.0, 2112.0, 1932.0, 1998.0, 2242.0, 2238.0, 1918.0, 2386.0, 2268.0, 2205.0, 2669.0, 6168.0, 3596.0, 1233.0, 2445.0, 1960.0, 2585.0, 2252.0, 2467.0, 2123.0, 1974.0, 2581.0, 2116.0, 2236.0, 2021.0, 2437.0, 2523.0, 2539.0, 2389.0, 2437.0, 2696.0, 2518.0, 2600.0, 2657.0, 2688.0, 2849.0, 2800.0, 2703.0, 3007.0, 2675.0, 3110.0, 2842.0, 2470.0, 2881.0, 2794.0, 2696.0, 2950.0, 3082.0, 2760.0, 2775.0, 3091.0, 2900.0, 2963.0, 2726.0, 2919.0, 2405.0, 2818.0, 2520.0, 2563.0, 2685.0, 2541.0, 2843.0, 2985.0, 2254.0, 2848.0, 2666.0, 2474.0, 2421.0, 2423.0, 2545.0, 2527.0, 2527.0, 2635.0, 2297.0, 2142.0, 2236.0, 2537.0, 2391.0, 2181.0, 2071.0, 2232.0, 2384.0, 2285.0, 2202.0, 1964.0, 2292.0, 2033.0, 2179.0, 2298.0, 2433.0, 2011.0, 1932.0, 2299.0, 2350.0, 2242.0, 1962.0, 1952.0, 1850.0, 1742.0, 1684.0, 1581.0, 1526.0, 1471.0, 1325.0, 1186.0, 1107.0, 1025.0, 916.0, 764.0, 707.0, 650.0, 600.0, 572.0, 495.0, 444.0, 399.0, 258.0, 268.0, 307.0, 327.0, 83.0, 221.0, 204.0, 184.0, 155.0, 139.0, 119.0, 196.0, 487.0, 538.0, 41.0, 91.0, 104.0, 112.0, 310.0, 294.0, 312.0, 319.0, 303.0, 722.0]}___END___)";
    char buffer[8192] = {};

    TCPServer server(10000);
    server.acceptConnection();
    
    while(true)
    {
        //std::cout<<"test"<<std::endl;
        //server.receiveData(buffer, sizeof(buffer));
        server.sendData(data); 
        std::cout << data << std::endl;
    }

    return 0;
}