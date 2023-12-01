#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
//#include "matplotlibcpp.h"

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

    class TCP_Client
    {
    public:
        TCP_Client();
        ~TCP_Client();

    private:


    };

    
}














/*
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

// C++-Bindungen für Matplotlib
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

namespace Robot
{
    class MobileRobot
    {
    public:
        MobileRobot() : scanMsg(0.0), odomMsg(0.0), cmdVelMsg(0.0) {}
        ~MobileRobot() {}

        double getScanMsg() const { return scanMsg; }
        void setScanMsg(double value) { scanMsg = value; }

        // Hier könnten ähnliche Getter und Setter für odomMsg und cmdVelMsg hinzugefügt werden.

    private:
        double scanMsg;
        double odomMsg;
        double cmdVelMsg;
    };

    class Socket : public MobileRobot
    {
    public:
        Socket() {}
        ~Socket() {}

        // Simuliere den Empfang von Scan-Daten
        void getScanData()
        {
            // Hier könnte der eigentliche Code stehen, um Daten über einen Socket zu empfangen.
            // In diesem Beispiel wird einfach nur eine simuliert.
            setScanMsg(static_cast<double>(rand() % 100) / 10.0);
        }

        // Hier könnten ähnliche Funktionen für odomData und cmdVelData hinzugefügt werden.

        // Funktion zum kontinuierlichen Empfangen und Plotten von Scan-Daten
        void plotContinuousScanData()
        {
            while (true)
            {
                // Daten empfangen
                getScanData();

                // Daten in Matplotlib plotten
                plotScanData();

                // Eine kurze Pause einfügen, um die Aktualisierungsgeschwindigkeit zu steuern
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }

    private:
        // Funktion zum Plotten von Scan-Daten mit Matplotlib
        void plotScanData()
        {
            // Matplotlib-Plotten
            plt::clf();  // Aktuelles Diagramm löschen, um es zu aktualisieren
            plt::plot({getScanMsg()}, "r-");  // Rote Linie für Scan-Daten

            // Diagramm anzeigen
            plt::pause(0.01);
            plt::draw();
        }
    };
}

int main()
{
    // Erstelle eine Instanz der Socket-Klasse
    Robot::Socket robotSocket;

    // Starte den Thread für das kontinuierliche Empfangen und Plotten von Scan-Daten
    std::thread scanThread(&Robot::Socket::plotContinuousScanData, &robotSocket);

    // Warte darauf, dass der Benutzer das Programm beendet
    std::cout << "Drücken Sie Enter, um das Programm zu beenden." << std::endl;
    std::cin.get();

    // Beende den Thread
    scanThread.join();

    return 0;
}

*/