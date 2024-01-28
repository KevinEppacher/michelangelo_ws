#include <SFML/Graphics.hpp>
#include <cmath>
#include <deque> // Für die Verwendung einer Warteschlange (Queue)
#include "Robot.h"


int main(int argc, char **argv)
{
    Robot::MobileRobot turtle(argv[1]);
    Robot::Visualizer visualizer;

    while (true)
    {
        turtle.run();
        visualizer.run();
    }

    return 0;
}












































/*


// Funktion zur Konvertierung von Polarkoordinaten in kartesische Koordinaten
sf::Vector2f polarToCartesian(float radius, float angleDegrees) {
    // Winkel von Grad in Bogenmaß umrechnen
    float angleRadians = angleDegrees * (M_PI / 180.0f);

    // Berechnen Sie die x- und y-Koordinaten
    float x = radius * std::cos(angleRadians);
    float y = radius * std::sin(angleRadians);

    return sf::Vector2f(x, y);
}

int main() {
    // Fenstergröße festlegen
    int screenWidth = 1200;
    int screenHeight = 1000;

    // Erstellen Sie ein SFML-Fenster
    sf::RenderWindow window(sf::VideoMode(screenWidth, screenHeight), "Punkte in Polar-Koordinaten");

    // Erstellen Sie ein SFML-Rechteck für den Hintergrund (schwarz)
    sf::RectangleShape background(sf::Vector2f(screenWidth, screenHeight));
    background.setFillColor(sf::Color::Black);

    // Berechnen Sie die Mitte des Bildschirms
    float centerX = static_cast<float>(screenWidth) / 2.0f;
    float centerY = static_cast<float>(screenHeight) / 2.0f;

    // Erstellen Sie ein SFML-Linienobjekt (weiß) für die Koordinatenachsen
    sf::VertexArray axes(sf::Lines, 4);
    axes[0].position = sf::Vector2f(0, centerY);
    axes[1].position = sf::Vector2f(screenWidth, centerY);
    axes[2].position = sf::Vector2f(centerX, 0);
    axes[3].position = sf::Vector2f(centerX, screenHeight);
    for (int i = 0; i < 4; ++i) {
        axes[i].color = sf::Color::White;
    }

    // Anfangsposition und Inkrement für den Winkel der Punkte
    float angle = 0.0f;
    float angleIncrement = 1.0f;

    // Erstellen Sie eine Warteschlange (Queue) für die Polarpunkte
    std::deque<std::pair<float, float>> polarPointQueue;

    // Maximale Anzahl von Punkten im Puffer
    const int maxBufferSize = 10;

    // Punkte in Polar-Koordinaten hinzufügen
    for (int i = 0; i < maxBufferSize; ++i) {
        polarPointQueue.push_back({100.0f + i, angle});
        angle += angleIncrement *200;
    }

    // Punkte in kartesischen Koordinaten umrechnen und darstellen
    sf::CircleShape pointShape(3);
    pointShape.setFillColor(sf::Color::Red);

    while (window.isOpen()) {
        // Ereignisse behandeln
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Zeichnen Sie den schwarzen Hintergrund auf den Bildschirm
        window.clear();

        // Zeichnen Sie die Koordinatenachsen
        window.draw(background);
        window.draw(axes);

        // Inkrementieren Sie den Winkel für die neuen Punkte und fügen Sie sie der Warteschlange hinzu
        angle += angleIncrement;
        polarPointQueue.push_back({100.0f + maxBufferSize, angle});

        // Entfernen Sie den ältesten Punkt, wenn die maximale Puffergröße überschritten ist
        if (polarPointQueue.size() > maxBufferSize) {
            polarPointQueue.pop_front();
        }

        // Zeichnen Sie die Punkte in Polar-Koordinaten
        for (const auto& polarPoint : polarPointQueue) {
            sf::Vector2f cartesianPoint = polarToCartesian(polarPoint.first, polarPoint.second);
            cartesianPoint += sf::Vector2f(centerX, centerY);
            pointShape.setPosition(cartesianPoint);
            window.draw(pointShape);
        }
        sf::sleep(sf::milliseconds(10));
        window.display();
    }

    return 0;
}

*/