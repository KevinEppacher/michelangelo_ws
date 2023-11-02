#!/bin/bash

# Stelle sicher, dass wir im richtigen Verzeichnis sind (dem Verzeichnis, in dem dieses Skript liegt)
cd "$(dirname "$0")"

# Wechsle zum "src"-Verzeichnis
cd ../src

# Kompiliere den Code
g++ -c main.cpp
g++ -o main main.o Robot.o

gcc TCPEchoClient.c DieWithError.c -o TCPEchoClient


# Führe die ausführbare Datei aus
./main

# optional: Aufräumen (lösche die erzeugten Objektdateien und die ausführbare Datei)
rm -f main main.o Robot.o