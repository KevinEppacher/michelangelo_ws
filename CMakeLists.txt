cmake_minimum_required(VERSION 3.0.2)
project(michelangelo_ws)

# Stelle sicher, dass die C++-Version auf C++11 oder höher eingestellt ist
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(main src/main.cpp src/Robot.cpp)
