cmake_minimum_required(VERSION 3.0.2)
project(michelangelo_ws)

# Ensure C++ version is set to C++11 or higher
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include FetchContent module
include(FetchContent)

# Declare dependency on nlohmann/json
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.10.5 # specify the version you need
)

# Make json available for use
FetchContent_MakeAvailable(json)

# Add your executables
add_executable(main src/main.cpp src/Robot.cpp)
add_executable(server src/server.cpp)

# Link nlohmann/json to your targets as necessary
target_link_libraries(main nlohmann_json::nlohmann_json)
target_link_libraries(server nlohmann_json::nlohmann_json)
