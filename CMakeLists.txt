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
  GIT_TAG v3.10.5
)
# Make json available for use
FetchContent_MakeAvailable(json)

# Declare Eigen dependency
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0  # Replace with the desired version
)
# Make Eigen available for use
FetchContent_MakeAvailable(eigen)

# Find SFML package
find_package(SFML 2.5 COMPONENTS system window graphics REQUIRED)

# Add your executables
add_executable(main src/main.cpp src/Robot.cpp)

# Link nlohmann/json to your main target
target_link_libraries(main nlohmann_json::nlohmann_json)

# Include Eigen for your main target
target_include_directories(main PRIVATE ${eigen_SOURCE_DIR})

# Link SFML with your main target
target_link_libraries(main sfml-system sfml-window sfml-graphics)
