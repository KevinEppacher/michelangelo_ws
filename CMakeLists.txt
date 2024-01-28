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

# Add your executables
add_executable(main src/main.cpp src/Robot.cpp)
add_executable(server src/server.cpp)

# Link nlohmann/json to your targets
target_link_libraries(main nlohmann_json::nlohmann_json)
target_link_libraries(server nlohmann_json::nlohmann_json)

# Include Eigen for your targets
target_include_directories(main PRIVATE ${eigen_SOURCE_DIR})
target_include_directories(server PRIVATE ${eigen_SOURCE_DIR})




add_executable(PCA src/pca.cpp)
target_include_directories(PCA PRIVATE ${eigen_SOURCE_DIR})

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

# populate matplotlib repository
include(FetchContent)
FetchContent_Declare(
    matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

# link python and numpy
target_link_libraries(PCA PRIVATE ${PYTHON_LIBRARIES} Python3::NumPy)

