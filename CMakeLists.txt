cmake_minimum_required(VERSION 3.0.2)
project(michelangelo_ws)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.10.5
)
FetchContent_MakeAvailable(json)

FetchContent_Declare(eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen)

FetchContent_Declare(matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_MakeAvailable(matplotlib)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
message(STATUS "Python3 include dirs: ${Python3_INCLUDE_DIRS}")

add_executable(main src/main.cpp src/Robot.cpp)
add_executable(server src/server.cpp)
add_executable(PCA src/pca.cpp)

target_include_directories(main PRIVATE ${eigen_SOURCE_DIR} ${matplotlib_SOURCE_DIR} ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_include_directories(server PRIVATE ${eigen_SOURCE_DIR} ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_include_directories(PCA PRIVATE ${eigen_SOURCE_DIR} ${matplotlib_SOURCE_DIR} ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

target_link_libraries(main PRIVATE nlohmann_json::nlohmann_json Python3::Python Python3::NumPy)
target_link_libraries(server PRIVATE nlohmann_json::nlohmann_json Python3::Python Python3::NumPy)
target_link_libraries(PCA PRIVATE nlohmann_json::nlohmann_json Python3::Python Python3::NumPy)
