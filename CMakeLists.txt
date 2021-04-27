cmake_minimum_required(VERSION 3.15)

project(CVProject4)

# Include headers
include_directories("/usr/local/include/")
include_directories("./include/")

# Create the executable
file(GLOB SRC "./src/*.cpp")
add_executable(project4 ./src/main.cpp ./src/OBJParser.cpp)
add_executable(detectRobustFeatures ./src/detectRobustFeatures.cpp)


find_package(OpenCV REQUIRED)
target_link_libraries(project4 ${OpenCV_LIBS})
target_link_libraries(detectRobustFeatures ${OpenCV_LIBS})


