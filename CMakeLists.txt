cmake_minimum_required(VERSION 2.8)
project( PoweLaneFinding )

#set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -pthread -lstdc++ -lm")

include_directories(include)
file(GLOB SOURCES "src/*.cpp")

find_package( OpenCV REQUIRED )
add_executable( PowerLaneFinding ${SOURCES} )

target_link_libraries( PowerLaneFinding ${OpenCV_LIBS} )
