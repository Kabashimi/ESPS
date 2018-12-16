cmake_minimum_required(VERSION 2.8)
project( DisplayCamera )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( DisplayCamera DisplayCamera.cpp)

include_directories(/usr/include)
include_directories(/usr/include/mysql)

target_link_libraries( DisplayCamera ${OpenCV_LIBS})
target_link_libraries( DisplayCamera pigpio)
target_link_libraries( DisplayCamera mysqlclient)