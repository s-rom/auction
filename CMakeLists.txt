cmake_minimum_required(VERSION 2.8.3)
project(auction)


find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    actionlib
)
add_compile_options(-std=c++11)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES RcSocket
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${boots_INCLUDE_DIRS}
)


add_executable(auction src/cpp_sandbox.cpp)
add_library(RcSocket src/RcSocket.cpp)
add_library(MessageSystem src/MessageSystem.cpp)

target_link_libraries(auction ${catkin_LIBRARIES})
target_link_libraries(auction ${boost_LIBRARIES})
target_link_libraries(auction RcSocket)
target_link_libraries(auction MessageSystem)


