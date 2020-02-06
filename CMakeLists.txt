cmake_minimum_required(VERSION 2.8.3)
project(auction)


find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    actionlib
    Boost REQUIRED COMPONENTS system
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
    ${boost_INCLUDE_DIRS}
)

#---- Executables ----#
add_executable(auction src/cpp_sandbox.cpp)
add_executable(sender src/message_sender.cpp)
add_executable(monitor src/Monitor.cpp)

#---- Libraries ----#
add_library(RcSocket src/RcSocket.cpp)
add_library(MessageSystem src/MessageSystem.cpp)
add_library(RobotManager src/RobotManager.cpp)

#---- Sender ----#
target_link_libraries(sender MessageSystem)
target_link_libraries(sender RcSocket)

#---- RobotManager ----#
target_link_libraries(RobotManager RcSocket)
target_link_libraries(RobotManager MessageSystem)

#---- Auction ----#
target_link_libraries(auction ${catkin_LIBRARIES})
target_link_libraries(auction ${boost_LIBRARIES})
target_link_libraries(auction RcSocket)
target_link_libraries(auction MessageSystem)
target_link_libraries(auction RobotManager)

#---- Monitor ----#
target_link_libraries(monitor ${boost_LIBRARIES})
target_link_libraries(monitor RcSocket)
target_link_libraries(monitor MessageSystem)