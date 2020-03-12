cmake_minimum_required(VERSION 2.8.3)
project(auction)

find_package(Boost REQUIRED COMPONENTS system)


find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    actionlib
)
add_compile_options(-std=c++11)
add_definitions(-DBOOST_LOG_DYN_LINK=1)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES RcSocket
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)


#---- Executables ----#
add_executable(auction src/Auction.cpp)
add_executable(monitor src/AuctionMonitor.cpp)
add_executable(auction_test src/MainTest.cpp)
add_executable(goal_sender src/GoalSender.cpp)

#---- CppCMS ----#
add_library(CppCMS STATIC IMPORTED GLOBAL)
set_target_properties(CppCMS PROPERTIES IMPORTED_LOCATION 
/usr/local/lib/libcppcms.so.1.2.1)

#---- Libraries ----#
add_library(RcSocket src/RcSocket.cpp)
add_library(MessageSystem src/MessageSystem.cpp)
add_library(RobotManager src/RobotManager.cpp)
add_library(WebService src/WebService.cpp)
add_library(Monitor src/Monitor.cpp)



#---- RobotManager ----#
target_link_libraries(RobotManager RcSocket)
target_link_libraries(RobotManager MessageSystem)

#---- Auction ----#
target_link_libraries(auction ${catkin_LIBRARIES})
target_link_libraries(auction ${Boost_LIBRARIES})
target_link_libraries(auction RcSocket)
target_link_libraries(auction MessageSystem)
target_link_libraries(auction RobotManager)

#---- Monitor ----#
target_link_libraries(Monitor ${catkin_LIBRARIES})
target_link_libraries(Monitor ${Boost_LIBRARIES})
target_link_libraries(Monitor RcSocket)
target_link_libraries(Monitor MessageSystem)

target_link_libraries(monitor WebService)
target_link_libraries(monitor ${Boost_LIBRARIES})
target_link_libraries(monitor Monitor)

#---- WebService ----#
target_link_libraries(WebService CppCMS)
target_link_libraries(WebService Monitor)

#---- GoalSender ----#
target_link_libraries(goal_sender ${catkin_LIBRARIES})
target_link_libraries(goal_sender ${Boost_LIBRARIES})


#---- Tests ----#
target_link_libraries(auction_test ${catkin_LIBRARIES})
target_link_libraries(auction_test ${Boost_LIBRARIES})
#target_link_libraries(auction_test RcSocket)
target_link_libraries(auction_test MessageSystem)
target_link_libraries(auction_test RobotManager)