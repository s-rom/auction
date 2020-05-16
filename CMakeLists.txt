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
add_executable(robot_node src/RobotNode.cpp)
add_executable(monitor src/MonitorNode.cpp)
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
add_library(GoalManager src/GoalManager.cpp)
add_library(vfh_algorithm src/vfh_algorithm.cc)
add_library(vfh_node src/vfh_node.cpp)

#--- vfh_algorithm ---#
target_link_libraries(vfh_node ${catkin_LIBRARIES})
target_link_libraries(vfh_node vfh_algorithm)


#--- GoalManager ---#
target_link_libraries(GoalManager ${catkin_LIBRARIES})
target_link_libraries(GoalManager ${Boost_LIBRARIES})

#---- RobotManager ----#
target_link_libraries(RobotManager RcSocket)
target_link_libraries(RobotManager MessageSystem)
target_link_libraries(RobotManager GoalManager)


#---- robot_node -----#
target_link_libraries(robot_node ${catkin_LIBRARIES})
target_link_libraries(robot_node ${Boost_LIBRARIES})
target_link_libraries(robot_node RcSocket)
target_link_libraries(robot_node MessageSystem)
target_link_libraries(robot_node RobotManager)
target_link_libraries(robot_node GoalManager)
target_link_libraries(robot_node vfh_node)

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
target_link_libraries(goal_sender GoalManager)

#---- Tests ----#
target_link_libraries(auction_test ${catkin_LIBRARIES})
target_link_libraries(auction_test ${Boost_LIBRARIES})
#target_link_libraries(auction_test RcSocket)
target_link_libraries(auction_test MessageSystem)
target_link_libraries(auction_test RobotManager)
target_link_libraries(auction_test GoalManager)


