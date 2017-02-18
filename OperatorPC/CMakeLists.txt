cmake_minimum_required(VERSION 2.8.4)
project(ur5_teleoperation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(orocos_kdl REQUIRED)
set(OMEGA6_INCLUDE_DIR "$ENV{OMEGA6HOME}/include")
set(OMEGA6_LIBRARY_DIR "$ENV{OMEGA6HOME}/lib/release/lin-x86_64-gcc")
set(OMEGA6_LIBRARIES "-ldrd -ldhd -lpthread -lusb-1.0 -lrt -ldl -lGL -lGLU -lglut")

include_directories(
        ${orocos_kdl_INCLUDE_DIRS}
        ${OMEGA6_INCLUDE_DIR}
        ../ControlBox/src/
)

set(PacketSource
        ../ControlBox/src/PacketManager.cpp
        ../ControlBox/src/PacketURState.cpp
        ../ControlBox/src/SerializablePacket.cpp
        )
set(PacketHeader
        ../ControlBox/src/PacketManager.h
        ../ControlBox/src/PacketTypes.h
        ../ControlBox/src/PacketURState.h
        ../ControlBox/src/URState.h)

link_directories(${OMEGA6_LIBRARY_DIR})

add_library(teleoperation_master
        teleop/omega6/Omega6DeltaPosCommander.cpp
        teleop/omega6/Omega6Interface.cpp
        teleop/omega6/Omega6VelocityCommander.cpp)
target_link_libraries(teleoperation_master
        ${OMEGA6_LIBRARIES})

add_executable(main
        main.cpp
        kinematics/RobotKinematics.cpp
        )
target_link_libraries(
        main
        teleoperation_master
        ${OMEGA6_LIBRARIES}
        ${orocos_kdl_LIBRARIES}
)

add_executable(comm_test comm_test.cpp ${PacketSource})
