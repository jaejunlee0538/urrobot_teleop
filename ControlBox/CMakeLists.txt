cmake_minimum_required(VERSION 2.8.4)
project(playground)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(ur-c-api-1.8 src)
add_library(SocketLibs
        src/TCPServer.cpp
        src/TCPClient.cpp
        src/TCPSocket.cpp
        src/SocketInterface.cpp
        src/TCPServer.h
        src/TCPClient.h
        src/TCPSocket.h
        src/SocketInterface.h)

add_library(PacketLibs
        src/PacketManager.cpp
        src/PacketURState.cpp
        src/SerializablePacket.cpp
        src/BytesBuffer.cpp
        src/PacketManager.h
        src/PacketURState.h
        src/SerializablePacket.h
        src/PacketTypes.h
        src/BytesBuffer.h)
############################################################################################
set(HEADERS
        src/Button.h
        src/URState.h
        src/PreciseClock.h
        src/VelocityPControl.h
        src/PacketURConfigCommands.h
        src/PacketJointTargetPos.h)

add_executable(run_joint_p_control
        src/main_JointPControl.cpp
        src/LoopInterface.cpp
        src/URRobotInterface.cpp
        src/PreciseClock.cpp
        src/URRobotJointPControl.cpp
        ${HEADERS}
        )
#set_target_properties(run_joint_p_control PROPERTIES COMPILE_DEFINITIONS "REAL_ROBOT")
#target_compile_definitions(run_joint_p_control PRIVATE FAKE_PRINT_VERBOSE)
target_compile_definitions(run_joint_p_control PRIVATE FAKE_LOOP_PERIOD=500)
target_link_libraries(run_joint_p_control pthread SocketLibs PacketLibs)

############################################################################################
add_executable(test_tcp_server
        test/TestTCPServer.cpp)
target_link_libraries(test_tcp_server SocketLibs PacketLibs)

add_executable(test_tcp_client test/TestTCPClient.cpp)
target_link_libraries(test_tcp_client SocketLibs PacketLibs)

add_executable(test_packet_manager_sender test/TestPacketManagerSender.cpp)
target_link_libraries(test_packet_manager_sender SocketLibs PacketLibs)

add_executable(test_packet_manager_receiver test/TestPacketManagerReceiver.cpp)
target_link_libraries(test_packet_manager_receiver SocketLibs PacketLibs)

add_executable(test_bytes_buffer test/TestBytesBuffer.cpp)
target_link_libraries(test_bytes_buffer SocketLibs PacketLibs)

add_executable(test_joint_p_control test/TestJointPControl.cpp src/ConioLinux.cpp)
target_link_libraries(test_joint_p_control SocketLibs PacketLibs)