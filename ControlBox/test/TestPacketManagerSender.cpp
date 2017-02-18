//
// Created by ub1404 on 17. 2. 18.
//
#include "PacketManager.h"
#include "PacketURState.h"
#include "PacketStringMessage.h"
#include "PacketQuit.h"

#include "TCPServer.h"
#include <iostream>
#include <sstream>

int main(int argc, char** argv){
  URState robot_state;
  robot_state.sequence = 0;

  PacketManager packet_manager;
  PacketURState state_packet(&robot_state);
  PacketStringMessage string_packet;
  PacketQuit quit_command;
  packet_manager.registerPacket(&quit_command);

  TCPServer server;
  if(server.initialize(50001) == -1){
    std::cerr<<"Error initializing server."<<std::endl;
    return -1;
  }

  while(true){
    if(!server.isConnected()){
      std::cerr<<"Waiting for connection..."<<std::endl;
      if(server.waitForConnection(1000)){
        std::cout<<"Connected"<<std::endl;
      }
    }else{
      std::cout<<"processing..."<<std::endl;
      if(server.isReadyToReceive(100)) {
        packet_manager.processIncomingData(&server);
      }
      std::cout<<"sending state packet"<<std::endl;
      PacketManager::sendPacket(&server, &state_packet);
      if(robot_state.sequence % 4) {
        std::ostringstream oss;
        oss << "Tick count = " << robot_state.sequence;
        string_packet._msg = oss.str();
        std::cout<<"sending string packet"<<std::endl;
        PacketManager::sendPacket(&server, &string_packet);
      }
      if(quit_command.isUpdated()){
        std::cout<<"Quit command received."<<std::endl;
        quit_command.clearUpdated();
        break;
      }
    }
    robot_state.sequence++;
    ::usleep(2000000);
  }
  return 0;
}