//
// Created by ub1404 on 17. 2. 18.
//
#include "PacketManager.h"
#include "PacketURState.h"
#include "PacketStringMessage.h"
#include "PacketQuit.h"

#include "TCPClient.h"
#include <iostream>
#include <signal.h>

bool running = true;
void signal_handler(int sig){
  running = false;
}
int main(int argc, char** argv){
  PacketManager packet_manager;
  PacketURState state_packet;
  PacketStringMessage string_packet;

  URState robot_state;
  robot_state.sequence = 0;
  state_packet.setURState(&robot_state);

  packet_manager.registerPacket(&state_packet);
  packet_manager.registerPacket(&string_packet);

  TCPClient client;
  if(client.initialize(50001) == -1){
    std::cerr<<"Error initializing server."<<std::endl;
    return -1;
  }

  if(client.connect("127.0.0.1") == -1){
    std::cerr<<"Cannot connect to server"<<std::endl;
    return -1;
  }
  signal(SIGINT, signal_handler);
  while(client.isConnected() && running){
    if(client.isReadyToReceive(100)) {
      packet_manager.processIncomingData(&client);

      if (string_packet.isUpdated()) {
        std::cerr << "String packet received:" << std::endl;
        std::cout << "\t" << string_packet._msg << std::endl;
        string_packet.clearUpdated();
      }

      if (state_packet.isUpdated()) {
        std::cout << "State packet received" << std::endl;
        std::cout << "\tcount = " << robot_state.sequence << std::endl;
        state_packet.clearUpdated();
      }
      ::usleep(500000);
    }
  }
  std::cout<<"closing communication..."<<std::endl;
  std::cout<<"Sending quit command"<<std::endl;

  PacketQuit quit_command;
  PacketManager::sendPacket(&client, &quit_command);
  ::sleep(2);
  client.close();
  std::cout<<"Bye Bye~"<<std::endl;
  return 0;
}