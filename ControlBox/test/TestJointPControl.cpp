//
// Created by ub1404 on 17. 2. 18.
//
#include "PacketManager.h"
#include "PacketURState.h"
#include "TCPClient.h"
#include <iostream>
#include <signal.h>
#include <PacketJointTargetPos.h>
#include "ConioLinux.h"
#include "PacketURConfigCommands.h"

bool running = true;
void signal_handler(int sig){
  running = false;
}

void keyBoardInput();

uint64_t state_packet_count = 0;
PacketURState state_packet;
PacketURConfigCommand config_command;
PacketJointTargetPos joint_target_command;

int main(int argc, char** argv){
  PacketManager packet_manager;

  URState robot_state;
  robot_state.sequence = 0;
  state_packet.setURState(&robot_state);
  packet_manager.registerPacket(&state_packet);
  config_command.clearAll();


  TCPClient tcp_client;
  if(tcp_client.initialize(50000) == -1){
    std::cerr<<"Error initializing server."<<std::endl;
    return -1;
  }

  if(tcp_client.connect("127.0.0.1") == -1){
    std::cerr<<"Cannot connect to server"<<std::endl;
    return -1;
  }
  signal(SIGINT, signal_handler);
  while(tcp_client.isConnected() && running){
    keyBoardInput();

    if(config_command.isUpdated()){
      //send configuration packet command
      PacketManager::sendPacket(&tcp_client, &config_command);
      config_command.clearAll();
      config_command.clearUpdated();
    }

    if(joint_target_command.isUpdated()){
      PacketManager::sendPacket(&tcp_client, &joint_target_command);
      joint_target_command.clearUpdated();
    }

    //receive packet from UR Robot
    if(tcp_client.isReadyToReceive(5)) {
      //parsing packet received
      packet_manager.processIncomingData(&tcp_client);

      if (state_packet.isUpdated()) {
        //if state packet is received
        std::cout << "State packet received : seq = ";
        std::cout << robot_state.sequence << std::endl;
        if(state_packet_count == 0){
          joint_target_command.setTargetPosition(&robot_state.q_act[0]);
        }
        state_packet_count++;
        state_packet.clearUpdated();
      }
    }
  }
  std::cout<<"closing communication..."<<std::endl;
  tcp_client.close();
  std::cout<<"Bye Bye~"<<std::endl;
  return 0;
}

void keyBoardInput(){
  if(kbhit()){
    int key =  getch();
    switch(key){
      case 'o':
        config_command.setCommand(PacketURConfigCommand::ConfigOpenInterface);
        config_command.setUpdated();
        break;
      case 'c':
        config_command.setCommand(PacketURConfigCommand::ConfigCloseInterface);
        config_command.setUpdated();
        break;
      case '+':
      if(state_packet_count > 0){
        joint_target_command.q_des[1] += 0.05;
        joint_target_command.setUpdated();
      }
        break;
      case '-':
        if(state_packet_count > 0){
          joint_target_command.q_des[1] -= 0.05;
          joint_target_command.setUpdated();
        }
        break;
    }
  }
}