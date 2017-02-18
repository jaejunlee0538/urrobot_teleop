//
// Created by ub1404 on 17. 2. 17.
//

#include "PacketManager.h"
#include <iostream>

uint8_t STX = 0xcc;

bool PacketManager::registerPacket(SerializablePacket* packet) {
  PacketsMap::iterator iter = _packets.find(packet->getTypeID());
  if(iter != _packets.end()){
    assert(false);
    return false; //packet with same type is already registered
  }
  _packets[packet->getTypeID()] = packet;
  return true;
}

void PacketManager::processIncomingData(SocketInterface* socket) {
  //parse length field
  packet_length_t pack_len;
  int len_rcv = socket->recvBytes(_raw_buffer ,PacketManager::lengthLength());
  if(len_rcv == 0 || len_rcv != PacketManager::lengthLength()){
//    std::cerr<<"len_rcv=0 || len_rcv!=sizeof(packet_length_t) - "<<__PRETTY_FUNCTION__<<std::endl;
    return;
  }
  BytesBuffer buffer;
  buffer.load(_raw_buffer, PacketManager::lengthLength());

  if(!buffer.unload(pack_len)){
    std::cerr<<"Failed to parse packet length"<<std::endl;
    return;
  }

  if(pack_len > MAX_PACKET_LENGTH){
    std::cerr<<"MAX_PACKET_LENGTH error"<<std::endl;
    //TODO : error message.
    return;
  }

  //read length bytes
  len_rcv = socket->recvBytes(_raw_buffer, pack_len);
  if(len_rcv != pack_len){
    std::cerr<<"len_rcv != pack_len"<<std::endl;
    return;
  }

  //try deserialize
  buffer.init(_raw_buffer, pack_len);
  packet_type_t pack_type = 5555;
  if(!buffer.unload(pack_type)){
    std::cerr<<"Failed to parse packet type"<<std::endl;
    return;
  }
  SerializablePacket * packet = getPacket(pack_type);
  if(packet == NULL){
    //TODO : message -> Unregistered packet type
    std::cerr<<"Unregistered packet type["<<pack_type<<"]"<<std::endl;
    return;
  }
//  std::cerr<<"Packet Type - "<<packet->getTypeID()<<std::endl;
  if(!packet->deserialize(buffer)){
    std::cerr<<"Deserialize failed"<<std::endl;
  }
}

SerializablePacket *PacketManager::getPacket(const packet_type_t& pack_type) {
  PacketsMap::iterator iter = _packets.find(pack_type);
  if(iter == _packets.end()){
    return NULL;
  }
  return iter->second;
}

PacketManager::~PacketManager() {

}

PacketManager::PacketManager() {

}

bool PacketManager::sendPacket(SocketInterface* socket,
                               SerializablePacket* packet) {
  BytesBuffer raw_packet;

  packet_length_t len = packet->getLength() + sizeof(SerializablePacket::type_id_t);
//  if(!raw_packet.load(STX)){
//    return false;
//  }
  if(!raw_packet.load(len)){
    return false;
  }
  if(!raw_packet.load(packet->getTypeID())){
    return false;
  }
  if(!packet->serialize(raw_packet)){
    return false;
  }
  char* buffer = new char[raw_packet.size()];
  raw_packet.copyToArray(buffer);
  socket->sendBytes(buffer, raw_packet.size());
  delete[] buffer;
  return true;
}