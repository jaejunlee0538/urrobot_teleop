//
// Created by ub1404 on 17. 2. 17.
//

#ifndef PLAYGROUND_MESSAGEMANAGER_H
#define PLAYGROUND_MESSAGEMANAGER_H
#include <assert.h>
#include <map>
#include <algorithm>
#include <string.h>
#include "SerializablePacket.h"
#include "BytesBuffer.h"
#include "SocketInterface.h"

#define MAX_PACKET_LENGTH 4096
class PacketManager {
public:
  typedef uint32_t packet_length_t;
  typedef SerializablePacket::type_id_t packet_type_t;

  PacketManager();

  ~PacketManager();

  bool registerPacket(SerializablePacket* packet);

  void processIncomingData(SocketInterface* socket);

  static bool sendPacket(SocketInterface* socket, SerializablePacket* packet);

  size_t lengthLength()const{
    return sizeof(packet_length_t);
  }
private:
  SerializablePacket * getPacket(const packet_type_t& pack_type);

protected:
  char _raw_buffer[MAX_PACKET_LENGTH];

  //map a packet type into a SerializablePacket instance
  typedef std::map<packet_type_t, SerializablePacket*> PacketsMap;
  PacketsMap _packets;
};


#endif //PLAYGROUND_MESSAGEMANAGER_H
