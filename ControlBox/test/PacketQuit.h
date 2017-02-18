//
// Created by ub1404 on 17. 2. 18.
//

#ifndef PLAYGROUND_PACKETQUIT_H
#define PLAYGROUND_PACKETQUIT_H
class PacketQuit:public SerializablePacket{
public:
  PacketQuit():SerializablePacket(10){}
  uint32_t getLength(){
    return 0;
  }
protected:
  bool serializeImpl(BytesBuffer& data_out){
    return true;
  }
  bool deserializeImpl(BytesBuffer& data_in){
    return true;
  }
};
#endif //PLAYGROUND_PACKETQUIT_H
