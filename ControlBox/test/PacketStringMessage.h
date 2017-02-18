//
// Created by ub1404 on 17. 2. 18.
//

#ifndef PLAYGROUND_PACKETSTRINGMESSAGE_H
#define PLAYGROUND_PACKETSTRINGMESSAGE_H
class PacketStringMessage:public SerializablePacket{
public:
  PacketStringMessage():SerializablePacket(11){}
  std::string _msg;
  uint32_t getLength(){
    return sizeof(std::string::size_type) + _msg.size();
  }
protected:
  bool serializeImpl(BytesBuffer& data_out){
    if(!data_out.load(_msg.size()))
      return false;
    if(!data_out.load(_msg.c_str(), _msg.size()))
      return false;
    return true;
  }
  bool deserializeImpl(BytesBuffer& data_in){
    std::string::size_type sz_str;
    if(!data_in.unload(sz_str))
      return false;
    _msg.resize(sz_str);
    if(!data_in.unload(&_msg[0], sz_str))
      return false;
    return true;
  }
};
#endif //PLAYGROUND_PACKETSTRINGMESSAGE_H
