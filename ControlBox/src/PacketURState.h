//
// Created by ub1404 on 17. 2. 17.
//

#ifndef PLAYGROUND_PACKETURSTATE_H
#define PLAYGROUND_PACKETURSTATE_H
#include "SerializablePacket.h"
#include "PacketTypes.h"
#include "URState.h"

class PacketURState :public SerializablePacket{
public:
  PacketURState(URState* ur_state=NULL)
      :SerializablePacket(PacketTypes::URState),_ur_state(ur_state)
  {

  }

  void setURState(URState* ur_state){
    this->_ur_state = ur_state;
  }

  uint32_t getLength(){
    return 401;
  }
private:
  uint8_t getFlags() const;
  void parseFlags(const uint8_t& flags);
protected:
  bool serializeImpl(BytesBuffer& data_out);
  bool deserializeImpl(BytesBuffer& data_in);
  URState *_ur_state;
};


#endif //PLAYGROUND_PACKETURSTATE_H
