//
// Created by ub1404 on 17. 2. 18.
//

#ifndef PLAYGROUND_PACKETJOINTTARGETPOS_H
#define PLAYGROUND_PACKETJOINTTARGETPOS_H
#include "SerializablePacket.h"

class PacketJointTargetPos : public SerializablePacket{
public:
  PacketJointTargetPos()
      :SerializablePacket(PacketTypes::JointTargetPos){

  }

  uint32_t getLength(){
    return sizeof(q_des[0]) * 6;
  }

  bool serializeImpl(BytesBuffer& data_out) {
    for(size_t i=0;i<6;i++){
      LOAD_AND_RETURN_IF_FAILED(data_out, q_des[i]);
    }
    return true;
  }

  bool deserializeImpl(BytesBuffer& data_in) {
    for(size_t i=0;i<6;i++){
      UNLOAD_AND_RETURN_IF_FAILED(data_in, q_des[i]);
    }
    return true;
  }

  void setTargetPosition(const double *q){
    for(size_t i=0;i<6;i++){
      q_des[i] = q[i];
    }
  }

  bool isSafeCommand(const double* q_max, const double* q_min){
    for(size_t i=0;i<6;i++){
      if(q_des[i] > q_max[i]){
        return false;
      }
      if(q_des[i] < q_min[i]){
        return false;
      }
    }
    return true;
  }

  double q_des[6];
};
#endif //PLAYGROUND_PACKETJOINTTARGETPOS_H
