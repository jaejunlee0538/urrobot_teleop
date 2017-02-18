//
// Created by ub1404 on 17. 2. 17.
//

#ifndef PLAYGROUND_SERIALIZABLE_H
#define PLAYGROUND_SERIALIZABLE_H
#include <stdint.h>
#include <string>
#include "BytesBuffer.h"
#include <assert.h>
#include <iostream>
class SerializablePacket {
public:
  typedef int32_t type_id_t;
  SerializablePacket(type_id_t type_id):_type_id(type_id){
    clearUpdated();
  }

  virtual ~SerializablePacket(){

  }

  bool clearUpdated(){
    _updated = false;
  }

  void setUpdated(){
    _updated = true;
  }

  bool isUpdated() const{
    return _updated;
  }

  bool serialize(BytesBuffer& data_out){
    return serializeImpl(data_out);
  }

  /*
   * make sure raw_packet pass 'isDeserializable' test.
   */
  bool deserialize(BytesBuffer& data_in){
    clearUpdated(); //clear updated flag.
    /*
     * With these line activated, cannot handle variable length packet.
     */
//    if(data_in.size() != getLength()){
//      std::cerr<<"buffer length does not match with packet length["<<data_in.size()<<"vs"<<getLength()<<"]"<<std::endl;
//      return false;
//    }
    if(!deserializeImpl(data_in)){
        return false;
    }
    setUpdated();
    return true;
  }

  inline const type_id_t& getTypeID() const{
    return _type_id;
  }

  /*
   * return packet length
   */
  virtual uint32_t getLength() = 0;
protected:
  /*
   * implement loading process of user-defined fields
   */
  virtual bool serializeImpl(BytesBuffer& data_out) = 0;

  /*
   * implement unloading process of user-defined fields
   */
  virtual bool deserializeImpl(BytesBuffer& data_in) = 0;

  /*
   * a class that inherits SerializablePacket must set _type_id member with a unique
   * type identifier within the whole packets.
   */
  type_id_t _type_id;

private:
  bool _updated;
};


#endif //PLAYGROUND_SERIALIZABLE_H
