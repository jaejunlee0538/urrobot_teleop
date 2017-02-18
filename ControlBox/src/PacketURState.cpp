//
// Created by ub1404 on 17. 2. 17.
//

#include "PacketURState.h"
#include <iostream>
uint8_t PacketURState::getFlags() const{
  uint8_t flags = 0x00;
  if(_ur_state->is_power_on_robot){
    BytesBufferHelper::SET_BIT(flags, 0);
  }
  if(_ur_state->is_security_stopped){
    BytesBufferHelper::SET_BIT(flags, 1);
  }
  if(_ur_state->is_emergency_stopped){
    BytesBufferHelper::SET_BIT(flags, 2);
  }
  if(_ur_state->is_extra_button_pressed){
    BytesBufferHelper::SET_BIT(flags, 3);
  }
  if(_ur_state->is_power_button_pressed){
    BytesBufferHelper::SET_BIT(flags, 4);
  }
  if(_ur_state->is_safety_signal_such_that_we_should_stop){
    BytesBufferHelper::SET_BIT(flags, 5);
  }
  if(_ur_state->is_robot_connected){
    BytesBufferHelper::SET_BIT(flags, 6);
  }
  return flags;
}

void PacketURState::parseFlags(const uint8_t& flags){
  _ur_state->is_power_on_robot = BytesBufferHelper::IS_BIT_SET(flags, 0);
  _ur_state->is_security_stopped = BytesBufferHelper::IS_BIT_SET(flags, 1);
  _ur_state->is_emergency_stopped = BytesBufferHelper::IS_BIT_SET(flags, 2);
  _ur_state->is_extra_button_pressed = BytesBufferHelper::IS_BIT_SET(flags, 3);
  _ur_state->is_power_button_pressed = BytesBufferHelper::IS_BIT_SET(flags, 4);
  _ur_state->is_safety_signal_such_that_we_should_stop= BytesBufferHelper::IS_BIT_SET(flags, 5);
  _ur_state->is_robot_connected = BytesBufferHelper::IS_BIT_SET(flags, 6);
}

bool PacketURState::serializeImpl(BytesBuffer& data_out){
  if(_ur_state == NULL)
    return false;
  LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->sequence);  //8
  LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->robot_mode_id); // 1
  for(size_t i=0;i<6;i++) {
    LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->joint_mode_id[i]);//1*6
  }
  for(size_t i=0;i<6;i++) {
    LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->q_act[i]);//8*6
  }
  for(size_t i=0;i<6;i++) {
    LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->qd_act[i]);//8*6
  }
  for(size_t i=0;i<6;i++) {
    LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->i_act[i]);//8*6
  }
  for(size_t i=0;i<6;i++) {
    LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->q_des[i]);//8*6
  }
  for(size_t i=0;i<6;i++) {
    LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->qd_des[i]);//8*6
  }
  for(size_t i=0;i<6;i++) {
    LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->qdd_des[i]);//8*6
  }
  for(size_t i=0;i<6;i++) {
    LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->i_des[i]);//8*6
  }
  for(size_t i=0;i<6;i++) {
    LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->moment_des[i]);//8*6
  }
  LOAD_AND_RETURN_IF_FAILED(data_out,getFlags());//1
  LOAD_AND_RETURN_IF_FAILED(data_out,_ur_state->digital_inputs);//1
  //8+1+6+(8+8+8+8+8+8+8+8)*6+1+1
  //8+1+6+64*6+1+1
  return true;
}
bool PacketURState::deserializeImpl(BytesBuffer& data_in){
  assert(_ur_state!=NULL);
  if(data_in.size() != getLength()) {
    std::cerr<<"Not enough buffer data"<<std::endl;
    return false;
  }

  UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->sequence);
  UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->robot_mode_id);
  for(size_t i=0;i<6;i++) {
    UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->joint_mode_id[i]);
  }
  for(size_t i=0;i<6;i++) {
    UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->q_act[i]);
  }
  for(size_t i=0;i<6;i++) {
    UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->qd_act[i]);
  }
  for(size_t i=0;i<6;i++) {
    UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->i_act[i]);
  }
  for(size_t i=0;i<6;i++) {
    UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->q_des[i]);
  }
  for(size_t i=0;i<6;i++) {
    UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->qd_des[i]);
  }
  for(size_t i=0;i<6;i++) {
    UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->qdd_des[i]);
  }
  for(size_t i=0;i<6;i++) {
    UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->i_des[i]);
  }
  for(size_t i=0;i<6;i++) {
    UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->moment_des[i]);
  }
  uint8_t flags;
  UNLOAD_AND_RETURN_IF_FAILED(data_in,flags);
  parseFlags(flags);

  UNLOAD_AND_RETURN_IF_FAILED(data_in,_ur_state->digital_inputs);
  return true;
}