//
// Created by ub1404 on 17. 2. 18.
//

#ifndef PLAYGROUND_PACKETURCONFIGCOMMANDS_H
#define PLAYGROUND_PACKETURCONFIGCOMMANDS_H

#include "SerializablePacket.h"
#include "PacketTypes.h"

class PacketURConfigCommand:public SerializablePacket{
public:
  enum ConfigCommandID{
    ConfigOpenInterface = 0,
    ConfigCloseInterface,
    ConfigPowerOn,
    ConfigPowerOff,
    ConfigSetReadyMode,
    ConfigSetRunningMode,
    ConfigSetFreedriveMode,
    ConfigSetSecurityStop,
    ConfigUnlockSecurityStop
  };
  PacketURConfigCommand():SerializablePacket(PacketTypes::ConfigCommands){
    clearAll();
  }

  uint32_t getLength(){
    return sizeof(_config_flags);
  }

  void clearAll(){
    _config_flags = 0;
  }

  void clearCommand(ConfigCommandID cmd_id){
    _config_flags = _config_flags&(~(1<<cmd_id));
  }

  bool isCommandSet(ConfigCommandID cmd_id){
    return (_config_flags&(1<<cmd_id)) > 0;
  }

  void setCommand(ConfigCommandID cmd_id){
    _config_flags = _config_flags|(1<<cmd_id);
  }

protected:
  bool serializeImpl(BytesBuffer& data_out){
    LOAD_AND_RETURN_IF_FAILED(data_out, _config_flags);
    return true;
  }
  bool deserializeImpl(BytesBuffer& data_in){
    UNLOAD_AND_RETURN_IF_FAILED(data_in, _config_flags);
    return true;
  }

  uint32_t _config_flags;
};
//
//#define DECLARE_CONFIG_PACKET(class_name, type_id) \
//class class_name : public SerializablePacket{\
//public:\
//  class_name():SerializablePacket(type_id){}\
//  uint32_t getLength(){return 0;}\
//protected:\
//  bool serializeImpl(BytesBuffer& data_out){return true;}\
//  bool deserializeImpl(BytesBuffer& data_in){return true;}\
//}
//
//DECLARE_CONFIG_PACKET(PacketConfigOpenInterface, PacketTypes::ConfigOpenInterface);
//DECLARE_CONFIG_PACKET(PacketConfigCloseInterface, PacketTypes::ConfigCloseInterface);
//DECLARE_CONFIG_PACKET(PacketConfigPowerOnRobot, PacketTypes::ConfigPowerOn);
//DECLARE_CONFIG_PACKET(PacketConfigPowerOffRobot, PacketTypes::ConfigPowerOff);
//DECLARE_CONFIG_PACKET(PacketConfigSetReadyMode, PacketTypes::ConfigSetReadyMode);
//DECLARE_CONFIG_PACKET(PacketConfigSetRunningMode, PacketTypes::ConfigSetRunningMode);
//DECLARE_CONFIG_PACKET(PacketConfigSetFreedriveMode, PacketTypes::ConfigSetFreedriveMode);
//DECLARE_CONFIG_PACKET(PacketConfigSetSecurityStop, PacketTypes::ConfigSetSecurityStop);
//DECLARE_CONFIG_PACKET(PacketConfigUnlockSecurityStop, PacketTypes::ConfigUnlockSecurityStop);



#endif //PLAYGROUND_PACKETURCONFIGCOMMAND_H
