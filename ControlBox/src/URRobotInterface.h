//
// Created by ub1404 on 15. 9. 17.
//

#ifndef UR_ROBOT_INTERFACE_H_
#define UR_ROBOT_INTERFACE_H_
#include <stdint.h>
#include "LoopInterface.h"
#ifdef REAL_ROBOT
#include <robotinterface.h>
#include <Configuration.h>
#include <microprocessor_commands.h>
#include <microprocessor_definitions.h>
#else
#include <fake/Configuration_fake.h>
    #include <fake/robotinterface_fake.h>
    #include <fake/microprocessor_commands_fake.h>
    #include <fake/microprocessor_definitions_fake.h>
#endif
#include "URState.h"
#include "PacketManager.h"
#include "PacketURState.h"
#include "TCPServer.h"
#include "PreciseClock.h"
#include "PacketURConfigCommands.h"

#define ROBOT_TYPE_UR5
namespace URRobot {
  extern const double HOME_POSITION[6];
  extern const double MIN_Q[6];
  extern const double MAX_Q[6];
  extern const double MAX_QD[6];
  extern const double MAX_QDD[6];
  extern const double MAX_TORQUE[6];
  extern const double ZERO_JOINTS_VECTOR[6];
  extern const double CONTROL_PERIOD;
}

class URRobotInterface{
public:
  URRobotInterface();

  bool initialize(int port);

  void powerOffRobot();

  void controlLoop();

protected:
  /*
   * override this to implement custom robot controller
   */
  virtual void controlRobot() {robotinterface_command_empty_command();}

  /*
   * override this to send out custom packets
   */
  virtual void sendCustomMessages() {}

  /*
   *
   */
  void delayNSteps(unsigned int n);

  /*
   *
   */
  bool registerPacket(SerializablePacket* packet);

private:
  void readRobotState();

  void processInputMessages();

  void sendStateMessage();

  void conductConfigCommands();
protected:
  URState robot_state;
  URState prev_robot_state;
private:
  TCPServer _tcp_server;
  PacketManager _packet_manager;

  PacketURState _packet_ur_state;
  PacketURConfigCommand _packet_config_cmd;
};

#endif
