//
// Created by ub1404 on 15. 9. 17.
//

#include "URRobotInterface.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>


#define MAX_MSG_BUFFER_SIZE 10
struct message_t msg_buffer[MAX_MSG_BUFFER_SIZE];
char text_buffer[MAX_MSG_BUFFER_SIZE][200];

namespace URRobot {
#if defined(ROBOT_TYPE_UR5)
  const double HOME_POSITION[6] = {0.0, -1.570796, 0.0, -1.570796, 0.0, 0.0};
  const double MIN_Q[6] = {-2 * M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI};
  const double MAX_Q[6] = {2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI};
  const double MAX_QD[6] = {M_PI, M_PI, M_PI, M_PI, M_PI, M_PI};
  const double MAX_QDD[6] = {15.0, 15.0, 15.0, 25.0, 25.0, 25.0};
  const double MAX_TORQUE[6] = {150.0, 150.0, 150.0, 28.0, 28.0, 28.0};
  const double CONTROL_PERIOD = 0.008;
  const double ZERO_JOINTS_VECTOR[6] = {0, };
#elif defined(ROBOT_TYPE_UR10)
#error "Joint Information for UR10 is not complete"
  //const double HOME_POSITION[6] = {0.0, -1.570796, 0.0, -1.570796, 0.0, 0.0};
  //const double MAX_Q[6] = {2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI};
  //const double MAX_QD[6] = {M_PI, M_PI, M_PI, M_PI, M_PI, M_PI};
  //const double MAX_QDD[6] = {15.0, 15.0, 15.0, 25.0, 25.0, 25.0};
  //const double MAX_TORQUE[6] = {150.0, 150.0, 150.0, 28.0, 28.0, 28.0};
  //const double ZERO_JOINTS_VECTOR[6] = {0, };
#else
#error "ROBOT_TYPE_XX is not defined."
#endif
}
URRobotInterface::URRobotInterface(){
  _packet_ur_state.setURState(&robot_state);
  registerPacket(&_packet_ur_state);
  registerPacket(&_packet_config_cmd);
  for(int i=0;i<MAX_MSG_BUFFER_SIZE;i++)
  {
    msg_buffer[i].text = text_buffer[i];
  }
}


void URRobotInterface::readRobotState(){
  prev_robot_state = robot_state;

  robot_state.sequence = robotinterface_get_step();
  robotinterface_get_actual(this->robot_state.q_act, this->robot_state.qd_act);
  robotinterface_get_actual_current(this->robot_state.i_act);
  robotinterface_get_tcp_force(this->robot_state.tcp_force);
  robotinterface_get_tcp_speed(this->robot_state.tcp_speed);
  robotinterface_get_target(this->robot_state.q_des, this->robot_state.qd_des, this->robot_state.qdd_des);
  robotinterface_get_target_current(this->robot_state.i_des);
  robotinterface_get_target_moment(this->robot_state.moment_des);
  // config states
  this->robot_state.robot_mode_id = robotinterface_get_robot_mode();
  this->robot_state.is_power_on_robot = robotinterface_is_power_on_robot();
  this->robot_state.is_security_stopped = robotinterface_is_security_stopped();
  this->robot_state.is_emergency_stopped = robotinterface_is_emergency_stopped();
  this->robot_state.is_extra_button_pressed = robotinterface_is_extra_button_pressed();
  this->robot_state.is_power_button_pressed = robotinterface_is_power_button_pressed();
  this->robot_state.is_safety_signal_such_that_we_should_stop =
      robotinterface_is_safety_signal_such_that_we_should_stop();
  this->robot_state.is_robot_connected = robotinterface_is_robot_connected();

  /* This is from the safety stop interface */
  for(int i=0;i<6;i++)
    this->robot_state.joint_mode_id[i] = robotinterface_get_joint_mode(i);
  if(this->robot_state.is_extra_button_pressed)
    this->robot_state.extra_button.update(Button::BUTTON_ON, robot_state.sequence * 0.008);
  else
    this->robot_state.extra_button.update(Button::BUTTON_OFF, robot_state.sequence * 0.008);

  // read error codes
  int msg_count = robotinterface_get_message_count();
  if(msg_count > MAX_MSG_BUFFER_SIZE)
    msg_count = MAX_MSG_BUFFER_SIZE;

  for(int i=0;i<msg_count;i++) {
    robotinterface_get_message(&msg_buffer[i]);
    printf("[message from ur] source : %d timestamp : %lu\nmsg : %s\n",msg_buffer[i].source, msg_buffer[i].timestamp,msg_buffer[i].text);
  }
}

void URRobotInterface::sendStateMessage() {
  if(_tcp_server.isConnected()){
    PacketManager::sendPacket(&_tcp_server, &_packet_ur_state);
  }
}

void URRobotInterface::processInputMessages() {
  PreciseClock clock;

  if(_tcp_server.isConnected()) {
    while(_tcp_server.isReadyToReceive(1)){
      _packet_manager.processIncomingData(&_tcp_server);
      if(clock.getElapsedTime_sec() > 0.003){
        break;
      }
    }
  }else{
    static ClockTimeout timeout_1sec(1.0);
    if(timeout_1sec.isTimeout()){
      printf("Wait for connection...\n");
      timeout_1sec.resetTimeout(true);
    }
    if(_tcp_server.waitForConnection(2)){
      printf("connected with %s!\n", _tcp_server.getOtherIP());
    }
  }
}

void URRobotInterface::controlLoop() {
  robotinterface_read_state_blocking();

  //update robot state
  readRobotState();
  //send out state packet
  sendStateMessage();
  //process incomming packets from outside
  processInputMessages();
  //conduct config commands
  conductConfigCommands();
  //call robot commands
  controlRobot();
  //send custom messages to out of the control box
  sendCustomMessages();

  robotinterface_send();
}


bool URRobotInterface::registerPacket(SerializablePacket* packet) {
  return _packet_manager.registerPacket(packet);
}
#ifdef REAL_ROBOT
namespace __URInterface_private {
  pid_t pid;
  struct sched_param sch_param;
}

bool URRobotInterface::initialize(int port) {
      if(_tcp_server.initialize(port) == -1){
    return false;
  }
  printf("TCP server initialized\n");
    printf("Loading robot configuration\n");
    configuration_load();

    printf("Setting RT priority\n");
    __URInterface_private::pid = getpid();
    __URInterface_private::sch_param.sched_priority = 99;
    if (sched_setscheduler(__URInterface_private::pid, SCHED_FIFO, &(__URInterface_private::sch_param)) != 0) {
        printf("- Priority not set, error: %i\n", errno);
        return false;
    }
    return true;
}
#else
bool URRobotInterface::initialize(int port){
  if(_tcp_server.initialize(port) == -1){
    return false;
  }
  printf("TCP server initialized\n");

  printf("Robot initialized\n");
  return true;
}
#endif

void URRobotInterface::powerOffRobot() {
  for(size_t i=0;i<20;i++) {
    robotinterface_read_state_blocking();
    robotinterface_power_off_robot();
  }
  delayNSteps(10);
  for(size_t i=0;i<20;i++) {
    robotinterface_read_state_blocking();
    robotinterface_close();
  }
  delayNSteps(10);
}

void URRobotInterface::delayNSteps(unsigned int n) {
#ifdef REAL_ROBOT
  for(unsigned int i=0;i<n;i++){
        robotinterface_read_state_blocking();
        robotinterface_command_empty_command();
        robotinterface_send();
    }
#else
  printf("%d step delay\n", n);
#endif
}

void URRobotInterface::conductConfigCommands() {
  if(_packet_config_cmd.isUpdated()){
    if(_packet_config_cmd.isCommandSet(PacketURConfigCommand::ConfigOpenInterface)){
      robotinterface_open(0);
    }
    if(_packet_config_cmd.isCommandSet(PacketURConfigCommand::ConfigCloseInterface)){
      robotinterface_close();
    }
    if(_packet_config_cmd.isCommandSet(PacketURConfigCommand::ConfigPowerOn)){
      robotinterface_power_on_robot();
    }
    if(_packet_config_cmd.isCommandSet(PacketURConfigCommand::ConfigPowerOff)){
      robotinterface_power_off_robot();
    }
    if(_packet_config_cmd.isCommandSet(PacketURConfigCommand::ConfigSetReadyMode)){
      robotinterface_set_robot_ready_mode();
    }
    if(_packet_config_cmd.isCommandSet(PacketURConfigCommand::ConfigSetRunningMode)){
      robotinterface_set_robot_running_mode();
    }
    if(_packet_config_cmd.isCommandSet(PacketURConfigCommand::ConfigSetFreedriveMode)){
      robotinterface_set_robot_freedrive_mode();
    }
    if(_packet_config_cmd.isCommandSet(PacketURConfigCommand::ConfigSetSecurityStop)){
//      robotinterface_security_stop()
      fprintf(stderr,"[PacketURConfigCommand::ConfigSetSecurityStop] is not implemented\n");
    }
    if(_packet_config_cmd.isCommandSet(PacketURConfigCommand::ConfigUnlockSecurityStop)){
      robotinterface_unlock_security_stop();
    }
    _packet_config_cmd.clearUpdated();
  }
}