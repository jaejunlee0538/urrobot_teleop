//
// Created by ub1404 on 17. 2. 18.
//

#include "URRobotJointPControl.h"

URRobotJointPControl::URRobotJointPControl() : _first_run(true) {
  //initializing joint position controllers
  double P_init[6] = {1, 1, 1, 1, 1, 1};
  for(int i=0;i<6;i++){
    _controller[i].init(P_init[i], -URRobot::MAX_Q[i], URRobot::MAX_Q[i], 0.5*URRobot::MAX_QD[i], 0.8*URRobot::MAX_QDD[i]);
  }

  this->registerPacket(&_packet_jnt_target);
}

void URRobotJointPControl::controlRobot() {
  if(_first_run){
    for(size_t i=0;i<6;i++){
      _q_des[i] = robot_state.q_act[i];
    }
    _first_run = false;
  }

  if(_packet_jnt_target.isUpdated()){
    if(_packet_jnt_target.isSafeCommand(URRobot::MAX_Q, URRobot::MIN_Q)) {
      for (size_t i = 0; i < 6; i++) {
        _q_des[i] = _packet_jnt_target.q_des[i];
      }
      fprintf(stderr, "Received JointTargetPos [");
      for(size_t i=0;i<6;i++){
        fprintf(stderr, "%.3lf ",_packet_jnt_target.q_des[i]);
      }
      fprintf(stderr, "]\n");
    }else{
      fprintf(stderr, "Received JointTargetPos is not safe. Rejected.\n\t");
      for(size_t i=0;i<6;i++){
        fprintf(stderr, "%.3lf ",_packet_jnt_target.q_des[i]);
      }
      fprintf(stderr, "\n");
    }
    _packet_jnt_target.clearUpdated();
  }

  double qd_cmd[6] = {0,0,0,0,0,0};
  for(size_t i=0;i<6;i++){
    qd_cmd[i] = _controller[i].update(
        robot_state.q_act[i],
        robot_state.qd_act[i],
        _q_des[i],
        URRobot::CONTROL_PERIOD);
  }
  robotinterface_command_velocity(qd_cmd);
}