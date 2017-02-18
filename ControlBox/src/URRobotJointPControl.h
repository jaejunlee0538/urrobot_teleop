//
// Created by ub1404 on 17. 2. 18.
//

#ifndef PLAYGROUND_URROBOTJOINTPCONTROL_H
#define PLAYGROUND_URROBOTJOINTPCONTROL_H
#include "URRobotInterface.h"
#include "VelocityPControl.h"
#include "PacketJointTargetPos.h"

class URRobotJointPControl :public URRobotInterface{
public:
  URRobotJointPControl();

protected:
  void controlRobot();

  PacketJointTargetPos _packet_jnt_target;

  double _q_des[6];
  VelocityPControl _controller[6];
  bool _first_run;
};


#endif //PLAYGROUND_URROBOTJOINTPCONTROL_H
