//
// Created by ub1404 on 15. 9. 3.
//

#include "RobotKinematics.h"
#include "RobotKinematicsModel.h"
#include <string>

UR5KinematicsKDLNRJL::UR5KinematicsKDLNRJL() {
    using namespace KinematicsModel;
    KDL::Chain ur5_chain;

    for (int i = 0; i < UR5::NR_JOINT; i++) {
        std::ostringstream jnt_name;
        jnt_name<<"joint_"<<i;
        ur5_chain.addSegment(
                KDL::Segment(KDL::Joint(jnt_name.str(), KDL::Joint::RotZ),
                KDL::Frame::DH_Craig1989(UR5::a[i], UR5::alpha[i], UR5::d[i], 0.0)));
    }

    KDL::JntArray q_max(UR5::NR_JOINT), q_min(UR5::NR_JOINT);
    for (int i = 0; i < 6; i++) {
        q_min(i) = UR5::joint_limit_lower[i];
        q_max(i) = UR5::joint_limit_upper[i];
    }

    this->init(ur5_chain, q_min, q_max);
    printf("UR5KinematicsNRJL initialized\n");
}