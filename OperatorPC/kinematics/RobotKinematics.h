//
// Created by ub1404 on 15. 9. 3.
//

#ifndef ROBOTCONTROL_ROBOTKINEMATICS_H
#define ROBOTCONTROL_ROBOTKINEMATICS_H

#include <kdl/chain.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <boost/shared_ptr.hpp>

typedef boost::shared_ptr<KDL::ChainFkSolverPos> ChainFkSolverPosPtr;
typedef boost::shared_ptr<KDL::ChainIkSolverVel> ChainIkSolverVelPtr;
typedef boost::shared_ptr<KDL::ChainIkSolverPos> ChainIkSolverPosPtr;
typedef boost::shared_ptr<KDL::ChainFkSolverVel> ChainFKSolverVelPtr;

class RobotKinematicsInterfaceKDL {
public:
    int getNRJoints(void) {
        return this->_kin_chain.getNrOfJoints();
    }

    int getForwardPositionKinematics(const KDL::JntArray &q_current, KDL::Frame &x_out, int segmentNr = -1) {
        if (chain_fk_solver_pos)
            return chain_fk_solver_pos->JntToCart(q_current, x_out, segmentNr);
        return -1;
    }

    int getForwardVelocityKinematics(const KDL::JntArrayVel &q_in, KDL::FrameVel &out, int segmentNr = -1) {
        if (chain_fk_solver_vel)
            return chain_fk_solver_vel->JntToCart(q_in, out, segmentNr);
        return -1;
    }

    int getInversePositionKinematics(const KDL::Frame &x_target, const KDL::JntArray &q_current, KDL::JntArray &q_out) {
        if (chain_ik_solver_pos)
            return this->chain_ik_solver_pos->CartToJnt(q_current, x_target, q_out);
        return -1;
    }

    virtual int getInverseVelocityKinematics(const KDL::JntArray &q_in, const KDL::Twist &v_in,
                                             KDL::JntArray &qdot_out) {
        if (chain_ik_solver_vel)
            return this->chain_ik_solver_vel->CartToJnt(q_in, v_in, qdot_out);
        return -1;
    }

protected:
    KDL::Chain _kin_chain;

    ChainFkSolverPosPtr chain_fk_solver_pos;
    ChainFKSolverVelPtr chain_fk_solver_vel;
    ChainIkSolverVelPtr chain_ik_solver_vel;
    ChainIkSolverPosPtr chain_ik_solver_pos;
};

#include <kdl/chainiksolverpos_nr_jl.hpp>

class RobotKinematicsKDLNRJL : public RobotKinematicsInterfaceKDL {
public:
    RobotKinematicsKDLNRJL() {

    }

    void init(const KDL::Chain &robot_chain, const KDL::JntArray &q_min, const KDL::JntArray &q_max) {
        this->_kin_chain = robot_chain;
        this->chain_fk_solver_pos.reset(new KDL::ChainFkSolverPos_recursive(this->_kin_chain));
        this->chain_fk_solver_vel.reset(new KDL::ChainFkSolverVel_recursive(this->_kin_chain));
//        this->chain_ik_solver_vel.reset(new KDL::ChainIkSolverVel_pinv(this->_kin_chain, 0.001, 100));
        this->chain_ik_solver_vel.reset(new KDL::ChainIkSolverVel_wdls(this->_kin_chain));
        this->chain_ik_solver_pos.reset(new KDL::ChainIkSolverPos_NR_JL(_kin_chain, q_min, q_max,
                                                                    *chain_fk_solver_pos,
                                                                    *chain_ik_solver_vel, 500, 1e-5));
    }
};

class UR5KinematicsKDLNRJL : public RobotKinematicsKDLNRJL {
public:
    UR5KinematicsKDLNRJL();
};


#endif //ROBOTCONTROL_ROBOTKINEMATICS_H
