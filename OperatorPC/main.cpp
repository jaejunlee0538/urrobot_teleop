//
// Created by ub1404 on 16. 11. 14.
//

#include "teleop/omega6/Omega6DeltaPosCommander.h"
#include "TCPClient.h"
#include "ur5_messages.h"
#include "kinematics/RobotKinematics.h"


KDL::JntArray toJntArray(const double* q, unsigned int len){
    KDL::JntArray jnt(len);
    for(unsigned int i=0;i<len;i++){
        jnt(i,0) = q[i];
    }
    return jnt;
}

KDL::Frame toFrame(const double* xyz, const double* orient){
    KDL::Frame frame;

}

int main(int argc, char** argv){
    if (argc != 2) {
        printf("usage : [prog] [IP]\n");
        return 0;
    }

    Omega6DeltaPosCommander deltaPosCommander;
    deltaPosCommander.init();
    if(!deltaPosCommander.isInitialized()){
        printf("Initializing omega6 failed\n");
        return 0;
    }

    TCPClient tcp_client;
    tcp_client.init();
    tcp_client.connect("192.168.2.100", 30005);
    if(!tcp_client.isConnected()){
        return 0;
    }

    deltaPosCommander.startHapticThread();

#define BUFFER_SIZE 1024
    char buffer[BUFFER_SIZE];
    size_t buffer_idx=0;
    bool first_run = true;
    double des_xyz[3];
    double des_orient[3];
    UR5KinematicsKDLNRJL ur5_kinematics;

    while(tcp_client.isConnected() && !deltaPosCommander.shouldStop()){
        printf("1\n");
        int read_len = tcp_client.read(&buffer, sizeof(URStatePacket)-buffer_idx);
        printf("2\n");

        if(read_len > 0){
            printf("3\n");

            buffer_idx += read_len;

            if(buffer_idx == sizeof(URStatePacket)){
                URStatePacket ur_state;
                ::memcpy((void*)&ur_state, (void*)buffer, sizeof(URStatePacket));
                buffer_idx = 0;
                if(first_run){
                    KDL::Frame frame;
                    if(ur5_kinematics.getForwardPositionKinematics(toJntArray(ur_state.q,6), frame)<0){
                        printf("FK error\n");
                        continue;
                    }
                    des_xyz[0] = frame.p[0];
                    des_xyz[1] = frame.p[1];
                    des_xyz[2] = frame.p[2];
                    first_run = false;
                    continue;
                }
                KDL::Frame fk_result;
                if(ur5_kinematics.getForwardPositionKinematics(toJntArray(ur_state.q,6), fk_result)<0){
                    printf("FK error--\n");
                }
                double R,P,Y;
                fk_result.M.GetRPY(R,P,Y);
//                fprintf(stdout,"%5.3lf %5.3lf %5.3lf %5.3lf %5.3lf %5.3lf\r",
//                        des_xyz[0]-fk_result.p[0],
//                        des_xyz[1]-fk_result.p[1],
//                        des_xyz[2]-fk_result.p[2],
//                        des_orient[0] - R,
//                        des_orient[1] - P,
//                        des_orient[2] - Y);
                std::vector<double> delta_pos, orient;
                deltaPosCommander.getDeltaPosition(delta_pos, orient);

                for(size_t i=0;i<3;i++){
                    des_xyz[i] += delta_pos[i];
                    des_orient[i] = orient[i];
                }

                KDL::JntArray des_q, current_q;
                current_q = toJntArray(ur_state.q, 6);
                KDL::Frame des_frame;
                des_frame.p = KDL::Vector(des_xyz[0], des_xyz[1], des_xyz[2]);
                des_frame.M.RPY(des_orient[0], des_orient[1], des_orient[2]);
                ur5_kinematics.getInversePositionKinematics(des_frame,current_q,des_q);

                URCommandPacket command;
                command.jntCmd = URJointCommandTypes::URJOINTCMD_Q;
                for(size_t i=0;i<6;i++){
                    command.q[i] = des_q(i,0);
                }
                fprintf(stdout,"%5.3lf %5.3lf %5.3lf %5.3lf %5.3lf %5.3lf\n",
                    des_q(0,0), des_q(1,0), des_q(2,0),
                        des_q(3,0), des_q(4,0), des_q(5,0));
//                tcp_client.write((void*)&command, sizeof(URCommandPacket));
            }
        }
    }

    deltaPosCommander.joinHapticThread();

    return 0;
}