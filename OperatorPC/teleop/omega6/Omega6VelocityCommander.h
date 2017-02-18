//
// Created by ub1404 on 15. 9. 14.
//

#ifndef UR_ROBOT_OMEGA6VELOCITYCOMMANDER_H
#define UR_ROBOT_OMEGA6VELOCITYCOMMANDER_H
#include "Omega6Interface.h"
#include <vector>
class Omega6VelocityCommander : public Omega6Interface {
public:
    Omega6VelocityCommander(){
        pthread_mutex_init(&this->mutex, NULL);
    }

    void getVelCommand(std::vector<double>& velcmd_xyz, std::vector<double>& velcmd_orient)
    {
        velcmd_xyz.resize(3);
        velcmd_orient.resize(3);

        pthread_mutex_lock(&mutex);
        for(int i=0;i<3;i++)
        {
            velcmd_xyz[i] = this->velcmd_xyz[i];
            velcmd_orient[i] = this->velcmd_orient[i];
        }
        pthread_mutex_unlock(&mutex);
    }

protected:
    void hapticLoopImplementation()
    {
        pthread_mutex_lock(&mutex);
        for(int i=0;i<3;i++)
        {
            this->velcmd_xyz[i] = this->vel_linear[i];
            this->velcmd_orient[i] = this->vel_angular[i];
        }
        pthread_mutex_unlock(&mutex);
    }
protected:
    double velcmd_xyz[3];
    double velcmd_orient[3];
    pthread_mutex_t mutex;


};


#endif //UR_ROBOT_OMEGA6VELOCITYCOMMANDER_H
