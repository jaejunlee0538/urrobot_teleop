//
// Created by ub1404 on 15. 9. 14.
//

#ifndef UR_ROBOT_OMEGA6DELTAPOSCOMMANDER_H
#define UR_ROBOT_OMEGA6DELTAPOSCOMMANDER_H
#include "../interface/FrictionMode.h"
#include "Omega6Interface.h"
#include <vector>
class Omega6DeltaPosCommander: public Omega6Interface {
public:
    Omega6DeltaPosCommander()
    {
        this->first_button_run = true;
    }

    void getDeltaPosition(std::vector<double>& delta_xyz,
                          std::vector<double>& orientation)
    {
        delta_xyz.resize(3);
        orientation.resize(3);
        pthread_mutex_lock(&this->mutex);
        for(int i=0;i<3;i++)
        {
            delta_xyz[i] = this->xyz_end[i] - this->xyz_start[i];
            orientation[i] = this->orient_end[i];
        }
        this->reset();
        pthread_mutex_unlock(&this->mutex);
    }
protected:
    void hapticLoopImplementation(int loopCount)
    {
        if(this->button.isPressed())
        {
            pthread_mutex_lock(&this->mutex);
            if(this->first_button_run){
                OMEGA6::copyData(this->xyz, this->xyz_start, 3);
                OMEGA6::copyData(this->orient, this->orient_start, 3);
                OMEGA6::copyData(this->xyz, this->xyz_end, 3);
                OMEGA6::copyData(this->orient, this->orient_end, 3);
                this->first_button_run = false;
            }
            else{
                OMEGA6::copyData(this->xyz, this->xyz_end, 3);
                OMEGA6::copyData(this->orient, this->orient_end, 3);
            }
            pthread_mutex_unlock(&this->mutex);
        }
        else{
            if(!this->first_button_run)
            {
                this->reset();
                this->first_button_run = true;
            }
        }


        double forces[3];
        friction_force.computeForce(this->xyz, this->vel_linear, forces, NULL);
        OMEGA6::setForces(forces);
    }

    inline void reset(){
        pthread_mutex_lock(&this->mutex);
        for(int i=0;i<3;i++)
        {
            this->xyz_start[i] = this->xyz_end[i];
            this->orient_start[i] = this->orient_end[i];
        }
        pthread_mutex_unlock(&this->mutex);
    }

private:
    double xyz_start[3], orient_start[3];
    double xyz_end[3], orient_end[3];
    bool first_button_run;

    pthread_mutex_t mutex;

    FrictionModeInterface friction_force;
};


#endif //UR_ROBOT_OMEGA6DELTAPOSCOMMANDER_H
