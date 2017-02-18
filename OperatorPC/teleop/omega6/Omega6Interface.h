//
// Created by ub1404 on 15. 9. 14.
//

#ifndef UR_ROBOT_OMEGA6INTERFACE_H
#define UR_ROBOT_OMEGA6INTERFACE_H
#include "Omega6Utils.h"
#include <pthread.h>
#include <vector>

/*
 * Two usages were intended.
 *  1. loop in single thread.
 *      - hapticLoop
 *
 *  2. loop in multi thread.
 *      - startHapticThread
 *      - jointHapticThread
 *      - hapticLoopThread
 *
 */
class Omega6Interface {
public:
    Omega6Interface()
    {
        is_initialized = false;
        loop_count = 0;
    }

    virtual ~Omega6Interface()
    {
        OMEGA6::closeDHD();
    }

    bool init(){
        if(OMEGA6::initDHD(true) == DHD_ERROR)
        {
            OMEGA6::printErrorMessage();
            is_initialized = false;
        }
        else
        {
            is_initialized = true;
        }
        return is_initialized;
    }

    bool isInitialized()
    {
        return is_initialized;
    }

    bool startHapticThread(){
        if(isInitialized())
        {
            pthread_create(&haptic_thread,NULL,&Omega6Interface::hapticLoopThread, (void*)this);
            return true;
        }
        return false;
    }

    void joinHapticThread(){
        if(!isInitialized())
            return;
        pthread_join(this->haptic_thread, NULL);
    }

    bool shouldStop(){
        return OMEGA6::shouldStop();
    }

    void hapticLoop(){
        button.update();
        this->cur_time = OMEGA6::getTime();
        OMEGA6::getPositionAndOrientationRad(this->xyz, this->orient);
        OMEGA6::getLinearVelocity(this->vel_linear);
        OMEGA6::getAngularVelocityRad(this->vel_angular);

        this->hapticLoopImplementation(this->loop_count);//Additional behavior

        OMEGA6::copyData(this->xyz, this->xyz_last, 3);
        OMEGA6::copyData(this->orient, this->orient_last, 3);
    }

protected:
    virtual void hapticLoopImplementation(int loopCount) = 0;
protected:
    OMEGA6::ButtonOmega6 button;

    double cur_time;
    double xyz[3], xyz_last[3];
    double orient[3], orient_last[3];
    double vel_linear[3], vel_angular[3];

private:
    static void* hapticLoopThread(void * instance){
        Omega6Interface* omega6 = static_cast<Omega6Interface*>(instance);

        printf("Omega6 Haptic Loop Started\n");
        while(!omega6->shouldStop())
        {
            omega6->hapticLoop();
        }
        printf("Terminating Omega6 Haptic Loop\n");
        return NULL;
    }

    pthread_t haptic_thread;
    bool is_initialized;

    int loop_count;

};


#endif //UR_ROBOT_OMEGA6INTERFACE_H
