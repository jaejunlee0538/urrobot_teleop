//
// Created by ub1404 on 15. 9. 17.
//

#ifndef PROJECT_UR5_MESSAGES_H
#define PROJECT_UR5_MESSAGES_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

template <typename T>
inline void SET_BIT(T& val, int flag_offset)
{
    val |= ((T)1)<<flag_offset;
}

template <typename T>
inline void CLEAR_BIT(T& val, int flag_offset)
{
    val &= ~(((T)1)<<flag_offset);
}

template <typename T>
inline void CLEAR_ALL_BIT(T& val)
{
    val = (T)0;
}

template <typename T>
inline bool IS_BIT_SET(T& val, int flag_offset)
{
    return (bool)(val & (((T)1)<<flag_offset));
}

template <typename T>
void printBits(T val)
{
    int N_bits = sizeof(T) * 8;
    for(int i=0;i<N_bits;i++)
    {
        if(i > 0 && i % 8 == 0)
            printf(" ");
        printf("%d", (bool)(val&(((T)1)<<(N_bits-i-1))));
    }
    printf("\n");
}

/****************ROBOT MODE*****************/
//defined at robotinterface.h
#define ROBOT_RUNNING_MODE 0
#define ROBOT_FREEDRIVE_MODE 1
#define ROBOT_READY_MODE 2
#define ROBOT_INITIALIZING_MODE 3
#define ROBOT_SECURITY_STOPPED_MODE 4
#define ROBOT_EMERGENCY_STOPPED_MODE 5
#define ROBOT_FATAL_ERROR_MODE 6
#define ROBOT_NO_POWER_MODE 7
#define ROBOT_NOT_CONNECTED_MODE 8
#define ROBOT_SHUTDOWN_MODE 9

/****************JOINT MODE*****************/
//defined at microprocessor_commands.h
#define JOINT_MODE_BAR 237
#define JOINT_PART_D_CALIBRATION_MODE 237
#define JOINT_BACKDRIVE_MODE 238
#define JOINT_POWER_OFF_MODE 239
#define JOINT_EMERGENCY_STOPPED_MODE 240
#define JOINT_CALVAL_INITIALIZATION_MODE 241
#define JOINT_ERROR_MODE 242
#define JOINT_FREEDRIVE_MODE 243
#define JOINT_SIMULATED_MODE 244
#define JOINT_NOT_RESPONDING_MODE 245
#define JOINT_MOTOR_INITIALISATION_MODE 246
#define JOINT_BOOTING_MODE 247
#define JOINT_PART_D_CALIBRATION_ERROR_MODE 248
#define JOINT_BOOTLOADER_MODE 249
#define JOINT_CALIBRATION_MODE 250
#define JOINT_SECURITY_STOPPED_MODE 251
#define JOINT_FAULT_MODE 252
#define JOINT_RUNNING_MODE 253
#define JOINT_INITIALISATION_MODE 254
#define JOINT_IDLE_MODE 255


enum URStateFlags {
    URFLAG_IS_POWER_ON_ROBOT = 0,
    URFLAG_IS_SECURITY_STOPPED,
    URFLAG_IS_EMERGENCY_STOPPED,
    URFLAG_IS_EXTRA_BUTTON_PRESSED,
    URFLAG_IS_POWER_BUTTON_PRESSED,
    URFLAG_IS_SAFETY_SIGNAL_XXX,
    URFLAG_IS_ROBOT_CONNECTED
};

enum URConfigCommandTypes {
    URCONFIG_OPEN_INTERFACE = 0,
    URCONFIG_CLOSE_INTERFACE,
    URCONFIG_POWER_ON_ROBOT,
    URCONFIG_POWER_OFF_ROBOT,
    URCONFIG_SET_ROBOT_READY_MODE,
    URCONFIG_SET_ROBOT_RUNNING_MODE,
    URCONFIG_SET_ROBOT_FREEDRIVE_MODE,
    URCONFIG_SET_SECURITY_STOP,
    URCONFIG_UNLOCK_SECURITY_STOP
};

enum URJointCommandTypes {
    URJOINTCMD_Q = 0,
    URJOINTCMD_Q_QD_QDD,
    URJOINTCMD_TORQUE
};

#pragma pack(push,1)
struct URStatePacket {
    uint8_t robot_mode;         //ROBOT_XXXX MACRO(not bit masking)
    double robot_time;
    uint8_t robot_state_flags;  //bit masking with URStateFlags

    uint8_t joint_modes[6];     //JOINT_XXXXX MACRO(not bit masking)
    double q[6];
    double qd[6];
    double current[6];
};
#pragma pack(pop)

#pragma pack(push,1)
struct URCommandPacket {
    uint32_t configCmd; //URConfigCommandTypes
    uint8_t jntCmd;     //URJointCommandTypes
    double q[6];
    double qd[6];
    double qdd[6];
    double torque[6];
};
#pragma pack(pop)


#endif //PROJECT_UR5_MESSAGES_H
