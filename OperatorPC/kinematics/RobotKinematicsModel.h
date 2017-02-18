//
// Created by ub1404 on 15. 9. 3.
//

#ifndef ROBOTCONTROL_ROBOTKINEMATICSMODEL_H
#define ROBOTCONTROL_ROBOTKINEMATICSMODEL_H

const double __PI__ = 3.14159265359;
const double __2PI__ = 2 * __PI__;

#define __D2R(x) (0.0174532925 * x)
#define __R2D(x) (57.2957795 * x)

namespace KinematicsModel {
    namespace LWA3 {
        const int NR_JOINT = 7;
//        const std::vector <std::string> joint_names = {"arm_1_joint",
//                                                       "arm_2_joint",
//                                                       "arm_3_joint",
//                                                       "arm_4_joint",
//                                                       "arm_5_joint",
//                                                       "arm_6_joint",
//                                                       "arm_7_joint"};

        const double a[NR_JOINT] = {0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000};
        const double alpha[NR_JOINT] = {-1.570796327, 1.570796327, -1.570796327, 1.570796327, -1.570796327, 1.570796327,
                                        -1.570796327};
        const double d[NR_JOINT] = {0.3000, 0.00000, 0.3280, 0.00000, 0.2765, 0.00000, 0.1717};
        const double theta[NR_JOINT] = {0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000};
        const int joint_direction[NR_JOINT] = {1, 1, 1, 1, 1, 1, 1};
        const double joint_limit_upper[NR_JOINT] = {3.1415, 1.97, 3.1415, 1.97, 3.1415, 1.97, 3.1415};
        const double joint_limit_lower[NR_JOINT] = {-3.1415, -1.97, -3.1415, -1.97, -3.1415, -1.97, -3.1415};
    }

    namespace UR5 {

        const int NR_JOINT = 6;
        const double __SAFETY_OFFSET = __D2R(20);
        //offset for safety
        const double __LIM1 = __PI__ * 2.0 - __SAFETY_OFFSET;    //Upper Joint Angle Limit
        const double __LIM2 = -1.0 * __PI__ * 2.0 + __SAFETY_OFFSET;    //Lower Joint Angle Limit

//        const std::vector <std::string> joint_names[NR_JOINT] = {"shoulder_pan_joint",
//                                                                 "shoulder_lift_joint",
//                                                                 "elbow_joint",
//                                                                 "wrist_1_joint",
//                                                                 "wrist_2_joint",
//                                                                 "wrist_3_joint"};

        const double a[NR_JOINT] = {0.00000, -0.42500, -0.39225, 0.00000, 0.00000, 0.00000};
        const double d[NR_JOINT] = {0.089159, 0.00000, 0.00000, 0.10915, 0.09465, 0.0823};
        const double alpha[NR_JOINT] = {1.570796327, 0, 0, 1.570796327, -1.570796327, 0};
        const double joint_offset[NR_JOINT] = {0, -1.570796327, 0, -1.570796327, 0, 0};
        const int joint_direction[NR_JOINT] = {-1, -1, 1, 1, 1, 1};
        const double joint_limit_upper[NR_JOINT] = {__LIM1, __LIM1, __LIM1, __LIM1, __LIM1, __LIM1};
        const double joint_limit_lower[NR_JOINT] = {__LIM2, __LIM2, __LIM2, __LIM2, __LIM2, __LIM2};

    }
}


#endif //ROBOTCONTROL_ROBOTKINEMATICSMODEL_H
