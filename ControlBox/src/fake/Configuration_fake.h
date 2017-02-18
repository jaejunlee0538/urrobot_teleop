//
// Created by ub1404 on 15. 8. 24.
//


#ifndef CONFIGURATION_FAKE_H_
#define CONFIGURATION_FAKE_H_

#include <stdio.h>

inline int calibration_load() {
    printf("calibration_load\n");
    return 1;
}

inline int calibration_save() {
    printf("calibration_save\n");
    return 1;
}

inline int configuration_load() {
    printf("configuration_load\n");
    return 1;
}

inline void check_for_valid_hw_combo() {
    printf("check_for_valid_hw_combo\n");
}

inline void configuration_print() {
    printf("configuration_print\n");
}

inline void calibration_print() {
    printf("calibration_print\n");
}

inline const char *getJointVersion(int const jointID) {
    printf("getJointVersion\n");
}

inline void setForceParamsToDefault() {
    printf("setForceParamsToDefault\n");
}
#endif