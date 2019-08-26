#ifndef __PID_H__
#define __PID_H__

#include "sys.h"
#include "ano.h"

struct pid {
    int error;
    int error_t_1;
    int Kp;
    int Kd;
    int delta_target;
    int output_target;
    unsigned char des_force;
};
int pid_control(struct pid* pid_data,unsigned char actual);
int pid_control2(struct pid* pid_data,unsigned char actual);
#endif
