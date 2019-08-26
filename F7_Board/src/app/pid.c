#include "pid.h"
#include "usart.h"
int pid_control(struct pid* pid_data,unsigned char actual)
{
    pid_data->error = actual - pid_data->des_force;
    pid_data->delta_target = pid_data->Kp * pid_data->error + pid_data->Kd * (pid_data->error - pid_data->error_t_1);
    pid_data->output_target = pid_data->output_target + pid_data->delta_target;

    //printf("%3d %3d %6d %6d\r\n",actual,pid_data->des_force,pid_data->delta_target,pid_data->output_target);

    pid_data->error_t_1 = pid_data->error;

    return pid_data->output_target;

}


int pid_control2(struct pid* pid_data,unsigned char actual)
{
    pid_data->error = actual - pid_data->des_force;
    pid_data->delta_target = pid_data->Kp * pid_data->error + pid_data->Kd * (pid_data->error - pid_data->error_t_1);
    pid_data->output_target = 	pid_data->delta_target;
    printf("%3d %3d %6d %6d\r\n",actual,pid_data->des_force,pid_data->delta_target,pid_data->output_target);

    pid_data->error_t_1 = pid_data->error;

    return pid_data->output_target;
}


