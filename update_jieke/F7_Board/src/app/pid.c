#include "pid.h"
#include "usart.h"

/********************
位置式：u(k) = Kp*e(k) + Ki*Σe + Kd*(e(k)-e(k-1))

增量式：u(k) = Δu(k)+u(k-1)
				Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(n) + Kd*(e(k)+e(k-2)-2*e(k-1))

*********************/
int pid_control(struct pid* pid_data,unsigned char actual)
{
			pid_data->error = actual - pid_data->des_force;
			pid_data->delta_target = pid_data->Kp * pid_data->error + pid_data->Kd * (pid_data->error - pid_data->error_t_1);
	
			//这里错，位置式增量式混用
			//pid_data->output_target = pid_data->output_target + pid_data->delta_target;
			pid_data->output_target = pid_data->delta_target;
	
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


