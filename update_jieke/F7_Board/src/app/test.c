#include "sys.h"
#include "delay.h" 
#include "led.h"  
#include "mpu.h" 
#include "usart.h"  
#include "lcd.h" 
#include "sdram.h"  
#include "usmart.h"
#include "key.h"
#include "can.h"
#include "copley_node1.h"
#include "copley_node2.h"
#include "uart.h"
#include "math.h"
#include "ANO.h"
#include "pid.h"
#include "test.h"
#include "can.h"
#include "soft_switch.h"

int NODE2_MIN  = -150000;  //node2最小限位
int NODE2_MAX  = 200000;  		//node2最大限位
int NODE2_1_PRE = -20000; 		//node2髋前预拉紧位置
int NODE2_2_PRE = -20000;


int NODE1_MIN = -220000;     //node1最小限位
int NODE1_MAX = 100000;      //node1最大限位
int NODE1_PRE = 10000; 		//node1髋前预拉紧位置

int NODE1_1_PRE = 10000;
int NODE1_2_PRE = 10000;
int NIDE1_INI_POS = 0;  //node1初始位置
int NODE2_INT_POS = 0; //node2初始化位置


int NODE2_ACTUAL_F_MAX = 0;
int NODE2_ACTUAL_B_MAX = 0;
int NODE1_ACTUAL_F_MAX = 0;
int NODE1_ACTUAL_B_MAX = 0;


#define NODE2_ACTUAL_F f4_data[3]  //node2 髋前实际力
#define NODE2_ACTUAL_B f4_data[4]  //node2 髋后实际力

#define NODE2_TARGET_F f4_data[6]  //node2 髋前目标力
#define NODE2_TARGET_B f4_data[8]  //node2 髋后目标力


#define NODE1_ACTUAL_F f4_data[1]  //node1 髋前实际力
#define NODE1_ACTUAL_B f4_data[2]  //node1 髋后实际力

#define NODE1_TARGET_F f4_data[5]  //node1 髋前目标力
#define NODE1_TARGET_B f4_data[7]  //node1 髋后目标力





struct pid node2_1 = {
	.error = 0,
	.error_t_1 = 0,
	.Kp = 80,
	.Kd = 40,
	.delta_target = 0,
	.output_target = 0,
	.des_force = 0,
};



struct pid node2_2 = {
	.error = 0,
	.error_t_1 = 0,
	.Kp = -800,
	.Kd = -50,
	.delta_target = 0,
	.output_target = 0,
	.des_force = 0,
};


struct pid node1_1 = {
	.error = 0,
	.error_t_1 = 0,
	.Kp = 60,
	.Kd = 10,
	.delta_target = 0,
	.output_target = 0,
	.des_force = 0,
};



struct pid node1_2 = {
	.error = 0,
	.error_t_1 = 0,
	.Kp = 50,
	.Kd = 20,
	.delta_target = 0,
	.output_target = 0,
	.des_force = 0,
};






int test_flag1 = 0;   //测试1开关
int test_flag2 = 0;   //测试2开关



float ano_data[10]; 			 //匿名发送包

int actual_position2 = 0; //记录绝对位置 node2
int actual_position1 = 0; //记录绝对位置 node1

int node2_1_flag = 0; //2髋前拉一次标志
int node2_2_flag = 0; //2髋后拉一次标志


int node1_1_flag = 0; //1髋前拉一次标志
int node1_2_flag = 0; //1髋后拉一次标志

int node1_time = 0;   //node1髋前执行次数 在髋前执行完后预拉紧
int node2_time = 0;

int main(void)
{   

	Stm32_Clock_Init(432,25,2,9);//设置时钟,216Mhz
  delay_init(216);			//延时初始化  
	uart_init(108,921600);		//初始化串口波特率为921600 
	printf("Start system init\r\n");
	USART3_Init(54,500000); //匿名串口
	usmart_dev.init(108);		//初始化USMART
	MPU_Memory_Protection();	//保护相关存储区域
	SDRAM_Init();				//初始化SDRAM 

 	CAN1_Mode_Init(1,7,10,3,0);	//CAN初始化,波特率1000Kbps  

	copley_node1.init();
	copley_node2.init();

	
	printf("Can init end\r\n");
  init_switch();
	
	
	while(copley_node1.nmt_state!=0x00||copley_node2.nmt_state!=0x00){
			delay_ms(5);
			set_led(get_switc_status());	
	}
	
	printf("Start node init \r\n");
		
	copley_node1.start(OP_MODE_PPM);
  copley_node2.start(OP_MODE_PPM);
	copley_node1.ppm_init();
	copley_node2.ppm_init();
	
	delay_ms(5);	
	printf("Node init complete\r\n");
	USART2_Init(54,460800);

	delay_ms(1000);

	//set_prepare_param();

 	while(1)
	{ 
		USART2_Send_char('R');
		delay_ms(1);
		
		
		//printf("%d %d %d %d\r\n",NODE2_ACTUAL_F,NODE2_ACTUAL_B,NODE1_ACTUAL_F,NODE1_ACTUAL_B);
		
		
		if(!get_switc_status()){
			

				if(NODE2_TARGET_B > 5){   //目标生成，髋后
					  node2_2.des_force = NODE2_TARGET_B;
						actual_position2 = pid_control(&node2_2,NODE2_ACTUAL_B);	
						node2_1_flag = 0;
						node2_2_flag = 1;
				}else if(NODE2_TARGET_F > 5){       //髋前，直接放松cable到初始位置
						//node2_1.des_force = NODE2_TARGET_F;
					  
						//髋前信号开始进行预拉紧
						//node2_1.des_force = 5;
						//actual_position2 = pid_control(&node2_1,NODE2_ACTUAL_B);
					
						node2_1_flag = 1;
						node2_2_flag = 0;
				}else{				
						if(node2_1_flag == 1 && node2_2_flag == 0){ //髋前产生目标力结束，开始以力为目标控制，新的一套pid防止超调过大
								node2_1.des_force = 5;
								actual_position2 = pid_control(&node2_1,NODE2_ACTUAL_B); 
								//actual_position2 = 20000;
								//node2_1.output_target = NODE2_1_PRE;
								//actual_position2 = NODE2_1_PRE;
						}
						
						//髋后拉完的预拉紧，可以直接取消
							else if(node2_1_flag == 0 && node2_2_flag == 1){
									actual_position2 =0;
						}
				}		
				

							
				if(actual_position2 <= NODE2_MIN){  //限位
						actual_position2 = NODE2_MIN;
				}
				if(actual_position2 >= NODE2_MAX){						
						actual_position2= NODE2_MAX;
				}
				copley_node2.ppm_set_target(actual_position2);  //实际控制
				
				delay_ms(2);
				
				if(NODE1_TARGET_B > 5){  //目标生成，髋后
					  node1_2.des_force = NODE1_TARGET_B;
						actual_position1 = pid_control(&node1_2,NODE1_ACTUAL_B);	
						node1_1_flag = 0;
						node1_2_flag = 1;
				}else if(NODE1_TARGET_F > 5){       //目标生成,髋前
					
						//髋前信号开始进行预拉紧
						//node1_1.des_force = 5;
						//actual_position1 = pid_control(&node1_1,NODE1_ACTUAL_B);
					
						node1_1_flag = 1;
						node1_2_flag = 0;
				}else{				
						if(node1_1_flag == 1 && node1_2_flag == 0){
							  node1_1.des_force = 5;
							  actual_position1 = pid_control(&node1_1,NODE1_ACTUAL_B);

						}
						else if(node1_1_flag == 0 && node1_2_flag == 1){
								actual_position1 = 0;
						}												
				}		
				

							
				if(actual_position1 <= NODE1_MIN){  //限位
						actual_position1 = NODE1_MIN;
				}
				if(actual_position1 >= NODE1_MAX){						
						actual_position1= NODE1_MAX;
				}
				
				copley_node1.ppm_set_target(actual_position1);  //实际控制
				
	
		}
		
	/*匿名发送数据曲线*/
		ano_data[0] = NODE2_ACTUAL_F;  	
		ano_data[1] = NODE2_ACTUAL_B;  		
		ano_data[2] =	NODE2_TARGET_F;  
		ano_data[3] = NODE2_TARGET_B;  
		
		ano_data[4] = NODE1_ACTUAL_F;  
		ano_data[5] = NODE1_ACTUAL_B;  	
		ano_data[6] =	NODE1_TARGET_F;  
		ano_data[7] = NODE1_TARGET_B; 	
		
		
		ano_data[8] = actual_position1/4000; 
		ano_data[9] = actual_position2/4000;
		ANO_send(0xF1,(unsigned char*)ano_data,sizeof(float),sizeof(ano_data));

		set_led(~get_switc_status());		
		
		delay_ms(2);	
	} 

}



void set_prepare_param(void){


	
		struct pid node2_1 = {
			.error = 0,
			.error_t_1 = 0,
			.Kp = 100,
			.Kd = 0,
			.delta_target = 0,
			.output_target = 0,
			.des_force = 0,
		};



		struct pid node2_2 = {
			.error = 0,
			.error_t_1 = 0,
			.Kp = -100,
			.Kd = 0,
			.delta_target = 0,
			.output_target = 0,
			.des_force = 0,
		};


		struct pid node1_1 = {
			.error = 0,
			.error_t_1 = 0,
			.Kp = -100,
			.Kd = 0,
			.delta_target = 0,
			.output_target = 0,
			.des_force = 0,
		};



		struct pid node1_2 = {
			.error = 0,
			.error_t_1 = 0,
			.Kp = 100,
			.Kd = 0,
			.delta_target = 0,
			.output_target = 0,
			.des_force = 0,
		};
		
		int pre_1_1 = 0;
		int pre_1_2 = 0;
		int pre_2_1 = 0;
		int pre_2_2 = 0;
		
		int cnt = 0;
		int temp = 0;
	
		int PRE_SET_TIME = 1000;
		
		while(cnt < PRE_SET_TIME){
			

				USART2_Send_char('R');
				delay_ms(1);				
				node1_1.des_force = 7;
				pre_1_1 = pid_control(&node1_1,NODE1_ACTUAL_F);
				delay_ms(10);
				cnt++;
			
				if(pre_1_1 <= NODE1_MIN){  //限位
						pre_1_1 = NODE1_MIN;
				}
				if(pre_1_1 >= NODE1_MAX){//NODE1_MAX){						
						pre_1_1 = NODE1_MAX;
				}
				printf("pre_1_1 %d %d\r\n",pre_1_1,NODE1_ACTUAL_F);
				copley_node1.ppm_set_target(pre_1_1);  //实际控制
				
				if(cnt > PRE_SET_TIME - 10){
						temp += pre_1_1;
				}
		}
		pre_1_1 = temp/10;
		printf("result pre_1_1 %d %d\r\n",pre_1_1,NODE1_ACTUAL_F);
		

		cnt = 0;
		temp = 0;
		while(cnt < PRE_SET_TIME){
				USART2_Send_char('R');
				delay_ms(1);				
				node1_2.des_force = 7;
				pre_1_2 = pid_control(&node1_2,NODE1_ACTUAL_B);
				delay_ms(10);
				cnt++;
			
				if(pre_1_2 <= NODE1_MIN){  //限位
						pre_1_2 = NODE1_MIN;
				}
				if(pre_1_2 >= NODE1_MAX){//NODE1_MAX){						
						pre_1_2 = NODE1_MAX;
				}
				printf("pre_1_2 %d %d\r\n",pre_1_2,NODE1_ACTUAL_B);
				copley_node1.ppm_set_target(pre_1_2);  //实际控制			
				
				if(cnt > PRE_SET_TIME - 10){
						temp += pre_1_2;
				}
		}
		pre_1_2 = temp/10;
		printf("result pre_1_2 %d %d\r\n",pre_1_2,NODE1_ACTUAL_B);
		
		
		cnt = 0;
		temp = 0;
		while(cnt < PRE_SET_TIME){
				USART2_Send_char('R');
				delay_ms(1);				
				node2_1.des_force = 7;
				pre_2_1 = pid_control(&node2_1,NODE2_ACTUAL_F);
				delay_ms(10);
				cnt++;
			
				if(pre_2_1 <= NODE2_MIN){  //限位
						pre_2_1 = NODE2_MIN;
				}
				if(pre_2_1 >= NODE2_MAX){//NODE1_MAX){						
						pre_2_1 = NODE2_MAX;
				}
				printf("pre_2_1 %d %d\r\n",pre_2_1,NODE2_ACTUAL_F);
				copley_node2.ppm_set_target(pre_2_1);  //实际控制		
				
				if(cnt > PRE_SET_TIME - 10){
						temp += pre_2_1;
				}
		}
		pre_2_1 = temp/10;
		printf("result pre_2_1 %d %d\r\n",pre_2_1,NODE2_ACTUAL_F);
		
		
		cnt = 0;
		temp = 0;
		while(cnt < PRE_SET_TIME){
				USART2_Send_char('R');
				delay_ms(1);				
				node2_2.des_force = 7;
				pre_2_2 = pid_control(&node2_2,NODE2_ACTUAL_B);
				delay_ms(10);
				cnt++;
			
				if(pre_2_2 <= NODE2_MIN){  //限位
						pre_2_2 = NODE2_MIN;
				}
				if(pre_2_2 >= NODE2_MAX){//NODE1_MAX){						
						pre_2_2 = NODE2_MAX;
				}
				printf("pre_2_2 %d %d\r\n",pre_2_2,NODE2_ACTUAL_B);
				copley_node2.ppm_set_target(pre_2_2);  //实际控制		
				
				if(cnt > PRE_SET_TIME - 10){
						temp += pre_2_2;
				}
		}
		pre_2_2 = temp/10;
		printf("result pre_2_2 %d %d\r\n",pre_2_2,NODE2_ACTUAL_B);
		
		float FACTOR_LIMIT = 1.5;
		float FACTOR_PRE_TENSE = 0.3;
		
		if(pre_1_1 > pre_1_2){
			NODE1_MAX = pre_1_1*FACTOR_LIMIT;
			NODE1_MIN = pre_1_2*FACTOR_LIMIT;
		}else{
			NODE1_MAX = pre_1_2*FACTOR_LIMIT;
			NODE1_MIN = pre_1_1*FACTOR_LIMIT;
		}
		
		if(pre_2_1 > pre_2_2){
			NODE2_MAX = pre_2_1*FACTOR_LIMIT;
			NODE2_MIN = pre_2_2*FACTOR_LIMIT;
		}else{
			NODE2_MAX = pre_2_2*FACTOR_LIMIT;
			NODE2_MIN = pre_2_1*FACTOR_LIMIT;
		}
		
		NODE1_1_PRE = pre_1_1*FACTOR_PRE_TENSE;
		NODE1_2_PRE = pre_1_2*FACTOR_PRE_TENSE;
		NODE2_1_PRE = pre_2_1*FACTOR_PRE_TENSE;
		NODE2_2_PRE = pre_2_2*FACTOR_PRE_TENSE;
		

		printf("The pre_set result is: %d %d %d %d \r\n",pre_1_1,pre_1_2,pre_2_1,pre_2_2);
		printf("The pre_set result is: %d %d %d %d \r\n",NODE1_MAX,NODE1_MIN,NODE2_MAX,NODE2_MIN);
		printf("The pre set result is: %d %d %d %d \r\n",NODE1_1_PRE,NODE1_2_PRE,NODE2_1_PRE,NODE2_2_PRE);
		
		copley_node1.ppm_set_target(0);
		copley_node2.ppm_set_target(0);

}



void set_start(int flag){
		test_flag1 = flag;
	  printf(" %d ",test_flag1);
}




void set_start2(int flag){
		test_flag2 = flag;
	  printf(" %d ",test_flag2);
}

void show_pid(void){
	//printf(" node1:Kp:%d  Kd:%d node2:Kp %d  Kd %d ",node1_1.Kp,node1_1.Kd,node2_1.Kp,node2_1.Kd);
}

void set_kp(int kp){
		node2_1.Kp = kp;	
		printf(" node2:%d ",node2_1.Kp);
}

void set_kd(int kd){
		node2_1.Kd = kd;	
		printf(" node2:%d ",node2_1.Kd);		
}
void read_sdo(uint16_t index,uint8_t subindex){
		printf("  0x%X  0x%X ",index,subindex);
		struct can_msg msg;

		creat_sdo_msg(&msg,copley_node2.node_id,SDO_READ_OD,index,subindex,NULL,0);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
}


void set_sdo(void){
		struct can_msg msg;
		uint8_t data[4] = {0};
		
		//6 Set MOTION PROFILE TYPE to Trapezoidal profile mode
		creat_sdo_msg(&msg,copley_node1.node_id,SDO_WRITE_OD,0x6086,0x00,data,2);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
}



//This is to generate desire force via now T
#define pi 3.141592653
int N =0;
float tp = 0.4, to = 0.6, Fp = 100;
float y[100] = {0.0};
void generate_force(){
	
	int np = N*tp;
	int no = N*(to - tp);
	for(int i =0; i< N; i++)
	{
		if(i < np)
		{
			y[i] = Fp*sin(i*pi/(2*np));
		}
		else if(i<(no+np))
		{
			y[i] = Fp*cos((pi*(i-np)/(2*no)));
		
		}
		else y[i] = 0;
	
	}

}






















