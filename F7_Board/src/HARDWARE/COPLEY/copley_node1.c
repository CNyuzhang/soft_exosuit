#include "copley_node1.h"
#include "copley_control.h"
#include "usart.h" 
#include "delay.h" 

static void node1_init(void);
copley_device copley_node1 = {
		.node_id = 0x01,
		.rpdo_id = {0x201,0x301,0x401,0x501,0x601},
		.tpdo_id = {0x180,0x281,0x381,0x481,0x491},
		.heart_beat = 0x701,
		.sdo_id = 0x581,
		//.copley_dev = NULL,
		.control_word = 0,
		.status_word = 0,
		.nmt_state = 0xFF,
		.operation_mode = 0,
		.error_code = 0,
		.csp_target = 0,
		.csv_target = 0,
		.cst_target = 0,
		.actual_p = 0,
		.actual_v =0,
		.actual_t = 0,
		.init = node1_init,
};



static void node1_update(can_msg_t copley_msg){
			  if(copley_msg->id == copley_node1.heart_beat){					//收到心跳帧
						if(NODE1_DEBUG){
							printf("Receive a heartbeat msg\r\n");
						}
						
						copley_node1.nmt_state = copley_msg->data[0];
						
						if(NODE1_DEBUG){
							switch(copley_node1.nmt_state){
								case NMT_STA_INI:
									printf("Node1: The NMT state is: INI \r\n");break;
								case NMT_STA_STOP:
									printf("Node1: The NMT state is: STOP\r\n");break;
								case NMT_STA_OPER:
									printf("Node1: The NMT state is: OPER\r\n");break;
								case NMT_STA_PRE_OP:
									printf("Node1: The NMT state is: PRE_OP\r\n");break;
							}						
						  printf("Node1: The NMT state is: %d\r\n",copley_node1.nmt_state);
					 }
				}
				if(copley_msg->id == copley_node1.tpdo_id[0]){					//收到TPDO帧
						if(NODE1_DEBUG){
							printf("Node1: Receive a TPDO0 msg\r\n");
					  }
						
					  copley_node1.status_word = ( copley_msg->data[1]<<8 )| copley_msg->data[0];
						copley_node1.status_register = (copley_msg->data[5]<<24)|(copley_msg->data[4]<<16)|(copley_msg->data[3]<<8)|(copley_msg->data[2]);
						//printf("status word %X status register %X\r\n",copley_node1.status_word,copley_node1.status_register);
						
						if(NODE1_DEBUG){
							switch(copley_node1.status_word & DEV_STATE_MSDK){
								case DEV_STA_NOT_RD_TO_SW_ON:
									printf("Node1: The status world is: NOT_RD_TO_SW_ON \r\n");break;
								case DEV_STA_SW_ON_DISABLE:
									printf("Node1: The status world is: SW_ON_DISABLE \r\n");break;
								case DEV_STA_RD_TO_SW_ON:
									printf("Node1: The status world is: RD_TO_SW_ON \r\n");break;
								case DEV_STA_SWITCH_ON:
									printf("Node1: The status world is: SWITCH_ON \r\n");break;
								case DEV_STA_OPER_EN:
									printf("Node1: The status world is: OPER_EN \r\n");break;
								case DEV_STA_QK_STOP:
									printf("Node1: The status world is: QK_STOP \r\n");break;
								case DEV_STA_FLT_RE_AC:
									printf("Node1: The status world is: FLT_RE_AC \r\n");break;
								case DEV_STA_FAULT:
									printf("Node1: The status world is:FAULT \r\n");break;
							}						
							printf("Node1: The status world is: %X \r\n",copley_node1.status_word);
							printf("Node1: The operation mode is: %d \r\n",copley_node1.operation_mode);
							printf("Node1: The error code is:%X \r\n",copley_node1.error_code);
					}
				}
				
				if(copley_msg->id == copley_node1.tpdo_id[1]){    //收到TPDO帧
					  if(NODE1_DEBUG){
								printf("Node1: Receive a TPDO1 msg\r\n");
						}
						
						
					  copley_node1.error_code = ( copley_msg->data[1]<<8 )| copley_msg->data[0];
						copley_node1.operation_mode = copley_msg->data[2];
						//printf("error code %X operation mode %X\r\n",copley_node1.error_code,	copley_node1.operation_mode);
						
						
						if(NODE1_DEBUG){
								switch(copley_node1.status_word & DEV_STATE_MSDK){
									case DEV_STA_NOT_RD_TO_SW_ON:
										printf("Node1: The status world is: NOT_RD_TO_SW_ON \r\n");break;
									case DEV_STA_SW_ON_DISABLE:
										printf("Node1: The status world is: SW_ON_DISABLE \r\n");break;
									case DEV_STA_RD_TO_SW_ON:
										printf("Node1: The status world is: RD_TO_SW_ON \r\n");break;
									case DEV_STA_SWITCH_ON:
										printf("Node1: The status world is: SWITCH_ON \r\n");break;
									case DEV_STA_OPER_EN:
										printf("Node1: The status world is: OPER_EN \r\n");break;
									case DEV_STA_QK_STOP:
										printf("Node1: The status world is: QK_STOP \r\n");break;
									case DEV_STA_FLT_RE_AC:
										printf("Node1: The status world is: FLT_RE_AC \r\n");break;
									case DEV_STA_FAULT:
										printf("Node1: The status world is:FAULT \r\n");break;
								}						
								printf("Node1: The status world is: %X \r\n",copley_node1.status_word);
								printf("Node1: The operation mode is: %d \r\n",copley_node1.operation_mode);
								printf("Node1: The error code is:%X \r\n",copley_node1.error_code);
					}
				}
				
				if(copley_msg->id == copley_node1.tpdo_id[2]){    //收到TPDO帧
					  if(NODE1_DEBUG){
								printf("Node1: Receive a TPDO2 msg\r\n");
						}
					  copley_node1.actual_target_p = (copley_msg->data[3]<<24 )|(copley_msg->data[2]<<16 )|( copley_msg->data[1]<<8 )| copley_msg->data[0];
						copley_node1.actual_p		 = (copley_msg->data[7]<<24)|(copley_msg->data[6]<<16)|(copley_msg->data[5]<<8)|copley_msg->data[4];
					
						//printf("actual_target_p %d actual p %d\r\n", copley_node1.actual_target_p,copley_node1.actual_p);
					  if(NODE1_DEBUG){					
								switch(copley_node1.status_word & DEV_STATE_MSDK){
									case DEV_STA_NOT_RD_TO_SW_ON:
										printf("Node1: The status world is: NOT_RD_TO_SW_ON \r\n");break;
									case DEV_STA_SW_ON_DISABLE:
										printf("Node1: The status world is: SW_ON_DISABLE \r\n");break;
									case DEV_STA_RD_TO_SW_ON:
										printf("Node1: The status world is: RD_TO_SW_ON \r\n");break;
									case DEV_STA_SWITCH_ON:
										printf("Node1: The status world is: SWITCH_ON \r\n");break;
									case DEV_STA_OPER_EN:
										printf("Node1: The status world is: OPER_EN \r\n");break;
									case DEV_STA_QK_STOP:
										printf("Node1: The status world is: QK_STOP \r\n");break;
									case DEV_STA_FLT_RE_AC:
										printf("Node1: The status world is: FLT_RE_AC \r\n");break;
									case DEV_STA_FAULT:
										printf("Node1: The status world is:FAULT \r\n");break;
								}						
								printf("Node1: The status world is: %X \r\n",copley_node1.status_word);
								printf("Node1: The operation mode is: %d \r\n",copley_node1.operation_mode);
								printf("Node1: The error code is:%X \r\n",copley_node1.error_code);
								printf("Node1:The actual position is:%d \r\n",copley_node1.actual_p);
					}
				}
				
				if(copley_msg->id == copley_node1.tpdo_id[3]){    //收到TPDO帧
						if(NODE1_DEBUG){
								printf("Node1: Receive a TPDO3 msg\r\n");
						}
						
						
					  copley_node1.status_word = ( copley_msg->data[1]<<8 )| copley_msg->data[0];
						copley_node1.actual_v		 = (copley_msg->data[5]<<24)|(copley_msg->data[4]<<16)|(copley_msg->data[3]<<8)|copley_msg->data[2];
						
						if(NODE1_DEBUG){
								switch(copley_node1.status_word & DEV_STATE_MSDK){
									case DEV_STA_NOT_RD_TO_SW_ON:
										printf("Node1: The status world is: NOT_RD_TO_SW_ON \r\n");break;
									case DEV_STA_SW_ON_DISABLE:
										printf("Node1: The status world is: SW_ON_DISABLE \r\n");break;
									case DEV_STA_RD_TO_SW_ON:
										printf("Node1: The status world is: RD_TO_SW_ON \r\n");break;
									case DEV_STA_SWITCH_ON:
										printf("Node1: The status world is: SWITCH_ON \r\n");break;
									case DEV_STA_OPER_EN:
										printf("Node1: The status world is: OPER_EN \r\n");break;
									case DEV_STA_QK_STOP:
										printf("Node1: The status world is: QK_STOP \r\n");break;
									case DEV_STA_FLT_RE_AC:
										printf("Node1: The status world is: FLT_RE_AC \r\n");break;
									case DEV_STA_FAULT:
										printf("Node1: The status world is:FAULT \r\n");break;
								}						
								printf("Node1: The status world is: %X \r\n",copley_node1.status_word);
								printf("Node1: The operation mode is: %d \r\n",copley_node1.operation_mode);
								printf("Node1: The error code is:%X \r\n",copley_node1.error_code);
								printf("Node1:The actual volocity is:%d \r\n",copley_node1.actual_v);
					}
				}
				
				
				if(copley_msg->id == copley_node1.tpdo_id[4]){					//收到TPDO帧
						if(NODE1_DEBUG){
								printf("Node1: Receive a TPDO4 msg\r\n");
						}
					  copley_node1.status_word = ( copley_msg->data[1]<<8 )| copley_msg->data[0];
						copley_node1.actual_t		 = (copley_msg->data[5]<<24)|(copley_msg->data[4]<<16)|(copley_msg->data[3]<<8)|copley_msg->data[2];
						
						if(NODE1_DEBUG){
								switch(copley_node1.status_word & DEV_STATE_MSDK){
									case DEV_STA_NOT_RD_TO_SW_ON:
										printf("Node1: The status world is: NOT_RD_TO_SW_ON \r\n");break;
									case DEV_STA_SW_ON_DISABLE:
										printf("Node1: The status world is: SW_ON_DISABLE \r\n");break;
									case DEV_STA_RD_TO_SW_ON:
										printf("Node1: The status world is: RD_TO_SW_ON \r\n");break;
									case DEV_STA_SWITCH_ON:
										printf("Node1: The status world is: SWITCH_ON \r\n");break;
									case DEV_STA_OPER_EN:
										printf("Node1: The status world is: OPER_EN \r\n");break;
									case DEV_STA_QK_STOP:
										printf("Node1: The status world is: QK_STOP \r\n");break;
									case DEV_STA_FLT_RE_AC:
										printf("Node1: The status world is: FLT_RE_AC \r\n");break;
									case DEV_STA_FAULT:
										printf("Node1: The status world is:FAULT \r\n");break;
								}						
								printf("Node1: The status world is: %X \r\n",copley_node1.status_word);
								printf("Node1: The operation mode is: %d \r\n",copley_node1.operation_mode);
								printf("Node1: The error code is:%X \r\n",copley_node1.error_code);
								printf("Node1:The actual torque is:%d \r\n",copley_node1.actual_t);
					}
				}
				
				if(copley_msg->id == copley_node1.sdo_id ){  //收到SDO响应
						if(NODE1_DEBUG){
								printf("Node1: Receive a SDO respon\r\n");
								printf("Node1: id:0x%X \r\r\n",copley_msg->id);
								printf("Node1: ide:0x%X \r\r\n",copley_msg->ide);
								printf("Node1: rtr:0x%X \r\r\n",copley_msg->rtr);
								printf("Node1: rlc:0x%X \r\r\n",copley_msg->len);		  
								for(u8 i=0;i<8;i++){
									printf("Node1: data[%d]: 0x%X \r\r\n",i,copley_msg->data[i]);
								}
					  }
				}

}


static void other_init_msg(void)
{
		struct can_msg msg;
		uint8_t data[4] = {0x01,0x06,0x00,0x00}; //设置RPDO4的cob-id
		creat_sdo_msg(&msg,copley_node1.node_id,SDO_WRITE_OD,0x1404,0x01,data,sizeof(data));
		//rt_device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct can_msg));	
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
		
//		pdo_disable(copley_node1.node_id,0x1800,0x1A00);   //映射error register
//		pdo_map(copley_node1.node_id,0x1A00,0x02,0x10020020);
//		pdo_enable(copley_node1.node_id,0x1800,0x0181,0x1A00,2);
//		
//		pdo_disable(copley_node1.node_id,0x1801,0x1A01);   //映射error code
//		pdo_map(copley_node1.node_id,0x1A01,0x01,0x603F0010);
//		pdo_enable(copley_node1.node_id,0x1801,0x0281,0x1A01,2);
//		
//		pdo_disable(copley_node1.node_id,0x1802,0x1A02);   //设置rpdo2的type
//		pdo_set_type(copley_node1.node_id,0x1802,0xFF);
//		pdo_enable(copley_node1.node_id,0x1802,0x0381,0x1A02,2);		
		
		
		
		
}

static void node1_start(uint8_t mode)
{
	  //printf("Node1: start\r\n");
		struct can_msg msg;
		creat_nmt_msg(&msg,copley_node1.node_id,NMT_START_NODE); //start node 进入optional 状态
		//rt_device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
	

		other_init_msg();

		uint8_t rpdo_data[8]={0};
			
		copley_node1.operation_mode = mode;
		copley_node1.control_word = CMD_EN_VOL;

		rpdo_data[0] = copley_node1.control_word & 0xFF;  //CMD_EN_VOL
		rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node1.operation_mode;
		create_rpdo_msg(&msg,copley_node1.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);	
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);


		copley_node1.control_word = CMD_SW_ON;
		rpdo_data[0] = copley_node1.control_word & 0xFF;  //CMD_SW_ON
		rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node1.operation_mode;;
		create_rpdo_msg(&msg,copley_node1.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
    CAN1_Send_Msg(&msg,8);
		delay_ms(5);		
		
		copley_node1.control_word = CMD_EN_OP;
		rpdo_data[0] = copley_node1.control_word & 0xFF;  //CMD_EN_OP
		rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node1.operation_mode;
		create_rpdo_msg(&msg,copley_node1.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);

	
}

static void node1_fault_reset(void)
{
		struct can_msg msg;
		uint8_t rpdo_data[8]={0};
		copley_node1.control_word = FAULT_RST;
		rpdo_data[0] = copley_node1.control_word & 0xFF;  //FAULT_RST
		rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node1.operation_mode;
		create_rpdo_msg(&msg,copley_node1.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
}



int node1_set_target(uint8_t target, int32_t val)
{

			struct can_msg msg;
		  uint8_t rpdo_data[8]={0};			
			if(target != TARGET_CSP && target != TARGET_CSV && target!= TARGET_CST){
					printf(" Invalid arg \r\n");
					return -1;						
			}else{
					switch(target){
						case(TARGET_CSP):{
								copley_node1.actual_p	= val;
								rpdo_data[0] = copley_node1.control_word & 0xFF; 
								rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;
								rpdo_data[2] = copley_node1.actual_p & 0xFF;
								rpdo_data[3] = (copley_node1.actual_p >> 8) & 0xFF;
								rpdo_data[4] = (copley_node1.actual_p >> 16) & 0xFF;
								rpdo_data[5] = (copley_node1.actual_p >> 24) & 0xFF;
								create_rpdo_msg(&msg,copley_node1.rpdo_id[2],rpdo_data,6);	
								//rt_device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct can_msg));
							CAN1_Send_Msg(&msg,8);

						};break;
						case(TARGET_CSV):{
								copley_node1.actual_v	= val;
								rpdo_data[0] = copley_node1.control_word & 0xFF; 
								rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;
								rpdo_data[2] = copley_node1.actual_v & 0xFF;
								rpdo_data[3] = (copley_node1.actual_v >> 8) & 0xFF;
								rpdo_data[4] = (copley_node1.actual_v >> 16) & 0xFF;
								rpdo_data[5] = (copley_node1.actual_v >> 24) & 0xFF;
								create_rpdo_msg(&msg,copley_node1.rpdo_id[3],rpdo_data,6);	
								//rt_device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct can_msg));	
                CAN1_Send_Msg(&msg,8);							
								
						};break;
						case(TARGET_CST):{
								copley_node1.actual_t	= val;
								rpdo_data[0] = copley_node1.control_word & 0xFF; 
								rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;
								rpdo_data[2] = copley_node1.actual_t & 0xFF;
								rpdo_data[3] = (copley_node1.actual_t >> 8) & 0xFF;
								rpdo_data[4] = (copley_node1.actual_t >> 16) & 0xFF;
								rpdo_data[5] = (copley_node1.actual_t >> 24) & 0xFF;
								create_rpdo_msg(&msg,copley_node1.rpdo_id[4],rpdo_data,6);	
								//rt_device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));		
                CAN1_Send_Msg(&msg,8);								
						};break;						
					}
					
		 }
		return 0;		
		
}

//发送NMT帧
static int set_nmt_state(uint8_t node_id, uint8_t op)
{
	  struct can_msg epos_msg;
	
		if( op!=NMT_START_NODE && op!=NMT_STOP_NODE && op!=NMT_ENTER_OP && op!=RESET_NODE && op!=ERSET_COMMU ){
				  printf(" Invalid arg \r\n");
				  return -1;
		}
		
		creat_nmt_msg((can_msg_t)&epos_msg,node_id,op);
			
	  //rt_device_write(copley_node1.copley_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));
    CAN1_Send_Msg(&epos_msg,8);			

		return 0;
}

//发送命令和设置工作模式（通过RPDO）
static void node1_nmt_reset(void){
		set_nmt_state( copley_node1.node_id, RESET_NODE );

}


static void node1_state_reset(void)
{
		struct can_msg msg;
		uint8_t rpdo_data[8]={0};
		copley_node1.control_word = CMD_DIS_VOL;
		rpdo_data[0] = copley_node1.control_word & 0xFF;  //CMD_DIS_VOL
		rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node1.operation_mode;
		create_rpdo_msg(&msg,copley_node1.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
}

static void node1_quick_stop(void)
{
		struct can_msg msg;
		uint8_t rpdo_data[8]={0};
		copley_node1.control_word = CMD_QK_STOP;
		rpdo_data[0] = copley_node1.control_word & 0xFF;  //CMD_QK_STOP
		rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node1.operation_mode;
		create_rpdo_msg(&msg,copley_node1.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
	
}

static void node1_set_op_mode(uint8_t op_mode)
{
		node1_state_reset();
	  node1_start(op_mode);
		
}


static void send_sync(void)
{
		struct can_msg msg;
	  creat_sync_msg((can_msg_t)&msg);
	  //device_write(copley_node1.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		CAN1_Send_Msg(&msg,8);
}


static void node1_ppm_init(void)
{
		struct can_msg msg;
		uint8_t data[4] = {0};
		uint8_t rpdo_data[8] = {0};
		
		//6 Set MOTION PROFILE TYPE to Trapezoidal profile mode，梯形模式
		creat_sdo_msg(&msg,copley_node1.node_id,SDO_WRITE_OD,0x6086,0x00,data,2);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);

		//设置change immediately 为 1	,立刻改变目标
		copley_node1.control_word = copley_node1.control_word | CTL_W_PPM_CHA_IMM_BIT; 	//设置change immediately 为 1	
		rpdo_data[0] = copley_node1.control_word & 0xFF; 
		rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;			
		create_rpdo_msg(&msg,copley_node1.rpdo_id[0],rpdo_data,2);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);

		//设置adb/rel 为 0	,绝对位置
		copley_node1.control_word = copley_node1.control_word & (~CTL_W_PPM_ABS_REL_BIT); //设置adb/rel 为 0
		rpdo_data[0] = copley_node1.control_word & 0xFF; 
		rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;			
		create_rpdo_msg(&msg,copley_node1.rpdo_id[0],rpdo_data,2);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);		
}


void node1_ppm_set_target(int target)
{
		struct can_msg msg;
		uint8_t rpdo_data[8] = {0};
		
		copley_node1.set_target(1,target);  //设置目标		
		
		//如果状态字set point ack位为1 或 控制字 new set point位为1 则将new set point 设为0
		//if((copley_node1.status_word & DEV_STA_SET_P_ACK) || (copley_node1.control_word & CTL_W_PPM_INI_BIT)){
				//printf("%d %d\r\n",copley_node1.status_word & DEV_STA_SET_P_ACK,copley_node1.control_word & CTL_W_PPM_INI_BIT);
				copley_node1.control_word = copley_node1.control_word & (~CTL_W_PPM_INI_BIT); //则将new set point 设为0
				rpdo_data[0] = copley_node1.control_word & 0xFF; 
				rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;			
				create_rpdo_msg(&msg,copley_node1.rpdo_id[0],rpdo_data,2);
				CAN1_Send_Msg(&msg,8);
				delay_ms(2);
		//}
		
		//设置new set point  为 1	,启动电机
		copley_node1.control_word = copley_node1.control_word | CTL_W_PPM_INI_BIT; //设置new set point  为 1
		rpdo_data[0] = copley_node1.control_word & 0xFF; 
		rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;			
		create_rpdo_msg(&msg,copley_node1.rpdo_id[0],rpdo_data,2);
		CAN1_Send_Msg(&msg,8);			
				
}


void node1_halt(void)
{
		struct can_msg msg;
		uint8_t rpdo_data[8] = {0};	
		//设置halt  为 1	,停止电机
		copley_node1.control_word = copley_node1.control_word | CTL_W_HALT_BIT; //设置halt  为 1
		rpdo_data[0] = copley_node1.control_word & 0xFF; 
		rpdo_data[1] = (copley_node1.control_word >> 8) & 0xFF;			
		create_rpdo_msg(&msg,copley_node1.rpdo_id[0],rpdo_data,2);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
}







//static void node1_init(device_t dev)	
static void node1_init(void)
{
		//copley_node1.copley_dev = dev;
		copley_node1.update = node1_update;
		copley_node1.start = node1_start;
		copley_node1.falut_reset = node1_fault_reset;
		copley_node1.set_target = node1_set_target;
		copley_node1.nmt_reset = node1_nmt_reset;
		copley_node1.state_reset = node1_state_reset;
		copley_node1.quick_stop = node1_quick_stop;
		copley_node1.set_op_mode = node1_set_op_mode;
		copley_node1.sync = send_sync;
		copley_node1.ppm_init = node1_ppm_init;
		copley_node1.ppm_set_target = node1_ppm_set_target;
		copley_node1.halt = node1_halt;
		
}



