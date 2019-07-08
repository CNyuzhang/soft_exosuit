#include "copley_node2.h"
#include "copley_control.h"
#include "usart.h" 
#include "delay.h" 

static void node2_init(void);
copley_device copley_node2 = {
		.node_id = 0x02,
		.rpdo_id = {0x202,0x302,0x402,0x502,0x602},
		.tpdo_id = {0x182,0x282,0x382,0x482,0x492},
		.heart_beat = 0x702,
		.sdo_id = 0x582,
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
		.init = node2_init,
};



static void node2_update(can_msg_t copley_msg){
			  if(copley_msg->id == copley_node2.heart_beat){       //收到心跳帧
					  if(NODE2_DEBUG){
								printf("Receive a heartbeat msg\r\n");
						}
						copley_node2.nmt_state = copley_msg->data[0];
						
						if(NODE2_DEBUG){
								switch(copley_node2.nmt_state){
									case NMT_STA_INI:
										printf("node2: The NMT state is: INI \r\n");break;
									case NMT_STA_STOP:
										printf("node2: The NMT state is: STOP\r\n");break;
									case NMT_STA_OPER:
										printf("node2: The NMT state is: OPER\r\n");break;
									case NMT_STA_PRE_OP:
										printf("node2: The NMT state is: PRE_OP\r\n");break;
								}							
								printf("node2: The NMT state is: %d\r\n",copley_node2.nmt_state);
					 }
				}
				
				if(copley_msg->id == copley_node2.tpdo_id[0]){    //收到TPDO帧
						if(NODE2_DEBUG){
								printf("node2: Receive a TPDO0 msg\r\n");
						}
					  copley_node2.status_word = ( copley_msg->data[1]<<8 )| copley_msg->data[0];
						copley_node2.status_register = (copley_msg->data[5]<<24)|(copley_msg->data[4]<<16)|(copley_msg->data[3]<<8)|(copley_msg->data[2]);
						//printf("status word %X status register %X\r\n",copley_node2.status_word,copley_node2.status_register);
						//printf("%4X %4X %4X\r\n",copley_node2.control_word & CTL_W_PPM_INI_BIT,copley_node2.status_word & DEV_STA_SET_P_ACK,copley_node2.status_word);
						
						if(NODE2_DEBUG){
								printf("node2: Receive a TPDO0 msg\r\n");							
								switch(copley_node2.status_word & DEV_STATE_MSDK){
									case DEV_STA_NOT_RD_TO_SW_ON:
										printf("node2: The status world is: NOT_RD_TO_SW_ON \r\n");break;
									case DEV_STA_SW_ON_DISABLE:
										printf("node2: The status world is: SW_ON_DISABLE \r\n");break;
									case DEV_STA_RD_TO_SW_ON:
										printf("node2: The status world is: RD_TO_SW_ON \r\n");break;
									case DEV_STA_SWITCH_ON:
										printf("node2: The status world is: SWITCH_ON \r\n");break;
									case DEV_STA_OPER_EN:
										printf("node2: The status world is: OPER_EN \r\n");break;
									case DEV_STA_QK_STOP:
										printf("node2: The status world is: QK_STOP \r\n");break;
									case DEV_STA_FLT_RE_AC:
										printf("node2: The status world is: FLT_RE_AC \r\n");break;
									case DEV_STA_FAULT:
										printf("node2: The status world is:FAULT \r\n");break;
								}						
								printf("node2: The status world is: %X \r\n",copley_node2.status_word);
								printf("node2: The operation mode is: %d \r\n",copley_node2.operation_mode);
								printf("node2: The error code is:%X \r\n",copley_node2.error_code);
					}
				}
				
				if(copley_msg->id == copley_node2.tpdo_id[1]){					//收到TPDO帧
						if(NODE2_DEBUG){
								printf("node2: Receive a TPDO1 msg\r\n");
						}
					  copley_node2.error_code = ( copley_msg->data[1]<<8 )| copley_msg->data[0];
						copley_node2.operation_mode = copley_msg->data[2];
						//printf("error code %X operation mode %X\r\n",copley_node2.error_code,	copley_node2.operation_mode);
						
						if(NODE2_DEBUG){
								switch(copley_node2.status_word & DEV_STATE_MSDK){
									case DEV_STA_NOT_RD_TO_SW_ON:
										printf("node2: The status world is: NOT_RD_TO_SW_ON \r\n");break;
									case DEV_STA_SW_ON_DISABLE:
										printf("node2: The status world is: SW_ON_DISABLE \r\n");break;
									case DEV_STA_RD_TO_SW_ON:
										printf("node2: The status world is: RD_TO_SW_ON \r\n");break;
									case DEV_STA_SWITCH_ON:
										printf("node2: The status world is: SWITCH_ON \r\n");break;
									case DEV_STA_OPER_EN:
										printf("node2: The status world is: OPER_EN \r\n");break;
									case DEV_STA_QK_STOP:
										printf("node2: The status world is: QK_STOP \r\n");break;
									case DEV_STA_FLT_RE_AC:
										printf("node2: The status world is: FLT_RE_AC \r\n");break;
									case DEV_STA_FAULT:
										printf("node2: The status world is:FAULT \r\n");break;
								}						
								printf("node2: The status world is: %X \r\n",copley_node2.status_word);
								printf("node2: The operation mode is: %d \r\n",copley_node2.operation_mode);
								printf("node2: The error code is:%X \r\n",copley_node2.error_code);
					}
				}
				
				if(copley_msg->id == copley_node2.tpdo_id[2]){    //收到TPDO帧
						if(NODE2_DEBUG){
								printf("node2: Receive a TPDO2 msg\r\n");
						}
						
						
					  copley_node2.actual_target_p = (copley_msg->data[3]<<24 )|(copley_msg->data[2]<<16 )|( copley_msg->data[1]<<8 )| copley_msg->data[0];
						copley_node2.actual_p		 = (copley_msg->data[7]<<24)|(copley_msg->data[6]<<16)|(copley_msg->data[5]<<8)|copley_msg->data[4];
					
						//printf("actual_target_p %d actual p %d\r\n", copley_node2.actual_target_p,copley_node2.actual_p);
						
						
						if(NODE2_DEBUG){
								switch(copley_node2.status_word & DEV_STATE_MSDK){
									case DEV_STA_NOT_RD_TO_SW_ON:
										printf("node2: The status world is: NOT_RD_TO_SW_ON \r\n");break;
									case DEV_STA_SW_ON_DISABLE:
										printf("node2: The status world is: SW_ON_DISABLE \r\n");break;
									case DEV_STA_RD_TO_SW_ON:
										printf("node2: The status world is: RD_TO_SW_ON \r\n");break;
									case DEV_STA_SWITCH_ON:
										printf("node2: The status world is: SWITCH_ON \r\n");break;
									case DEV_STA_OPER_EN:
										printf("node2: The status world is: OPER_EN \r\n");break;
									case DEV_STA_QK_STOP:
										printf("node2: The status world is: QK_STOP \r\n");break;
									case DEV_STA_FLT_RE_AC:
										printf("node2: The status world is: FLT_RE_AC \r\n");break;
									case DEV_STA_FAULT:
										printf("node2: The status world is:FAULT \r\n");break;
								}						
								printf("node2: The status world is: %X \r\n",copley_node2.status_word);
								printf("node2: The operation mode is: %d \r\n",copley_node2.operation_mode);
								printf("node2: The error code is:%X \r\n",copley_node2.error_code);
								printf("node2:The actual position is:%d \r\n",copley_node2.actual_p);
					}
				}
				
				if(copley_msg->id == copley_node2.tpdo_id[3]){    //收到TPDO帧
						if(NODE2_DEBUG){
								printf("node2: Receive a TPDO3 msg\r\n");
						}
					  copley_node2.status_word = ( copley_msg->data[1]<<8 )| copley_msg->data[0];
						copley_node2.actual_v		 = (copley_msg->data[5]<<24)|(copley_msg->data[4]<<16)|(copley_msg->data[3]<<8)|copley_msg->data[2];
						//printf("actual_target_p %d actual p %d\r\n", copley_node2.actual_target_p,copley_node2.actual_p);						
						
						if(NODE2_DEBUG){
								switch(copley_node2.status_word & DEV_STATE_MSDK){
									case DEV_STA_NOT_RD_TO_SW_ON:
										printf("node2: The status world is: NOT_RD_TO_SW_ON \r\n");break;
									case DEV_STA_SW_ON_DISABLE:
										printf("node2: The status world is: SW_ON_DISABLE \r\n");break;
									case DEV_STA_RD_TO_SW_ON:
										printf("node2: The status world is: RD_TO_SW_ON \r\n");break;
									case DEV_STA_SWITCH_ON:
										printf("node2: The status world is: SWITCH_ON \r\n");break;
									case DEV_STA_OPER_EN:
										printf("node2: The status world is: OPER_EN \r\n");break;
									case DEV_STA_QK_STOP:
										printf("node2: The status world is: QK_STOP \r\n");break;
									case DEV_STA_FLT_RE_AC:
										printf("node2: The status world is: FLT_RE_AC \r\n");break;
									case DEV_STA_FAULT:
										printf("node2: The status world is:FAULT \r\n");break;
								}						
								printf("node2: The status world is: %X \r\n",copley_node2.status_word);
								printf("node2: The operation mode is: %d \r\n",copley_node2.operation_mode);
								printf("node2: The error code is:%X \r\n",copley_node2.error_code);
								printf("node2:The actual volocity is:%d \r\n",copley_node2.actual_v);
					}
				}
				
				
				if(copley_msg->id == copley_node2.tpdo_id[4]){    //收到TPDO帧
						if(NODE2_DEBUG){
								printf("node2: Receive a TPDO4 msg\r\n");
						}
					  copley_node2.status_word = ( copley_msg->data[1]<<8 )| copley_msg->data[0];
						copley_node2.actual_t		 = (copley_msg->data[5]<<24)|(copley_msg->data[4]<<16)|(copley_msg->data[3]<<8)|copley_msg->data[2];
						
						if(NODE2_DEBUG){
								switch(copley_node2.status_word & DEV_STATE_MSDK){
									case DEV_STA_NOT_RD_TO_SW_ON:
										printf("node2: The status world is: NOT_RD_TO_SW_ON \r\n");break;
									case DEV_STA_SW_ON_DISABLE:
										printf("node2: The status world is: SW_ON_DISABLE \r\n");break;
									case DEV_STA_RD_TO_SW_ON:
										printf("node2: The status world is: RD_TO_SW_ON \r\n");break;
									case DEV_STA_SWITCH_ON:
										printf("node2: The status world is: SWITCH_ON \r\n");break;
									case DEV_STA_OPER_EN:
										printf("node2: The status world is: OPER_EN \r\n");break;
									case DEV_STA_QK_STOP:
										printf("node2: The status world is: QK_STOP \r\n");break;
									case DEV_STA_FLT_RE_AC:
										printf("node2: The status world is: FLT_RE_AC \r\n");break;
									case DEV_STA_FAULT:
										printf("node2: The status world is:FAULT \r\n");break;
								}						
								printf("node2: The status world is: %X \r\n",copley_node2.status_word);
								printf("node2: The operation mode is: %d \r\n",copley_node2.operation_mode);
								printf("node2: The error code is:%X \r\n",copley_node2.error_code);
								printf("node2:The actual torque is:%d \r\n",copley_node2.actual_t);
					}
				}
				
				if(copley_msg->id == copley_node2.sdo_id ){  //收到SDO响应
						if(NODE2_DEBUG){
								printf("node2: Receive a SDO respon\r\n");
								printf("node2: id:0x%X \r\r\n",copley_msg->id);
								printf("node2: ide:0x%X \r\r\n",copley_msg->ide);
								printf("node2: rtr:0x%X \r\r\n",copley_msg->rtr);
								printf("node2: rlc:0x%X \r\r\n",copley_msg->len);		  
								for(u8 i=0;i<8;i++){
									printf("node2: data[%d]: 0x%X \r\r\n",i,copley_msg->data[i]);
								}
					 }
				}

}


static void other_init_msg(void)
{
		struct can_msg msg;
		uint8_t data[4] = {0x02,0x06,0x00,0x00}; //设置RPDO4的cob-id
		creat_sdo_msg(&msg,copley_node2.node_id,SDO_WRITE_OD,0x1404,0x01,data,sizeof(data));
		//rt_device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct can_msg));	
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
//		pdo_disable(copley_node2.node_id,0x1800,0x1A00);   //映射error register
//		pdo_map(copley_node2.node_id,0x1A00,0x02,0x10020020);
//		pdo_enable(copley_node2.node_id,0x1800,0x0182,0x1A00,2);
//		
//		pdo_disable(copley_node2.node_id,0x1801,0x1A01);   //映射error code
//		pdo_map(copley_node2.node_id,0x1A01,0x01,0x603F0010);
//		pdo_enable(copley_node2.node_id,0x1801,0x0282,0x1A01,2);
//		
//		
//		pdo_disable(copley_node2.node_id,0x1802,0x1A02);   //设置rpdo2的type
//		pdo_set_type(copley_node2.node_id,0x1802,0xFF);
//		pdo_map(copley_node2.node_id,0x1A02,0x01,0x607A0020);  //map target position
//		pdo_enable(copley_node2.node_id,0x1802,0x0382,0x1A02,2);		
		
}

static void node2_start(uint8_t mode)
{
	  //printf("node2: start\r\n");
		struct can_msg msg;
		creat_nmt_msg(&msg,copley_node2.node_id,NMT_START_NODE); //start node 进入optional 状态
		//rt_device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
	
		other_init_msg();

		uint8_t rpdo_data[8]={0};
			
		copley_node2.operation_mode = mode;
		copley_node2.control_word = CMD_EN_VOL;

		rpdo_data[0] = copley_node2.control_word & 0xFF;  //CMD_EN_VOL
		rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node2.operation_mode;
		create_rpdo_msg(&msg,copley_node2.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);	
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);


		copley_node2.control_word = CMD_SW_ON;
		rpdo_data[0] = copley_node2.control_word & 0xFF;  //CMD_SW_ON
		rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node2.operation_mode;;
		create_rpdo_msg(&msg,copley_node2.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
    CAN1_Send_Msg(&msg,8);
		delay_ms(5);		
		
		copley_node2.control_word = CMD_EN_OP;
		rpdo_data[0] = copley_node2.control_word & 0xFF;  //CMD_EN_OP
		rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node2.operation_mode;
		create_rpdo_msg(&msg,copley_node2.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
	
}

static void node2_fault_reset(void)
{
		struct can_msg msg;
		uint8_t rpdo_data[8]={0};
		copley_node2.control_word = FAULT_RST;
		rpdo_data[0] = copley_node2.control_word & 0xFF;  //FAULT_RST
		rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node2.operation_mode;
		create_rpdo_msg(&msg,copley_node2.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
}



int node2_set_target(uint8_t target, int32_t val)
{

			struct can_msg msg;
		  uint8_t rpdo_data[8]={0};			
			if(target != TARGET_CSP && target != TARGET_CSV && target!= TARGET_CST){
					printf(" Invalid arg \r\n");
					return -1;						
			}else{
					switch(target){
						case(TARGET_CSP):{
								copley_node2.actual_p	= val;
								rpdo_data[0] = copley_node2.control_word & 0xFF; 
								rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;
								rpdo_data[2] = copley_node2.actual_p & 0xFF;
								rpdo_data[3] = (copley_node2.actual_p >> 8) & 0xFF;
								rpdo_data[4] = (copley_node2.actual_p >> 16) & 0xFF;
								rpdo_data[5] = (copley_node2.actual_p >> 24) & 0xFF;
								create_rpdo_msg(&msg,copley_node2.rpdo_id[2],rpdo_data,6);	
								//rt_device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct can_msg));
							CAN1_Send_Msg(&msg,8);

						};break;
						case(TARGET_CSV):{
								copley_node2.actual_v	= val;
								rpdo_data[0] = copley_node2.control_word & 0xFF; 
								rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;
								rpdo_data[2] = copley_node2.actual_v & 0xFF;
								rpdo_data[3] = (copley_node2.actual_v >> 8) & 0xFF;
								rpdo_data[4] = (copley_node2.actual_v >> 16) & 0xFF;
								rpdo_data[5] = (copley_node2.actual_v >> 24) & 0xFF;
								create_rpdo_msg(&msg,copley_node2.rpdo_id[3],rpdo_data,6);	
								//rt_device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct can_msg));	
                CAN1_Send_Msg(&msg,8);							
								
						};break;
						case(TARGET_CST):{
								copley_node2.actual_t	= val;
								rpdo_data[0] = copley_node2.control_word & 0xFF; 
								rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;
								rpdo_data[2] = copley_node2.actual_t & 0xFF;
								rpdo_data[3] = (copley_node2.actual_t >> 8) & 0xFF;
								rpdo_data[4] = (copley_node2.actual_t >> 16) & 0xFF;
								rpdo_data[5] = (copley_node2.actual_t >> 24) & 0xFF;
								create_rpdo_msg(&msg,copley_node2.rpdo_id[4],rpdo_data,6);	
								//rt_device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));		
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
			
	  //rt_device_write(copley_node2.copley_dev , -1 , (void*)&epos_msg ,sizeof(struct rt_can_msg));
    CAN1_Send_Msg(&epos_msg,8);			

		return 0;
}

//发送命令和设置工作模式（通过RPDO）
static void node2_nmt_reset(void){
		set_nmt_state( copley_node2.node_id, RESET_NODE );

}


static void node2_state_reset(void)
{
		struct can_msg msg;
		uint8_t rpdo_data[8]={0};
		copley_node2.control_word = CMD_DIS_VOL;
		rpdo_data[0] = copley_node2.control_word & 0xFF;  //CMD_DIS_VOL
		rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node2.operation_mode;
		create_rpdo_msg(&msg,copley_node2.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
}

static void node2_quick_stop(void)
{
		struct can_msg msg;
		uint8_t rpdo_data[8]={0};
		copley_node2.control_word = CMD_QK_STOP;
		rpdo_data[0] = copley_node2.control_word & 0xFF;  //CMD_QK_STOP
		rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;
		rpdo_data[2] = copley_node2.operation_mode;
		create_rpdo_msg(&msg,copley_node2.rpdo_id[1],rpdo_data,3);
		//rt_device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
	
}

static void node2_set_op_mode(uint8_t op_mode)
{
		node2_state_reset();
	  node2_start(op_mode);
		
}


static void send_sync(void)
{
		struct can_msg msg;
	  creat_sync_msg((can_msg_t)&msg);
	  //device_write(copley_node2.copley_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		CAN1_Send_Msg(&msg,8);
}


static void node2_ppm_init(void)
{
		struct can_msg msg;
		uint8_t data[4] = {0};
		uint8_t rpdo_data[8] = {0};
		
		//6 Set MOTION PROFILE TYPE to Trapezoidal profile mode，梯形模式
		creat_sdo_msg(&msg,copley_node2.node_id,SDO_WRITE_OD,0x6086,0x00,data,2);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);

		//设置change immediately 为 1	,立刻改变目标
		copley_node2.control_word = copley_node2.control_word | CTL_W_PPM_CHA_IMM_BIT; 	//设置change immediately 为 1	
		rpdo_data[0] = copley_node2.control_word & 0xFF; 
		rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;			
		create_rpdo_msg(&msg,copley_node2.rpdo_id[0],rpdo_data,2);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);

		//设置adb/rel 为 0	,绝对位置
		copley_node2.control_word = copley_node2.control_word & (~CTL_W_PPM_ABS_REL_BIT); //设置adb/rel 为 0
		rpdo_data[0] = copley_node2.control_word & 0xFF; 
		rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;			
		create_rpdo_msg(&msg,copley_node2.rpdo_id[0],rpdo_data,2);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);		
}


void node2_ppm_set_target(int target)
{
		struct can_msg msg;
		uint8_t rpdo_data[8] = {0};
		
		copley_node2.set_target(1,target);  //设置目标		
		
		//如果状态字set point ack位为1 或 控制字 new set point位为1 则将new set point 设为0
		//printf("Node2: %X %X\r\n",copley_node2.status_word & DEV_STA_SET_P_ACK,copley_node2.control_word & CTL_W_PPM_INI_BIT);
		//if((copley_node2.status_word & DEV_STA_SET_P_ACK) || (copley_node2.control_word & CTL_W_PPM_INI_BIT)){
			//	printf("Node2: %X %X\r\n",copley_node2.status_word & DEV_STA_SET_P_ACK,copley_node2.control_word & CTL_W_PPM_INI_BIT);	
				//printf("%d %d\r\n",copley_node2.status_word & DEV_STA_SET_P_ACK,copley_node2.control_word & CTL_W_PPM_INI_BIT);		
				copley_node2.control_word = copley_node2.control_word & (~CTL_W_PPM_INI_BIT); //则将new set point 设为0
				rpdo_data[0] = copley_node2.control_word & 0xFF; 
				rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;			
				create_rpdo_msg(&msg,copley_node2.rpdo_id[0],rpdo_data,2);
				CAN1_Send_Msg(&msg,8);
				delay_ms(2);
		//}
		
		//设置new set point  为 1	,启动电机
		copley_node2.control_word = copley_node2.control_word | CTL_W_PPM_INI_BIT; //设置new set point  为 1
		rpdo_data[0] = copley_node2.control_word & 0xFF; 
		rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;			
		create_rpdo_msg(&msg,copley_node2.rpdo_id[0],rpdo_data,2);
		CAN1_Send_Msg(&msg,8);			
				
}

void node2_halt(void)
{
		struct can_msg msg;
		uint8_t rpdo_data[8] = {0};	
		//设置halt  为 1	,停止电机
		copley_node2.control_word = copley_node2.control_word | CTL_W_HALT_BIT; //设置halt  为 1
		rpdo_data[0] = copley_node2.control_word & 0xFF; 
		rpdo_data[1] = (copley_node2.control_word >> 8) & 0xFF;			
		create_rpdo_msg(&msg,copley_node2.rpdo_id[0],rpdo_data,2);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
}


//static void node2_init(device_t dev)	
static void node2_init(void)
{
		//copley_node2.copley_dev = dev;
		copley_node2.update = node2_update;
		copley_node2.start = node2_start;
		copley_node2.falut_reset = node2_fault_reset;
		copley_node2.set_target = node2_set_target;
		copley_node2.nmt_reset = node2_nmt_reset;
		copley_node2.state_reset = node2_state_reset;
		copley_node2.quick_stop = node2_quick_stop;
		copley_node2.set_op_mode = node2_set_op_mode;
		copley_node2.sync = send_sync;
		copley_node2.ppm_init = node2_ppm_init;
		copley_node2.ppm_set_target = node2_ppm_set_target;	
		copley_node2.halt = node2_halt;
}



