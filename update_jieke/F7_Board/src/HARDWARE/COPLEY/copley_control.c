#include "copley_control.h"
#include <stdlib.h>
#include "usart.h" 
#include "can.h"
#include "delay.h"

/*帧构造函数*/


//构造NMT数据
void creat_nmt_msg(can_msg_t msg,uint32_t node_id,uint8_t op)
{
		msg->id = 0x00;
	  msg->rtr = 0;
	  msg->ide = 0;
	  msg->len = 2;
	  msg->data[0] = op;
	  msg->data[1] =  node_id;
}

//构造SYNC帧
void creat_sync_msg(can_msg_t msg)
{
		msg->id = 0x80;
		msg->rtr = 0;
		msg->len = 0;
	  msg->ide = 0;
	  for(u8 i=0;i < 8;i++ ){
			msg->data[i] = 0x00;
		}
}

void send_sync(void)
{
		struct can_msg msg;
		creat_sync_msg(&msg);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);		
}

//构造RPDO帧
void create_rpdo_msg(can_msg_t msg,
	                   uint32_t cob_id,
										 uint8_t* buf,
                     uint8_t len)
{
		msg->id = cob_id;
		msg->rtr = 0;
	  msg->ide = 0;
	  if( len <= 8 ){
			msg->len = len;
			for(u8 i = 0; i < len; i++ ){
				msg->data[i] = buf[i];
			}
		}else{
			printf("PDO data should less than 8 bytes\n");
		}
}


//构造SDO数据帧
void creat_sdo_msg(can_msg_t msg,   //信息指针
	                 uint32_t node_id,//node id
                   uint32_t op, //读或写
                   uint16_t index, //索引
                   uint8_t subindex,//子索引
                   uint8_t* buf,uint8_t len)//数据和长度
{
		msg->ide = 0;
	  msg->rtr = 0;
	  msg->len = 8;
		switch(op){
			case SDO_READ_OD:{
				msg->id = 0x600+node_id;
				msg->data[0] = 0x40;
				msg->data[1] = index & 0x00FF;
				msg->data[2] = ( index >> 8 );
				msg->data[3] = subindex;

				msg->data[4] = 0x00;
				msg->data[5] = 0x00;
				msg->data[6] = 0x00;
				msg->data[7] = 0x00;
			};break;
			case SDO_WRITE_OD:{
				if(len <= 4){
					msg->id = 0x600+node_id;
					msg->data[0] = 0x23 | ( ( (4-len) << 2 ) & 0x0C );
				  msg->data[1] = index & 0x00FF;
				  msg->data[2] = ( index >> 8 );
				  msg->data[3] = subindex;
					msg->data[4] = buf[0];
					msg->data[5] = buf[1];
					msg->data[6] = buf[2];
					msg->data[7] = buf[3];
				}else{
					printf("Too much data!\n");
				}
			};break;
		}
}


void pdo_disable(uint8_t node_id,uint16_t id_index,uint16_t n_index)
{
		struct can_msg msg;
		uint8_t data[4] = {0x00,0x00,0x00,0x80};   //COB-ID失能
		creat_sdo_msg(&msg,node_id,SDO_WRITE_OD,id_index,0x01,data,4);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);		
		
		uint8_t data2[4] = {0x00,0x00,0x00,0x00};   //Num of map失能
		creat_sdo_msg(&msg,node_id,SDO_WRITE_OD,n_index,0x00,data2,1);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);		
	
}


void pdo_map(uint8_t node_id,uint16_t index,uint8_t sub_index,uint32_t map_data)
{
		struct can_msg msg;
		uint8_t data[8]={0};
		
		data[0] = map_data&0xFF;
		data[1] = (map_data>>8)&0xFF;
		data[2] = (map_data>>16)&0xFF;
		data[3] = (map_data>>24)&0xFF;
		creat_sdo_msg(&msg,node_id,SDO_WRITE_OD,index,sub_index,data,4);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);		
	
}


void pdo_enable(uint8_t node_id,uint16_t id_index,uint16_t cob_id,uint16_t n_index,uint8_t num)
{
		struct can_msg msg;
		uint8_t data[4] = {0};
  	
		data[0] = cob_id&0xFF;						 //COB-ID使能	
		data[1] = (cob_id>>8)&0xFF;
		creat_sdo_msg(&msg,node_id,SDO_WRITE_OD,id_index,0x01,data,4);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
		
		data[0] = num;  //Num of map使能
		creat_sdo_msg(&msg,node_id,SDO_WRITE_OD,n_index,0x00,data,1);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);
		
}


void pdo_set_type(uint8_t node_id,uint16_t index,uint8_t type)
{
		struct can_msg msg;
		uint8_t data[4] = {0};

		data[0] = type&0xFF;						 //设置type
		creat_sdo_msg(&msg,node_id,SDO_WRITE_OD,index,0x02,data,1);
		CAN1_Send_Msg(&msg,8);
		delay_ms(5);

		
}





/*测试函数*/
/*
static uint16_t control_word;
static uint8_t mode;

static int copley_nmt_init(int argc,void** argv)
{
		struct can_msg msg;
		creat_nmt_msg(&msg,1,NMT_START_NODE); //start node 进入optional 状态
		//rt_device_write(coplay_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
	
		return 0;
}

//MSH_CMD_EXPORT(copley_nmt_init,copley_nmt_init);

static void init_msg(void)
{
		struct can_msg msg;
		uint8_t data[4] = {0x01,0x06,0x00,0x00};
		creat_sdo_msg(&msg,1,SDO_WRITE_OD,0x1404,0x01,data,sizeof(data));
		//rt_device_write(coplay_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));	
		//rt_thread_mdelay(5);
}



static int copley_state_init(int argc,void ** argv)
{
		struct can_msg msg;
	
	  //通过RPDO设置control 和 mode
		uint32_t cobid = 0x301;
		mode = OP_MODE_CSV;
		uint8_t rpdo_data[8]={0};
		
		control_word = CMD_EN_VOL;
		rpdo_data[0] = control_word & 0xFF;  //CMD_EN_VOL
		rpdo_data[1] = (control_word >> 8) & 0xFF;
		rpdo_data[2] = mode;
		create_rpdo_msg(&msg,cobid,rpdo_data,3);
		//rt_device_write(coplay_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);


		control_word = CMD_SW_ON;
		rpdo_data[0] = control_word & 0xFF;  //CMD_SW_ON
		rpdo_data[1] = (control_word >> 8) & 0xFF;
		rpdo_data[2] = mode;
		create_rpdo_msg(&msg,cobid,rpdo_data,3);
		//rt_device_write(coplay_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));
		//rt_thread_mdelay(5);		
		
		control_word = CMD_EN_OP;
		rpdo_data[0] = control_word & 0xFF;  //CMD_EN_OP
		rpdo_data[1] = (control_word >> 8) & 0xFF;
		rpdo_data[2] = mode;
		create_rpdo_msg(&msg,cobid,rpdo_data,3);
		//rt_device_write(coplay_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));	
		//rt_thread_mdelay(5);
		init_msg();

		return 0;
}

//MSH_CMD_EXPORT(copley_state_init,copley state init);


static int copley_ctl(int argc,void ** argv)
{
		uint32_t target;
		if(argc >=2){
			target = atoi(argv[1]);
		}else{
			printf("Too less arg\n");
		}
		struct can_msg msg;
		uint32_t cobid = 0x501;
		uint8_t rpdo_data[8]={0};
		
		rpdo_data[0] = control_word & 0xFF; 
		rpdo_data[1] = (control_word >> 8) & 0xFF;
		rpdo_data[2] = target & 0xFF;
		rpdo_data[3] = (target >> 8) & 0xFF;
		rpdo_data[4] = (target >> 16) & 0xFF;
		rpdo_data[5] = (target >> 24) & 0xFF;
		create_rpdo_msg(&msg,cobid,rpdo_data,6);
		//rt_device_write(coplay_dev , -1 , (void*)&msg ,sizeof(struct rt_can_msg));	

		return 0;

}
//MSH_CMD_EXPORT(copley_ctl,copley ctl);
*/

