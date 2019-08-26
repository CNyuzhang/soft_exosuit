#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/7/19
//�汾��V1.0 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

struct can_msg
{
    uint32_t id  : 29;
    uint32_t ide : 1;
    uint32_t rtr : 1;
    uint32_t rsv : 1;
    uint32_t len : 8;
    uint32_t priv : 8;
    uint32_t hdr : 8;
    uint32_t reserved : 8;
    uint8_t data[8];
};//��Ϣ�ṹ��
typedef struct can_msg *can_msg_t;



	
//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE			1		 			//0,��ʹ��;1,ʹ��.								    
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat);	//��������
u8 CAN1_Msg_Pend(u8 fifox);								//��ѯ���䱨��
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat);//��������
u8 CAN1_Tx_Staus(u8 mbox);  							//���ط���״̬
//u8 CAN1_Send_Msg(u8* msg,u8 len);						//��������
u8 CAN1_Send_Msg(struct can_msg *msg,u8 len);						//��������
u8 CAN1_Receive_Msg(u8 *buf);							//��������
#endif

















