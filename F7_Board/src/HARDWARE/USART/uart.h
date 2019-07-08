#ifndef __USART_H__
#define __USART_H__
#include "sys.h"


#define EN_USART2_RX 1
#define EN_USART3_RX 1


void USART2_Init(u32 pclk1,u32 bound);
extern uint8_t f4_data_index;
void USART2_Send_char(u8 data);
extern uint8_t f4_data[10]; 



void USART3_Init(u32 pclk1,u32 bound);
void USART3_Send_char(u8 data);
void USART3_Send_Data(u8 *buf,u8 len);

#endif
