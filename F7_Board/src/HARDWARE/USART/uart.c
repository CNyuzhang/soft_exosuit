#include "uart.h"
#include "sys.h"
#include <stdlib.h>
#include "usart.h"

#if EN_USART2_RX   		//���ʹ���˽���   	  
 
 
uint8_t f4_data[10]; 
uint8_t f4_data_index=0;

void USART2_IRQHandler(void)
{
	u8 res;	    
	if(USART2->ISR&(1<<5))//���յ�����
	{	 
		res = USART2->RDR;
		//printf("%d\r\n",res);

		f4_data[f4_data_index++] = res;
	  if(f4_data[0]!='T') {
			f4_data_index=0;
		}
		
	  if(f4_data_index<10) return ;
		
		if(f4_data[9]!='E') {
			f4_data_index=0;
			return ;
		}
	  //printf("Get a frame len:%d \r\n",f4_data_index);
		f4_data_index = 0;
		
	  	
	}  											 
} 
#endif

//��ʼ��IO ����2
//pclk1:PCLK1ʱ��Ƶ��(Mhz),APB1һ��Ϊ54Mhz
//bound:������	  
void USART2_Init(u32 pclk1,u32 bound)
{
	u32 temp;   
	temp=(pclk1*1000000+bound/2)/bound;	//�õ�USARTDIV@OVER8=0,���������������

	RCC->AHB1ENR|=1<<0;   		//ʹ��PORTA��ʱ��   
	GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);	//PA2,PA3,���ù���,���� 
 	GPIO_AF_Set(GPIOA,2,7);		//PA2,AF7
	GPIO_AF_Set(GPIOA,3,7);		//PA3,AF7  	   
 
	RCC->APB1ENR|=1<<17;  		//ʹ�ܴ���2ʱ��  
	RCC->APB1RSTR|=1<<17;   	//��λ����2
	RCC->APB1RSTR&=~(1<<17);	//ֹͣ��λ	   	   
	//����������
 	USART2->BRR=temp; 			//����������	
	USART2->CR1=0;		 		//����CR1�Ĵ���
	USART2->CR1|=0<<28;	 		//����M1=0
	USART2->CR1|=0<<12;	 		//����M0=0&M1=0,ѡ��8λ�ֳ� 
	USART2->CR1|=0<<15; 		//����OVER8=0,16�������� 
	USART2->CR1|=1<<3;  		//���ڷ���ʹ�� 
#if EN_USART2_RX		  		//���ʹ���˽���
	//ʹ�ܽ����ж� 
	USART2->CR1|=1<<2;  		//���ڽ���ʹ��
	USART2->CR1|=1<<5;    		//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(3,3,USART2_IRQn,2);//��2��������ȼ� 
#endif
	USART2->CR1|=1<<0;  		//����ʹ��

}


void USART2_Send_char(u8 data)
{
		while((USART2->ISR&0X40)==0);//�ȴ����ͽ���	
		USART2->TDR = data;		
}


//RS485����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ����64���ֽ�)
void USART2_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  for(t=0;t<len;t++)			//ѭ����������
	{
		while((USART2->ISR&0X40)==0);//�ȴ����ͽ���		  
		USART2->TDR=buf[t];
	}	 
	while((USART2->ISR&0X40)==0);//�ȴ����ͽ���	
}



/*
*����3����
*/


#if EN_USART3_RX   		//���ʹ���˽���   	  

void USART3_IRQHandler(void)
{
	u8 res;	    
	if(USART3->ISR&(1<<5))//���յ�����
	{	 
		res = USART3->RDR;
		printf("%d\r\n",res);
	}  											 
} 
#endif


//��ʼ��IO ����3
//pclk1:PCLK1ʱ��Ƶ��(Mhz),APB1һ��Ϊ54Mhz
//bound:������	  
void USART3_Init(u32 pclk1,u32 bound)
{
	u32 temp;   
	temp=(pclk1*1000000+bound/2)/bound;	//�õ�USARTDIV@OVER8=0,���������������

	RCC->AHB1ENR|=1<<1;   		//ʹ��PORTB��ʱ��   **//
	GPIO_Set(GPIOB,PIN10|PIN11,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);	//PA2,PA3,���ù���,���� 
 	GPIO_AF_Set(GPIOB,11,7);		//PA11,AF7 **//
	GPIO_AF_Set(GPIOB,10,7);		//PA10,AF7 **// 	   
 
	RCC->APB1ENR|=1<<18;  		//ʹ�ܴ���3ʱ�� ** //
	RCC->APB1RSTR|=1<<18;   	//��λ����3 ** //
	RCC->APB1RSTR&=~(1<<18);	//ֹͣ��λ	** //
	
	//����������
 	USART3->BRR=temp; 			//����������	
	USART3->CR1=0;		 		//����CR1�Ĵ���
	USART3->CR1|=0<<28;	 		//����M1=0
	USART3->CR1|=0<<12;	 		//����M0=0&M1=0,ѡ��8λ�ֳ� 
	USART3->CR1|=0<<15; 		//����OVER8=0,16�������� 
	USART3->CR1|=1<<3;  		//���ڷ���ʹ�� 
#if EN_USART3_RX		  		//���ʹ���˽���
	//ʹ�ܽ����ж� 
	USART3->CR1|=1<<2;  		//���ڽ���ʹ��
	USART3->CR1|=1<<5;    		//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(3,0,USART3_IRQn,2);//��2��������ȼ� 
#endif
	USART3->CR1|=1<<0;  		//����ʹ��

}

void USART3_Send_char(u8 data)
{
		while((USART3->ISR&0X40)==0);//�ȴ����ͽ���	
		USART3->TDR = data;		
}

//USART3����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ����64���ֽ�)
void USART3_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  for(t=0;t<len;t++)			//ѭ����������
	{
		while((USART3->ISR&0X40)==0);//�ȴ����ͽ���		  
		USART3->TDR=buf[t];
	}	 
	while((USART3->ISR&0X40)==0);//�ȴ����ͽ���	
}




