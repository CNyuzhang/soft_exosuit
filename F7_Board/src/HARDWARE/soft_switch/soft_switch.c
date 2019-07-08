#include "soft_switch.h"


void init_switch(void)
{
//		RCC->AHB1ENR|=1<<1;	//ʹ��PORTBʱ�� 
//		GPIO_Set(GPIOB,PIN10,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PD); //PB10�����������  ----U3_TX	
//		GPIO_Set(GPIOB,PIN11,GPIO_MODE_IN,0,0,GPIO_PUPD_PU); 														//PB11����Ϊ��������----U3_RX 
	
	
		RCC->AHB1ENR|=1<<0;   	//ʹ��PORTA��ʱ�� 
		GPIO_Set(GPIOA,PIN9,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PD); //PA9�����������  ----U1_TX	
		GPIO_Set(GPIOA,PIN10,GPIO_MODE_IN,0,0,GPIO_PUPD_PU); 														//PA10����Ϊ��������----U1_RX 	
			
		GPIO_Set(GPIOB,PIN1,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); 	//PB1 ����----LED��
	
}


void set_led(uint8_t x)
{
		GPIO_Pin_Set(GPIOB,PIN1,x); 
}


uint8_t get_switc_status(void)
{
		return GPIO_Pin_Get(GPIOA,PIN10);//GPIO_Pin_Get(GPIOB,PIN11);
}






