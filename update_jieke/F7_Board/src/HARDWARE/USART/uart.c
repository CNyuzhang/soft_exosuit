#include "uart.h"
#include "sys.h"
#include <stdlib.h>
#include "usart.h"

#if EN_USART2_RX   		//如果使能了接收   	  
 
 
uint8_t f4_data[10]; 
uint8_t f4_data_index=0;

void USART2_IRQHandler(void)
{
	u8 res;	    
	if(USART2->ISR&(1<<5))//接收到数据
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

//初始化IO 串口2
//pclk1:PCLK1时钟频率(Mhz),APB1一般为54Mhz
//bound:波特率	  
void USART2_Init(u32 pclk1,u32 bound)
{
	u32 temp;   
	temp=(pclk1*1000000+bound/2)/bound;	//得到USARTDIV@OVER8=0,采用四舍五入计算

	RCC->AHB1ENR|=1<<0;   		//使能PORTA口时钟   
	GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);	//PA2,PA3,复用功能,上拉 
 	GPIO_AF_Set(GPIOA,2,7);		//PA2,AF7
	GPIO_AF_Set(GPIOA,3,7);		//PA3,AF7  	   
 
	RCC->APB1ENR|=1<<17;  		//使能串口2时钟  
	RCC->APB1RSTR|=1<<17;   	//复位串口2
	RCC->APB1RSTR&=~(1<<17);	//停止复位	   	   
	//波特率设置
 	USART2->BRR=temp; 			//波特率设置	
	USART2->CR1=0;		 		//清零CR1寄存器
	USART2->CR1|=0<<28;	 		//设置M1=0
	USART2->CR1|=0<<12;	 		//设置M0=0&M1=0,选择8位字长 
	USART2->CR1|=0<<15; 		//设置OVER8=0,16倍过采样 
	USART2->CR1|=1<<3;  		//串口发送使能 
#if EN_USART2_RX		  		//如果使能了接收
	//使能接收中断 
	USART2->CR1|=1<<2;  		//串口接收使能
	USART2->CR1|=1<<5;    		//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(3,3,USART2_IRQn,2);//组2，最低优先级 
#endif
	USART2->CR1|=1<<0;  		//串口使能

}


void USART2_Send_char(u8 data)
{
		while((USART2->ISR&0X40)==0);//等待发送结束	
		USART2->TDR = data;		
}


//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void USART2_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  for(t=0;t<len;t++)			//循环发送数据
	{
		while((USART2->ISR&0X40)==0);//等待发送结束		  
		USART2->TDR=buf[t];
	}	 
	while((USART2->ISR&0X40)==0);//等待发送结束	
}



/*
*串口3代码
*/


#if EN_USART3_RX   		//如果使能了接收   	  

void USART3_IRQHandler(void)
{
	u8 res;	    
	if(USART3->ISR&(1<<5))//接收到数据
	{	 
		res = USART3->RDR;
		printf("%d\r\n",res);
	}  											 
} 
#endif


//初始化IO 串口3
//pclk1:PCLK1时钟频率(Mhz),APB1一般为54Mhz
//bound:波特率	  
void USART3_Init(u32 pclk1,u32 bound)
{
	u32 temp;   
	temp=(pclk1*1000000+bound/2)/bound;	//得到USARTDIV@OVER8=0,采用四舍五入计算

	RCC->AHB1ENR|=1<<1;   		//使能PORTB口时钟   **//
	GPIO_Set(GPIOB,PIN10|PIN11,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);	//PA2,PA3,复用功能,上拉 
 	GPIO_AF_Set(GPIOB,11,7);		//PA11,AF7 **//
	GPIO_AF_Set(GPIOB,10,7);		//PA10,AF7 **// 	   
 
	RCC->APB1ENR|=1<<18;  		//使能串口3时钟 ** //
	RCC->APB1RSTR|=1<<18;   	//复位串口3 ** //
	RCC->APB1RSTR&=~(1<<18);	//停止复位	** //
	
	//波特率设置
 	USART3->BRR=temp; 			//波特率设置	
	USART3->CR1=0;		 		//清零CR1寄存器
	USART3->CR1|=0<<28;	 		//设置M1=0
	USART3->CR1|=0<<12;	 		//设置M0=0&M1=0,选择8位字长 
	USART3->CR1|=0<<15; 		//设置OVER8=0,16倍过采样 
	USART3->CR1|=1<<3;  		//串口发送使能 
#if EN_USART3_RX		  		//如果使能了接收
	//使能接收中断 
	USART3->CR1|=1<<2;  		//串口接收使能
	USART3->CR1|=1<<5;    		//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(3,0,USART3_IRQn,2);//组2，最低优先级 
#endif
	USART3->CR1|=1<<0;  		//串口使能

}

void USART3_Send_char(u8 data)
{
		while((USART3->ISR&0X40)==0);//等待发送结束	
		USART3->TDR = data;		
}

//USART3发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void USART3_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  for(t=0;t<len;t++)			//循环发送数据
	{
		while((USART3->ISR&0X40)==0);//等待发送结束		  
		USART3->TDR=buf[t];
	}	 
	while((USART3->ISR&0X40)==0);//等待发送结束	
}




