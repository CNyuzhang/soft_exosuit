#include "pcf8574.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//PCF8574驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/7/15
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//初始化PCF8574
//返回值:0,初始化成功
//      1,初始化失败
u8 PCF8574_Init(void)
{
	u8 temp=0;		  	    																 
	RCC->AHB1ENR|=1<<1;     	//使能PORTB时钟 
	GPIO_Set(GPIOB,PIN12,GPIO_MODE_IN,0,0,GPIO_PUPD_PU);	//PB12设置为上拉输入  
	IIC_Init();					//IIC初始化 	
	//检查PCF8574是否在位
    IIC_Start();    	 	   
	IIC_Send_Byte(PCF8574_ADDR);//写地址			   
	temp=IIC_Wait_Ack();		//等待应答,通过判断是否有ACK应答,来判断PCF8574的状态
    IIC_Stop();					//产生一个停止条件
    PCF8574_WriteOneByte(0XFF);	//默认情况下所有IO输出高电平
	return temp;
} 

//读取PCF8574的8位IO值
//返回值:读到的数据
u8 PCF8574_ReadOneByte(void)
{				  
	u8 temp=0;		  	    																 
    IIC_Start();    	 	   
	IIC_Send_Byte(PCF8574_ADDR|0X01);   //进入接收模式			   
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();							//产生一个停止条件	    
	return temp;
}
//向PCF8574写入8位IO值  
//DataToWrite:要写入的数据
void PCF8574_WriteOneByte(u8 DataToWrite)
{				   	  	    																 
    IIC_Start();  
    IIC_Send_Byte(PCF8574_ADDR|0X00);   //发送器件地址0X40,写数据 	 
	IIC_Wait_Ack();	    										  		   
	IIC_Send_Byte(DataToWrite);    	 	//发送字节							   
	IIC_Wait_Ack();      
    IIC_Stop();							//产生一个停止条件 
	delay_ms(10);	 
}

//设置PCF8574某个IO的高低电平
//bit:要设置的IO编号,0~7
//sta:IO的状态;0或1
void PCF8574_WriteBit(u8 bit,u8 sta)
{
    u8 data;
    data=PCF8574_ReadOneByte(); //先读出原来的设置
    if(sta==0)data&=~(1<<bit);     
    else data|=1<<bit;
    PCF8574_WriteOneByte(data); //写入新的数据
}

//读取PCF8574的某个IO的值
//bit：要读取的IO编号,0~7
//返回值:此IO的值,0或1
u8 PCF8574_ReadBit(u8 bit)
{
    u8 data;
    data=PCF8574_ReadOneByte(); //先读取这个8位IO的值 
    if(data&(1<<bit))return 1;
    else return 0;   
}
    
    
