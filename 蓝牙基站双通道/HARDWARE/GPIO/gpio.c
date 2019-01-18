#include "gpio.h"



void io_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOG, ENABLE);	 //使能PB、PE端口时钟
/***********************************蜂鸣器************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOE, &GPIO_InitStructure);			
	GPIO_SetBits(GPIOE,GPIO_Pin_14);				
/***********************************报警灯************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //报警灯1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOE, &GPIO_InitStructure);				
	GPIO_SetBits(GPIOE,GPIO_Pin_15);						 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				 //报警灯2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOB, &GPIO_InitStructure);				
	GPIO_SetBits(GPIOB,GPIO_Pin_10);						 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				 //报警灯3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOB, &GPIO_InitStructure);				
	GPIO_SetBits(GPIOB,GPIO_Pin_11);						 
/***********************************继电器************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);			
	GPIO_SetBits(GPIOC,GPIO_Pin_9);				
/***********************************进出门光电************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;				
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		
	GPIO_Init(GPIOG, &GPIO_InitStructure);				
/**********************************单双通道选择************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		
	GPIO_Init(GPIOE, &GPIO_InitStructure);	
/***********************************RFID************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;	          //PD9:标签   PD10:标志位			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		
	GPIO_Init(GPIOD, &GPIO_InitStructure);				
}

void alarm_shut(void)
{
	GPIO_SetBits(GPIOE,GPIO_Pin_14);		
	GPIO_SetBits(GPIOE,GPIO_Pin_15);						 
	GPIO_SetBits(GPIOB,GPIO_Pin_10);	
	GPIO_SetBits(GPIOB,GPIO_Pin_11);						 
}

void alarm_open(u8 channel)
{
	if(channel == DUAL_1)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_14);		
		GPIO_ResetBits(GPIOE,GPIO_Pin_15);						 
		GPIO_ResetBits(GPIOB,GPIO_Pin_10);	
	}
	else
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_14);		
		GPIO_ResetBits(GPIOB,GPIO_Pin_10);	
		GPIO_ResetBits(GPIOB,GPIO_Pin_11);						 
	}
}
