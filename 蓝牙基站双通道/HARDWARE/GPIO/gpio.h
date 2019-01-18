#ifndef __GPIO_H
#define __GPIO_H	 

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#define    SIGNAL             0
#define    DUAL_1             0
#define    DUAL_2             1


#define   DUAL_CHANNEL           GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)

#define   RfidTag                GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9)
#define   RfidFlag               GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10)


void io_init(void);
void alarm_shut(void);
void alarm_open(u8 channel);



#endif
