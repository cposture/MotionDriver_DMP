#ifndef LED_H
#define LED_H

#include "stm32f10x.h"

/* LED的GPIO口定义 */
#define LED_1_GPIO				GPIOB
#define LED_1_GPIO_PIN		GPIO_Pin_0
#define LED_1_RCC					RCC_APB2Periph_GPIOB

#define LED_2_GPIO				GPIOC
#define LED_2_GPIO_PIN		GPIO_Pin_4
#define LED_2_RCC					RCC_APB2Periph_GPIOC

#define LED_3_GPIO				GPIOC
#define LED_3_GPIO_PIN		GPIO_Pin_3
#define LED_3_RCC					RCC_APB2Periph_GPIOC

/* 带参宏，可以像内联函数一样使用 */
#define LED1(a)	if (a)	\
					GPIO_SetBits(LED_1_GPIO,LED_1_GPIO_PIN);\
					else		\
					GPIO_ResetBits(LED_1_GPIO,LED_1_GPIO_PIN)

#define LED2(a)	if (a)	\
					GPIO_SetBits(LED_2_GPIO,LED_2_GPIO_PIN);\
					else		\
					GPIO_ResetBits(LED_2_GPIO,LED_2_GPIO_PIN)

#define LED3(a)	if (a)	\
					GPIO_SetBits(LED_3_GPIO,LED_3_GPIO_PIN);\
					else		\
					GPIO_ResetBits(LED_3_GPIO,LED_3_GPIO_PIN)
					
/* 直接操作寄存器的方法控制IO */
#define	digitalHi(gpio,gpio_pin)				{gpio->BSRR =gpio_pin;}			//设置为高电平		
#define digitalLo(gpio,gpio_pin)				{gpio->BRR	=gpio_pin;}			//输出低电平
#define digitalToggle(gpio,gpio_pin)		{gpio->ODR ^=gpio_pin;}			//输出反转状态

/* 定义控制IO的宏 */
#define LED1_TOGGLE		digitalToggle(LED_1_GPIO,LED_1_GPIO_PIN)
#define LED1_OFF			digitalHi(LED_1_GPIO,LED_1_GPIO_PIN)
#define LED1_ON				digitalLo(LED_1_GPIO,LED_1_GPIO_PIN)

#define LED2_TOGGLE		digitalToggle(LED_2_GPIO,LED_2_GPIO_PIN)
#define LED2_OFF			digitalHi(LED_2_GPIO,LED_2_GPIO_PIN)
#define LED2_ON				digitalLo(LED_2_GPIO,LED_2_GPIO_PIN)

#define LED3_TOGGLE		digitalToggle(LED_3_GPIO,LED_3_GPIO_PIN)
#define LED3_OFF			digitalHi(LED_3_GPIO,LED_3_GPIO_PIN)
#define LED3_ON				digitalLo(LED_3_GPIO,LED_3_GPIO_PIN)

void LED_GPIO_Config(void);

#endif /* LED_H */
