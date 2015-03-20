#include "led.h" 

void LED_GPIO_Config(void)
{		
		/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*开启GPIOB和GPIOF的外设时钟*/
		RCC_APB2PeriphClockCmd( LED_1_RCC | LED_2_RCC | LED_3_RCC, ENABLE); 

		/*选择要控制的GPIOB引脚*/															   
		GPIO_InitStructure.GPIO_Pin = LED_1_GPIO_PIN;	

		/*设置引脚模式为通用推挽输出*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*调用库函数，初始化GPIOB0*/
		GPIO_Init(LED_1_GPIO, &GPIO_InitStructure);	
		
		/*选择要控制的GPIOF引脚*/															   
		GPIO_InitStructure.GPIO_Pin = LED_2_GPIO_PIN;

		/*调用库函数，初始化GPIOF7*/
		GPIO_Init(LED_2_GPIO, &GPIO_InitStructure);
		
		/*选择要控制的GPIOF引脚*/															   
		GPIO_InitStructure.GPIO_Pin = LED_3_GPIO_PIN;

		/*调用库函数，初始化GPIOF7*/
		GPIO_Init(LED_3_GPIO, &GPIO_InitStructure);			  

		/* 关闭所有led灯	*/
		GPIO_SetBits(LED_1_GPIO, LED_1_GPIO_PIN);
		
		/* 关闭所有led灯	*/
		GPIO_SetBits(LED_2_GPIO, LED_2_GPIO_PIN);	 
		
		/* 关闭所有led灯	*/
		GPIO_SetBits(LED_3_GPIO, LED_3_GPIO_PIN);	
}
