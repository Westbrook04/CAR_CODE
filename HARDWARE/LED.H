#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"

#define LED_TASK_PRIO		3     //Task priority //任务优先级
#define LED_STK_SIZE 		128   //Task stack size //任务堆栈大小


/*--------Buzzer control pin--------*/
#define Buzzer_PORT GPIOG
#define Buzzer_PIN GPIO_Pin_11
#define Buzzer PGout(11)
/*----------------------------------*/

/*--------LED control pin--------*/
#define LED_PORT GPIOE
#define LED_PIN GPIO_Pin_3
#define LED PEout(3) 
/*----------------------------------*/

void LED_Init(void);  
void Buzzer_Init(void); 
void Led_Flash(u16 time);
void led_task(void *pvParameters);
extern int Led_Count;
#endif
