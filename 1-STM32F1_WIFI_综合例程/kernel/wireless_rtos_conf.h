#ifndef WIRELESS_RTOS_CONF_H_H_H
#define WIRELESS_RTOS_CONF_H_H_H
/* ******************************** NOTED ********************************************/
/* *IoT_lwos task���ȼ�������Խ�����ȼ�Խ�ͣ��������ȼ�1�����ȼ�0�� ****************/
/* *UCOS II  task���ȼ�������Խ�����ȼ�Խ�ͣ��������ȼ�1�����ȼ�0�� ****************/
/* *UCOS III task���ȼ�������Խ�����ȼ�Խ�ͣ��������ȼ�1�����ȼ�0�� ****************/
/* *FreeRTOS task���ȼ�������Խ�����ȼ�Խ�ߣ��������ȼ�1�����ȼ�0�� ****************/
/* *rtthreas task���ȼ�������Խ�����ȼ�Խ�ͣ��������ȼ�1�����ȼ�0�� ****************/

/* ticks/s,�˴�����Ϊ100����ô100ticks/s��һ��ticks��10ms */
#define CONF_RTOS_TICKS_PER_SEC 100
/* MAX prio */
#define CONF_RTOS_MAX_PRIO	32
/* memory size */
#define CONF_RTOS_MEM_CNT_SIZE	40
#define CONF_RTOS_MEM_BLK_SIZE	512
#define CONF_RTOS_MEM_SIZE	(CONF_RTOS_MEM_CNT_SIZE*CONF_RTOS_MEM_BLK_SIZE)
/* IDLE task stack size */
#define CONF_IDLE_TASK_STACK_SIZE	128
/* ISR stack size */
#define CONF_ISR_STACK_SIZE			512

//#define IOT_LWOS_ENABLE	0
//#define USOCII_ENABLE		0
//#define USOCIII_ENABLE		0
//#define FREERTOS_ENABLE	1
//#define RTTHREAD_ENABLE	0

#endif
