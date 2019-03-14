#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "can.h"
#include "key.h"
#include "lwip_comm.h"
#include "LAN8720.h"
#include "usmart.h"
#include "timer.h"
#include "mpu6050_AT.h"

#include "sram.h"
#include "malloc.h"
#include "lwip_comm.h"
#include "includes.h"
#include "lwipopts.h"
#include "tcp_server_demo.h"
#include "tcp_client_demo.h"

#include "MPU6050.h"		
#include "IMU_Yesense.h"	
#include "ultrasonic.h"		
#include "driver.h"
#include "message.h"
 
#include "usart6.h"
#include "usart3.h"
#include "usart2.h"
#include "common.h" 




//KEY����
#define KEY_TASK_PRIO 		12
//�����ջ��С
#define KEY_STK_SIZE		128	
//�����ջ
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
//������
void key_task(void *pdata);   

//LED����
//�������ȼ�
#define LED_TASK_PRIO		7
//�����ջ��С
#define LED_STK_SIZE		64
//�����ջ
OS_STK	LED_TASK_STK[LED_STK_SIZE];
//������
void led_task(void *pdata);  


//�������ȼ�
#define IMU_TASK_PRIO	9
//�����ջ��С
#define IMU_STK_SIZE	128
//�����ջ
OS_STK	IMU_TASK_STK[IMU_STK_SIZE];
//������
void imu_task(void *pdata);

//START����
//�������ȼ�
#define START_TASK_PRIO		11
//�����ջ��С
#define START_STK_SIZE		128
//�����ջ
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata); 



int main(void)
{
	delay_init(168);       	//��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//�жϷ�������
	uart_init(115200);   	//���ڲ���������
	usart6_init(115200);  //��ʼ������6������Ϊ9600
	usart3_init(9600);  //��ʼ������3������Ϊ9600  //��ʼ������3������Ϊ115200
	//usart2_init(115200);
	usmart_dev.init(84);	//��ʼ��USMART
	LED_Init(); 			//LED��ʼ��
	//KEY_Init();  			//������ʼ��
	TIM2_Int_Init(1000000-1,84-1);//����1s��ʱ��84M����Ƶϵ��84������ֵ1000000��
    TIM_SetCompare1(TIM2,200000); //200ms��ת
	
    CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������500Kbps 
	
	InitDriverCmdQueue();
    InitDriverCmdQueue();
		
	FSMC_SRAM_Init(); 		//SRAM��ʼ��
	mymem_init(SRAMIN);  	//��ʼ���ڲ��ڴ��
	mymem_init(SRAMEX);  	//��ʼ���ⲿ�ڴ��
	mymem_init(SRAMCCM); 	//��ʼ��CCM�ڴ��
	
	OSInit(); 					//UCOS��ʼ��	
	OSTaskCreate(start_task,(void*)0,(OS_STK*)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO);
	OSStart(); //����UCOS
}

//start����
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr;
	pdata = pdata ;
	
	OSStatInit();  			//��ʼ��ͳ������
	OS_ENTER_CRITICAL();  	//���ж�
		
#if	LWIP_DHCP
	lwip_comm_dhcp_creat();	//����DHCP����
#endif
    if(can_task_init())//Can��ʼ��
	{
		printf("main.c  Can init fail!!!!\r\n");
	}	
	InitDriver();
	printf("main.c  Init driver,Wait a moment......\r\n");
    
    if(mpu_task_init()) 	//mpu��ʼ��
	{
		printf("main.c  mpu init fail!!!!\r\n");	
	}
    
    if(lwip_comm_init()) 	//lwip��ʼ��
	{
		printf("main.c  Lwip init fail!!!!\r\n");	
	}
	else{
	  //if(tcp_client_init()) 									//��ʼ��tcp_client(����tcp_client�߳�)
	  {
	 		printf("main.c  tcp_client init fail!!!!\r\n");	
	  }
	  if(tcp_server_init()) 									//��ʼ��tcp_server(����tcp_server�߳�)
	  {
	 		printf("main.c  tcp_server init fail!!!!\r\n");	
	  }
  }

  OSTaskCreate(led_task,(void*)0,(OS_STK*)&LED_TASK_STK[LED_STK_SIZE-1],LED_TASK_PRIO); 	//����LED����
	
  OSTaskSuspend(OS_PRIO_SELF); //����start_task����
	OS_EXIT_CRITICAL();  //���ж�
}



//led����
void led_task(void *pdata)
{
	int i=0;
	while(1)
	{
		//MotionFeedbackHandle(i);
        //printf("led task running!\r\n");
        synchronizationToRslidar();
        MotionFeedbackHandle3(i);
		if(i>50)
		{
			LED0 = !LED0;
			i=0;
		}
		i++;
        delay_ms(5);
		//OSTimeDlyHMSM(0,0,0,10);  //��ʱ500ms
 	}
}


