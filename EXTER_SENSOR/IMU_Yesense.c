#include "includes.h"
#include "IMU_Yesense.h"												    
#include "usart6.h"	
#include "Queue.h"
#include "ConvertData.h"			


#define MPUQUEUECOUNT 10

//客户端任务优先级
#define IMUYesense_SERVER_PRIO		20
//任务堆栈大小	
#define IMUYesense_STK_SIZE 		128
//任务堆栈	
__align(8) OS_STK IMUYesense_TASK_STK[IMUYesense_STK_SIZE];
//任务函数

int IMUYaw;



static void IMUYesense_thread(void *arg)
{
	float a[3],w[3],Angle[3],T;
	while(1)
	{
		if(USART6_RX_STA&0X8000 )		                            //接收到一次数据了 && USART6_RX_BUF[0]==0x55
		{ 				
					if(USART6_RX_BUF[76]==0x40)
					{						
						IMUYaw=(int)(USART6_RX_BUF[86]<<0|USART6_RX_BUF[87]<<8|USART6_RX_BUF[88]<<16|USART6_RX_BUF[89]<<24)*0.000001;
						//printf("%2x,%2x,%2x,%2x\r\n",USART6_RX_BUF[86],USART6_RX_BUF[87],USART6_RX_BUF[88],USART6_RX_BUF[89]);
						//printf("%d\r\n",IMUYaw);
					}
					
				  USART6_RX_STA=0;			
				}				
				delay_ms(3);
	}
	  
}




//创建IMUYesense线程
//返回值:0 IMUYesense创建成功
//		其他 IMUYesense创建失败
u8 IMUYesense_task_init(void)
{
  INT8U res;
	OS_CPU_SR cpu_sr;
	
	OS_ENTER_CRITICAL();	//关中断
	res = OSTaskCreate(IMUYesense_thread,(void*)0,(OS_STK*)&IMUYesense_TASK_STK[IMUYesense_STK_SIZE-1],IMUYesense_SERVER_PRIO); //创建TCP服务器线程
	OS_EXIT_CRITICAL();		//开中断
	
	return res;
}

















		  






