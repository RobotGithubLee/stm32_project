#include "timer.h"
#include "led.h"
#include "lwip_comm.h"
#include "sys.h"  
#include "stdio.h"	 	 
#include "string.h"	
#include <ucos_ii.h>

/////
#include "mpu6050_AT.h"

extern short aacx;
extern short aacy;
extern short aacz;
extern short gyrox;
extern short gyroy;
extern short gyroz;
extern short temp;

static u32 timeCount = 0;
static u8 isTimer = 0; 

struct times
{
    u8 hour;
    u8 minute;
    u8 second;
    u8 milSecond;
}timeStamp;
char gpsBuff[200];
u16  gpsLen;
///// 
extern vu16 USART6_RX_STA;
extern vu16 USART3_RX_STA;

extern u8 USART6_RX_BUF[400];


void TIM2_Int_Init(u32 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM3ʱ��
	
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;           //GPIOB0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
    
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�TIM_OCPolarity_High
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
 
    TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPEʹ�� 
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//��ʱ��2�жϷ�����
void TIM2_IRQHandler(void)
{  
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //����ж�
	{
		memset(gpsBuff,0,200);
        LED1=!LED1;
        timeCount = OSTimeGet();//ms
        isTimer = 1;
        //printf("timeCount = %d\r\n",timeCount);
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ
}

void synchronizationToRslidar(void)
{
    u8 i;
    u16  timeMs,timeS;
    u8 checkSum=0;
    if(isTimer)
    {
        timeS = timeCount/1000;
        timeMs = timeCount%1000;
        //sprintf("timeS = %d,timeMs=%d\r\n",timeS,timeMs);
        timeStamp.hour = timeS/3600;
        while(timeStamp.hour>=24)
        {
            timeS -= 24*3600;
            timeStamp.hour = timeS/3600;
        }
        timeStamp.minute = (timeS%3600)/60;
        timeStamp.second = (timeS%3600)%60;
        timeStamp.milSecond = timeMs/10;
        //printf("%02d,%02d,%02d,%02d\r\n",timeStamp.hour,timeStamp.minute,timeStamp.second,timeStamp.milSecond);
        sprintf(gpsBuff,"$GPRMC,%02d%02d%02d.%02d,A,4546.40891,N,12639.65641,E,1.045,328.42,191118,,,A*",\
                timeStamp.hour,timeStamp.minute,timeStamp.second,timeStamp.milSecond);
        
        //  sprintf(gpsBuff,"$GPRMC,080655.00,A,4546.40891,N,12639.65641,E,1.045,328.42,170809,,,A*");        
        gpsLen = strlen(gpsBuff);
        
        for(i=1;i<gpsLen-1;i++)
        {
            checkSum^=gpsBuff[i];
        }
        sprintf(gpsBuff+gpsLen,"%02X\r\n",checkSum);
        gpsLen = strlen(gpsBuff);
        //printf("%d,%c,%d,%d,%x\r\n",gpsBuff[0],gpsBuff[1],gpsBuff[0]+gpsBuff[1],gpsLen,checkSum);
        for(i=0;i<gpsLen;i++)
        {
            while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET){};  //�ȴ��ϴδ������ 
            USART_SendData(USART3,gpsBuff[i]); 	 //�������ݵ�����3  PB10 TX
        }
        
        isTimer = 0;
    }
}

//��ʱ��6�жϷ������		    
void TIM5_IRQHandler(void)
{ 	
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)//�Ǹ����ж�
	{	 			   
		USART6_RX_STA|=1<<15;	//��ǽ������
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update  );  //���TIM5�����жϱ�־    
		TIM_Cmd(TIM5, DISABLE);  //�ر�TIM5 
		//printf("%s",USART6_RX_BUF);
		//USART6_RX_STA=0;	
	}	    
}
 
//ͨ�ö�ʱ���жϳ�ʼ��
//����ʼ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��		 
void TIM5_Int_Init(u16 arr,u16 psc)
{	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);//TIM5ʱ��ʹ��    
	
	//��ʱ��TIM5��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM5�ж�,��������ж�

	 	  
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
}



//��ʱ��7�жϷ������		    
void TIM7_IRQHandler(void)
{ 	
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//�Ǹ����ж�
	{	 			   
		USART3_RX_STA|=1<<15;	//��ǽ������
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //���TIM7�����жϱ�־    
		TIM_Cmd(TIM7, DISABLE);  //�ر�TIM7 
	}	    
}
 
//ͨ�ö�ʱ���жϳ�ʼ��
//����ʼ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��		 
void TIM7_Int_Init(u16 arr,u16 psc)
{	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//TIM7ʱ��ʹ��    
	
	//��ʱ��TIM7��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM7�ж�,��������ж�

	 	  
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
}

//ͨ�ö�ʱ��3�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//��ʱ��3�жϷ�����
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{
		//LocalTime +=10; //��10
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}
