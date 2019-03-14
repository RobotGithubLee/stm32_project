#include "includes.h"
#include "message.h"
#include "delay.h"

#include "Queue.h"
#include "ConvertData.h"
#include "kinematics.h"
#include "driver.h"
#include "can.h"
#include "usart6.h"
#include "mpu6050_AT.h"
#include "stdbool.h"

#define PI 3.1415926

const float  aRatio=PI*16/34/180;   //max Angle=16°，UP max=34;
const float  wRatio=0.0001;
const float  vRatio=1000*0.75/405;    //Max v=0.75 m/s, *1000->mm/s, 405->max data 
u8 isAuto=0;
extern u8 wifiSendBuf(u8 *buf,u8 len);
extern short Yaw;
extern int IMUYaw;

//信息反馈队列
struct interQueue *FMQ,FeedbackMsgQueue;

STM FeedbackMsgArrry[QUEUEMAXCOUNT];

struct interQueue *joyCmdQueuePt,joyCmdQueue;

//接收命令队列
struct interQueue *DCQ,DriverCmdQueue;
SCM  DriverCmdArray[QUEUEMAXCOUNT];


void * MsgGrp[256];			//消息队列存储地址,最大支持256个消息

int InitDriverCmdQueue(void)
{
	u8 sendFramData[8];
	//Init FeedBack MsgQueue
	memset((void*)FeedbackMsgArrry,0x00,sizeof(FeedbackMsgArrry));

	FeedbackMsgQueue.QNode=FeedbackMsgArrry;

	FMQ=&FeedbackMsgQueue;

	FMQ->MaxCount=QUEUEMAXCOUNT;

	
	//Init Fornt Drive CmdQueue
	memset((void*)DriverCmdArray,0x00,sizeof(DriverCmdArray));

	DriverCmdQueue.QNode=DriverCmdArray;

	DCQ=&DriverCmdQueue;

	DCQ->MaxCount=QUEUEMAXCOUNT;

    InitQueueinter(DCQ);
	
	
	//Init  Drive: setTacc and Enable Drive
	setTVelocityFram(POS_MAX_SPEED,sendFramData);
	EnDriverCmdQueue(FRONT_ID,sendFramData,0x8);
	EnDriverCmdQueue(LEFT_ID,sendFramData,0x8);
	EnDriverCmdQueue(RIGHT_ID,sendFramData,0x8);
	
	//Enable Drive
	EnableFram(sendFramData);
	EnDriverCmdQueue(FRONT_ID,sendFramData,0x8);
	EnDriverCmdQueue(LEFT_ID,sendFramData,0x8);
	EnDriverCmdQueue(RIGHT_ID,sendFramData,0x8);

	return 0;
}



int MessageHandle(u8 *msg,int len)
{
	STM RxMsg;
	u8 i=0,n=0;
	
	while(i<len-EXTERN_BUFSIZE)
	{
		if(*(msg+i)==0xAA)
		{			
			n=*(msg+i+4)+EXTERN_BUFSIZE;
			if(n>sizeof(STM))
                break;
            if(i+n <= len)//否则访问越界
                memcpy(&RxMsg,msg+i,n-1);
            else
                break;
			RxMsg.Bcc=msg[i+n-1];
			i+=n;
			//XOR check	
            if(getXorCheck(RxMsg.data,RxMsg.len)==RxMsg.Bcc)
            {
                switch(RxMsg.Cmd) 
                    {
                        case 0x01:
                            isAuto=RxMsg.data[0];
                          printf("message.c:  isAuto=%d\r\n",isAuto); 
                        break;
                        
                        case 0x60:
                            if(!isAuto)
                            {
                                AngleMotionCmdHandle(RxMsg.Seq,RxMsg.data,RxMsg.len);
                              //MotionFeedbackHandle(RxMsg.Seq);
//                                printf("message.c: 60 RxMsg.Cmd=%2x\r\n",RxMsg.Cmd); 
                            }
                        break;
                            
                        case 0x80:
                                if(isAuto)
                                {
                                    MotionCmdHandle(RxMsg.Seq,RxMsg.data,RxMsg.len);
                                    //MotionFeedbackHandle(RxMsg.Seq);
//                                    printf("message.c: 80 RxMsg.Cmd=%2x\r\n",RxMsg.Cmd); 
                                }
                            break;
                         case 0x81:
                                MotionFeedbackHandle(RxMsg.Seq);
                            break;	
                    }
            }
            else
                printf("message.c:  XOR check error\r\n");
		}			
		else
			i++;
	}
	return 0;
}



int MotionCmdHandle(u8 Seq,u8* data, u8 len)
{
	int ExpPos=0,ExpRightV=0,ExpLeftV=0;
	u8 sendFramData[8];
	float w,v;
	if(len!=0x4)
	{
		printf("message.c: MotionCmdHandle check error\r\n");
		return -1;
	}
	
	w=hexbyte2short(data)*wRatio;
	if(w>0.5)
		w=0.5;
	else if(w<-0.5)
		w=-0.5;	
	v=hexbyte2short(data+2);
	if(v>2000)
		v=2000;
	else if(v<-1000)
		v=-1000;
	ExpPos=getExpPosition(w,v);
	ExpLeftV=getExpLeftVelocity(v);
	ExpRightV=getExpRightVelocity(v);
    printf("message.c:EXP: w=%f v=%f\r\n",w,v);
	 
	setPositionFram(ExpPos,sendFramData);
	EnDriverCmdQueue(FRONT_ID,sendFramData,0x8);
	
	setVelocityFram(ExpLeftV,sendFramData);
	EnDriverCmdQueue(LEFT_ID,sendFramData,0x8);
	
	setVelocityFram(ExpRightV,sendFramData);
	EnDriverCmdQueue(RIGHT_ID,sendFramData,0x8);
	
	return 0;
}


 
int AngleMotionCmdHandle(u8 Seq,u8* data, u8 len)
{
	int ExpPos=0,ExpRightV=0,ExpLeftV=0;
	u8 sendFramData[8];
	float Angle,v;
	if(len!=0x4)
	{
		printf("message.c: MotionCmdHandle check error\r\n");
		return -1;
	}
	
	Angle=hexbyte2short(data)*aRatio;	
	v=hexbyte2short(data+2)*vRatio;
	//printf("v=%f %d %2x %2x \t\n",v,hexbyte2short(data+2),data[2],data[3]);
	ExpPos=Angle2Position(Angle);
	ExpLeftV=getExpLeftVelocity(v);
	ExpRightV=getExpRightVelocity(v);
	

//	printf("Angle=%f v=%f ExpPos=%d  ExpLeftV=%d  ExpRightV=%d\r\n",Angle,v,ExpPos,ExpLeftV,ExpRightV);
	
	setPositionFram(ExpPos,sendFramData);
	EnDriverCmdQueue(FRONT_ID,sendFramData,0x8);
	
	setVelocityFram(ExpLeftV,sendFramData);
	EnDriverCmdQueue(LEFT_ID,sendFramData,0x8);
	
	setVelocityFram(ExpRightV,sendFramData);
	EnDriverCmdQueue(RIGHT_ID,sendFramData,0x8);
	
	return 0;
}

void queryCanFun(int ID,u8* sendFramData,u8 len)
{
    u8 readLen;
    u8 readData[8];
	u32 readID;
    int i=0;
	SCM sendCanmsg;
	sendCanmsg.ID=ID;
	for(i=0;i<len;i++)
	{
		sendCanmsg.data[i]=sendFramData[i];
	}
	sendCanmsg.len=len;
    
    SendCan(sendCanmsg);
    delay_ms(2);
    readLen=ReadCan(readData,&readID);/**/
    if(readLen)
        FeedbackMessge(readID,readData,readLen);
    //return sendCanmsg;
}

/*
* 查询前轮位置、后轮速度,查询命令入队列
*/
int MotionFeedbackHandle3(u8 Seq)
{
	u8 sendFramData[8];
	u8 sendData[36];
    u32 odomTimeStamp;
    /*查询各电机位置*/
	getPositionFram(sendFramData);
	EnDriverCmdQueue(FRONT_ID,sendFramData,0x8);
//	EnDriverCmdQueue(LEFT_ID,sendFramData,0x8);
//	EnDriverCmdQueue(RIGHT_ID,sendFramData,0x8);
	/*查询各后轮速度*/
	getVelocityFram(sendFramData);
	EnDriverCmdQueue(LEFT_ID,sendFramData,0x8);
	EnDriverCmdQueue(RIGHT_ID,sendFramData,0x8);
	             
	//delay_ms(1);               
	short2hexbyte((short)(getCurAngleVel()/wRatio),sendData);    //w
	short2hexbyte((short)(getCurVelocity()),sendData+2);    //v
	short2hexbyte((short)IMUYaw,sendData+4);    // IMU angle
	odomTimeStamp = OSTimeGet();
	int2hexbyte(getLeftPosition(),sendData+6);    //LeftPosition
	int2hexbyte(getRightPosition(),sendData+10);    //RightPosition
	int2hexbyte(odomTimeStamp,sendData+14);
    
    IMU_DATA_ IMUData = getMpuData();
    //printf("message.c osTime1 = %d\r\n",IMUData.timeStamp); 
    short2hexbyte(IMUData.aacx,sendData+18);
    short2hexbyte(IMUData.aacy,sendData+20);
    short2hexbyte(IMUData.aacz,sendData+22);
    short2hexbyte(IMUData.gyrox,sendData+24);
    short2hexbyte(IMUData.gyroy,sendData+26);
    short2hexbyte(IMUData.gyroz,sendData+28);
    short2hexbyte(IMUData.temp,sendData+30);
    int2hexbyte(IMUData.timeStamp,sendData+32);
    EnFeedBackMsgQueue(0x81,0x00,Seq,0x24,sendData);
	 
	//printf("message.c:CUR: Left=%d Right=%d\r\n",getLeftPosition(),getRightPosition());
	
	return 0;
}



int MotionFeedbackHandle(u8 Seq)
{
	u8 sendFramData[8];
	u8 sendData[14];
    /*查询各电机位置*/
	getPositionFram(sendFramData);
	EnDriverCmdQueue(FRONT_ID,sendFramData,0x8);
	EnDriverCmdQueue(LEFT_ID,sendFramData,0x8);
	EnDriverCmdQueue(RIGHT_ID,sendFramData,0x8);
	/*查询各后轮速度*/
	getVelocityFram(sendFramData);
	EnDriverCmdQueue(LEFT_ID,sendFramData,0x8);
	EnDriverCmdQueue(RIGHT_ID,sendFramData,0x8);
	
	delay_ms(1);               
	short2hexbyte((short)(getCurAngleVel()/wRatio),sendData);    //w
	short2hexbyte((short)(getCurVelocity()),sendData+2);    //v
	short2hexbyte((short)IMUYaw,sendData+4);    // IMU angle
	
	int2hexbyte(getLeftPosition(),sendData+6);    //LeftPosition
	int2hexbyte(getRightPosition(),sendData+10);    //RightPosition
    
	EnFeedBackMsgQueue(0x81,0x00,Seq,0x0E,sendData);
	 
	//printf("message.c:CUR: Left=%d Right=%d\r\n",getLeftPosition(),getRightPosition());
	
	return 0;
}


int EnDriverCmdQueue(int ID,u8* sendFramData,u8 len)
{
	int i=0;
	SCM sendCanmsg;
	sendCanmsg.ID=ID;
	for(i=0;i<len;i++)
	{
		sendCanmsg.data[i]=sendFramData[i];
	}
	sendCanmsg.len=len;
	
	EnQueueinter(sizeof(SCM),&sendCanmsg,DCQ);

	return 0;
}


int EnFeedBackMsgQueue(u8 Cmd,u8 state,u8 Seq,u8 len,u8 *data)
{
	STM feedBackMsg;
    u8 i;
	feedBackMsg.Head=0xCC;
    feedBackMsg.Seq=Seq;
	feedBackMsg.Cmd=Cmd;
	feedBackMsg.state=state;
	feedBackMsg.len=len;
    
    memset(feedBackMsg.data,sizeof(feedBackMsg.data),0);
	memcpy(feedBackMsg.data,data,len);
	feedBackMsg.Bcc=getXorCheck(data,len);

    feedBackMsg.data[len] = feedBackMsg.Bcc;
    feedBackMsg.Bcc=0;
    for(i=0;i<len;i++)
        feedBackMsg.Bcc ^= feedBackMsg.data[i];  
    if(feedBackMsg.Bcc != feedBackMsg.data[len])
        printf("Error,checkSum\r\n");
	EnQueueinter(sizeof(STM),&feedBackMsg,FMQ);
	u6_Send(&feedBackMsg.Head,len+EXTERN_BUFSIZE);
	return 0;
}

int InitDriver(void)
{
	int i=0;
	SCM sendCanmsg;
	for(i=1;i<=3;i++)
	{
		sendCanmsg.ID=i;
		setTVelocityFram(POS_MAX_SPEED,sendCanmsg.data);
		sendCanmsg.len=0x8;
		EnQueueinter(sizeof(SCM),&sendCanmsg,DCQ);
		
		delay_ms(2);		
		EnableFram(sendCanmsg.data);
		EnQueueinter(sizeof(SCM),&sendCanmsg,DCQ);
		
		delay_ms(2);
		getStautsFram(sendCanmsg.data);
		EnQueueinter(sizeof(SCM),&sendCanmsg,DCQ);
		
	}
	
	//打开抱闸,IO1置位On
	delay_ms(2);
	sendCanmsg.ID=0x1;
	SetIoOn(sendCanmsg.data);
	EnQueueinter(sizeof(SCM),&sendCanmsg,DCQ);
	
	return 0;
}
