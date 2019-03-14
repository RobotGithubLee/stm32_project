#include "tcp_server_demo.h"
#include "lwip/opt.h"
#include "lwip_comm.h"
#include "led.h"
#include "lwip/lwip_sys.h"
#include "lwip/api.h"
#include "Queue.h"
#include "message.h"
#include "stdbool.h"




u8 tcp_server_recvbuf[TCP_SERVER_RX_BUFSIZE];	//TCP�ͻ��˽������ݻ�����
u8 tcp_server_sendbuf[TCP_SERVER_SX_BUFSIZE];	
u8 tcp_server_flag;								//TCP���������ݷ��ͱ�־λ

extern struct interQueue *FMQ;


//TCP�ͻ�������
#define TCPSERVER_PRIO		6
//�����ջ��С
#define TCPSERVER_STK_SIZE	300
//�����ջ
OS_STK TCPSERVER_TASK_STK[TCPSERVER_STK_SIZE];

//tcp����������
static void tcp_server_thread(void *arg)
{
	OS_CPU_SR cpu_sr;
	u32 data_len = 0;
	struct pbuf *q;
	err_t err,recv_err;
	u8 remot_addr[4];
	struct netconn *conn, *newconn=NULL, *connSecond=NULL;
	static ip_addr_t ipaddr;
	static u16_t 			port;
    static bool newConFlag = false;
	
    STM sendMsg;
	
	LWIP_UNUSED_ARG(arg);

	conn = netconn_new(NETCONN_TCP);  //����һ��TCP����
	netconn_bind(conn,IP_ADDR_ANY,TCP_SERVER_PORT);  //�󶨶˿� 8�Ŷ˿�
	netconn_listen(conn);  		//�������ģʽ
	conn->recv_timeout = 20;  	//��ֹ�����߳� �ȴ�10ms
	while (1) 
	{
		//printf("tcp_server_demo.c �ȴ�����%d\r\n",err);
        err = netconn_accept(conn,&connSecond);  //������������
        //printf("tcp_server_demo.c �յ�����%d,%d\r\n",err,(int)(connSecond));
		if(err==ERR_OK)//���µ�����
        {   
            if(newconn!=NULL&&newConFlag)//֮ǰ��һ������
            {
                
                netconn_close(newconn);//�޳�֮ǰ������
				netconn_delete(newconn);
                newconn = connSecond;
                //printf("tcp_server_demo.c new����\r\n");
            }
            else//
            {
                newconn = connSecond;
            }
            newconn->recv_timeout = 20;
            newConFlag = true;
        }
		if ((err == ERR_OK || newConFlag)&& newconn!=NULL)    //���������ӵ�����
		{ 
			
            struct netbuf *recvbuf;
            if(err == ERR_OK)
           {
                netconn_getaddr(newconn,&ipaddr,&port,0); //��ȡԶ��IP��ַ�Ͷ˿ں�
                
                remot_addr[3] = (uint8_t)(ipaddr.addr >> 24); 
                remot_addr[2] = (uint8_t)(ipaddr.addr>> 16);
                remot_addr[1] = (uint8_t)(ipaddr.addr >> 8);
                remot_addr[0] = (uint8_t)(ipaddr.addr);
                printf("tcp_server_demo.c ����%d.%d.%d.%d�����Ϸ�����,�����˿ں�Ϊ:%d\r\n",remot_addr[0], remot_addr[1],remot_addr[2],remot_addr[3],port);
			}
			//while(1)
            if(newConFlag)
			{
				
				if(FMQ->QueueCount>0) //������Ҫ����
				{
					GetAndDelQueueinter(sizeof(STM), &sendMsg, FMQ);
				/*	err = netconn_write(newconn ,&sendMsg,sendMsg.len+6,NETCONN_COPY);
				  if(err != ERR_OK)
					{
						printf("tcp_server_demo.c ����ʧ��\r\n");
					}*/
				}
//				printf("----1\r\n"); 
				if((recv_err = netconn_recv(newconn,&recvbuf)) == ERR_OK)  	//���յ�����
				{		
					OS_ENTER_CRITICAL(); //���ж�
					memset(tcp_server_recvbuf,0,TCP_SERVER_RX_BUFSIZE);  //���ݽ��ջ���������
					for(q=recvbuf->p;q!=NULL;q=q->next)  //����������pbuf����
					{
						//�ж�Ҫ������TCP_SERVER_RX_BUFSIZE�е������Ƿ����TCP_SERVER_RX_BUFSIZE��ʣ��ռ䣬�������
						//�Ļ���ֻ����TCP_SERVER_RX_BUFSIZE��ʣ�೤�ȵ����ݣ�����Ļ��Ϳ������е�����
						if(q->len > (TCP_SERVER_RX_BUFSIZE-data_len)) 
                        {
							memcpy(tcp_server_recvbuf+data_len,q->payload,(TCP_SERVER_RX_BUFSIZE-data_len));//��������
                            //data_len = TCP_SERVER_RX_BUFSIZE;
                        }
						else 
                        {
							memcpy(tcp_server_recvbuf+data_len,q->payload,q->len);
                        }
                        data_len += q->len;  	
                        
						if(data_len > TCP_SERVER_RX_BUFSIZE) 
						{
							data_len=TCP_SERVER_RX_BUFSIZE;
							printf("tcp_server_demo.c data_len > TCP_SERVER_RX_BUFSIZE\r\n");
                            //newConFlag = false;
							break; //����TCP�ͻ��˽�������,����	
						}
					}
					OS_EXIT_CRITICAL();  //���ж�		
//                    printf("||||2\r\n");                    
					MessageHandle(tcp_server_recvbuf,data_len);
											
					printf("tcp_server_demo.c  tcp_server_recvbuf[0]=%x,data_len=%d\r\n",tcp_server_recvbuf[0],data_len);  //ͨ�����ڷ��ͽ��յ�������
                    data_len=0;  //������ɺ�data_lenҪ���㡣
					netbuf_delete(recvbuf);
				}else if(recv_err == ERR_CLSD)  //�ر�����
				{
					netconn_close(newconn);
					netconn_delete(newconn);
					newConFlag = false;
                    printf("tcp_server_demo.c ����:%d.%d.%d.%d�Ͽ��������������\r\n",remot_addr[0], remot_addr[1],remot_addr[2],remot_addr[3]);
					
                    //break;
				}

			}
		}
         OSTimeDlyHMSM(0,0,0,5);
	}
}


//����TCP�������߳�
//����ֵ:0 TCP�����������ɹ�
//		���� TCP����������ʧ��
INT8U tcp_server_init(void)
{
	INT8U res;
	OS_CPU_SR cpu_sr;
	
	OS_ENTER_CRITICAL();	//���ж�
	res = OSTaskCreate(tcp_server_thread,(void*)0,(OS_STK*)&TCPSERVER_TASK_STK[TCPSERVER_STK_SIZE-1],TCPSERVER_PRIO); //����TCP�������߳�
	OS_EXIT_CRITICAL();		//���ж�
	
	return res;
}


