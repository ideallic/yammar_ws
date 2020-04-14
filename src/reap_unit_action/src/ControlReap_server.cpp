// for action
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "reap_unit_action/ControlReapAction.h"

// for CAN
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <bitset>

// CAN相关函数
VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。

void *receive_func(void* param)  //接收线程。
{
	// rec是指一个数据帧，一个rec的类型是VCI_CAN_OBJ。rec.Datelen才是一个数据帧的长度。
	int i,j;
	int ind=0;
	int reclen=0; //接受一次数据之后的实际数据帧数量
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。但是一次接受不一定能够接到3000个数据帧。

	int *run=(int*)param;//线程启动，退出控制。

	while((*run)&0x0f)
	{
		// VCI_Receive(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pReceive,UINT Len,INT WaitTime);
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				printf("Index:%04d  ",count);count++;//序号递增
				printf(" ID:%02X ", rec[j].ID);
				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				printf(" data:0x");	//数据
				for(i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
				}
				printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
				printf("\n");
			}
		}
		//ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。		
	}
	printf("run thread exit\n");//退出接收线程	
	pthread_exit(0);
}

void transmit_msg(VCI_CAN_OBJ send[1])
{

	if(VCI_Transmit(VCI_USBCAN2,0,0,send,1) == 1)
	{
		printf("Index:%04d  ",count);count++;
		printf("CAN1 TX ID:0x%08X",send[0].ID);
		if(send[0].ExternFlag==0) printf(" Standard ");
		if(send[0].ExternFlag==1) printf(" Extend   ");
		if(send[0].RemoteFlag==0) printf(" Data   ");
		if(send[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send[0].DataLen;i++)
		{
			printf(" %02X",send[0].Data[i]);
		}

		printf("\n");
	}
}

typedef actionlib::SimpleActionServer<reap_unit_action::ControlReapAction> Server;

// 收到action的goal后调用的回调函数
void execute(const reap_unit_action::ControlReapGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    reap_unit_action::ControlReapFeedback feedback;

    ROS_INFO("Dishwasher %d is working.", goal->dishwasher_id);

    // 假设洗盘子的进度，并且按照1hz的频率发布进度feedback
    for(int i=1; i<=3; i++)
    {
        feedback.percent_complete = i * 10;
        as->publishFeedback(feedback);
        r.sleep();
    }

	// 当action完成后，向客户端返回结果
    ROS_INFO("Dishwasher %d finish working.", goal->dishwasher_id);


    // 进行CAN信号发送
    printf(">>start CAN device !\r\n");//指示程序已运行
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		// exit(1);
	}

	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x00;/*波特率1000 Kbps  Timing0=0x00 Timing1= 0x14*/
	config.Timing1=0x14;
	config.Mode=0;//正常模式		

	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)//CAN1
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}

	//if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)//CAN2
	//{
	//printf(">>Init can2 error\n");
	//VCI_CloseDevice(VCI_USBCAN2,0);

	//}
	//if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	//{
	//	printf(">>Start can2 error\n");
	//	VCI_CloseDevice(VCI_USBCAN2,0);

	//}
	//需要发送的帧，结构体设置//正常发送,数据帧,扩展帧,即CAN帧Data有8个字节,标准帧有效ID为11位,扩展帧29位
	//DataLen定义为3,即Data[0]、Data[1]、Data[2]是有效的
	VCI_CAN_OBJ send[1];
	send[0].ID=0x28;
	send[0].SendType=0;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;

	// int i=0;
	// for(i=0;i<send[0].DataLen;i++)
	// {
	//         send[0].Data[i] = i;
	// }
	
	// // 读取第40号电机型号
	// send[0].Data[0] = 0x04;
	// send[0].Data[1] = 0x28;
	// send[0].Data[2] = 0x01;
	// send[0].Data[3] = 0x00;
	// send[0].Data[4] = 0x00;
	// send[0].Data[5] = 0x00;
	// send[0].Data[6] = 0x00;
	// send[0].Data[7] = 0x00;
	
	
	send[0].Data[0] = 0x04;
	send[0].Data[1] = 0x28;
	send[0].Data[2] = 0x2A;
	send[0].Data[3] = 0x00;
	send[0].Data[4] = 0x00;
	send[0].Data[5] = 0x00;
	send[0].Data[6] = 0x00;
	send[0].Data[7] = 0x00;


	int m_run0=1;
	pthread_t threadid;
	int ret;
	ret=pthread_create(&threadid,NULL,receive_func,&m_run0);
	// 这一小段的接收函数有什么用处呢？应该是打开接收的线程

	transmit_msg(send);
	// int times = 1;
	// while(times--)
	// {
	//         if(VCI_Transmit(VCI_USBCAN2,0,0,send,1) == 1)
	//         {
	//                 printf("Index:%04d  ",count);count++;
	//                 printf("CAN1 TX ID:0x%08X",send[0].ID);
	//                 if(send[0].ExternFlag==0) printf(" Standard ");
	//                 if(send[0].ExternFlag==1) printf(" Extend   ");
	//                 if(send[0].RemoteFlag==0) printf(" Data   ");
	//                 if(send[0].RemoteFlag==1) printf(" Remote ");
	//                 printf("DLC:0x%02X",send[0].DataLen);
	//                 printf(" data:0x");
        //
	//                 for(int i=0;i<send[0].DataLen;i++)
	//                 {
	//                         printf(" %02X",send[0].Data[i]);
	//                 }
        //
	//                 printf("\n");
	//         }
	//         else
	//         {
	//                 break;
	//         }
        //
	//         //if(VCI_Transmit(VCI_USBCAN2, 0, 1, send, 1) == 1)
	//         //{
	//         //printf("Index:%04d  ",count);count++;
	//         //printf("CAN2 TX ID:0x%08X", send[0].ID);
	//         //if(send[0].ExternFlag==0) printf(" Standard ");
	//         //if(send[0].ExternFlag==1) printf(" Extend   ");
	//         //if(send[0].RemoteFlag==0) printf(" Data   ");
	//         //if(send[0].RemoteFlag==1) printf(" Remote ");
	//         //printf("DLC:0x%02X",send[0].DataLen);
	//         //printf(" data:0x");
	//         //for(i = 0; i < send[0].DataLen; i++)
	//         //{
	//         // printf(" %02X", send[0].Data[i]);
	//         //}
	//         //printf("\n");
	//         //send[0].ID+=1;
	//         //}
	//         //else break;
	// }

	usleep(2000000);


	send[0].ID=0x28;
	send[0].SendType=0;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;

	send[0].Data[0] = 0x08;
	send[0].Data[1] = 0x28;
	send[0].Data[2] = 0x90;
	send[0].Data[3] = 0x00;
	send[0].Data[4] = 0xD0;
	send[0].Data[5] = 0x05;
	send[0].Data[6] = 0x00;
	send[0].Data[7] = 0x00;

	transmit_msg(send);
	// times = 1;
	// while(times--)
	// {
	//         if(VCI_Transmit(VCI_USBCAN2,0,0,send,1) == 1)
	//         {
	//                 printf("Index:%04d  ",count);count++;
	//                 printf("CAN1 TX ID:0x%08X",send[0].ID);
	//                 if(send[0].ExternFlag==0) printf(" Standard ");
	//                 if(send[0].ExternFlag==1) printf(" Extend   ");
	//                 if(send[0].RemoteFlag==0) printf(" Data   ");
	//                 if(send[0].RemoteFlag==1) printf(" Remote ");
	//                 printf("DLC:0x%02X",send[0].DataLen);
	//                 printf(" data:0x");
        //
	//                 for(int i=0;i<send[0].DataLen;i++)
	//                 {
	//                         printf(" %02X",send[0].Data[i]);
	//                 }
        //
	//                 printf("\n");
	//         }
	//         else
	//         {
	//                 break;
	//         }
        //
	//         //if(VCI_Transmit(VCI_USBCAN2, 0, 1, send, 1) == 1)
	//         //{
	//         //printf("Index:%04d  ",count);count++;
	//         //printf("CAN2 TX ID:0x%08X", send[0].ID);
	//         //if(send[0].ExternFlag==0) printf(" Standard ");
	//         //if(send[0].ExternFlag==1) printf(" Extend   ");
	//         //if(send[0].RemoteFlag==0) printf(" Data   ");
	//         //if(send[0].RemoteFlag==1) printf(" Remote ");
	//         //printf("DLC:0x%02X",send[0].DataLen);
	//         //printf(" data:0x");
	//         //for(i = 0; i < send[0].DataLen; i++)
	//         //{
	//         // printf(" %02X", send[0].Data[i]);
	//         //}
	//         //printf("\n");
	//         //send[0].ID+=1;
	//         //}
	//         //else break;
	// }


	//需要发送的帧，结构体设置//正常发送,数据帧,扩展帧,即CAN帧Data有8个字节,标准帧有效ID为11位,扩展帧29位
	//DataLen定义为3,即Data[0]、Data[1]、Data[2]是有效的
	send[0].ID=0x27;
	send[0].SendType=0;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;

	// int i=0;
	// for(i=0;i<send[0].DataLen;i++)
	// {
	//         send[0].Data[i] = i;
	// }
	
	// // 读取第40号电机型号
	// send[0].Data[0] = 0x04;
	// send[0].Data[1] = 0x28;
	// send[0].Data[2] = 0x01;
	// send[0].Data[3] = 0x00;
	// send[0].Data[4] = 0x00;
	// send[0].Data[5] = 0x00;
	// send[0].Data[6] = 0x00;
	// send[0].Data[7] = 0x00;
	
	
	send[0].Data[0] = 0x04;
	send[0].Data[1] = 0x27;
	send[0].Data[2] = 0x2A;
	send[0].Data[3] = 0x00;
	send[0].Data[4] = 0x00;
	send[0].Data[5] = 0x00;
	send[0].Data[6] = 0x00;
	send[0].Data[7] = 0x00;

	transmit_msg(send);

	// times = 1;
	// while(times--)
	// {
	//         if(VCI_Transmit(VCI_USBCAN2,0,0,send,1) == 1)
	//         {
	//                 printf("Index:%04d  ",count);count++;
	//                 printf("CAN1 TX ID:0x%08X",send[0].ID);
	//                 if(send[0].ExternFlag==0) printf(" Standard ");
	//                 if(send[0].ExternFlag==1) printf(" Extend   ");
	//                 if(send[0].RemoteFlag==0) printf(" Data   ");
	//                 if(send[0].RemoteFlag==1) printf(" Remote ");
	//                 printf("DLC:0x%02X",send[0].DataLen);
	//                 printf(" data:0x");
        //
	//                 for(int i=0;i<send[0].DataLen;i++)
	//                 {
	//                         printf(" %02X",send[0].Data[i]);
	//                 }
        //
	//                 printf("\n");
	//         }
	//         else
	//         {
	//                 break;
	//         }
        //
	//         //if(VCI_Transmit(VCI_USBCAN2, 0, 1, send, 1) == 1)
	//         //{
	//         //printf("Index:%04d  ",count);count++;
	//         //printf("CAN2 TX ID:0x%08X", send[0].ID);
	//         //if(send[0].ExternFlag==0) printf(" Standard ");
	//         //if(send[0].ExternFlag==1) printf(" Extend   ");
	//         //if(send[0].RemoteFlag==0) printf(" Data   ");
	//         //if(send[0].RemoteFlag==1) printf(" Remote ");
	//         //printf("DLC:0x%02X",send[0].DataLen);
	//         //printf(" data:0x");
	//         //for(i = 0; i < send[0].DataLen; i++)
	//         //{
	//         // printf(" %02X", send[0].Data[i]);
	//         //}
	//         //printf("\n");
	//         //send[0].ID+=1;
	//         //}
	//         //else break;
	// }

	usleep(2000000);


	send[0].ID=0x27;
	send[0].SendType=0;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;

	send[0].Data[0] = 0x08;
	send[0].Data[1] = 0x27;
	send[0].Data[2] = 0x90;
	send[0].Data[3] = 0x00;
	send[0].Data[4] = 0xD0;
	send[0].Data[5] = 0x05;
	send[0].Data[6] = 0x00;
	send[0].Data[7] = 0x00;

	transmit_msg(send);
	// times = 1;
	// while(times--)
	// {
	//         if(VCI_Transmit(VCI_USBCAN2,0,0,send,1) == 1)
	//         {
	//                 printf("Index:%04d  ",count);count++;
	//                 printf("CAN1 TX ID:0x%08X",send[0].ID);
	//                 if(send[0].ExternFlag==0) printf(" Standard ");
	//                 if(send[0].ExternFlag==1) printf(" Extend   ");
	//                 if(send[0].RemoteFlag==0) printf(" Data   ");
	//                 if(send[0].RemoteFlag==1) printf(" Remote ");
	//                 printf("DLC:0x%02X",send[0].DataLen);
	//                 printf(" data:0x");
        //
	//                 for(int i=0;i<send[0].DataLen;i++)
	//                 {
	//                         printf(" %02X",send[0].Data[i]);
	//                 }
        //
	//                 printf("\n");
	//         }
	//         else
	//         {
	//                 break;
	//         }
        //
	//         //if(VCI_Transmit(VCI_USBCAN2, 0, 1, send, 1) == 1)
	//         //{
	//         //printf("Index:%04d  ",count);count++;
	//         //printf("CAN2 TX ID:0x%08X", send[0].ID);
	//         //if(send[0].ExternFlag==0) printf(" Standard ");
	//         //if(send[0].ExternFlag==1) printf(" Extend   ");
	//         //if(send[0].RemoteFlag==0) printf(" Data   ");
	//         //if(send[0].RemoteFlag==1) printf(" Remote ");
	//         //printf("DLC:0x%02X",send[0].DataLen);
	//         //printf(" data:0x");
	//         //for(i = 0; i < send[0].DataLen; i++)
	//         //{
	//         // printf(" %02X", send[0].Data[i]);
	//         //}
	//         //printf("\n");
	//         //send[0].ID+=1;
	//         //}
	//         //else break;
	// }

	usleep(10000000);//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
	m_run0=0;//线程关闭指令。
	pthread_join(threadid,NULL);//等待线程关闭。
	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
	usleep(100000);//延时100ms。
	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
	//除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
	//goto ext;
    as->setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_reap_server");
    ros::NodeHandle n;

	// 定义一个服务器，do_dishes就是topic
    Server server(n, "control_reap", boost::bind(&execute, _1, &server), false);
	
	// 服务器开始运行
    server.start();

    ros::spin();

    return 0;
}
