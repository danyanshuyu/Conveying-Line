/********************************** (C) COPYRIGHT ******************************
* File Name          : CH563.C
* Author             : WCH
* Version            : V1.0
* Date               : 2013/11/15
* Description        : CH563NET库演示文件
*                      (1)、CH563 Examples by KEIL;
*                      (2)、本程序用于演示UDP通讯，单片机收到数据后，回传给远端，ARM966,Thumb,小端
*******************************************************************************/



/******************************************************************************/
/* 头文件包含*/
#include <stdio.h>
#include <string.h>
#include "CH563SFR.H"
#include "Extern_Variables.h"

#include "CH563NET.H"
#define CH563NET_DBG                          1

/* 下面的缓冲区和全局变量必须要定义，库中调用 */
__align(16)UINT8    CH563MACRxDesBuf[(RX_QUEUE_ENTRIES )*16];        /* MAC接收描述符缓冲区，16字节对齐 */
__align(4) UINT8    CH563MACRxBuf[RX_QUEUE_ENTRIES*RX_BUF_SIZE];	 /* MAC接收缓冲区，4字节对齐 */
__align(4) SOCK_INF SocketInf[CH563NET_MAX_SOCKET_NUM];			     /* Socket信息表，4字节对齐 */
const UINT16 MemNum[8] = {CH563NET_NUM_IPRAW,
	                     CH563NET_NUM_UDP,
	                     CH563NET_NUM_TCP,
	                     CH563NET_NUM_TCP_LISTEN,
	                     CH563NET_NUM_TCP_SEG,
	                     CH563NET_NUM_IP_REASSDATA,
	                     CH563NET_NUM_PBUF,
	                     CH563NET_NUM_POOL_BUF
	                     };
const UINT16 MemSize[8] = {CH563NET_MEM_ALIGN_SIZE(CH563NET_SIZE_IPRAW_PCB),
	                      CH563NET_MEM_ALIGN_SIZE(CH563NET_SIZE_UDP_PCB),
	                      CH563NET_MEM_ALIGN_SIZE(CH563NET_SIZE_TCP_PCB),
	                      CH563NET_MEM_ALIGN_SIZE(CH563NET_SIZE_TCP_PCB_LISTEN),
	                      CH563NET_MEM_ALIGN_SIZE(CH563NET_SIZE_TCP_SEG),
	                      CH563NET_MEM_ALIGN_SIZE(CH563NET_SIZE_IP_REASSDATA),
	                      CH563NET_MEM_ALIGN_SIZE(CH563NET_SIZE_PBUF) + CH563NET_MEM_ALIGN_SIZE(0),
	                      CH563NET_MEM_ALIGN_SIZE(CH563NET_SIZE_PBUF) + CH563NET_MEM_ALIGN_SIZE(CH563NET_SIZE_POOL_BUF)
	                     };
__align(4)UINT8 Memp_Memory[CH563NET_MEMP_SIZE];
__align(4)UINT8 Mem_Heap_Memory[CH563NET_RAM_HEAP_SIZE];
__align(4)UINT8 Mem_ArpTable[CH563NET_RAM_ARP_TABLE_SIZE];
/******************************************************************************/
/* 本演示程序的相关宏 */
#define RECE_BUF_LEN                          1600                              /* 接收缓冲区的大小 */
/* CH563相关定义 */
const UINT8 MACAddr[6] = {0x02,0x03,0x04,0x05,0x06,0x07};                       /* CH563MAC地址 */
      UINT8 IPAddr[4] = {192,168,1,12};                                         /* CH563IP地址 *///最后一位地址根据实际拨码盘上的ip更改
const UINT8 GWIPAddr[4] = {192,168,1,1};                                        /* CH563网关 */
const UINT8 IPMask[4] = {255,255,255,0};                                        /* CH563子网掩码 */
const UINT8 DESIP[4] = {192,168,1,100};                                         /* 目的IP地址 */
      UINT8 ID_Number = 0;														/* 下位机板子编号 */
UINT8 SocketId;                                                                 /* 保存socket索引，可以不用定义 */
UINT8 SocketRecvBuf[RECE_BUF_LEN];                                              /* socket接收缓冲区 */
UINT8 FirstBuf_Rx[RECE_BUF_LEN];                                                /* 第一层缓冲区，socket库调用 */
UINT8 SecondBuf_Rx[RECE_BUF_LEN];                                               /* 第二层缓冲区，用于保证数据完整性 */
UINT32 Front = 0;																//SecondBuf_Rx队列当前填充起始位
volatile BUFF_UDP_DATA Buff_Rx;			                                        //过程数据



/*******************************************************************************
* Function Name  : IRQ_Handler
* Description    : IRQ中断服务函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__irq void IRQ_Handler( void )
{
    if(R32_INT_FLAG & 0x8000)                                                   /* 以太网中断 */
    {                                                                           /* 以太网中断中断服务函数 */
        CH563NET_ETHIsr();
    }
    if(R32_INT_FLAG & RB_IF_TMR0)                                               /* 定时器0中断 */
    {
         CH563NET_TimeIsr(CH563NETTIMEPERIOD);                                  /* 定时器中断服务函数 */
         R8_TMR0_INT_FLAG |= 0xff;                                              /* 清除定时器中断标志 */
    }
	if(R32_INT_FLAG & RB_IF_TMR1)                                               /* 定时器1中断 */
    {

		 Proc_plc_code();                                                       /* 每过5ms执行一次PLC扫描程序 */
		 PLC_Count++;
         R8_TMR1_INT_FLAG |= 0xff;                                              /* 清除定时器中断标志 */
    }

}

__irq void FIQ_Handler( void )
{
    while(1);
}

/*******************************************************************************
* Function Name  : SysTimeInit
* Description    : 系统定时器初始化，CH563@100MHZ TIME0 10ms，根据CH563NETTIMEPERIOD
*                ：来初始化定时器。
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTimeInit(void)
{
    R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R32_TMR0_COUNT = 0x00000000; 
    R32_TMR0_CNT_END = 0x186a0 * CH563NETTIMEPERIOD;                            /* 设置为10MS定时 */
    R8_TMR0_INTER_EN |= RB_TMR_IE_CYC_END;
    R8_TMR0_CTRL_MOD = RB_TMR_COUNT_EN;
}

/*******************************************************************************
* Function Name  : InitSysHal
* Description    : 硬件初始化操作，开启TIM0，ETH中断
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitSysHal(void)
{
    R8_INT_EN_IRQ_0 |= RB_IE_IRQ_TMR0;                                          /* 开启TIM0中断 */
    R8_INT_EN_IRQ_1 |= RB_IE_IRQ_ETH;                                           /* 开启ETH中断 */
    R8_INT_EN_IRQ_GLOB |= RB_IE_IRQ_GLOB;                                       /* 开启IRQ全局中断 */
}

/*******************************************************************************
* Function Name  : mStopIfError
* Description    : 调试使用，显示错误代码
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void mStopIfError(UINT8 iError)
{
    if (iError == CH563NET_ERR_SUCCESS) return;                                 /* 操作成功 */
#if CH563NET_DBG
    printf("Error: %02X\n", (UINT16)iError);                                    /* 显示错误 */
#endif    
}

/*******************************************************************************
* Function Name  : CH563NET_LibInit
* Description    : 库初始化操作
* Input          : ip      ip地址指针
*                ：gwip    网关ip地址指针
*                : mask    掩码指针
*                : macaddr MAC地址指针 
* Output         : None
* Return         : 执行状态
*******************************************************************************/
UINT8 CH563NET_LibInit(const UINT8 *ip,const UINT8 *gwip,const UINT8 *mask,const UINT8 *macaddr)
{
    UINT8 i;
    struct _CH563_CFG cfg;
    if(CH563NET_GetVer() != CH563NET_LIB_VER)return 0xfc;             /* 获取库的版本号，检查是否和头文件一致 */
    CH563NETConfig = LIB_CFG_VALUE;									  /* 将配置信息传递给库的配置变量 */
    cfg.RxBufSize = RX_BUF_SIZE; 
    cfg.TCPMss   = CH563NET_TCP_MSS;
    cfg.HeapSize = CH563_MEM_HEAP_SIZE;
    cfg.ARPTableNum = CH563NET_NUM_ARP_TABLE;
    cfg.MiscConfig0 = CH563NET_MISC_CONFIG0;
	CH563NET_ConfigLIB(&cfg);
    i = CH563NET_Init(ip,gwip,mask,macaddr);
    return (i); 				     /* 库初始化 */
}

/*******************************************************************************
* Function Name  : CH563NET_HandleSockInt
* Description    : Socket中断处理函数
* Input          : sockeid  socket索引
*                ：initstat 中断状态
* Output         : None
* Return         : None
*******************************************************************************/
void CH563NET_HandleSockInt(UINT8 sockeid,UINT8 initstat)
{
    UINT32 Len_First;
	UINT16 Lengh;
	UINT32 len;	
	UINT8  *p = FirstBuf_Tx;
	int i;

    if(initstat & SINT_STAT_RECV)                                                     /* 接收中断 */
    {
        Len_First = CH563NET_SocketRecvLen(sockeid,NULL);                             /* 会将当前接收指针传递给precv*/
#if CH563NET_DBG
        printf("Receive Len = %02x\n",Len_First);                           
#endif

		CH563NET_SocketRecv(sockeid,FirstBuf_Rx,&Len_First);                         /* 将接收缓冲区的数据读到FirstBuf_Rx中*/

		memcpy(&SecondBuf_Rx[Front],FirstBuf_Rx,Len_First);							//将FirstBuf_Rx中的数据拷贝到SecondBuf_Rx中
		
		Front += Len_First;
		
		/*以下程序用于验证数据完整性*/
		while(1) 
		{																			
			if(Front >= 4)
			{
				Lengh = (SecondBuf_Rx[0]<<8|SecondBuf_Rx[1]);

				if(Front >= Lengh)
				{
					memcpy((void *)&Buff_Rx,SecondBuf_Rx,Lengh);					//将SecondBuf_Rx中的数据拷贝到结构体Buff_Rx中，	注意u8到u16，0x12 0x34 变为 0x3412

					for(i = 0;i < (Front - Lengh);i++)								//切取完整数据后，将SecondBuf_Rx后面的数据前移
					{
						SecondBuf_Rx[i]	= SecondBuf_Rx[i+Lengh];
					}
					
					Front -= Lengh;

					Data_Process();													//成功接收数据后进行处理

					p = FirstBuf_Tx;												//进入循环前，p指针重新指向FirstBuf_Tx首地址

					while(1)
			        {
			           len = LENGH_TX;
			           CH563NET_SocketSend(sockeid,p,&len);                            // 将FirstBuf_Tx中的数据发送
			           LENGH_TX -= len;                                                // 将总长度减去已经发送完毕的长度 
			           p += len;                                                       // 将缓冲区指针偏移
			           if(LENGH_TX)continue;                                           // 如果数据未发送完毕，则继续发送
			           break;                                                          // 发送完毕，退出 
			        }  
				}
				else
				{
					break;
				}				
			}
			if(Front >= 4) continue;
			break;
		}	 
		 	
    }
    if(initstat & SINT_STAT_CONNECT)                                            /* TCP连接中断 */
    {
    }
    if(initstat & SINT_STAT_DISCONNECT)                                         /* TCP断开中断 */
    {
    }
    if(initstat & SINT_STAT_TIM_OUT)                                            /* TCP超时中断 */
    {
    }

}

/*******************************************************************************
* Function Name  : CH563NET_HandleGloableInt
* Description    : 全局中断处理函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH563NET_HandleGlobalInt(void)
{
    UINT8 initstat;
    UINT8 i;
    UINT8 socketinit;
    
    initstat = CH563NET_GetGlobalInt();                                          /* 读全局中断状态并清除 */
    if(initstat & GINT_STAT_UNREACH)                                            /* 不可达中断 */
    {
#if CH563NET_DBG
        printf("UnreachCode ：%d\n",CH563Inf.UnreachCode);                      /* 查看不可达代码 */
        printf("UnreachProto ：%d\n",CH563Inf.UnreachProto);                    /* 查看不可达协议类型 */
        printf("UnreachPort ：%d\n",CH563Inf.UnreachPort);                      /* 查询不可达端口 */
#endif       
    }
   if(initstat & GINT_STAT_IP_CONFLI)                                           /* IP冲突中断 */
   {
   
   }
   if(initstat & GINT_STAT_PHY_CHANGE)                                          /* PHY改变中断 */
   {
       i = CH563NET_GetPHYStatus();                                             /* 获取PHY状态 */
#if CH563NET_DBG
       printf("GINT_STAT_PHY_CHANGE %02x\n",i);
#endif   
   }
   if(initstat & GINT_STAT_SOCKET)                                              /* Socket中断 */
   {
       for(i = 0; i < CH563NET_MAX_SOCKET_NUM; i ++)                     
       {
           socketinit = CH563NET_GetSocketInt(i);                               /* 读socket中断并清零 */
           if(socketinit)CH563NET_HandleSockInt(i,socketinit);                  /* 如果有中断则清零 */
       }    
   }
}

/*******************************************************************************
* Function Name  : CH563NET_CreatUpdSocket
* Description    : 创建UDP socket
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH563NET_CreatUpdSocket(void)
{
   UINT8 i;                                                             
   SOCK_INF TmpSocketInf;                                                       /* 创建临时socket变量 */

   memset((void *)&TmpSocketInf,0,sizeof(SOCK_INF));                            /* 库内部会将此变量复制，所以最好将临时变量先全部清零 */
   memcpy((void *)TmpSocketInf.IPAddr,DESIP,4);                                 /* 设置目的IP地址 */
   TmpSocketInf.DesPort = 27000;                                                 /* 设置目的端口 */
   TmpSocketInf.SourPort = 28000;                                                /* 设置源端口 */
   TmpSocketInf.ProtoType = PROTO_TYPE_UDP;                                     /* 设置socekt类型 */
   TmpSocketInf.RecvStartPoint = (UINT32)SocketRecvBuf;                         /* 设置接收缓冲区的接收缓冲区 */
   TmpSocketInf.RecvBufLen = RECE_BUF_LEN ;                                     /* 设置接收缓冲区的接收长度 */
   i = CH563NET_SocketCreat(&SocketId,&TmpSocketInf);                           /* 创建socket，将返回的socket索引保存在SocketId中 */
   mStopIfError(i);                                                             /* 检查错误 */
}


/*********************************** endfile **********************************/
