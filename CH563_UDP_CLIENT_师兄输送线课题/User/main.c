/********************************** (C) COPYRIGHT ******************************
* File Name          : main.c
* Author             : LN
* Version            : V1.0
* Date               : 2017/04/25
* Description        : 自动化运输线_带网络通讯模块_下位机逻辑动作控制器
*                      串口0输出监控信息,115200bps;
*******************************************************************************/

/******************************************************************************/
/* 头文件包含*/
#include <stdio.h>
#include <string.h>
#include "CH563SFR.H"
#include "SYSFREQ.H"
#include "CH563NET.H"
#include "ISPXT563.H"
#include "Extern_Variables.h"

#define CH563NET_DBG                          1

/* 变量定义 */
volatile BUFF_UDP_DATA Buff_Tx;			                                        //从上位机接收的缓存数据
volatile UINT8 FirstBuf_Tx[1000];												//定义第一层数据发送缓冲区
UINT32  LENGH_TX;														//数据处理后，Socket库函数发送给上位机所调用的字符长度
UINT16  EEPROM_CODE[512]; 		                                        //1024字节缓存，用于存放PLC代码，在RAM中运行
UINT8   PLC_Count = 0;													//PLC扫描时间计数，满足次数时，向上位机发送一次开关量输出状态

/* 函数声明 */
UINT8 Reverse(UINT8 a,UINT8 n );

/*******************************************************************************
* Function Name  : Init_GPIO
* Description    : 为开关量输入输出初始化GPIO	(22+20+32)=74
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Init_GPIO( void )
{
	/* GPIO  输入输出设置,0输入,1输出 */
    R8_PD_DIR_3 &= ~(1<<7);														//PD31 PB0~6,PB16~19 PD8~11为输入
    R8_PB_DIR_0 &= ~(0x7F);														//16位输入
	R8_PB_DIR_2 &= ~(0x0F);
	R8_PD_DIR_1 &= ~(0x0F);

	R8_PD_DIR_1 |= 0xF0;													    //PD12~15 PA0~3,PA4~7 PA12~15,PA8~11 PA16~17为输出
	R32_PA_DIR  |= 0x0003FFFF;														//22位输出

	/* GPIO  上拉设置,置1启用上拉,置0关闭上拉 */							   //下拉也可以，但不能浮空，未接入开关量时，数据容易受干扰而不断变化
	R8_PD_PU_3 |= 1<<7;													
	R8_PB_PU_0 |= 0x7F;                                                   
	R8_PB_PU_2 |= 0x0F;														
	R8_PD_PU_1 |= 0x0F;

	R8_PD_OUT_1 |= 0xF0;														//置1，高电平时输出无效
	R32_PA_OUT  |= 0x0003FFFF;

	/* ip的硬件输入，板子编号的硬件输入 */
	R8_PB_DIR_1 &= ~(0xF3);														//PB 8  9  12  13  14  15  PD 0  1			PD 2  3  4  5  6  7  
	R8_PD_DIR_0 &= ~(0xFF);														//SW 1  2  3   4   5   6      7  8			SW 9  10 11 12 13 14
	
	/* 下拉 */
//	R8_PB_PD_1 |= 0xF3;
//	R8_PD_PD_0 |= 0xFF;
}

/*******************************************************************************
* Function Name  : mInitSTDIO
* Description    : 为printf和getkey输入输出初始化串口
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void mInitSTDIO( void )
{
    UINT32    x, x2;

    x = 10 * FREQ_SYS * 2 / 16 / 115200;                                        /* 115200bps */
    x2 = x % 10;
    x /= 10;
    if ( x2 >= 5 ) x ++;                                                        /* 四舍五入 */
    R8_UART1_LCR = 0x80;                                                        /* DLAB位置1 */
    R8_UART1_DIV = 1;                                                           /* 预分频 */
    R8_UART1_DLM = x>>8;
    R8_UART1_DLL = x&0xff;

    R8_UART1_LCR = RB_LCR_WORD_SZ ;                                             /* 设置字节长度为8 */
    R8_UART1_FCR = RB_FCR_FIFO_TRIG|RB_FCR_TX_FIFO_CLR|RB_FCR_RX_FIFO_CLR |    
                   RB_FCR_FIFO_EN ;                                             /* 设置FIFO触发点为14，清发送和接收FIFO，FIFO使能 */
    R8_UART1_IER = RB_IER_TXD_EN;                                               /* TXD enable */
    R32_PB_SMT |= RXD1|TXD1;                                                    /* RXD0 schmitt input, TXD0 slow rate */
    R32_PB_PD &= ~ RXD1;                                                        /* disable pulldown for RXD0, keep pullup */
    R32_PB_DIR |= TXD1;                                                         /* TXD0 output enable */
}

/*******************************************************************************
* Function Name  : fputc
* Description    : 通过串口输出监控信息
* Input          : c-- writes the character specified by c 
*                  *f--the output stream pointed to by *f
* Output         : None
* Return         : None
*******************************************************************************/

int fputc( int c, FILE *f )
{
    R8_UART1_THR = c;                                                           /* 发送数据 */
    while( ( R8_UART1_LSR & RB_LSR_TX_FIFO_EMP ) == 0 );                        /* 等待数据发送 */
    return( c );
}

/*******************************************************************************
* Function Name  : CH563_EEPROM
* Description    : EEPROM操作子程序
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void CH563_EEPROM( void ) 
{

  //  CH563_EEPROM_ERASE(0x2000,0x1000);                                           //  擦除0X2000地址开始的4K数据 

  //  CH563_EEPROM_WRITE( 0x2000,my_buffer,64 );                                   //  往地址0X2000写64字节的数据
 
  //  CH563_EEPROM_READ( 0x2000,my_buffer,64 );                                    //  EEPROM读取地址0x2000数据，读取64字节 
 
}

/*******************************************************************************
* Function Name  : InitTIM1
* Description    : 初始化定时器1，及其中断配置		5ms
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitTIM1(void)
{
    R8_TMR1_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R32_TMR1_COUNT = 0x00000000; 
    R32_TMR1_CNT_END = 0x186a0 * 5;                                             /* 设置为5MS定时 */
    R8_TMR1_INTER_EN |= RB_TMR_IE_CYC_END;
    R8_TMR1_CTRL_MOD = RB_TMR_COUNT_EN;										//启动定时器功能


	R8_INT_EN_IRQ_0 |= RB_IE_IRQ_TMR1;                                          /* 开启TIM1中断 */
    R8_INT_EN_IRQ_GLOB |= RB_IE_IRQ_GLOB;                                       /* 开启IRQ全局中断 */
}


/*******************************************************************************
* Function Name  : Data_Process
* Description    : 接收上位机的数据并处理
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Data_Process()
{
    UINT16  LENGH  = ( Buff_Rx.Lengh>>8 | Buff_Rx.Lengh<<8 );		// 包长度,存储在Buff_Rx中的数据高低位已被互换了一次，这里要换回来
	UINT16  R_Head = ( Buff_Rx.Head>>8 | Buff_Rx.Head<<8 );			// 接收的报文头
	UINT16  T_Head = 0;												// 发送的报文头
	UINT16  T_IP = (IPAddr[3]>>8 | IPAddr[3]<<8);					// 发送的下位机IP的第四位
	UINT16V Input_Port = 0;							                // 该变量对应硬件中读出的开关量输入
	UINT32  Output_Port = 0x00000000;								// 该变量对应写入的开关量输出
	int i,Num;
	
    if( R_Head == PLC_Q )									        // 接受到的报文头为 PLC代码
	{	
        T_Head = LOWER_ANSW;
		
		CH563_EEPROM_ERASE( PLC_Code_Addr,0x1000);                              //  擦除0x0000地址开始的4K数据
		CH563_EEPROM_WRITE( PLC_Code_Addr,Buff_Rx.Data.PLC_Cmd,LENGH-4 );       //  将PLC程序写入EEPROM中，起始地址0x0000
		CH563_EEPROM_READ ( 0,EEPROM_CODE,LENGH-4 );                            //  EEPROM读取地址0x0000数据，读取LENGH-4字节到EEPROM_CODE中

		for(i=0;i<(LENGH/2-2);i++)				                                // 小端模式的u8 0x12 0x34转换u16 为0x3412，故此处要换回来
		{
			EEPROM_CODE[i] = (UINT16)(EEPROM_CODE[i]<<8|EEPROM_CODE[i]>>8);	
		}

		R8_TMR1_CTRL_MOD = RB_TMR_COUNT_EN;									    // 开定时器1，开始PLC代码扫描

	}
	if( R_Head == GET_IN )									        // 上位机获取开关量输入
	{	
		
		T_Head = GET_IN;
		Buff_Tx.Lengh = ( 0x0008>>8 | 0x0008<<8 );
		Buff_Tx.Head  =	T_Head;
		LENGH_TX = 8;		
		Input_Port = (UINT16)( (R8_PB_PIN_2 & 0x0F) | ((R8_PD_PIN_1<<4) & 0xF0))<<8 | (UINT16)( (R8_PD_PIN_3>>7) & 0x01 | R8_PB_PIN_0<<1 );	 
																   // 从GPIO中读出开关量的信号,PB16~19 PD8~11,PD31 PB0~6
																                              //IN     7~0   ,    15~8
		//Input_Port = 0xFF00 | ((R8_PD_PIN_0 & 0xFC)>>2);//拨码开关模块测试
		//Input_Port = 0xFF00 | ((R8_PB_PIN_1 & 0x03)<<4 | (R8_PB_PIN_1 & 0x30)<<2 | (R8_PB_PIN_1 & 0xC0)>>6 | (R8_PD_PIN_0 & 0x03)<<2);//拨码盘模块测试
		Buff_Tx.Data.IO_Data[0] = T_IP;
		Buff_Tx.Data.IO_Data[1] = Input_Port ;
		memcpy(FirstBuf_Tx,(void *)&Buff_Tx,LENGH_TX);		   //将Buff_Tx中的数据拷贝到结构体FirstBuf_Tx中，	注意u16到u8，0x3412 变为 0x12 0x34

	}
	if( R_Head == SET_OUT )									        // 上位机设置开关量输出
	{																//上位机发送OUT格式 7~0,15~8,23~16,31~24(只有0~21位有效)
																	//                0x12   34   56   78
																	//储存在Buff_Rx.Data.IO_Data[0]和[1]中的格式（小端模式，高字节高地址，低字节低地址）
																	//				  0x3412       0x7856
		T_Head =  SET_OUT;
		Buff_Tx.Lengh = ( 0x000a>>8 | 0x000a<<8 );
		Buff_Tx.Head  =	T_Head;
		LENGH_TX = 10;
		
		/*从上位机广播帧中找出本地IP对应IO端口Output_Port值*/
		for(i = 0;i < (LENGH-4)/6;i += 3)
		{
			if(Buff_Rx.Data.IO_Data[i] == T_IP)
			{
			    Output_Port = Buff_Rx.Data.IO_Data[i+2]<<16 | (UINT32) Buff_Rx.Data.IO_Data[i+1] ;	 
																   // 对应GPIO中要写入的开关量输出信号，OUT0~21
				break;
			}
		}
				
	
		/*设置对应每一个开关量的值*/
		for(Num=0;Num<=21;Num++)
		{			
			if( (Output_Port>>Num & 0x00000001) == 1 )		 // 如果该信号量设置为1，则输出端无效				
			{
				if(0<=Num && Num<=3)
				R8_PD_OUT_1 |= (1<<(Num+4));					//开关量输出与GPIO引脚对应关系
				else if(Num<=11)								//OUT  21 20 19 18 17 16 15 14 13 12 11 10  9 8 7 6 5 4     3  2  1  0
				R8_PA_OUT_0 |= (1<<(Num-4));					//PA   17 16 11 10  9  8 15 14 13 12  7  6  5 4 3 2 1 0 PD 15 14 13 12 
				else if(Num<=15)								  		   
				R8_PA_OUT_1 |= (1<<(Num-8));					
				else if(Num<=19)
				R8_PA_OUT_1 |= (1<<(Num-16));
				else if(Num<=21)
				R8_PA_OUT_2 |= (1<<(Num-20));
			}
			else if((Output_Port>>Num & 0x00000001) == 0 )	 // 如果设置值为0，则对应位输出有效,即低电平有效
			{
				if(0<=Num && Num<=3)
				R8_PD_OUT_1 &= ~(1<<(Num+4));
				else if(Num<=11)
				R8_PA_OUT_0 &= ~(1<<(Num-4));
				else if(Num<=15)
				R8_PA_OUT_1 &= ~(1<<(Num-8));
				else if(Num<=19)
				R8_PA_OUT_1 &= ~(1<<(Num-16));
				else if(Num<=21)
				R8_PA_OUT_2 &= ~(1<<(Num-20));
			}
		}
		Buff_Tx.Data.IO_Data[0] = T_IP ;
		Buff_Tx.Data.IO_Data[1] = Buff_Rx.Data.IO_Data[i+1] ;
		Buff_Tx.Data.IO_Data[2] = Buff_Rx.Data.IO_Data[i+2] ;
		memcpy(FirstBuf_Tx,(void *)&Buff_Tx,LENGH_TX);		   //将Buff_Tx中的数据拷贝到结构体FirstBuf_Tx中，	注意u16到u8，0x3412 变为 0x12 0x34

	}
    if( R_Head == GET_OUT )									        // 上位机获取开关量输出
	{	
		
		T_Head = GET_OUT;
		Buff_Tx.Lengh = ( 0x000a>>8 | 0x000a<<8 );
		Buff_Tx.Head  =	T_Head;
		LENGH_TX = 10;		
		Output_Port = 0xFFFFFFFF;
		
		for(Num=0;Num<=21;Num++)
		{
			if(*((UINT16 *)EEPROM_BUFF + Num + 0x30) == 1)
			Output_Port &= ~(1<<Num);	
		}
		Buff_Tx.Data.IO_Data[0] = T_IP;
		Buff_Tx.Data.IO_Data[1] =  (UINT16)Output_Port;
	    Buff_Tx.Data.IO_Data[2] =  (UINT16)(Output_Port>>16) ;

		memcpy(FirstBuf_Tx,(void *)&Buff_Tx,LENGH_TX);		 //将Buff_Tx中的数据拷贝到结构体FirstBuf_Tx中，	注意u16到u8，0x3412 变为 0x12 0x34
	
	}
	if( R_Head == SEND_IP_ID )									        // 下位机发送本地IP和ID
	{

		T_Head = SEND_IP_ID;
		Buff_Tx.Lengh = ( 0x0008>>8 | 0x0008<<8 );
	    Buff_Tx.Head  =	T_Head;
	    LENGH_TX = 8;
	
	    Buff_Tx.Data.IO_Data[0] =  (UINT16)IPAddr[3]<<8;
		Buff_Tx.Data.IO_Data[1] =  (UINT16)ID_Number<<8;
	    
		memcpy(FirstBuf_Tx,(void *)&Buff_Tx,LENGH_TX);
		
	}
	if( R_Head == SET_M000 )									        // 上位机设置“移载出”模块执行移载动作，M000为移载变量，默认为0不执行
	{
		EEPROM_BUFF[0+2*0x30] = 1;					      //设置PLC变量M000的值为1，PLC移载动作执行完之后，M000自动复位为0
	}

	//此处添加程序
}

/*******************************************************************************
* Function Name  : Reverse
* Description    : 二进制数逆序排列得到的新数
* Input          : UINT8 a 为待排序数  待排位 为0~（n-1）
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 Reverse(UINT8 a,UINT8 n )                                                        
{
    UINT8 temp = 0;
	int i;
    for (i = n-1; i >= 0; i--)
    	temp |= (UINT8)(((a>>i)&0x01)<<(n-1-i));	
    return temp;
	
}

/*******************************************************************************
* Function Name  : GetLocal_IP_ID
* Description    : 获得拨码盘和拨码开关设定的IP和ID
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GetLocal_IP_ID()                                                        
{
	IPAddr[3] = ((R8_PB_PIN_1 & 0x03)<<4 | (R8_PB_PIN_1 & 0x30)<<2 | (R8_PB_PIN_1 & 0xC0)>>6 | (R8_PD_PIN_0 & 0x03)<<2);
	ID_Number = ((R8_PD_PIN_0 & 0xFC)>>2);	
}

/*******************************************************************************
* Function Name  : PLC_PowerOn_Init()
* Description    : 用于设备重新上电时，读取EEPROM中保存的PLC代码
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PLC_PowerOn_Init()                                                        
{
	UINT8 i;

	CH563_EEPROM_READ ( 0,EEPROM_CODE,512-4 );                                  //  EEPROM读取地址0x0000数据，读取512-4字节到EEPROM_CODE中

		for(i=0;i<(512/2-2);i++)				                                // 小端模式的u8 0x12 0x34转换u16 为0x3412，故此处要换回来
		{
			EEPROM_CODE[i] = (UINT16)(EEPROM_CODE[i]<<8|EEPROM_CODE[i]>>8);	
		}	
}


/*******************************************************************************
* Function Name  : main
* Description    : 主函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main( void ) 
{

    UINT8 i = 0,Num;
//	UINT32 Output_Port;
//	UINT32 len;	
//	UINT8  *p = FirstBuf_Tx;
//	UINT8  DES_IP[4] = {192,168,1,100};                                         /* 目的IP地址 */
//	UINT8  *ip = DES_IP;

    Init_GPIO( );																//初始化GPIO
    mInitSTDIO( );                                                              /* 为了让计算机通过串口监控演示过程 */
	InitTIM1( );																//初始化定时器1，中断周期5ms
	GetLocal_IP_ID();															//获得拨码盘和拨码开关设定的IP和ID

    i = CH563NET_LibInit(IPAddr,GWIPAddr,IPMask,MACAddr);                       /* 库初始化 */
    mStopIfError(i);                                                            /* 检查错误 */
   
    SysTimeInit();                                                              /* 系统定时器初始化 */
    InitSysHal();                                                               /* 初始化中断 */
    CH563NET_CreatUpdSocket();                                                  /* 创建UDP Socket */

//	PLC_PowerOn_Init();															//上电初始化PLC程序

#if CH563NET_DBG
    printf("CH563IPLibInit Success\n");
#endif 

	while(1)
    {
        CH563NET_MainTask();                                                    /* CH563NET库主任务函数，需要在主循环中不断调用 */
        
		if(CH563NET_QueryGlobalInt())CH563NET_HandleGlobalInt();                /* 查询中断，如果有中断，则调用全局中断处理函数 */
		
		/*以下函数用于下位机执行PLC扫描程序时，每过100毫秒向上位机发送一次开关量输出状态*/		
		/*if(PLC_Count > 250)
		{
			Output_Port = 0xFFFFFFFF;
			for(Num=0;Num<=21;Num++)
			{
				if(*((UINT16 *)EEPROM_BUFF + Num + 0x30) == 1)
				Output_Port &= ~(1<<Num);	
			}
			Buff_Tx.Lengh = ( 0x0008>>8 | 0x0008<<8 );
		    Buff_Tx.Head  =	( GET_OUT>>8 | GET_OUT<<8 );
		    LENGH_TX = 8;
			Buff_Tx.Data.IO_Data[0] =  (UINT16)(Output_Port>>16)>>8 | (UINT16)(Output_Port>>16)<<8 ;
		    Buff_Tx.Data.IO_Data[1] =  (UINT16)Output_Port>>8 | (UINT16)Output_Port<<8 ;

			memcpy(FirstBuf_Tx,(void *)&Buff_Tx,LENGH_TX);

			p = FirstBuf_Tx;												//进入循环前，p指针重新指向FirstBuf_Tx首地址

			while(1)
	        {
	           len = LENGH_TX;
	           //CH563NET_SocketSend(SocketId,p,&len);                            // 将FirstBuf_Tx中的数据发送(只向创建socket时指定的目标IP和端口发送数据)
			   CH563NET_SocketUdpSendTo(SocketId,p,&len,ip,27000);               // 向指定的目的IP，端口发送UDP包（可以向任意的IP和端口发送数据）
	           LENGH_TX -= len;                                                // 将总长度减去已经发送完毕的长度 
	           p += len;                                                       // 将缓冲区指针偏移
	           if(LENGH_TX)continue;                                           // 如果数据未发送完毕，则继续发送
	           break;                                                          // 发送完毕，退出 
	        } 
			
			PLC_Count = 0;
		}*/ 
    }
}

/*********************************** endfile **********************************/



