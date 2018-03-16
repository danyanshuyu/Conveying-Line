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

#define CH563NET_DBG                      1    

/* 变量定义 */
volatile BUFF_UDP_DATA Buff_Tx;			                                        //从上位机接收的缓存数据
volatile UINT8 FirstBuf_Tx[1000];												//定义第一层数据发送缓冲区
UINT32  LENGH_TX;														//数据处理后，Socket库函数发送给上位机所调用的字符长度
UINT16  EEPROM_CODE[512]; 		                                        //1024字节缓存，用于存放PLC代码，在RAM中运行
UINT8   PLC_Count = 0;													//PLC扫描时间计数，满足次数时，向上位机发送一次开关量输出状态


UINT8  PLC_PowerOn_Init_Flag = 1;
UINT32  PLC_PowerOn_Init_Count = 0;

/* 函数声明 */
UINT8 Reverse(UINT8 a,UINT8 n );
void BarCode_Upload(UINT8 RcvNUM);
void SetOutput(UINT32 Value);
void PLC_PowerOn_Init();	

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
* Function Name  : Uart1_Init
* Description    : 串口1初始化
* Input          : baud-串口波特率，最高为主频1/8
* Output         : None
* Return         : None
*******************************************************************************/

void Uart1_Init( UINT32 baud )
{
    UINT32    x, x2;

    x = 10 * FREQ_SYS * 2 / 16 / baud;                                                /* 115200bps */
    x2 = x % 10;
    x /= 10;                                                                        
    if ( x2 >= 5 ) x ++; 															/* 四舍五入 */
    R8_UART1_LCR = RB_LCR_DLAB;                                                 /* DLAB位置1 */
    R8_UART1_DIV = 1;                                                           /* 预分频 */
    R8_UART1_DLM = x>>8;
    R8_UART1_DLL = x&0xff;

    R8_UART1_LCR = RB_LCR_WORD_SZ ;                                             /* 设置字节长度为8    */
    R8_UART1_FCR = RB_FCR_FIFO_TRIG|RB_FCR_TX_FIFO_CLR|RB_FCR_RX_FIFO_CLR |    
                   RB_FCR_FIFO_EN ;                                             /* 设置FIFO触发点为28，清发送和接收FIFO，FIFO使能 */
    R8_UART1_IER = RB_IER_TXD_EN | RB_IER_LINE_STAT |RB_IER_THR_EMPTY | 
                   RB_IER_RECV_RDY  ;                                           /* TXD enable */
    R8_UART1_MCR = RB_MCR_OUT2;                                                
    R8_INT_EN_IRQ_0 |= RB_IE_IRQ_UART1;                                            /* 串口中断输出使能 */
    R32_PB_SMT |= RXD1|TXD1;                                                    /* RXD1 schmitt input, TXD1 slow rate */
    R32_PB_PD  &= ~ RXD1;                                                       /* disable pulldown for RXD1, keep pullup */
    R32_PB_DIR |= TXD1;                                                         /* TXD1 output enable */
}

/*******************************************************************************
* Function Name  : UART1_SendByte
* Description    : 串口1发送一字节子程序
* Input          : dat -要发送的数据
* Output         : None
* Return         : None
*******************************************************************************/

void UART1_SendByte( UINT8 dat )   
{        
    R8_UART1_THR  = dat;
    while( ( R8_UART1_LSR & RB_LSR_TX_ALL_EMP ) == 0 );                         /* 等待数据发送 */       
}

/*******************************************************************************
* Function Name  : UART1_SendStr
* Description    : 串口1发送字符串子程序
* Input          : *str -要发送字符串的指针
* Output         : None
* Return         : None
*******************************************************************************/

void UART1_SendStr( UINT8 *str )   
{
    while( 1 ){
        if( *str == '\0' ) break;
        UART1_SendByte( *str++ );
    }
}

/*******************************************************************************
* Function Name  : UART1_RcvByte
* Description    : 串口1接收一字节子程序
* Input          : None
* Output         : None
* Return         : Rcvdat -接收到的数据
*******************************************************************************/

UINT8 UART1_RcvByte( void )    
{
    UINT8 Rcvdat = 0;
    
    if( !( ( R8_UART1_LSR  ) & ( RB_LSR_OVER_ERR |RB_LSR_PAR_ERR  | RB_LSR_FRAME_ERR |  RB_LSR_BREAK_ERR  ) ) ){
        while( ( R8_UART1_LSR & RB_LSR_DATA_RDY  ) == 0 );                      /* 等待数据准备好 */ 
        Rcvdat = R8_UART1_RBR;                                                  /* 从接收缓冲寄存器读出数据 */ 
    }
    else{
        R8_UART1_RBR;                                                           /* 有错误清除 */
    }
    return( Rcvdat );
}

/*******************************************************************************
* Function Name  : Seril1Send
* Description    : 串口1发送多字节子程序
* Input          : *Data -待发送数据区指针
*                  Num   -发送数据长度
* Output         : None
* Return         : None
*******************************************************************************/

void  Seril1Send( UINT8 *Data, UINT8 Num )                        
{
    do{
        while( ( R8_UART1_LSR & RB_LSR_TX_FIFO_EMP ) == 0 );                    /* 等待数据发送完毕 */ 
        R8_UART1_THR  = *Data++;  
    }while( --Num );
}

/*******************************************************************************
* Function Name  : Seril1Rcv
* Description    : 禁用FIFO,串口1接收多字节子程序
* Input          : *pbuf -接收缓冲区指针
* Output         : None
* Return         : RcvNum -接收到数据的数据长度
*******************************************************************************/

UINT8  Seril1Rcv( UINT8 *pbuf )    
{
    UINT8 RcvNum = 0;

    if( !( ( R8_UART1_LSR  ) & ( RB_LSR_OVER_ERR |RB_LSR_PAR_ERR  | RB_LSR_FRAME_ERR |  RB_LSR_BREAK_ERR  ) ) ){
        while( ( R8_UART1_LSR & RB_LSR_DATA_RDY  ) == 0 );                      /* 等待数据准备好 */ 
        do{
            *pbuf++ = R8_UART1_RBR;                                             /* 从接收缓冲寄存器读出数据 */ 
            RcvNum++;
        }while( ( R8_UART1_LSR & RB_LSR_DATA_RDY   ) == 0x01 );
    }
    else{
        R8_UART1_RBR;
    }
    return( RcvNum );
}

/*******************************************************************************
* Function Name  : UART1Send_FIFO
* Description    : 启用FIFO,一次最多32字节，CH432串口1发送多字节子程序
* Input          : *Data -待发送数据区指针
*                  Num   -发送数据长度
* Output         : None
* Return         : None
*******************************************************************************/

void UART1Send_FIFO( UINT8 *Data, UINT8 Num ) 
{
    int i;

    while( 1 ){
        while( ( R8_UART1_LSR & RB_LSR_TX_ALL_EMP ) == 0 );                     /* 等待数据发送完毕，THR,TSR全空 */
        if( Num <= 32){                                                         /* FIFO长度为32，数据长度不满32字节，一次发送完成 */
            do{
                R8_UART1_THR=*Data++;
            }while(--Num) ;
            break;
        }
        else{                                                                   /* FIFO长度为32，数据长度超过32字节，分多次发送，一次32字节 */
            for(i=0;i<32;i++){
                R8_UART1_THR=*Data++;
            }
            Num -= 32;
        }
    }
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
* Function Name  : BarCode_Upload(RcvNum)
* Description    : 条形码数据上传给上位机
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BarCode_Upload(UINT8 RcvNUM)
{
    UINT32 len;
    UINT32 totallen;
	UINT8  BarCode_Buff_Tx[50];
    UINT8 *p = BarCode_Buff_Tx;
	UINT16  LENGH;
	UINT16  T_IP = (IPAddr[3]>>8 | IPAddr[3]<<8);					// 发送的下位机IP的第四位
	BUFF_UDP_DATA Buff_Tx;			                                //发送给上位机的缓存数据
	UINT8 * P_Buff_Tx = (void *)&Buff_Tx;

	UINT8  DES_IP[4] = {192,168,1,100};                                         /* 目的IP地址 */
	UINT8  *ip = DES_IP;

	LENGH = RcvNUM+12;

	Buff_Tx.Start =	0xAA;
	Buff_Tx.Lengh = (LENGH>>8 | LENGH<<8);
	Buff_Tx.Packet_Flag = 0x0000;
	Buff_Tx.SourSite = T_IP;
	Buff_Tx.DesSite = 0xffff;
	Buff_Tx.Head  =	(BARCODE>>8 | BARCODE<<8);
	Buff_Tx.End =	0xBB;

	BarCode_Buff_Tx[0] = Buff_Tx.Start;
	memcpy(BarCode_Buff_Tx+1,P_Buff_Tx+2,10);		      //将Buff_Tx中的数据拷贝到BarCode_Buff_Tx中，	注意u16到u8，0x3412 变为 0x12 0x34
	memcpy(BarCode_Buff_Tx+11,Uart_Buf,RcvNUM);
	BarCode_Buff_Tx[LENGH-1] = Buff_Tx.End;
		  
	  len = LENGH;
	  totallen = len;
        while(1)
        {
           len = totallen;
           //CH563NET_SocketSend(SocketId,p,&len);                                 /* 将MyBuf中的数据发送 */
		   CH563NET_SocketUdpSendTo( SocketId,p,&len,ip,27000);
           totallen -= len;                                                     /* 将总长度减去以及发送完毕的长度 */
           p += len;                                                            /* 将缓冲区指针偏移*/
           if(totallen)continue;                                                /* 如果数据未发送完毕，则继续发送*/
           break;                                                               /* 发送完毕，退出 */
        }

	
}

/*******************************************************************************
* Function Name  : ResetOutput()
* Description    : 设置io全部输出
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetOutput(UINT32 Value)
{
	UINT32 Output_Port;	
	int Num;
	
	Output_Port = Value;
	
	
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
	int i,Num,j;
	UINT8 * P_Buff_Tx = (void *)&Buff_Tx;							//定义一个指向结构体Buff_Tx起始地址的指针

	UINT8 a[12]= {0xAA,0x00 ,0x0C, 0x00 ,0x00 ,0xFF ,0xFF ,0xFF ,0xFF ,0x00 ,0xFB ,0xBB};
	UINT32 len = 12;	
	UINT8  *p = a;
	UINT8  DES_IP[4] = {192,168,1,4};                                         /* 目的IP地址 */
	UINT8  *ip = DES_IP;

	
	Buff_Tx.Start = 0xAA;
	Buff_Tx.Packet_Flag = 0x0000;
	Buff_Tx.SourSite = T_IP;
	Buff_Tx.DesSite = 0xffff;
	Buff_Tx.End = 0xBB;


    if( R_Head == PLC_Q )									        // 接受到的报文头为 PLC代码
	{	
		R8_INT_EN_IRQ_0 &= ~RB_IE_IRQ_TMR1;                                          //关闭TIM1中断   
		
		Control_PLC_End = 0;                            //PLC扫描立即终止
		
		SetOutput(0xffffffff);                          //IO全部清零
				
		memset(EEPROM_CODE,0,sizeof(EEPROM_CODE));     //PLC代码缓存清空
		memset(EEPROM_BUFF,0,sizeof(EEPROM_BUFF));     //PLC寄存器缓存清空
		
		T_Head = LOWER_ANSW;
		
		CH563_EEPROM_ERASE( PLC_Code_Addr,0x1000);                              //  擦除0x0000地址开始的4K数据
		CH563_EEPROM_WRITE( PLC_Code_Addr,Buff_Rx.Data.PLC_Cmd,LENGH-12 );       //  将PLC程序写入EEPROM中，起始地址0x0000
		CH563_EEPROM_READ ( 0,EEPROM_CODE,LENGH-12 );                            //  EEPROM读取地址0x0000数据，读取LENGH-12字节到EEPROM_CODE中

		for(i=0;i<(LENGH/2-6);i++)				                                // 小端模式的u8 0x12 0x34转换u16 为0x3412，故此处要换回来
		{
			EEPROM_CODE[i] = (UINT16)(EEPROM_CODE[i]<<8|EEPROM_CODE[i]>>8);	
		}
    
		
		Control_PLC_End = 1;                            //PLC扫描全局控制量开启   R8_INT_EN_IRQ_0 |= RB_IE_IRQ_TMR1;                                          //开启TIM1中断 
		
		
	}
	if( R_Head == GET_IN )									        // 上位机获取开关量输入
	{	
		
		T_Head = ( GET_IN>>8 | GET_IN<<8 );
		Buff_Tx.Lengh = ( 0x000e>>8 | 0x000e<<8 );
		Buff_Tx.Head  =	T_Head;
		LENGH_TX = 14;		
		Input_Port = (UINT16)( (R8_PB_PIN_2 & 0x0F) | ((R8_PD_PIN_1<<4) & 0xF0))<<8 | (UINT16)( (R8_PD_PIN_3>>7) & 0x01 | R8_PB_PIN_0<<1 );	 
																   // 从GPIO中读出开关量的信号,PB16~19 PD8~11,PD31 PB0~6
																                              //IN     7~0   ,    15~8
		//Input_Port = 0xFF00 | ((R8_PD_PIN_0 & 0xFC)>>2);//拨码开关模块测试
		//Input_Port = 0xFF00 | ((R8_PB_PIN_1 & 0x03)<<4 | (R8_PB_PIN_1 & 0x30)<<2 | (R8_PB_PIN_1 & 0xC0)>>6 | (R8_PD_PIN_0 & 0x03)<<2);//拨码盘模块测试

		Buff_Tx.Data.IO_Data[0] = Input_Port ;
		
		FirstBuf_Tx[0] = Buff_Tx.Start;
		memcpy(FirstBuf_Tx+1,P_Buff_Tx+2,LENGH_TX-2);		      //将Buff_Tx中的数据拷贝到结构体FirstBuf_Tx中，	注意u16到u8，0x3412 变为 0x12 0x34
		FirstBuf_Tx[LENGH_TX-1] = Buff_Tx.End;
	}
	if( R_Head == SET_OUT )									        // 上位机设置开关量输出
	{																//上位机发送OUT格式 7~0,15~8,23~16,31~24(只有0~21位有效)
																	//                0x12   34   56   78
																	//储存在Buff_Rx.Data.IO_Data[0]和[1]中的格式（小端模式，高字节高地址，低字节低地址）
																	//	协议中的0x1234	-> Buff_Rx中的0x3412	-> 	协议中的0x1234	         
		T_Head =  ( SET_OUT>>8 | SET_OUT<<8 );
		Buff_Tx.Lengh = ( 0x0010>>8 | 0x0010<<8 );
		Buff_Tx.Head  =	T_Head;
		LENGH_TX = 16;
		
		R8_INT_EN_IRQ_0 &= ~RB_IE_IRQ_TMR1;                                          //关闭TIM1中断   
		
		Control_PLC_End = 0;                            //PLC扫描立即终止
						
		memset(EEPROM_CODE,0,sizeof(EEPROM_CODE));     //PLC代码缓存清空
		memset(EEPROM_BUFF,0,sizeof(EEPROM_BUFF));     //PLC寄存器缓存清空
		
		T_Head = LOWER_ANSW;
		
		
		/*从上位机广播帧中找出本地IP对应IO端口Output_Port值*/
		/*
		for(i = 0;i < (LENGH-4)/6;i += 3)
		{
			if(Buff_Rx.Data.IO_Data[i] == T_IP)
			{
			    Output_Port = Buff_Rx.Data.IO_Data[i+2]<<16 | (UINT32) Buff_Rx.Data.IO_Data[i+1] ;	 
																   // 对应GPIO中要写入的开关量输出信号，OUT0~21
				break;
			}
		}
		*/
		Output_Port = Buff_Rx.Data.IO_Data[1]<<16 | (UINT32) Buff_Rx.Data.IO_Data[0] ;

		SetOutput(Output_Port);
				
		Buff_Tx.Data.IO_Data[0] = Buff_Rx.Data.IO_Data[0] ;
		Buff_Tx.Data.IO_Data[1] = Buff_Rx.Data.IO_Data[1] ;
		
		FirstBuf_Tx[0] = Buff_Tx.Start;
		memcpy(FirstBuf_Tx+1,P_Buff_Tx+2,LENGH_TX-2);		      //将Buff_Tx中的数据拷贝到结构体FirstBuf_Tx中，	注意u16到u8，0x3412 变为 0x12 0x34
		FirstBuf_Tx[LENGH_TX-1] = Buff_Tx.End;

	}
    if( R_Head == GET_OUT )									        // 上位机获取开关量输出
	{	
		
		T_Head = ( GET_OUT>>8 | GET_OUT<<8 );
		Buff_Tx.Lengh = ( 0x0010>>8 | 0x0010<<8 );
		Buff_Tx.Head  =	T_Head;
		LENGH_TX = 16;		
		Output_Port = 0xFFFFFFFF;
		
		for(Num=0;Num<=21;Num++)
		{
			if(*((UINT16 *)EEPROM_BUFF + Num + 0x30) == 1)
			Output_Port &= ~(1<<Num);	
		}
		Buff_Tx.Data.IO_Data[0] =  (UINT16) Output_Port;
	    Buff_Tx.Data.IO_Data[1] =  (UINT16)(Output_Port>>16) ;

	    FirstBuf_Tx[0] = Buff_Tx.Start;
		memcpy(FirstBuf_Tx+1,P_Buff_Tx+2,LENGH_TX-2);		      //将Buff_Tx中的数据拷贝到结构体FirstBuf_Tx中，	注意u16到u8，0x3412 变为 0x12 0x34
		FirstBuf_Tx[LENGH_TX-1] = Buff_Tx.End;
	
	}
	if( R_Head == SEND_IP_ID )									        // 下位机发送本地IP和ID
	{

		T_Head = ( SEND_IP_ID>>8 | SEND_IP_ID<<8 );
		Buff_Tx.Lengh = ( 0x000e>>8 | 0x000e<<8 );
	    Buff_Tx.Head  =	T_Head;
	    LENGH_TX = 14;
	
	    Buff_Tx.Data.IO_Data[0] =  (UINT16)IPAddr[3]<<8;
	    
		FirstBuf_Tx[0] = Buff_Tx.Start;
		memcpy(FirstBuf_Tx+1,P_Buff_Tx+2,LENGH_TX-2);		      //将Buff_Tx中的数据拷贝到结构体FirstBuf_Tx中，	注意u16到u8，0x3412 变为 0x12 0x34
		FirstBuf_Tx[LENGH_TX-1] = Buff_Tx.End;
		
	}
	if( R_Head == SET_M000 )									        // 上位机设置“移载出”模块执行移载动作，M000为移载变量，默认为0不执行
	{
		EEPROM_BUFF[0+2*0x30] = 1;					      //设置PLC变量M000的值为1，PLC移载动作执行完之后，M000自动复位为0
	}

	
	if( R_Head == PLC_START )									        //上位机控制全部启动
	{
		printf("START\n");
		//PLC_PowerOn_Init();											//由上位机控制上电初始化PLC程序
	}

	if( R_Head == PLC_STOP )									        //上位机控制全部停止
	{
		R8_INT_EN_IRQ_0 &= ~RB_IE_IRQ_TMR1;                                          //关闭TIM1中断   
		
		Control_PLC_End = 0;                            //PLC扫描立即终止
		
		SetOutput(0xffffffff);                          //IO全部清零
				
		memset(EEPROM_CODE,0,sizeof(EEPROM_CODE));     //PLC代码缓存清空
		memset(EEPROM_BUFF,0,sizeof(EEPROM_BUFF));     //PLC寄存器缓存清空
		
		T_Head = LOWER_ANSW;
	}
	

	if( R_Head == TEST )
	{
		printf("R_Head == TEST\n");
		CH563NET_SocketUdpSendTo(SocketId,p,&len,ip,28000);
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
	MACAddr[5] = IPAddr[3];
		
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
    
	  R8_INT_EN_IRQ_0 &= ~RB_IE_IRQ_TMR1;                                          //关闭TIM1中断   
		
		Control_PLC_End = 0;                            //PLC扫描立即终止
		
		SetOutput(0xffffffff);                          //IO全部清零
				
		memset(EEPROM_CODE,0,sizeof(EEPROM_CODE));     //PLC代码缓存清空
		memset(EEPROM_BUFF,0,sizeof(EEPROM_BUFF));     //PLC寄存器缓存清空
	
	
	  CH563_EEPROM_READ ( 0,EEPROM_CODE,512-4 );                                  //  EEPROM读取地址0x0000数据，读取512-4字节到EEPROM_CODE中

		for(i=0;i<(512/2-2);i++)				                                // 小端模式的u8 0x12 0x34转换u16 为0x3412，故此处要换回来
		{
			EEPROM_CODE[i] = (UINT16)(EEPROM_CODE[i]<<8|EEPROM_CODE[i]>>8);	
		}
	
		Control_PLC_End = 1;                            //PLC扫描全局控制量开启
    R8_INT_EN_IRQ_0 |= RB_IE_IRQ_TMR1;                                          //开启TIM1中断 
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
   
	  R32_TMR1_CNT_END = 0x186a0 * 5;                                             //设置为5MS定时	
  	R8_TMR1_CTRL_MOD = RB_TMR_ALL_CLEAR;   	
	  R8_TMR1_CTRL_MOD = RB_TMR_COUNT_EN;										                     //定时器计数功使能    
	  R32_TMR1_COUNT = 0x00000000; 
    R8_TMR1_INTER_EN |= RB_TMR_IE_CYC_END;
    
	  R8_INT_EN_IRQ_0 |= RB_IE_IRQ_TMR1;                                          /* 开启TIM1中断 */
    R8_INT_EN_IRQ_GLOB |= RB_IE_IRQ_GLOB;                                       /* 开启IRQ全局中断 */
}



void IRQ_InitPD( void )	//PD0为SCL，PD1为SDA
{
    R32_PD_PU  |= 0x00000001;                                                   /* GPIO D0上拉设置，置1表示上拉 */ 
    R32_PD_DIR &= ~0x00000001;                                                  /* GPIO D0方向设置为输入 , direction: 0=in, 1=out */

			 
	R32_PD_PU  |= 0x00000002;                                                   /* GPIO D1上拉设置，置1表示上拉 */ 
    R32_PD_DIR &= ~0x00000002;                                                  /* GPIO D1方向设置为输入 , direction: 0=in, 1=out */


    R32_INT_ENABLE_PD |= 0x00000001;                                            /* GPIO D0中断使能 ： 1-使能，0-禁止 */
    R32_INT_MODE_PD   |= 0x00000001;                                            /* GPIO D0中断方式 ：1-边沿中断,0-电平中断 */      
    R32_INT_POLAR_PD  |= 0x00000001;                                            /* GPIO D0中断极性 ：1-上升沿中断/高电平，0-下降沿中断/低电平 */      
    
    R32_INT_STATUS_PD  = 0xffffffff;                                               /* 中断标志写1清零 */
    R8_INT_EN_IRQ_1   |= RB_IE_IRQ_PD;                                          /* GPIO D组中断使能 */ 
     
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
	UINT8 a[12]= {0xAA,0x00 ,0x0C, 0x00 ,0x00 ,0xFF ,0xFF ,0xFF ,0xFF ,0x00 ,0xFB ,0xBB};
	//UINT32 Output_Port;
	UINT32 len = 12;	
	UINT8  *p = a;
	UINT8  DES_IP[4] = {192,168,1,2};                                         /* 目的IP地址 */
	UINT8  *ip = DES_IP;

    Init_GPIO( );																//初始化GPIO
    //mInitSTDIO( );                                                              /* 为了让计算机通过串口监控演示过程 */
	Uart1_Init( 9600);
	
	InitTIM1( );																//初始化定时器1，中断周期5ms
	//GetLocal_IP_ID();															//获得拨码盘和拨码开关设定的IP和ID

	IRQ_InitPD( );

    i = CH563NET_LibInit(IPAddr,GWIPAddr,IPMask,MACAddr);                       /* 库初始化 */
    mStopIfError(i);                                                            /* 检查错误 */
   
    SysTimeInit();                                                              /* 系统定时器初始化 */
    InitSysHal();                                                               /* 初始化中断 */	
	CH563NET_CreatUdpSocket();	

#if CH563NET_DBG
    printf("CH563IPLibInit Success\n");
#endif 


   
	while(1)
    {
        CH563NET_MainTask();                                                    /* CH563NET库主任务函数，需要在主循环中不断调用 */
        
		    if(CH563NET_QueryGlobalInt())CH563NET_HandleGlobalInt();                /* 查询中断，如果有中断，则调用全局中断处理函数 */
		
		/* 
		if(PLC_PowerOn_Init_Flag == 1 && PLC_PowerOn_Init_Count == 1000)	  //上电延时5s后初始化PLC程序
		{
			CH563NET_SocketUdpSendTo(SocketId,p,&len,ip,28000);              // 向指定的目的IP，端口发送UDP包（可以向任意的IP和端口发送数据）
			//PLC_PowerOn_Init();
			printf("CH563NET_SocketUdpSendTo\n");
			PLC_PowerOn_Init_Count = 0;															
			//PLC_PowerOn_Init_Flag = 0;
        }
		*/
				
				
				
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



