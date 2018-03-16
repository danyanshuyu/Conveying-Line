/********************************** (C) COPYRIGHT ******************************
* File Name          : main.c
* Author             : LN
* Version            : V1.0
* Date               : 2017/04/25
* Description        : �Զ���������_������ͨѶģ��_��λ���߼�����������
*                      ����0��������Ϣ,115200bps;
*******************************************************************************/

/******************************************************************************/
/* ͷ�ļ�����*/
#include <stdio.h>
#include <string.h>
#include "CH563SFR.H"
#include "SYSFREQ.H"
#include "CH563NET.H"
#include "ISPXT563.H"
#include "Extern_Variables.h"

#define CH563NET_DBG                      1    

/* �������� */
volatile BUFF_UDP_DATA Buff_Tx;			                                        //����λ�����յĻ�������
volatile UINT8 FirstBuf_Tx[1000];												//�����һ�����ݷ��ͻ�����
UINT32  LENGH_TX;														//���ݴ����Socket�⺯�����͸���λ�������õ��ַ�����
UINT16  EEPROM_CODE[512]; 		                                        //1024�ֽڻ��棬���ڴ��PLC���룬��RAM������
UINT8   PLC_Count = 0;													//PLCɨ��ʱ��������������ʱ������λ������һ�ο��������״̬


UINT8  PLC_PowerOn_Init_Flag = 1;
UINT32  PLC_PowerOn_Init_Count = 0;

/* �������� */
UINT8 Reverse(UINT8 a,UINT8 n );
void BarCode_Upload(UINT8 RcvNUM);
void SetOutput(UINT32 Value);
void PLC_PowerOn_Init();	

/*******************************************************************************
* Function Name  : Init_GPIO
* Description    : Ϊ���������������ʼ��GPIO	(22+20+32)=74
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Init_GPIO( void )
{
	/* GPIO  �����������,0����,1��� */
    R8_PD_DIR_3 &= ~(1<<7);														//PD31 PB0~6,PB16~19 PD8~11Ϊ����
    R8_PB_DIR_0 &= ~(0x7F);														//16λ����
	R8_PB_DIR_2 &= ~(0x0F);
	R8_PD_DIR_1 &= ~(0x0F);

	R8_PD_DIR_1 |= 0xF0;													    //PD12~15 PA0~3,PA4~7 PA12~15,PA8~11 PA16~17Ϊ���
	R32_PA_DIR  |= 0x0003FFFF;														//22λ���

	/* GPIO  ��������,��1��������,��0�ر����� */							   //����Ҳ���ԣ������ܸ��գ�δ���뿪����ʱ�����������ܸ��Ŷ����ϱ仯
	R8_PD_PU_3 |= 1<<7;													
	R8_PB_PU_0 |= 0x7F;                                                   
	R8_PB_PU_2 |= 0x0F;														
	R8_PD_PU_1 |= 0x0F;

	R8_PD_OUT_1 |= 0xF0;														//��1���ߵ�ƽʱ�����Ч
	R32_PA_OUT  |= 0x0003FFFF;

	/* ip��Ӳ�����룬���ӱ�ŵ�Ӳ������ */
	R8_PB_DIR_1 &= ~(0xF3);														//PB 8  9  12  13  14  15  PD 0  1			PD 2  3  4  5  6  7  
	R8_PD_DIR_0 &= ~(0xFF);														//SW 1  2  3   4   5   6      7  8			SW 9  10 11 12 13 14
	
	/* ���� */
//	R8_PB_PD_1 |= 0xF3;
//	R8_PD_PD_0 |= 0xFF;
}

/*******************************************************************************
* Function Name  : mInitSTDIO
* Description    : Ϊprintf��getkey���������ʼ������
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
    if ( x2 >= 5 ) x ++;                                                        /* �������� */
    R8_UART1_LCR = 0x80;                                                        /* DLABλ��1 */
    R8_UART1_DIV = 1;                                                           /* Ԥ��Ƶ */
    R8_UART1_DLM = x>>8;
    R8_UART1_DLL = x&0xff;

    R8_UART1_LCR = RB_LCR_WORD_SZ ;                                             /* �����ֽڳ���Ϊ8 */
    R8_UART1_FCR = RB_FCR_FIFO_TRIG|RB_FCR_TX_FIFO_CLR|RB_FCR_RX_FIFO_CLR |    
                   RB_FCR_FIFO_EN ;                                             /* ����FIFO������Ϊ14���巢�ͺͽ���FIFO��FIFOʹ�� */
    R8_UART1_IER = RB_IER_TXD_EN;                                               /* TXD enable */
    R32_PB_SMT |= RXD1|TXD1;                                                    /* RXD0 schmitt input, TXD0 slow rate */
    R32_PB_PD &= ~ RXD1;                                                        /* disable pulldown for RXD0, keep pullup */
    R32_PB_DIR |= TXD1;                                                         /* TXD0 output enable */
}

/*******************************************************************************
* Function Name  : fputc
* Description    : ͨ��������������Ϣ
* Input          : c-- writes the character specified by c 
*                  *f--the output stream pointed to by *f
* Output         : None
* Return         : None
*******************************************************************************/

int fputc( int c, FILE *f )
{
    R8_UART1_THR = c;                                                           /* �������� */
    while( ( R8_UART1_LSR & RB_LSR_TX_FIFO_EMP ) == 0 );                        /* �ȴ����ݷ��� */
    return( c );
}


/*******************************************************************************
* Function Name  : Uart1_Init
* Description    : ����1��ʼ��
* Input          : baud-���ڲ����ʣ����Ϊ��Ƶ1/8
* Output         : None
* Return         : None
*******************************************************************************/

void Uart1_Init( UINT32 baud )
{
    UINT32    x, x2;

    x = 10 * FREQ_SYS * 2 / 16 / baud;                                                /* 115200bps */
    x2 = x % 10;
    x /= 10;                                                                        
    if ( x2 >= 5 ) x ++; 															/* �������� */
    R8_UART1_LCR = RB_LCR_DLAB;                                                 /* DLABλ��1 */
    R8_UART1_DIV = 1;                                                           /* Ԥ��Ƶ */
    R8_UART1_DLM = x>>8;
    R8_UART1_DLL = x&0xff;

    R8_UART1_LCR = RB_LCR_WORD_SZ ;                                             /* �����ֽڳ���Ϊ8    */
    R8_UART1_FCR = RB_FCR_FIFO_TRIG|RB_FCR_TX_FIFO_CLR|RB_FCR_RX_FIFO_CLR |    
                   RB_FCR_FIFO_EN ;                                             /* ����FIFO������Ϊ28���巢�ͺͽ���FIFO��FIFOʹ�� */
    R8_UART1_IER = RB_IER_TXD_EN | RB_IER_LINE_STAT |RB_IER_THR_EMPTY | 
                   RB_IER_RECV_RDY  ;                                           /* TXD enable */
    R8_UART1_MCR = RB_MCR_OUT2;                                                
    R8_INT_EN_IRQ_0 |= RB_IE_IRQ_UART1;                                            /* �����ж����ʹ�� */
    R32_PB_SMT |= RXD1|TXD1;                                                    /* RXD1 schmitt input, TXD1 slow rate */
    R32_PB_PD  &= ~ RXD1;                                                       /* disable pulldown for RXD1, keep pullup */
    R32_PB_DIR |= TXD1;                                                         /* TXD1 output enable */
}

/*******************************************************************************
* Function Name  : UART1_SendByte
* Description    : ����1����һ�ֽ��ӳ���
* Input          : dat -Ҫ���͵�����
* Output         : None
* Return         : None
*******************************************************************************/

void UART1_SendByte( UINT8 dat )   
{        
    R8_UART1_THR  = dat;
    while( ( R8_UART1_LSR & RB_LSR_TX_ALL_EMP ) == 0 );                         /* �ȴ����ݷ��� */       
}

/*******************************************************************************
* Function Name  : UART1_SendStr
* Description    : ����1�����ַ����ӳ���
* Input          : *str -Ҫ�����ַ�����ָ��
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
* Description    : ����1����һ�ֽ��ӳ���
* Input          : None
* Output         : None
* Return         : Rcvdat -���յ�������
*******************************************************************************/

UINT8 UART1_RcvByte( void )    
{
    UINT8 Rcvdat = 0;
    
    if( !( ( R8_UART1_LSR  ) & ( RB_LSR_OVER_ERR |RB_LSR_PAR_ERR  | RB_LSR_FRAME_ERR |  RB_LSR_BREAK_ERR  ) ) ){
        while( ( R8_UART1_LSR & RB_LSR_DATA_RDY  ) == 0 );                      /* �ȴ�����׼���� */ 
        Rcvdat = R8_UART1_RBR;                                                  /* �ӽ��ջ���Ĵ����������� */ 
    }
    else{
        R8_UART1_RBR;                                                           /* �д������ */
    }
    return( Rcvdat );
}

/*******************************************************************************
* Function Name  : Seril1Send
* Description    : ����1���Ͷ��ֽ��ӳ���
* Input          : *Data -������������ָ��
*                  Num   -�������ݳ���
* Output         : None
* Return         : None
*******************************************************************************/

void  Seril1Send( UINT8 *Data, UINT8 Num )                        
{
    do{
        while( ( R8_UART1_LSR & RB_LSR_TX_FIFO_EMP ) == 0 );                    /* �ȴ����ݷ������ */ 
        R8_UART1_THR  = *Data++;  
    }while( --Num );
}

/*******************************************************************************
* Function Name  : Seril1Rcv
* Description    : ����FIFO,����1���ն��ֽ��ӳ���
* Input          : *pbuf -���ջ�����ָ��
* Output         : None
* Return         : RcvNum -���յ����ݵ����ݳ���
*******************************************************************************/

UINT8  Seril1Rcv( UINT8 *pbuf )    
{
    UINT8 RcvNum = 0;

    if( !( ( R8_UART1_LSR  ) & ( RB_LSR_OVER_ERR |RB_LSR_PAR_ERR  | RB_LSR_FRAME_ERR |  RB_LSR_BREAK_ERR  ) ) ){
        while( ( R8_UART1_LSR & RB_LSR_DATA_RDY  ) == 0 );                      /* �ȴ�����׼���� */ 
        do{
            *pbuf++ = R8_UART1_RBR;                                             /* �ӽ��ջ���Ĵ����������� */ 
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
* Description    : ����FIFO,һ�����32�ֽڣ�CH432����1���Ͷ��ֽ��ӳ���
* Input          : *Data -������������ָ��
*                  Num   -�������ݳ���
* Output         : None
* Return         : None
*******************************************************************************/

void UART1Send_FIFO( UINT8 *Data, UINT8 Num ) 
{
    int i;

    while( 1 ){
        while( ( R8_UART1_LSR & RB_LSR_TX_ALL_EMP ) == 0 );                     /* �ȴ����ݷ�����ϣ�THR,TSRȫ�� */
        if( Num <= 32){                                                         /* FIFO����Ϊ32�����ݳ��Ȳ���32�ֽڣ�һ�η������ */
            do{
                R8_UART1_THR=*Data++;
            }while(--Num) ;
            break;
        }
        else{                                                                   /* FIFO����Ϊ32�����ݳ��ȳ���32�ֽڣ��ֶ�η��ͣ�һ��32�ֽ� */
            for(i=0;i<32;i++){
                R8_UART1_THR=*Data++;
            }
            Num -= 32;
        }
    }
}

/*******************************************************************************
* Function Name  : CH563_EEPROM
* Description    : EEPROM�����ӳ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void CH563_EEPROM( void ) 
{

  //  CH563_EEPROM_ERASE(0x2000,0x1000);                                           //  ����0X2000��ַ��ʼ��4K���� 

  //  CH563_EEPROM_WRITE( 0x2000,my_buffer,64 );                                   //  ����ַ0X2000д64�ֽڵ�����
 
  //  CH563_EEPROM_READ( 0x2000,my_buffer,64 );                                    //  EEPROM��ȡ��ַ0x2000���ݣ���ȡ64�ֽ� 
 
}



/*******************************************************************************
* Function Name  : BarCode_Upload(RcvNum)
* Description    : �����������ϴ�����λ��
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
	UINT16  T_IP = (IPAddr[3]>>8 | IPAddr[3]<<8);					// ���͵���λ��IP�ĵ���λ
	BUFF_UDP_DATA Buff_Tx;			                                //���͸���λ���Ļ�������
	UINT8 * P_Buff_Tx = (void *)&Buff_Tx;

	UINT8  DES_IP[4] = {192,168,1,100};                                         /* Ŀ��IP��ַ */
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
	memcpy(BarCode_Buff_Tx+1,P_Buff_Tx+2,10);		      //��Buff_Tx�е����ݿ�����BarCode_Buff_Tx�У�	ע��u16��u8��0x3412 ��Ϊ 0x12 0x34
	memcpy(BarCode_Buff_Tx+11,Uart_Buf,RcvNUM);
	BarCode_Buff_Tx[LENGH-1] = Buff_Tx.End;
		  
	  len = LENGH;
	  totallen = len;
        while(1)
        {
           len = totallen;
           //CH563NET_SocketSend(SocketId,p,&len);                                 /* ��MyBuf�е����ݷ��� */
		   CH563NET_SocketUdpSendTo( SocketId,p,&len,ip,27000);
           totallen -= len;                                                     /* ���ܳ��ȼ�ȥ�Լ�������ϵĳ��� */
           p += len;                                                            /* ��������ָ��ƫ��*/
           if(totallen)continue;                                                /* �������δ������ϣ����������*/
           break;                                                               /* ������ϣ��˳� */
        }

	
}

/*******************************************************************************
* Function Name  : ResetOutput()
* Description    : ����ioȫ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetOutput(UINT32 Value)
{
	UINT32 Output_Port;	
	int Num;
	
	Output_Port = Value;
	
	
	  /*���ö�Ӧÿһ����������ֵ*/
		for(Num=0;Num<=21;Num++)
		{			
			if( (Output_Port>>Num & 0x00000001) == 1 )		 // ������ź�������Ϊ1�����������Ч				
			{
				if(0<=Num && Num<=3)
				R8_PD_OUT_1 |= (1<<(Num+4));					//�����������GPIO���Ŷ�Ӧ��ϵ
				else if(Num<=11)								//OUT  21 20 19 18 17 16 15 14 13 12 11 10  9 8 7 6 5 4     3  2  1  0
				R8_PA_OUT_0 |= (1<<(Num-4));					//PA   17 16 11 10  9  8 15 14 13 12  7  6  5 4 3 2 1 0 PD 15 14 13 12 
				else if(Num<=15)								  		   
				R8_PA_OUT_1 |= (1<<(Num-8));					
				else if(Num<=19)
				R8_PA_OUT_1 |= (1<<(Num-16));
				else if(Num<=21)
				R8_PA_OUT_2 |= (1<<(Num-20));
			}
			else if((Output_Port>>Num & 0x00000001) == 0 )	 // �������ֵΪ0�����Ӧλ�����Ч,���͵�ƽ��Ч
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
* Description    : ������λ�������ݲ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Data_Process()
{
    UINT16  LENGH  = ( Buff_Rx.Lengh>>8 | Buff_Rx.Lengh<<8 );		// ������,�洢��Buff_Rx�е����ݸߵ�λ�ѱ�������һ�Σ�����Ҫ������
	UINT16  R_Head = ( Buff_Rx.Head>>8 | Buff_Rx.Head<<8 );			// ���յı���ͷ
	UINT16  T_Head = 0;												// ���͵ı���ͷ
	UINT16  T_IP = (IPAddr[3]>>8 | IPAddr[3]<<8);					// ���͵���λ��IP�ĵ���λ
	UINT16V Input_Port = 0;							                // �ñ�����ӦӲ���ж����Ŀ���������
	UINT32  Output_Port = 0x00000000;								// �ñ�����Ӧд��Ŀ��������
	int i,Num,j;
	UINT8 * P_Buff_Tx = (void *)&Buff_Tx;							//����һ��ָ��ṹ��Buff_Tx��ʼ��ַ��ָ��

	UINT8 a[12]= {0xAA,0x00 ,0x0C, 0x00 ,0x00 ,0xFF ,0xFF ,0xFF ,0xFF ,0x00 ,0xFB ,0xBB};
	UINT32 len = 12;	
	UINT8  *p = a;
	UINT8  DES_IP[4] = {192,168,1,4};                                         /* Ŀ��IP��ַ */
	UINT8  *ip = DES_IP;

	
	Buff_Tx.Start = 0xAA;
	Buff_Tx.Packet_Flag = 0x0000;
	Buff_Tx.SourSite = T_IP;
	Buff_Tx.DesSite = 0xffff;
	Buff_Tx.End = 0xBB;


    if( R_Head == PLC_Q )									        // ���ܵ��ı���ͷΪ PLC����
	{	
		R8_INT_EN_IRQ_0 &= ~RB_IE_IRQ_TMR1;                                          //�ر�TIM1�ж�   
		
		Control_PLC_End = 0;                            //PLCɨ��������ֹ
		
		SetOutput(0xffffffff);                          //IOȫ������
				
		memset(EEPROM_CODE,0,sizeof(EEPROM_CODE));     //PLC���뻺�����
		memset(EEPROM_BUFF,0,sizeof(EEPROM_BUFF));     //PLC�Ĵ����������
		
		T_Head = LOWER_ANSW;
		
		CH563_EEPROM_ERASE( PLC_Code_Addr,0x1000);                              //  ����0x0000��ַ��ʼ��4K����
		CH563_EEPROM_WRITE( PLC_Code_Addr,Buff_Rx.Data.PLC_Cmd,LENGH-12 );       //  ��PLC����д��EEPROM�У���ʼ��ַ0x0000
		CH563_EEPROM_READ ( 0,EEPROM_CODE,LENGH-12 );                            //  EEPROM��ȡ��ַ0x0000���ݣ���ȡLENGH-12�ֽڵ�EEPROM_CODE��

		for(i=0;i<(LENGH/2-6);i++)				                                // С��ģʽ��u8 0x12 0x34ת��u16 Ϊ0x3412���ʴ˴�Ҫ������
		{
			EEPROM_CODE[i] = (UINT16)(EEPROM_CODE[i]<<8|EEPROM_CODE[i]>>8);	
		}
    
		
		Control_PLC_End = 1;                            //PLCɨ��ȫ�ֿ���������   R8_INT_EN_IRQ_0 |= RB_IE_IRQ_TMR1;                                          //����TIM1�ж� 
		
		
	}
	if( R_Head == GET_IN )									        // ��λ����ȡ����������
	{	
		
		T_Head = ( GET_IN>>8 | GET_IN<<8 );
		Buff_Tx.Lengh = ( 0x000e>>8 | 0x000e<<8 );
		Buff_Tx.Head  =	T_Head;
		LENGH_TX = 14;		
		Input_Port = (UINT16)( (R8_PB_PIN_2 & 0x0F) | ((R8_PD_PIN_1<<4) & 0xF0))<<8 | (UINT16)( (R8_PD_PIN_3>>7) & 0x01 | R8_PB_PIN_0<<1 );	 
																   // ��GPIO�ж������������ź�,PB16~19 PD8~11,PD31 PB0~6
																                              //IN     7~0   ,    15~8
		//Input_Port = 0xFF00 | ((R8_PD_PIN_0 & 0xFC)>>2);//���뿪��ģ�����
		//Input_Port = 0xFF00 | ((R8_PB_PIN_1 & 0x03)<<4 | (R8_PB_PIN_1 & 0x30)<<2 | (R8_PB_PIN_1 & 0xC0)>>6 | (R8_PD_PIN_0 & 0x03)<<2);//������ģ�����

		Buff_Tx.Data.IO_Data[0] = Input_Port ;
		
		FirstBuf_Tx[0] = Buff_Tx.Start;
		memcpy(FirstBuf_Tx+1,P_Buff_Tx+2,LENGH_TX-2);		      //��Buff_Tx�е����ݿ������ṹ��FirstBuf_Tx�У�	ע��u16��u8��0x3412 ��Ϊ 0x12 0x34
		FirstBuf_Tx[LENGH_TX-1] = Buff_Tx.End;
	}
	if( R_Head == SET_OUT )									        // ��λ�����ÿ��������
	{																//��λ������OUT��ʽ 7~0,15~8,23~16,31~24(ֻ��0~21λ��Ч)
																	//                0x12   34   56   78
																	//������Buff_Rx.Data.IO_Data[0]��[1]�еĸ�ʽ��С��ģʽ�����ֽڸߵ�ַ�����ֽڵ͵�ַ��
																	//	Э���е�0x1234	-> Buff_Rx�е�0x3412	-> 	Э���е�0x1234	         
		T_Head =  ( SET_OUT>>8 | SET_OUT<<8 );
		Buff_Tx.Lengh = ( 0x0010>>8 | 0x0010<<8 );
		Buff_Tx.Head  =	T_Head;
		LENGH_TX = 16;
		
		R8_INT_EN_IRQ_0 &= ~RB_IE_IRQ_TMR1;                                          //�ر�TIM1�ж�   
		
		Control_PLC_End = 0;                            //PLCɨ��������ֹ
						
		memset(EEPROM_CODE,0,sizeof(EEPROM_CODE));     //PLC���뻺�����
		memset(EEPROM_BUFF,0,sizeof(EEPROM_BUFF));     //PLC�Ĵ����������
		
		T_Head = LOWER_ANSW;
		
		
		/*����λ���㲥֡���ҳ�����IP��ӦIO�˿�Output_Portֵ*/
		/*
		for(i = 0;i < (LENGH-4)/6;i += 3)
		{
			if(Buff_Rx.Data.IO_Data[i] == T_IP)
			{
			    Output_Port = Buff_Rx.Data.IO_Data[i+2]<<16 | (UINT32) Buff_Rx.Data.IO_Data[i+1] ;	 
																   // ��ӦGPIO��Ҫд��Ŀ���������źţ�OUT0~21
				break;
			}
		}
		*/
		Output_Port = Buff_Rx.Data.IO_Data[1]<<16 | (UINT32) Buff_Rx.Data.IO_Data[0] ;

		SetOutput(Output_Port);
				
		Buff_Tx.Data.IO_Data[0] = Buff_Rx.Data.IO_Data[0] ;
		Buff_Tx.Data.IO_Data[1] = Buff_Rx.Data.IO_Data[1] ;
		
		FirstBuf_Tx[0] = Buff_Tx.Start;
		memcpy(FirstBuf_Tx+1,P_Buff_Tx+2,LENGH_TX-2);		      //��Buff_Tx�е����ݿ������ṹ��FirstBuf_Tx�У�	ע��u16��u8��0x3412 ��Ϊ 0x12 0x34
		FirstBuf_Tx[LENGH_TX-1] = Buff_Tx.End;

	}
    if( R_Head == GET_OUT )									        // ��λ����ȡ���������
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
		memcpy(FirstBuf_Tx+1,P_Buff_Tx+2,LENGH_TX-2);		      //��Buff_Tx�е����ݿ������ṹ��FirstBuf_Tx�У�	ע��u16��u8��0x3412 ��Ϊ 0x12 0x34
		FirstBuf_Tx[LENGH_TX-1] = Buff_Tx.End;
	
	}
	if( R_Head == SEND_IP_ID )									        // ��λ�����ͱ���IP��ID
	{

		T_Head = ( SEND_IP_ID>>8 | SEND_IP_ID<<8 );
		Buff_Tx.Lengh = ( 0x000e>>8 | 0x000e<<8 );
	    Buff_Tx.Head  =	T_Head;
	    LENGH_TX = 14;
	
	    Buff_Tx.Data.IO_Data[0] =  (UINT16)IPAddr[3]<<8;
	    
		FirstBuf_Tx[0] = Buff_Tx.Start;
		memcpy(FirstBuf_Tx+1,P_Buff_Tx+2,LENGH_TX-2);		      //��Buff_Tx�е����ݿ������ṹ��FirstBuf_Tx�У�	ע��u16��u8��0x3412 ��Ϊ 0x12 0x34
		FirstBuf_Tx[LENGH_TX-1] = Buff_Tx.End;
		
	}
	if( R_Head == SET_M000 )									        // ��λ�����á����س���ģ��ִ�����ض�����M000Ϊ���ر�����Ĭ��Ϊ0��ִ��
	{
		EEPROM_BUFF[0+2*0x30] = 1;					      //����PLC����M000��ֵΪ1��PLC���ض���ִ����֮��M000�Զ���λΪ0
	}

	
	if( R_Head == PLC_START )									        //��λ������ȫ������
	{
		printf("START\n");
		//PLC_PowerOn_Init();											//����λ�������ϵ��ʼ��PLC����
	}

	if( R_Head == PLC_STOP )									        //��λ������ȫ��ֹͣ
	{
		R8_INT_EN_IRQ_0 &= ~RB_IE_IRQ_TMR1;                                          //�ر�TIM1�ж�   
		
		Control_PLC_End = 0;                            //PLCɨ��������ֹ
		
		SetOutput(0xffffffff);                          //IOȫ������
				
		memset(EEPROM_CODE,0,sizeof(EEPROM_CODE));     //PLC���뻺�����
		memset(EEPROM_BUFF,0,sizeof(EEPROM_BUFF));     //PLC�Ĵ����������
		
		T_Head = LOWER_ANSW;
	}
	

	if( R_Head == TEST )
	{
		printf("R_Head == TEST\n");
		CH563NET_SocketUdpSendTo(SocketId,p,&len,ip,28000);
	}

	
	//�˴���ӳ���
}

/*******************************************************************************
* Function Name  : Reverse
* Description    : ���������������еõ�������
* Input          : UINT8 a Ϊ��������  ����λ Ϊ0~��n-1��
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
* Description    : ��ò����̺Ͳ��뿪���趨��IP��ID
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
* Description    : �����豸�����ϵ�ʱ����ȡEEPROM�б����PLC����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PLC_PowerOn_Init()                                                        
{
	UINT8 i;
    
	  R8_INT_EN_IRQ_0 &= ~RB_IE_IRQ_TMR1;                                          //�ر�TIM1�ж�   
		
		Control_PLC_End = 0;                            //PLCɨ��������ֹ
		
		SetOutput(0xffffffff);                          //IOȫ������
				
		memset(EEPROM_CODE,0,sizeof(EEPROM_CODE));     //PLC���뻺�����
		memset(EEPROM_BUFF,0,sizeof(EEPROM_BUFF));     //PLC�Ĵ����������
	
	
	  CH563_EEPROM_READ ( 0,EEPROM_CODE,512-4 );                                  //  EEPROM��ȡ��ַ0x0000���ݣ���ȡ512-4�ֽڵ�EEPROM_CODE��

		for(i=0;i<(512/2-2);i++)				                                // С��ģʽ��u8 0x12 0x34ת��u16 Ϊ0x3412���ʴ˴�Ҫ������
		{
			EEPROM_CODE[i] = (UINT16)(EEPROM_CODE[i]<<8|EEPROM_CODE[i]>>8);	
		}
	
		Control_PLC_End = 1;                            //PLCɨ��ȫ�ֿ���������
    R8_INT_EN_IRQ_0 |= RB_IE_IRQ_TMR1;                                          //����TIM1�ж� 
}

/*******************************************************************************
* Function Name  : InitTIM1
* Description    : ��ʼ����ʱ��1�������ж�����		5ms
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitTIM1(void)
{
   
	  R32_TMR1_CNT_END = 0x186a0 * 5;                                             //����Ϊ5MS��ʱ	
  	R8_TMR1_CTRL_MOD = RB_TMR_ALL_CLEAR;   	
	  R8_TMR1_CTRL_MOD = RB_TMR_COUNT_EN;										                     //��ʱ��������ʹ��    
	  R32_TMR1_COUNT = 0x00000000; 
    R8_TMR1_INTER_EN |= RB_TMR_IE_CYC_END;
    
	  R8_INT_EN_IRQ_0 |= RB_IE_IRQ_TMR1;                                          /* ����TIM1�ж� */
    R8_INT_EN_IRQ_GLOB |= RB_IE_IRQ_GLOB;                                       /* ����IRQȫ���ж� */
}



void IRQ_InitPD( void )	//PD0ΪSCL��PD1ΪSDA
{
    R32_PD_PU  |= 0x00000001;                                                   /* GPIO D0�������ã���1��ʾ���� */ 
    R32_PD_DIR &= ~0x00000001;                                                  /* GPIO D0��������Ϊ���� , direction: 0=in, 1=out */

			 
	R32_PD_PU  |= 0x00000002;                                                   /* GPIO D1�������ã���1��ʾ���� */ 
    R32_PD_DIR &= ~0x00000002;                                                  /* GPIO D1��������Ϊ���� , direction: 0=in, 1=out */


    R32_INT_ENABLE_PD |= 0x00000001;                                            /* GPIO D0�ж�ʹ�� �� 1-ʹ�ܣ�0-��ֹ */
    R32_INT_MODE_PD   |= 0x00000001;                                            /* GPIO D0�жϷ�ʽ ��1-�����ж�,0-��ƽ�ж� */      
    R32_INT_POLAR_PD  |= 0x00000001;                                            /* GPIO D0�жϼ��� ��1-�������ж�/�ߵ�ƽ��0-�½����ж�/�͵�ƽ */      
    
    R32_INT_STATUS_PD  = 0xffffffff;                                               /* �жϱ�־д1���� */
    R8_INT_EN_IRQ_1   |= RB_IE_IRQ_PD;                                          /* GPIO D���ж�ʹ�� */ 
     
}


/*******************************************************************************
* Function Name  : main
* Description    : ������
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
	UINT8  DES_IP[4] = {192,168,1,2};                                         /* Ŀ��IP��ַ */
	UINT8  *ip = DES_IP;

    Init_GPIO( );																//��ʼ��GPIO
    //mInitSTDIO( );                                                              /* Ϊ���ü����ͨ�����ڼ����ʾ���� */
	Uart1_Init( 9600);
	
	InitTIM1( );																//��ʼ����ʱ��1���ж�����5ms
	//GetLocal_IP_ID();															//��ò����̺Ͳ��뿪���趨��IP��ID

	IRQ_InitPD( );

    i = CH563NET_LibInit(IPAddr,GWIPAddr,IPMask,MACAddr);                       /* ���ʼ�� */
    mStopIfError(i);                                                            /* ������ */
   
    SysTimeInit();                                                              /* ϵͳ��ʱ����ʼ�� */
    InitSysHal();                                                               /* ��ʼ���ж� */	
	CH563NET_CreatUdpSocket();	

#if CH563NET_DBG
    printf("CH563IPLibInit Success\n");
#endif 


   
	while(1)
    {
        CH563NET_MainTask();                                                    /* CH563NET��������������Ҫ����ѭ���в��ϵ��� */
        
		    if(CH563NET_QueryGlobalInt())CH563NET_HandleGlobalInt();                /* ��ѯ�жϣ�������жϣ������ȫ���жϴ����� */
		
		/* 
		if(PLC_PowerOn_Init_Flag == 1 && PLC_PowerOn_Init_Count == 1000)	  //�ϵ���ʱ5s���ʼ��PLC����
		{
			CH563NET_SocketUdpSendTo(SocketId,p,&len,ip,28000);              // ��ָ����Ŀ��IP���˿ڷ���UDP���������������IP�Ͷ˿ڷ������ݣ�
			//PLC_PowerOn_Init();
			printf("CH563NET_SocketUdpSendTo\n");
			PLC_PowerOn_Init_Count = 0;															
			//PLC_PowerOn_Init_Flag = 0;
        }
		*/
				
				
				
				/*���º���������λ��ִ��PLCɨ�����ʱ��ÿ��100��������λ������һ�ο��������״̬*/		
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

			p = FirstBuf_Tx;												//����ѭ��ǰ��pָ������ָ��FirstBuf_Tx�׵�ַ

			while(1)
	        {
	           len = LENGH_TX;
	           //CH563NET_SocketSend(SocketId,p,&len);                            // ��FirstBuf_Tx�е����ݷ���(ֻ�򴴽�socketʱָ����Ŀ��IP�Ͷ˿ڷ�������)
			   CH563NET_SocketUdpSendTo(SocketId,p,&len,ip,27000);               // ��ָ����Ŀ��IP���˿ڷ���UDP���������������IP�Ͷ˿ڷ������ݣ�
	           LENGH_TX -= len;                                                // ���ܳ��ȼ�ȥ�Ѿ�������ϵĳ��� 
	           p += len;                                                       // ��������ָ��ƫ��
	           if(LENGH_TX)continue;                                           // �������δ������ϣ����������
	           break;                                                          // ������ϣ��˳� 
	        } 
			
			PLC_Count = 0;
		}*/ 
    }
}

/*********************************** endfile **********************************/



