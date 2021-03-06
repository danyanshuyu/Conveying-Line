/******************************************************************************/
/* 头文件包含*/
#include "CH563SFR.H"


#ifndef _EXTERN_VARIABLES
#define _EXTERN_VARIABLES

/*宏定义*/
//////////////////////////////		报文头
//===========================上传到上位机指令包头======================
#define		LOWER_ANSW		0x00B0		        // 对没有参数回的数据的应答包头


//===========================从上位机接受包头指令======================
#define		GET_IN			0x00FF				//上位机获取开关量输入
#define		SET_OUT			0x00FE				//上位机设置开关量输出
#define		GET_OUT			0x00FD				//上位机获取开关量输出
#define		PLC_START 	    0x00FB				//上位机控制PLC全部启动
#define		PLC_STOP	    0x00FA				//上位机控制PLC全部停止
#define		HANDSHAKE	    0x00EE				//上位机控制下位机之间通信握手
#define		SEND_IP_ID	    0x00A1		        //上位机获取下位机本地IP和ID
#define		PLC_Q			0x00A2		        //上位机发送下位机执行的PLC代码

#define		SET_MXXX	    0x00FC				//下位机之间远程IO通信
#define		BARCODE 	    0x00B1				//下位机发送识别到的条形码
#define		ALARM	        0x00B2				//下位机发送报警信号
#define		HANDSUCCESS	    0x00ED				//下位机发送通信握手成功信号

/*TCP、UDP收发数据相关 */
typedef union _BUFF_DATA
{
	UINT16  PLC_Cmd[512];
	UINT16  IO_Data[100];
	UINT16  BarCode_Data[100];
	UINT16  Mxxx_Data[100];
	UINT16  IP_Data[100];
}BUFF_DATA;


typedef struct _BUFF_UDP_DATA			 //考虑字节对齐，UINT8  Start;UINT8  End;应放在一起，待改进
{	
	UINT8  Start;						//起始位
	UINT16 Lengh;						//切片长度
	UINT16 Packet_Flag;		            //包标记数
	UINT16 SourSite;			        //源站点号
	UINT16 DesSite;		                //目标站点号
	UINT16 Head;						//包头 
	BUFF_DATA       Data;				//数据
	UINT8  End;						    //停止位
}BUFF_UDP_DATA;
extern volatile BUFF_UDP_DATA Buff_Rx;			//从上位机接收的缓存数据
extern volatile BUFF_UDP_DATA Buff_Tx;			//发送给上位机的缓存数据
extern volatile UINT8 FirstBuf_Tx[1000];	    //定义第一层数据发送缓冲区
extern UINT32  LENGH_TX;				        //数据处理后，Socket库函数发送给上位机所调用的字符长度
extern UINT16  EEPROM_CODE[512]; 		        //1024字节缓存，用于存放PLC代码，在RAM中运行
extern UINT16  EEPROM_BUFF[512]; 		        //1024字节缓存，用于存放PLC输入输出开关量状态，在RAM中运行


/* PLC相关定义 */
#define PLC_Code_Addr      0x0000					//PLC代码在EEPROM中存放的起始地址为0x0000
#define PLC_BUFF_Addr      0x1000				    //PLC变量在EEPROM中存放的起始地址为0x1000

typedef struct _Ctr_Value
{
	UINT32 ActionFlag : 1;  	  //判断计数器能否计数，当从0变为1时，计数一次，
	UINT32 CountFlag : 1;		  //计数器是否在工作，0表示未计数，1表示计数
	UINT32 Value :30;			  //保存计数器的计数值

}CtrValue;

typedef struct _Timer
{
    UINT32 Flag : 1;		          //判断定时器是否正在工作，0表示停止，1表示工作
	UINT32 Value : 31;	          //保存定时器的更新值

}Timer;

/*
//缓存EEPROM_BUFF中的偏移地址	   实际EEPROM中的偏移地址
	X	0x00							0x0000
	Y	0x30							0x0060
	M	0x60							0x00C0
	C	0xA0							0x0140
	T	0xC0							0x0180
*/
#define X 1
#define Y 2
#define M 3
#define C 4
#define T 5

#define LD 	 0xFF00
#define LDI	 0xFE00
#define AND	 0xFD00
#define ANI	 0xFC00
#define OR 	 0xFB00
#define ORI	 0xFA00
#define ANB  0xF900 
#define ORB  0xF800

#define SET  0xF700
#define RST  0xF600

#define MPS  0xF500
#define MRD  0xF400
#define MPP  0xF300

#define OUT  0xF200
#define NTO	 0xF100
#define END	 0xF000
#define NOOP 0xEF00

#define OUTI 22


#define TMR	 31
#define CTR	 32
#define AXL	 33
#define AXZ	 34
#define AXR	 35
#define DECB 36
#define ROTB 37

#define CALL 41
#define JMP	 42
#define JMPE 43
#define CALLE 46

#define LD_X       0xFF01
#define LD_Y       0xFF02
#define LD_M       0xFF03
#define LD_C       0xFF04
#define LD_T       0xFF05
#define LDI_X      0xFE01
#define LDI_Y      0xFE02
#define LDI_M      0xFE03
#define LDI_C      0xFE04
#define LDI_T      0xFE05
#define AND_X      0xFD01
#define AND_Y      0xFD02
#define AND_M      0xFD03
#define AND_C      0xFD04
#define AND_T      0xFD05
#define ANI_X      0xFC01
#define ANI_Y      0xFC02
#define ANI_M      0xFC03
#define ANI_C      0xFC04
#define ANI_T      0xFC05
#define OR_X       0xFB01
#define OR_Y       0xFB02
#define OR_M       0xFB03
#define OR_C       0xFB04
#define OR_T       0xFB05
#define ORI_X      0xFA01
#define ORI_Y      0xFA02
#define ORI_M      0xFA03
#define ORI_C      0xFA04
#define ORI_T      0xFA05
#define ANB_       0xF900
#define ORB_       0xF800
#define SET_Y      0xF702
#define SET_M      0xF703
#define SET_C      0xF704
#define SET_T      0xF705
#define RST_Y      0xF602
#define RST_M      0xF603
#define RST_C      0xF604
#define RST_T      0xF605
#define MPS_       0xF500
#define MRD_       0xF400
#define MPP_       0xF300
#define OUT_Y      0xF202
#define OUT_M      0xF203
#define OUT_C      0xF204
#define OUT_T      0xF205
#define NTO_       0xF100
#define END_       0xF000


/* CH563相关定义 */
extern UINT8 MACAddr[6];                             /* CH563MAC地址 */
extern UINT8 IPAddr[4];                              /* CH563IP地址 */
extern const UINT8 GWIPAddr[4];                      /* CH563网关 */
extern const UINT8 IPMask[4];                        /* CH563子网掩码 */
extern const UINT8 DESIP[4];                         /* 目的IP地址 */
extern UINT8 ID_Number;                              /* 下位机板子编号 */
extern UINT8 SocketId;                               /* 保存socket索引，用于与主机通信 */



extern UINT8  Uart_Buf[ 500 ];

extern UINT8  Control_PLC_End;

extern UINT8  PLC_Loop_Flag;
extern UINT8  UDP_Buff_Queue_Count;


/*远程IO相关*/
extern UINT8 Mxxx_Send_1[16];
extern UINT32 Mxxx_Send_1_len;
extern UINT8  Mxxx_Des_IP_1[4];                                         


#endif

/*********************************** endfile **********************************/