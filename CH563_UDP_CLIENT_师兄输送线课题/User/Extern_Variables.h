/******************************************************************************/
/* ͷ�ļ�����*/
#include "CH563SFR.H"


#ifndef _EXTERN_VARIABLES
#define _EXTERN_VARIABLES

/*�궨��*/
//////////////////////////////		����ͷ
//===========================�ϴ�����λ��ָ���ͷ======================
#define		LOWER_ANSW		0x00B0		        // ��û�в����ص����ݵ�Ӧ���ͷ


//===========================����λ�����ܰ�ͷָ��======================
#define		SEND_IP_ID	    0x00A1		        //��λ�����ͱ���IP��ID
#define		PLC_Q			0x00A2		        //��λ��ִ�е�PLC����
#define		GET_IN			0x00FF				//��λ����ȡ����������
#define		SET_OUT			0x00FE				//��λ�����ÿ��������
#define		GET_OUT			0x00FD				//��λ����ȡ���������
#define		SET_M000	    0x00FC				//��λ�����á����س���ģ��ִ�����ض�����Ĭ�ϲ�ִ��


/*TCP��UDP�շ�������� */
typedef union _BUFF_DATA
{
	UINT16  PLC_Cmd[100];
	UINT16  IO_Data[100];
}BUFF_DATA;


typedef struct _BUFF_UDP_DATA
{	
    volatile UINT16 Lengh;						//��Ƭ����
	volatile UINT16 Head;		                //��ͷ 
	BUFF_DATA       Data;
}BUFF_UDP_DATA;
extern volatile BUFF_UDP_DATA Buff_Rx;			//����λ�����յĻ�������
extern volatile BUFF_UDP_DATA Buff_Tx;			//���͸���λ���Ļ�������
extern volatile UINT8 FirstBuf_Tx[1000];	    //�����һ�����ݷ��ͻ�����
extern UINT32  LENGH_TX;				        //���ݴ����Socket�⺯�����͸���λ�������õ��ַ�����
extern UINT16  EEPROM_CODE[512]; 		        //1024�ֽڻ��棬���ڴ��PLC���룬��RAM������
extern UINT16  EEPROM_BUFF[512]; 		        //1024�ֽڻ��棬���ڴ��PLC�������������״̬����RAM������
extern UINT8 SocketId;                          /* ����socket���������Բ��ö��� */

/* PLC��ض��� */
#define PLC_Code_Addr      0x0000					//PLC������EEPROM�д�ŵ���ʼ��ַΪ0x0000
#define PLC_BUFF_Addr      0x1000				    //PLC������EEPROM�д�ŵ���ʼ��ַΪ0x1000

typedef struct _Ctr_Value
{
	UINT32 ActionFlag : 1;  	  //�жϼ������ܷ����������0��Ϊ1ʱ������һ�Σ�
	UINT32 CountFlag : 1;		  //�������Ƿ��ڹ�����0��ʾδ������1��ʾ����
	UINT32 Value :30;			  //����������ļ���ֵ

}CtrValue;

typedef struct _Timer
{
    UINT32 Flag : 1;		          //�ж϶�ʱ���Ƿ����ڹ�����0��ʾֹͣ��1��ʾ����
	UINT32 Value : 31;	          //���涨ʱ���ĸ���ֵ

}Timer;

extern UINT8   PLC_Count;
/*
//����EEPROM_BUFF�е�ƫ�Ƶ�ַ	   ʵ��EEPROM�е�ƫ�Ƶ�ַ
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


/* CH563��ض��� */
extern const UINT8 MACAddr[6];                       /* CH563MAC��ַ */
extern UINT8 IPAddr[4];                              /* CH563IP��ַ */
extern const UINT8 GWIPAddr[4];                      /* CH563���� */
extern const UINT8 IPMask[4];                        /* CH563�������� */
extern const UINT8 DESIP[4];                         /* Ŀ��IP��ַ */
extern UINT8 ID_Number;                              /* ��λ�����ӱ�� */


/* �ⲿ�������� */

#endif

/*********************************** endfile **********************************/