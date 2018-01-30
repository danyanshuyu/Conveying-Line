/******************************************************************************/
/* ͷ�ļ�����*/
#include "Extern_Variables.h"

UINT16  EEPROM_BUFF[512]; 		       //1024�ֽڻ��棬��RAM������

/*�¶κ������ڵó�PLC��ֵ*/
UINT16 GetValue(UINT16 Num,UINT16 type)						// ����˵����Num��PLC�����ţ�type��PLC��������					
{	UINT16 Offset;											// ����һ��ƫ�ñ���ֵ�����ڱ���PLC����������
	UINT16 *pAddr;											// ����һ����ַ����
	UINT16 Data;											// ����һ���������������淵����
	UINT16V Input_Port = 0;							        // �ñ�����ӦӲ���ж����Ŀ���������

	Offset = type;											// X1 Y2 M3 C4 T5

	/*���³������ڻ�üĴ����ڻ���EEPROM_BUFF�еĴ洢��ַ����������0xA0��ʼ���������͵ļ�������0x30�ı�����ʼ*/

	if(type == C)
	{
		pAddr =(UINT16 *)EEPROM_BUFF + Num + (Offset-1)*0x30 + 0x10;		// �������������Ŷ�Ӧ�ĵ�ַ
	}
	else 
	{
		pAddr =(UINT16 *)EEPROM_BUFF + Num + (Offset-1)*0x30;
	}
	/*���³������ڻ�ȡ�����ͼĴ����е�ֵ��X���ͻ�ȡ��ʽ��GPIO����Ĵ�����ֵ*/
	if(type == X)												// ���PLC�ź�ΪX����
	{
	    Input_Port =(UINT16)( (R8_PD_PIN_3>>7) & 0x01 | R8_PB_PIN_0<<1 ) | (UINT16)( (R8_PB_PIN_2 & 0x0F) | ((R8_PD_PIN_1<<4) & 0xF0))<<8;	 
																// ��GPIO�ж������������źţ��ɸ�λ����λ��INPUT15~INPUT0
		Data = (~(Input_Port>>Num)&0x0001);					    // ȡ�ö�Ӧ�ź�Ϊ1������0
	}
	else													    // ����ź����Ͳ���X�������Ӧ�źŵ����ͼ���ŵĵ�ַ��ȡ��ֵ
		Data = *pAddr ;

	return Data;											    // ����ȡֵ
}

/*�¶κ�����������PLC������ֵ*/

void SetValue(UINT16 Num,UINT16 type,UINT16 Data)			// ����˵����PLC�����ţ�PLC�������ͣ����õ����ݣ�ֻ��0��1��1������Ч��0��ʾ�����Ч
{	UINT16 Offset;											// �ò������ڱ������������
	UINT16 *pAddr;											// �ò������ڱ�������ĵ�ַ

	if(type == Y)		                                    // ��������Ϊ2��Y����������в�������
	{	if(Data == 0)										// ������ź�������Ϊ0�����������1���ߵ�ƽ��Ч				
		{
			if(0<=Num && Num<=3)
			R8_PD_OUT_1 |= (1<<(Num+4));					//�����������GPIO���Ŷ�Ӧ��ϵ
			else if(Num<=11)								//OUT  21 20 19 18 17 16 15 14 13 12  11 10 9 8 7 6 5 4     3  2  1  0
			R8_PA_OUT_0 |= (1<<(Num-4));					//PA   17 16 11 10  9  8 15 14 13 12  7  6  5 4 3 2 1 0 PD 15 14 13 12 
			else if(Num<=15)								  		   
			R8_PA_OUT_1 |= (1<<(Num-8));					
			else if(Num<=19)
			R8_PA_OUT_1 |= (1<<(Num-16));
			else if(Num<=21)
			R8_PA_OUT_2 |= (1<<(Num-20));
		}
		else if(Data == 1)									// �������ֵΪ1����������ź���0���͵�ƽ��Ч
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

	Offset = type&0x000f;

	/*���³������ڻ�ô洢���Ĵ洢��ַ*/

	if(Offset == C)
	{
		pAddr = (UINT16 *)EEPROM_BUFF + Num + ((Offset-1)*0x30 + 0x10);
	}
	else 
	{
		pAddr = (UINT16 *)EEPROM_BUFF + Num + (Offset-1)*0x30;
	}
	
	*pAddr = Data;											// ��PLC�߼���д������
}

/*�ú������ڼ��������㣬���������ã�����������������Ĵ�����������������ʱ�����Ч��Ҫ�����ڶ�ʱ���Ĺ���*/

void CntProcess(UINT16 stack,UINT16 Cnt_set,UINT16 Cnt_num,UINT16 Rst)		// ��������������������ֵ���������ţ���λ
{	CtrValue *pCtrValue;									// ����������ṹ��ָ�����
	UINT32 *pCtrValueAndFlag;								// ����ָ����������ڴ�ż�������ŵĵ�ַ

	pCtrValueAndFlag = (UINT32 *)(EEPROM_BUFF + 0x180);		// ��������ŵ�������ݿ�ʼ��ַ		 0X180

	pCtrValue = (CtrValue *)(pCtrValueAndFlag + Cnt_num);	// �õ���ͬ�������ŵĵ�ַ���õ�ַ�ж�Ӧ�����ΪPLC�����״̬
	
	/*���³������ڼ�������������*/

	if(Rst == 1) 											// ��λ���ؿ�����λ���ȼ���,��ʱΪ��λ
	{	pCtrValue->ActionFlag = stack;						// �жϼ������ܷ����������0��Ϊ1ʱ������һ��
		pCtrValue->CountFlag = 0;							// �������Ƿ��ڹ�����0��ʾδ������1��ʾ����
		pCtrValue->Value = Cnt_num;							// ����������ļ���ֵ��
						
		SetValue(Cnt_num,4,0);								// �趨Cnt_num�ż����������Ϊ0���ö�ʱ����Ч
	}
	else													// ��λ�ź���Ч
	{	if(stack == 1)										// �൱������������Ч
		{	if(pCtrValue->CountFlag == 0)					// ��ʼ״̬�����������δ���ڼ�����״̬����ʼ����
			{	// ��ʼ����
				pCtrValue->Value = Cnt_set;					// �趨������������ֵ
				pCtrValue->Value -= 1;						// ��������һ
				pCtrValue->ActionFlag =1;					// ���������Լ���
				pCtrValue->CountFlag = 1;					// ���������ڼ���״̬
			}
			else											// ���������ڼ���״̬����һ��������Ȼ����
			{	// ��������
				if(pCtrValue->ActionFlag == 0)				// �����������������˴��ӵڶ����������忪ʼ������
				{	if(pCtrValue->Value>0)					// �������ֵ��Ϊ0����������
					{	pCtrValue->Value -= 1;				// ����ֵ��һ
					}

					pCtrValue->ActionFlag = 1;				// ÿ����һ�κ󣬽���λ��1��Ϊ�´μ�����׼��
					
					if(pCtrValue->Value == 0)				// ����ֵΪ0����ʾ����������
					{	SetValue(Cnt_num,4,1);				// ������������PLC�����Ч
					}								
				}
				else
				{	pCtrValue->ActionFlag = 1;				// �ò�������
				}					
			}
		}
		else												// ���������Ͽ�ʱ����������ƽ��0��Ϊ�´μ�����׼��
		{	pCtrValue->ActionFlag = 0;
		}
	}
}

/*���溯�����ڴ���PLC��ʱ������*/

void TimerProcess(UINT16 stack,UINT16 Timer_set,UINT16 Timer_num)		// ����˵������ʱ���ж���������ʱ������ֵ����ʱ����
{	UINT32 *pTimerValueAndFlag ;							// ���ڶ�ʱ�������ڴ�
	Timer *pTimer ;											// ���嶨ʱ������ָ��

	pTimerValueAndFlag = (UINT32 *)(EEPROM_BUFF + 0x100);	// �õ���ʱ��������ŵĵ�ַ			 0X100
															// ���ڶ�ʱ�������ڴ�
	pTimer = (Timer *)(pTimerValueAndFlag + Timer_num);		// �õ���ͬ��ʱ���Ŷ�Ӧ�ĵ�ַ
	
	/*���³��������ж��Ƿ���ж�ʱ������*/
					
	if(stack == 1)											// �����ʱ���������������㣬��ʼ��ʱ������
	{	
		if(pTimer->Flag == 0)								// ��ʱ����ʼ����״̬���ڹر�״̬����ʱ����������ʱ�����ж�ʱ��������ʼ��
		{	
			pTimer->Flag = 1;								// ��ʾ��ʼ����
			pTimer->Value = Timer_set;						// �趨��ʱ����ʱʱ��
			SetValue(Timer_num,5,0);						// ��Ӧ��ʱ�������Ч,�൱�ڸ�λһ�¶�ʱ��
		}
		else												// ��ʱ�����ڶ�ʱ״̬����������ʱ
		{	// ��������
			pTimer->Value -= 5;								// ��ʱ�Ĵ����ڵ���ֵ�ݼ���ÿ�����ڶ�ʱʱ��Ϊ5ms����˼�5
			if(pTimer->Value <= 4)							// ��ʱ��С��һ���ж�����ʱ����ʾ��ʱʱ�䵽
			{
				SetValue(Timer_num,5,1);					// ����Ӧ�Ķ�ʱ����λ��PLC�����Ч
				pTimer->Flag = 0;							// ��ʾ��������
			}
		}				
	}
	else													
	{													
		if(pTimer->Flag == 1)
		{    // ��������
			pTimer->Value -= 5;								// ��ʱ�Ĵ����ڵ���ֵ�ݼ���ÿ�����ڶ�ʱʱ��Ϊ5ms����˼�5
			if(pTimer->Value <= 4)							// ��ʱ��С��һ���ж�����ʱ����ʾ��ʱʱ�䵽
			{
				SetValue(Timer_num,5,1);					// ����Ӧ�Ķ�ʱ����λ��PLC�����Ч
				pTimer->Flag = 0;							// ��ʾ��������
			}
		}
		else
		{
			pTimer->Value = 0;									// ��ʱ����Чֵ��ʱʱ��Ϊ0
			SetValue(Timer_num,5,0);							// ��Ӧ��ʱ�������Ч,�൱�ڸ�λһ�¶�ʱ��
		}
	}

}


/*�ú�������PLCɨ��*/

void Proc_plc_code()
{	UINT16 stack1[30],stack2[30];							// ��ջ	30���������30���������LD��OUT�ֱ�������30�Ρ�
	UINT16 point1=0, point2=0;								// ��ջָ��
	UINT16 bEnd;											// �ж��Ƿ�ɨ�����

	/*���±��������м䴦�����*/

	UINT16 Data = 1;
	UINT16 a,b,i;	 

	UINT16 *pNow = (UINT16 *)EEPROM_CODE;					// ������ȡ���룬�ñ������PLC����ĳ�ʼ��ַ
	
	UINT16 Type;											// ָ�����ͣ����ӡ��롢������ȣ�
	UINT16 RegType;											// �Ĵ�������	
	UINT16 *pNum;											// �Ĵ��������

	UINT16 *pTimeSet;										// ��ʱ�����趨ֵ ,���º��ʱ��ֵ�� ��λms

	UINT16 *pCntSet;										// �������趨ֵ

	bEnd = 1;												// ��β�ж�ָ�1�����ڳ���ɨ��

	/*�¶γ�������PLC����ͼ��ɨ�裬ĩβ������Ч*/
	while(bEnd)												// ���ĩβ��־Ϊ1��PLCѭ��ɨ��										
	{	pNum = (UINT16 *)pNow;								// ����PLC������׵�ַ���õ�ַ�д�żĴ��������
		pNow++;										// ����PLC�������һ����ַ���������ͼ��ָ��ΪLD M023����������ָ��˳��Ϊ023,LD��M������Ǽ������Ͷ�ʱ���������¸���ַ��Ŵ�����ʱ��

		Type = (*pNow)&0xff00;                              //���ָ�����ͣ�LD,AND,OR�ȣ�								
		RegType = (*pNow)&0x00ff;							// ��üĴ������ͣ�X��Y��M�ȣ�
		pNow++;												// ��ַ����
		/*���³��������ж�PLCָ������*/
		switch(Type)
		{	case NTO:
				point1 = 0;
				point2 = 0;
				break;

			case NOOP: 										// ��ָ��
				point1 = 0; 								// ��ջָ��
				point2 = 0;									// ��ջָ��
				break;

			case END:										// ����ָ��
				bEnd = 0;
				break;

			case LD:										// ��������ӳ�������
				Data = GetValue(*pNum,RegType);				// �����Ӧ�Ĵ�������е�ֵ
				// ��ջ
				stack1[point1] = Data;						// ���üĴ����е�ֵ��ջ
				point1++;									// ջ���ӣ����ڱ�����һ���Ĵ�����ֵ
				break;

			case LDI:										// ������ӵ��ǳ��մ���	
				Data = GetValue(*pNum,RegType);				// �����Ӧ�Ĵ�����ֵ
				if(Data == 1)								// ���մ���Dataȡ��
				{	Data = 0;	}
				else
				{	Data = 1;	}
				stack1[point1] = Data;						// ���üĴ�����ֵ��ջ����
				point1++;									// ջָ�����ӣ����ڱ�����һ���Ĵ�����ֵ
				break;

			case AND:										// ָ������Ϊ�볣������ȡ��ָ��	
				Data = GetValue(*pNum,RegType);				// �õ���Ӧ�Ĵ�����ֵ
				stack1[point1-1] = stack1[point1-1] & Data;	// ��ǰһ���Ĵ�����ֵȡ�룬�õ������Ĵ�������Ľ��
				break;

			case ANI:										// ָ������Ϊ�볣�մ���ȡ��	
				Data = GetValue(*pNum,RegType);				// ��øüĴ�����ֵ
				if(Data == 1)								// Dataȡ��
				{	Data = 0;	}
				else
				{	Data = 1;	}							
				stack1[point1-1] = stack1[point1-1] & Data;	// ǰ�������Ĵ���ֵȡ��
				break;

			case OR:										// ָ������Ϊ�볣������ȡ��	
				Data = GetValue(*pNum,RegType);
				// �޸�ջֵ
				stack1[point1-1] = stack1[point1-1] | Data;
				break;

			case ORI:										// ָ������Ϊ�볣�մ���ȡ��	
				Data = GetValue(*pNum,RegType);
				if(Data == 1)		// Dataȡ��
				{	Data = 0;	}
				else
				{	Data = 1;	}
				// �޸�ջֵ
				stack1[point1-1] = stack1[point1-1] | Data;
				break;

			case OUT:										// ָ������Ϊ�����������
 				Data = stack1[point1-1];					// ������Ϊǰ��һϵ�мĴ����߼��Ľ��
				
				/*��������������Ĵ��������ͽ����������*/

				if( (RegType==0x02)||(RegType==0x03) )		// ����Ĵ���ΪY��M 
					SetValue(*pNum,RegType,Data);			// ȷ�����ֵ
				else if( RegType==0x04 )					// �������Ĵ���Ϊ������
				{	pCntSet = (UINT16 *)pNow;				// ��ü���ֵ			
					//pNow = pNow+2;						// ����ַ����
					pNow++;
					CntProcess(Data,*pCntSet,*pNum,0);		// ����������			
				}
				else if( RegType==0x05 )					// ����Ĵ�������Ϊ��ʱ��
				{	
					pTimeSet = (UINT16 *)pNow;				// �õ���ʱʱ��	
					pNow++;
					TimerProcess(Data,*pTimeSet,*pNum);		// ��ʱ������
				}
				break;

			case OUTI:										// 	ָ������Ϊ������մ���
				Data = stack1[point1-1];					// Data = stack1[point1];
				if(Data == 1)								// Dataȡ��
				{	Data = 0;	}
				else
				{	Data = 1;	}

				if( (RegType==0x02)||(RegType==0x03) )		// Y or M 
					SetValue(*pNum,RegType,Data);
				else if( RegType==0x04 )					// C
				{	pCntSet = (UINT16 *)pNow;				//��ʱ�趨ֵ
					//pNow = pNow+2;
					pNow++;
					CntProcess(Data,*pCntSet,*pNum,0);			
				}
				else if( RegType==0x05 )					// T
				{	pTimeSet = (UINT16 *)pNow;
					pNow++;

					TimerProcess(Data,*pTimeSet,*pNum);
				}
				break;

			case SET:										// ָ������Ϊ��λָ��		
				Data = stack1[point1-1];					// ���ǰ��һϵ�е��߼��Ľ��
				if(Data == 1)								// ����ǰ����߼����ȷ���������������߼���������λ
				{	SetValue(*pNum,RegType,1);
				}
				break;

			case RST:										// ָ������Ϊ��λ	
				Data = stack1[point1-1];
				if(Data == 1)								// �߼�������Ч
				{	if( (RegType==0x02)||(RegType==0x03) )	// Y or M 
						SetValue(*pNum,RegType,0);			// ������Ϊ0
					else if( RegType==0x04 )				// C
						CntProcess(Data,*pCntSet,*pNum,1);	// ��������λ�������Ч		
					else if( RegType==0x05 )				// T
						TimerProcess(0,*pTimeSet,*pNum);	// ��ʱ��ֹͣ����
				}
				break;

			case ANB:										// �������߼���ȡ�루�ο�����ͼ�Ļس��߼�ǰ������Ԫ�����룩
				a = stack1[point1-1];						// �õ���ǰ�߼��鵥Ԫ��ֵ
				b = stack1[point1-2];						// �õ�ǰһ���߼��鵥Ԫ��ֵ					
				point1--;
				Data = a & b;								// �ó������߼��鵥Ԫ����Ľ��
				stack1[point1-1] = Data;					// ��������Ľ��
				break;

			case ORB:		                               // �������߼��鵥Ԫȡ��
				a = stack1[point1-1];
				b = stack1[point1-2];
				point1--;
				Data = a | b;
				stack1[point1-1] = Data;
				break;

			case MPS:										// ��������Ϊ0,��1ջ��Ԫ�ط���ջ2
				Data = stack1[point1-1];
				stack2[point2] = Data;
				point2++;
				break;

			case MRD:										// ��������Ϊ0,��2ջ��Ԫ�ط���1ջ
				Data = stack2[point2-1];
				stack1[point1-1] = Data;
				break;

			case MPP:		// ��������Ϊ0,��2ջ��Ԫ�ط���1ջ,2ջ��ջ
				Data = stack2[point2-1];
				stack1[point1-1] = Data;
				point2--;
				break;
		}	
	}	
}






/*********************************** endfile **********************************/

