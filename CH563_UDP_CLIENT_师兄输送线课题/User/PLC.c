/******************************************************************************/
/* 头文件包含*/
#include "Extern_Variables.h"

UINT16  EEPROM_BUFF[512]; 		       //1024字节缓存，在RAM中运行

/*下段函数用于得出PLC的值*/
UINT16 GetValue(UINT16 Num,UINT16 type)						// 参数说明，Num：PLC变量号，type：PLC变量类型					
{	UINT16 Offset;											// 声明一个偏置变量值，用于保存PLC变量的类型
	UINT16 *pAddr;											// 声明一个地址变量
	UINT16 Data;											// 声明一个变量，用来保存返回量
	UINT16V Input_Port = 0;							        // 该变量对应硬件中读出的开关量输入

	Offset = type;											// X1 Y2 M3 C4 T5

	/*以下程序用于获得寄存器在缓存EEPROM_BUFF中的存储地址，计数器从0xA0开始，其余类型的计数器以0x30的倍数开始*/

	if(type == C)
	{
		pAddr =(UINT16 *)EEPROM_BUFF + Num + (Offset-1)*0x30 + 0x10;		// 计算变量及其序号对应的地址
	}
	else 
	{
		pAddr =(UINT16 *)EEPROM_BUFF + Num + (Offset-1)*0x30;
	}
	/*以下程序用于获取各类型寄存器中的值，X类型获取方式是GPIO输入寄存器的值*/
	if(type == X)												// 如果PLC信号为X类型
	{
	    Input_Port =(UINT16)( (R8_PD_PIN_3>>7) & 0x01 | R8_PB_PIN_0<<1 ) | (UINT16)( (R8_PB_PIN_2 & 0x0F) | ((R8_PD_PIN_1<<4) & 0xF0))<<8;	 
																// 从GPIO中读出开关量的信号，由高位到低位是INPUT15~INPUT0
		Data = (~(Input_Port>>Num)&0x0001);					    // 取得对应信号为1，还是0
	}
	else													    // 如果信号类型不是X，则从相应信号的类型及序号的地址中取出值
		Data = *pAddr ;

	return Data;											    // 返回取值
}

/*下段函数用于设置PLC变量的值*/

void SetValue(UINT16 Num,UINT16 type,UINT16 Data)			// 参数说明：PLC变量号，PLC变量类型，设置的数据：只有0和1，1代表有效，0表示输出无效
{	UINT16 Offset;											// 该参数用于保存变量的类型
	UINT16 *pAddr;											// 该参数用于保存变量的地址

	if(type == Y)		                                    // 变量类型为2（Y输出），进行参数设置
	{	if(Data == 0)										// 如果该信号量设置为0，将输出端置1，高电平无效				
		{
			if(0<=Num && Num<=3)
			R8_PD_OUT_1 |= (1<<(Num+4));					//开关量输出与GPIO引脚对应关系
			else if(Num<=11)								//OUT  21 20 19 18 17 16 15 14 13 12  11 10 9 8 7 6 5 4     3  2  1  0
			R8_PA_OUT_0 |= (1<<(Num-4));					//PA   17 16 11 10  9  8 15 14 13 12  7  6  5 4 3 2 1 0 PD 15 14 13 12 
			else if(Num<=15)								  		   
			R8_PA_OUT_1 |= (1<<(Num-8));					
			else if(Num<=19)
			R8_PA_OUT_1 |= (1<<(Num-16));
			else if(Num<=21)
			R8_PA_OUT_2 |= (1<<(Num-20));
		}
		else if(Data == 1)									// 如果设置值为1，将输出口信号置0，低电平有效
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

	/*以下程序用于获得存储器的存储地址*/

	if(Offset == C)
	{
		pAddr = (UINT16 *)EEPROM_BUFF + Num + ((Offset-1)*0x30 + 0x10);
	}
	else 
	{
		pAddr = (UINT16 *)EEPROM_BUFF + Num + (Offset-1)*0x30;
	}
	
	*pAddr = Data;											// 向PLC逻辑中写入数据
}

/*该函数用于计数器运算，计数器作用：计数输入条件满足的次数，当计数次数到时输出有效：要区别于定时器的功能*/

void CntProcess(UINT16 stack,UINT16 Cnt_set,UINT16 Cnt_num,UINT16 Rst)		// 计数器计数条件，计数值，计数器号，复位
{	CtrValue *pCtrValue;									// 定义计数器结构体指针变量
	UINT32 *pCtrValueAndFlag;								// 定义指针变量，用于存放计数器序号的地址

	pCtrValueAndFlag = (UINT32 *)(EEPROM_BUFF + 0x180);		// 计数器存放的相关数据开始地址		 0X180

	pCtrValue = (CtrValue *)(pCtrValueAndFlag + Cnt_num);	// 得到不同计数器号的地址，该地址中对应的输出为PLC的输出状态
	
	/*以下程序用于计数器计数处理*/

	if(Rst == 1) 											// 复位开关开，复位优先级高,此时为复位
	{	pCtrValue->ActionFlag = stack;						// 判断计数器能否计数，当从0变为1时，计数一次
		pCtrValue->CountFlag = 0;							// 计数器是否在工作，0表示未计数，1表示计数
		pCtrValue->Value = Cnt_num;							// 保存计数器的计数值
						
		SetValue(Cnt_num,4,0);								// 设定Cnt_num号计数器的输出为0，该定时器无效
	}
	else													// 复位信号无效
	{	if(stack == 1)										// 相当于输入条件有效
		{	if(pCtrValue->CountFlag == 0)					// 初始状态，如果计数器未处于计数的状态，开始计数
			{	// 开始计数
				pCtrValue->Value = Cnt_set;					// 设定计数器计数数值
				pCtrValue->Value -= 1;						// 计数器减一
				pCtrValue->ActionFlag =1;					// 计数器可以计数
				pCtrValue->CountFlag = 1;					// 计数器处于计数状态
			}
			else											// 计数器处于计数状态，下一个周期仍然计数
			{	// 继续计数
				if(pCtrValue->ActionFlag == 0)				// 计数器继续计数（此处从第二个计数脉冲开始计数）
				{	if(pCtrValue->Value>0)					// 如果计数值不为0，继续计数
					{	pCtrValue->Value -= 1;				// 计数值减一
					}

					pCtrValue->ActionFlag = 1;				// 每计数一次后，将该位置1，为下次计数做准备
					
					if(pCtrValue->Value == 0)				// 计数值为0，表示计数次数到
					{	SetValue(Cnt_num,4,1);				// 计数次数到，PLC输出有效
					}								
				}
				else
				{	pCtrValue->ActionFlag = 1;				// 该操作多余
				}					
			}
		}
		else												// 输入条件断开时，将计数电平置0，为下次计数做准备
		{	pCtrValue->ActionFlag = 0;
		}
	}
}

/*下面函数用于处理PLC定时器操作*/

void TimerProcess(UINT16 stack,UINT16 Timer_set,UINT16 Timer_num)		// 参数说明：定时器判断条件，定时器计数值，定时器号
{	UINT32 *pTimerValueAndFlag ;							// 用于定时器参数内存
	Timer *pTimer ;											// 定义定时器数据指针

	pTimerValueAndFlag = (UINT32 *)(EEPROM_BUFF + 0x100);	// 得到定时器参数存放的地址			 0X100
															// 用于定时器参数内存
	pTimer = (Timer *)(pTimerValueAndFlag + Timer_num);		// 得到不同定时器号对应的地址
	
	/*以下程序用于判断是否进行定时器操作*/
					
	if(stack == 1)											// 如果定时器的输入条件满足，开始定时器操作
	{	
		if(pTimer->Flag == 0)								// 定时器初始化的状态处于关闭状态，定时器条件满足时，进行定时器条件初始化
		{	
			pTimer->Flag = 1;								// 表示开始工作
			pTimer->Value = Timer_set;						// 设定定时器定时时间
			SetValue(Timer_num,5,0);						// 相应定时器输出无效,相当于复位一下定时器
		}
		else												// 定时器处于定时状态，将继续定时
		{	// 继续工作
			pTimer->Value -= 5;								// 定时寄存器内的数值递减，每个周期定时时间为5ms，因此减5
			if(pTimer->Value <= 4)							// 当时间小于一个中断周期时，表示定时时间到
			{
				SetValue(Timer_num,5,1);					// 将相应的定时器置位，PLC输出有效
				pTimer->Flag = 0;							// 表示结束工作
			}
		}				
	}
	else													
	{													
		if(pTimer->Flag == 1)
		{    // 继续工作
			pTimer->Value -= 5;								// 定时寄存器内的数值递减，每个周期定时时间为5ms，因此减5
			if(pTimer->Value <= 4)							// 当时间小于一个中断周期时，表示定时时间到
			{
				SetValue(Timer_num,5,1);					// 将相应的定时器置位，PLC输出有效
				pTimer->Flag = 0;							// 表示结束工作
			}
		}
		else
		{
			pTimer->Value = 0;									// 定时器有效值定时时间为0
			SetValue(Timer_num,5,0);							// 相应定时器输出无效,相当于复位一下定时器
		}
	}

}


/*该函数用于PLC扫描*/

void Proc_plc_code()
{	UINT16 stack1[30],stack2[30];							// 堆栈	30代表最多有30行输出，即LD和OUT分别最多出现30次。
	UINT16 point1=0, point2=0;								// 堆栈指针
	UINT16 bEnd;											// 判断是否扫描结束

	/*以下变量用于中间处理过程*/

	UINT16 Data = 1;
	UINT16 a,b,i;	 

	UINT16 *pNow = (UINT16 *)EEPROM_CODE;					// 用来读取代码，该变量存放PLC代码的初始地址
	
	UINT16 Type;											// 指令类型（连接、与、或、输出等）
	UINT16 RegType;											// 寄存器类型	
	UINT16 *pNum;											// 寄存器的组号

	UINT16 *pTimeSet;										// 定时器的设定值 ,更新后的时间值， 单位ms

	UINT16 *pCntSet;										// 计数器设定值

	bEnd = 1;												// 结尾判断指令赋1，用于程序扫描

	/*下段程序用于PLC梯形图的扫描，末尾变量有效*/
	while(bEnd)												// 如果末尾标志为1，PLC循环扫描										
	{	pNum = (UINT16 *)pNow;								// 保存PLC代码的首地址，该地址中存放寄存器的序号
		pNow++;										// 计算PLC代码的下一个地址，如果梯形图的指令为LD M023，发下来的指令顺序为023,LD　M，如果是计数器和定时器，还有下个地址存放次数和时间

		Type = (*pNow)&0xff00;                              //获得指令类型（LD,AND,OR等）								
		RegType = (*pNow)&0x00ff;							// 获得寄存器类型（X，Y，M等）
		pNow++;												// 地址增加
		/*以下程序用于判断PLC指令类型*/
		switch(Type)
		{	case NTO:
				point1 = 0;
				point2 = 0;
				break;

			case NOOP: 										// 空指令
				point1 = 0; 								// 堆栈指针
				point2 = 0;									// 堆栈指针
				break;

			case END:										// 结束指令
				bEnd = 0;
				break;

			case LD:										// 如果是连接常开触点
				Data = GetValue(*pNum,RegType);				// 获得相应寄存器序号中的值
				// 入栈
				stack1[point1] = Data;						// 将该寄存器中的值入栈
				point1++;									// 栈增加，用于保存下一个寄存器的值
				break;

			case LDI:										// 如果连接的是常闭触点	
				Data = GetValue(*pNum,RegType);				// 获得相应寄存器的值
				if(Data == 1)								// 常闭触点Data取反
				{	Data = 0;	}
				else
				{	Data = 1;	}
				stack1[point1] = Data;						// 将该寄存器的值入栈保存
				point1++;									// 栈指针增加，用于保存下一个寄存器的值
				break;

			case AND:										// 指令类型为与常开触点取与指令	
				Data = GetValue(*pNum,RegType);				// 得到相应寄存器的值
				stack1[point1-1] = stack1[point1-1] & Data;	// 与前一个寄存器的值取与，得到两个寄存器相与的结果
				break;

			case ANI:										// 指令类型为与常闭触点取与	
				Data = GetValue(*pNum,RegType);				// 获得该寄存器的值
				if(Data == 1)								// Data取反
				{	Data = 0;	}
				else
				{	Data = 1;	}							
				stack1[point1-1] = stack1[point1-1] & Data;	// 前后两个寄存器值取与
				break;

			case OR:										// 指令类型为与常开触点取或	
				Data = GetValue(*pNum,RegType);
				// 修改栈值
				stack1[point1-1] = stack1[point1-1] | Data;
				break;

			case ORI:										// 指令类型为与常闭触点取或	
				Data = GetValue(*pNum,RegType);
				if(Data == 1)		// Data取反
				{	Data = 0;	}
				else
				{	Data = 1;	}
				// 修改栈值
				stack1[point1-1] = stack1[point1-1] | Data;
				break;

			case OUT:										// 指令类型为常开触点输出
 				Data = stack1[point1-1];					// 输出结果为前面一系列寄存器逻辑的结果
				
				/*下面程序根据输出寄存器的类型进行输出处理*/

				if( (RegType==0x02)||(RegType==0x03) )		// 输出寄存器为Y或M 
					SetValue(*pNum,RegType,Data);			// 确定输出值
				else if( RegType==0x04 )					// 如果输出寄存器为计数器
				{	pCntSet = (UINT16 *)pNow;				// 获得计数值			
					//pNow = pNow+2;						// 将地址后移
					pNow++;
					CntProcess(Data,*pCntSet,*pNum,0);		// 计数器操作			
				}
				else if( RegType==0x05 )					// 如果寄存器类型为定时器
				{	
					pTimeSet = (UINT16 *)pNow;				// 得到定时时间	
					pNow++;
					TimerProcess(Data,*pTimeSet,*pNum);		// 定时器操作
				}
				break;

			case OUTI:										// 	指令类型为输出常闭触点
				Data = stack1[point1-1];					// Data = stack1[point1];
				if(Data == 1)								// Data取反
				{	Data = 0;	}
				else
				{	Data = 1;	}

				if( (RegType==0x02)||(RegType==0x03) )		// Y or M 
					SetValue(*pNum,RegType,Data);
				else if( RegType==0x04 )					// C
				{	pCntSet = (UINT16 *)pNow;				//定时设定值
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

			case SET:										// 指令类型为置位指令		
				Data = stack1[point1-1];					// 获得前面一系列的逻辑的结果
				if(Data == 1)								// 根据前面的逻辑结果确定输出，如果满足逻辑条件，置位
				{	SetValue(*pNum,RegType,1);
				}
				break;

			case RST:										// 指令类型为复位	
				Data = stack1[point1-1];
				if(Data == 1)								// 逻辑条件有效
				{	if( (RegType==0x02)||(RegType==0x03) )	// Y or M 
						SetValue(*pNum,RegType,0);			// 输出结果为0
					else if( RegType==0x04 )				// C
						CntProcess(Data,*pCntSet,*pNum,1);	// 计数器复位，输出无效		
					else if( RegType==0x05 )				// T
						TimerProcess(0,*pTimeSet,*pNum);	// 定时器停止工作
				}
				break;

			case ANB:										// 将两个逻辑块取与（参考梯形图的回程逻辑前两个或单元后相与）
				a = stack1[point1-1];						// 得到当前逻辑块单元的值
				b = stack1[point1-2];						// 得到前一个逻辑块单元的值					
				point1--;
				Data = a & b;								// 得出两个逻辑块单元相与的结果
				stack1[point1-1] = Data;					// 保存相与的结果
				break;

			case ORB:		                               // 将两个逻辑块单元取或
				a = stack1[point1-1];
				b = stack1[point1-2];
				point1--;
				Data = a | b;
				stack1[point1-1] = Data;
				break;

			case MPS:										// 参数个数为0,将1栈顶元素放入栈2
				Data = stack1[point1-1];
				stack2[point2] = Data;
				point2++;
				break;

			case MRD:										// 参数个数为0,将2栈顶元素放入1栈
				Data = stack2[point2-1];
				stack1[point1-1] = Data;
				break;

			case MPP:		// 参数个数为0,将2栈顶元素放入1栈,2栈出栈
				Data = stack2[point2-1];
				stack1[point1-1] = Data;
				point2--;
				break;
		}	
	}	
}






/*********************************** endfile **********************************/

