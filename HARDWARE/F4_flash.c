#include "F4_flash.h"

#define FLASH_SAVE_ADDR  0x08040000 	//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)
u16 Flash_Parameter[10];

float Position1=SERVO_INIT,Position2=SERVO_INIT,Position3=SERVO_INIT,Position4=SERVO_INIT; 
float Velocity1=0,Velocity2=0,Velocity3=0,Velocity4=0;     //���PWM����
float	Position_KP=2,Position_KI=0,Position_KD=1;  //λ�ÿ���PID���� 
short Moveit_Angle1_init=0,Moveit_Angle2_init=0,Moveit_Angle3_init=0,Moveit_Angle4_init=0; //�����ʼλ��ֵ΢��
int Servo_init_angle_adjust=0;//��������־λ

//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}  
//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
//pBuffer:����ָ��
//NumToWrite:����(16λ)��(����Ҫд���16λ���ݵĸ���.) 
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%2)return;	//�Ƿ���ַ
	FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);		//FLASH�����ڼ�,�����ֹ���ݻ���
 		
	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*2;	//д��Ľ�����ַ
	if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadHalfWord(addrx)!=0XFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V֮��!!
				if(status!=FLASH_COMPLETE)break;	//����������
			}else addrx+=2;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//д����
		{
			if(FLASH_ProgramHalfWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=2;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����
} 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(4λ)��
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
		ReadAddr+=2;//ƫ��2���ֽ�.	
	}
}


/**************************************************************************
�������ܣ���Flash��ȡָ������
��ڲ�������
����  ֵ����
**************************************************************************/
void Flash_Read(void)
{
	STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)Flash_Parameter,10);
	if(Flash_Parameter[0]==65535&&Flash_Parameter[1]==65535&&Flash_Parameter[2]==65535&&Flash_Parameter[3]==65535)
	{
    Moveit_Angle1_init=0,Moveit_Angle2_init=0,Moveit_Angle3_init=0,Moveit_Angle4_init=0;
	}
  else 
	{		
		Moveit_Angle1_init=Flash_Parameter[0];	//��ȡ�����е��΢���ĳ�ʼλ��
		Moveit_Angle2_init=Flash_Parameter[1];	
		Moveit_Angle3_init=Flash_Parameter[2];	
		Moveit_Angle4_init=Flash_Parameter[3];	
	
	}
}	
/**************************************************************************
�������ܣ���Flashд��ָ������
��ڲ�������
����  ֵ����
**************************************************************************/
void Flash_Write(void)
{
	Flash_Parameter[0]=(u16)Moveit_Angle1_init;	//�洢
	Flash_Parameter[1]=(u16)Moveit_Angle2_init;	//�洢	
	Flash_Parameter[2]=(u16)Moveit_Angle3_init;	//�洢	
	Flash_Parameter[3]=(u16)Moveit_Angle4_init;	//�洢		
	STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)Flash_Parameter,10);	
}	























