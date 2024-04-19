#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count; 
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;
extern int Time_count;
/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
�������ܣ���ȡ��ص�ѹ�������������������Լ졢��APP�������ݡ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
int Buzzer_count=25;
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
   {	
		int i=0;
		static int LowVoltage_1=0, LowVoltage_2=0;
		static int Servo_adjust_timecount;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //��������10Hz��Ƶ������
		
		//����ʱ���������ݷ�������������
		//The buzzer will beep briefly when the machine is switched on
		if(Time_count<50)Buzzer=1; 
		else if(Time_count>=51 && Time_count<100)Buzzer=0;
		 
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
		if(Buzzer_count<5)Buzzer_count++;
		if(Buzzer_count<5)Buzzer=1; //The buzzer is buzzing //����������
		else if(Buzzer_count==5)Buzzer=0;
		
		if(Beep_Control_Flag)
		{
			Buzzer = 1;
			Servo_adjust_timecount++;
			if(Servo_adjust_timecount == 10)
			{
				Beep_Control_Flag = 0;
				Buzzer = 0;
				Servo_adjust_timecount = 0;
				OLED_Refresh_Gram(); //ˢ����Ļ
        OLED_Clear();  //�����Ļ
		    OLED_Refresh_Gram();
			}
		}
		 
		//Read the battery voltage //��ȡ��ص�ѹ
		for(i=0;i<10;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/10;
		Voltage_All=0;
		
		if(LowVoltage_1==1)LowVoltage_1++; //Make sure the buzzer only rings for 0.5 seconds //ȷ��������ֻ��0.5��
		if(LowVoltage_2==1)LowVoltage_2++; //Make sure the buzzer only rings for 0.5 seconds //ȷ��������ֻ��0.5��
		if(Voltage>=12.6f)Voltage=12.6f;
		else if(10<=Voltage && Voltage<10.5f && LowVoltage_1<2)LowVoltage_1++; //10.5V, first buzzer when low battery //10.5V���͵���ʱ��������һ�α���
		else if(Voltage<10 && LowVoltage_1<2)LowVoltage_2++; //10V, when the car is not allowed to control, the buzzer will alarm the second time //10V��С����ֹ����ʱ�������ڶ��α���
					
		APP_Show();	 //Send data to the APP //��APP��������
	  oled_show(); //Tasks are displayed on the screen //��ʾ����ʾ����

   }
}  

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
�������ܣ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{  
   static int count=0;	 
	 int Car_Mode_Show;
	
			Divisor_Mode=2048/CAR_NUMBER+2;
		 Car_Mode_Show=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); 
		 if(Car_Mode_Show>2)Car_Mode_Show=2;
		 Voltage_Show=Voltage*100; 
		 count++;
	
	 //Collect the tap information of the potentiometer, 
	 //and display the car model to be fitted when the car starts up in real time
	 //�ɼ���λ����λ��Ϣ��ʵʱ��ʾС������ʱҪ�����С���ͺ�
	if(Servo_init_angle_adjust == 0)//�������ƣ�������΢��ģʽ
	{

		
		 //if(Check==0)//The car displays normally when the self-check mode is not enabled //û�п����Լ�ģʽʱС��������ʾ
		 {	
			 //The first line of the display displays the content//
			 //��ʾ����1����ʾ����//
			 switch(Car_Mode_Show)
			 {
				case Mec_Car:       OLED_ShowString(0,0,"Mec "); break; 
				case FourWheel_Car: OLED_ShowString(0,0,"4WD "); break; 
				case Tank_Car:      OLED_ShowString(0,0,"Tank"); break; 
			 }
			 
			 if(Car_Mode==Mec_Car)
			 {
				 //The Mec_car and omni_car show Z-axis angular velocity
				 //���֡�ȫ����С����ʾZ����ٶ�
				 OLED_ShowString(55,0,"GZ");
				 if( gyro[2]<0)  OLED_ShowString(80,0,"-"),OLED_ShowNumber(90,0,-gyro[2],5,12);
				 else            OLED_ShowString(80,0,"+"),OLED_ShowNumber(90,0, gyro[2],5,12);		
			 }
			 else if(Car_Mode==FourWheel_Car||Car_Mode==Tank_Car)
			 {
				 //Akm_Car, Diff_Car, FourWheel_Car and Tank_Car Displays gyroscope zero
				 //�����������١��������Ĵ�����ʾ���������
				 OLED_ShowString(55,0,"BIAS");
				 if( Deviation_gyro[2]<0)  OLED_ShowString(90,0,"-"),OLED_ShowNumber(100,0,-Deviation_gyro[2],3,12);  //Zero-drift data of gyroscope Z axis
				 else                      OLED_ShowString(90,0,"+"),OLED_ShowNumber(100,0, Deviation_gyro[2],3,12);	//������z�����Ư������	
			 }
			 //The first line of the display displays the content//
			 //��ʾ����1����ʾ����//
			 

			 //The second line of the display displays the content//
			 //��ʾ����2����ʾ����//
			 if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
			 {
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor A
				//���֡�ȫ���֡���������ʾ���A��Ŀ���ٶȺ͵�ǰʵ���ٶ�
				OLED_ShowString(0,10,"A");
				if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
															OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
				else                 	OLED_ShowString(15,10,"+"),
															OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 
				
				if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
															OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,10,"+"),
															OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);

			 }
			 else if(Car_Mode==Tank_Car)
			 {
				 //The Akm_Car, Diff_Car and Tank_Car show Z-axis angular velocity
				 //�����������١�̹��С����ʾZ����ٶ�
				 OLED_ShowString(00,10,"GYRO_Z:");
				 if( gyro[2]<0)  OLED_ShowString(60,10,"-"),
												 OLED_ShowNumber(75,10,-gyro[2],5,12);
				 else            OLED_ShowString(60,10,"+"),
												 OLED_ShowNumber(75,10, gyro[2],5,12);			
			 }	 
			 //The second line of the display displays the content//
			 //��ʾ����2����ʾ����//
			 
			 //Lines 3 and 4 of the display screen display content//
			 //��ʾ����3��4����ʾ����//
			 if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
			 {
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor B
				//���֡�ȫ���֡���������ʾ���B��Ŀ���ٶȺ͵�ǰʵ���ٶ�
				OLED_ShowString(0,20,"B");		
				if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
															OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
				else                 	OLED_ShowString(15,20,"+"),
															OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
				
				if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
															OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,20,"+"),
															OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor C
				//���֡�ȫ���֡���������ʾ���C��Ŀ���ٶȺ͵�ǰʵ���ٶ�
				OLED_ShowString(0,30,"C");
				if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
															OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
				else                 	OLED_ShowString(15,30,"+"),
															OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12); 
					
				if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
															OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,30,"+"),
															OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
			 }
			 else if(Car_Mode==Tank_Car)
			 {
				 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor A
				 //�����������١��Ĵ�����ʾ���A��Ŀ���ٶȺ͵�ǰʵ���ٶ�
				 OLED_ShowString(0,20,"L:");
				 if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
																OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
				 else                 	OLED_ShowString(15,20,"+"),
																OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12);  
				 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
																OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
				 else                 	OLED_ShowString(60,20,"+"),
																OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);
				 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor B
				 //�����������١��Ĵ�����ʾ���B��Ŀ���ٶȺ͵�ǰʵ���ٶ�
				 OLED_ShowString(0,30,"R:");
				 if( MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
																OLED_ShowNumber(20,30,-MOTOR_B.Target*1000,5,12);
				 else                 	OLED_ShowString(15,30,"+"),
																OLED_ShowNumber(20,30,  MOTOR_B.Target*1000,5,12);  
					
				 if( MOTOR_B.Encoder<0)	OLED_ShowString(60,30,"-"),
																OLED_ShowNumber(75,30,-MOTOR_B.Encoder*1000,5,12);
				 else                 	OLED_ShowString(60,30,"+"),
																OLED_ShowNumber(75,30, MOTOR_B.Encoder*1000,5,12);

			 }
			 //Lines 3 and 4 of the display screen display content//
			 //��ʾ����3��4����ʾ����//
			 
			 //Line 5 of the display displays the content//
			 //��ʾ����5����ʾ����//
			 if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
			 {
					//Mec_Car Display the target speed and current actual speed of motor D
					//����С����ʾ���D��Ŀ���ٶȺ͵�ǰʵ���ٶ�
					OLED_ShowString(0,40,"D");
					if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
																OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
					else                 	OLED_ShowString(15,40,"+"),
																OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12); 			
					if( MOTOR_D.Encoder<0)	OLED_ShowString(60,40,"-"),
																OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
					else                 	OLED_ShowString(60,40,"+"),
																OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);
			 }
			 
			
			 else if(Car_Mode==Tank_Car)
			 {
				 // The Diff_Car and Tank_Car displays the PWM values of the left and right motors
				 //����С�����Ĵ�����ʾ���ҵ����PWM����ֵ
																 OLED_ShowString(00,40,"MA");
				 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(20,40,"-"),
																 OLED_ShowNumber(30,40,-MOTOR_A.Motor_Pwm,4,12);
				 else                 	 OLED_ShowString(20,40,"+"),
																 OLED_ShowNumber(30,40, MOTOR_A.Motor_Pwm,4,12); 
																 OLED_ShowString(60,40,"MB");
				 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(80,40,"-"),
																 OLED_ShowNumber(90,40,-MOTOR_B.Motor_Pwm,4,12);
				 else                 	 OLED_ShowString(80,40,"+"),
																 OLED_ShowNumber(90,40, MOTOR_B.Motor_Pwm,4,12);
			 }
			 //Line 5 of the display displays the content//
			 //��ʾ����5����ʾ����//
				 
			 //Displays the current control mode //��ʾ��ǰ����ģʽ
			 if(PS2_ON_Flag==1)         OLED_ShowString(0,50,"PS2  ");
			 else if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP  ");
			 else if (Remote_ON_Flag==1)OLED_ShowString(0,50,"R-C  ");
			 else if (CAN_ON_Flag==1)   OLED_ShowString(0,50,"CAN  ");
			 else if (Usart1_ON_Flag==1) OLED_ShowString(0,50,"UART1");
			 else if (Usart5_ON_Flag==1) OLED_ShowString(0,50,"UART5");
			 else                       OLED_ShowString(0,50,"ROS  ");
				
			 //Displays whether controls are allowed in the current car
			 //��ʾ��ǰС���Ƿ��������
			 if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N");  
			 else                      OLED_ShowString(45,50,"OFF"); 
				
																	OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
																	OLED_ShowString(88,50,".");
																	OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
																	OLED_ShowString(110,50,"V");
			 if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);
			}
		 
//			/* self-check related */
//			/*�Լ����*/
//			//Display the self-check confirmation screen //��ʾ�Լ�ȷ�Ͻ���
//			if(Check==1&&!Checking&&Checked==0) 
//			{
//					OLED_ShowCheckConfirming();
//			}
//			//Display the interface for self-testing //��ʾ�Լ���н���
//			if(Check==1&&Checking&&Checked==0) 
//			{	
//					OLED_ShowChecking();
//			}	
//			//Show the results of self-test //��ʾ�Լ���
//			if(Check==1&&Checking&&Checked==1) 
//			{		
//				OLED_ShowCheckResult();
//			}
//			/* self-check related */
//			/*�Լ����*/
			
//			//Refresh the screen //ˢ����Ļ
//			if(Check==0)OLED_Refresh_Gram();
//			//The screen refresh rate in self-check mode is reduced by 10 times
//			//�Լ�ģʽ����Ļˢ��Ƶ�ʽ���10��
//			else {if(count>10)OLED_Refresh_Gram(),count=0;}	
	}
	else //΢��ģʽ��ʾ�ĸ������ƫ��ֵ 
	{
				//=============��һ��=======================//	
		OLED_ShowString(0,0,"Angle1_init:");
		if( Moveit_Angle1_init<0)	
		{
			OLED_ShowString(100,0,"-");
			OLED_ShowNumber(110,0, -Moveit_Angle1_init,3,12);
		}
		else
		{
			OLED_ShowString(100,00,"+");
			OLED_ShowNumber(110,0, Moveit_Angle1_init,3,12);
		}
						//=============�ڶ���=======================//	
		OLED_ShowString(0,10,"Angle2_init:");
		if( Moveit_Angle2_init<0)	
		{
			OLED_ShowString(100,10,"-");
			OLED_ShowNumber(110,10, -Moveit_Angle2_init,3,12);
		}
		else
		{
			OLED_ShowString(100,10,"+");
			OLED_ShowNumber(110,10, Moveit_Angle2_init,3,12);
		}
				//=============������=======================//	
		OLED_ShowString(0,20,"Angle3_init:");
		if( Moveit_Angle3_init<0)	
		{
			OLED_ShowString(100,20,"-");
			OLED_ShowNumber(110,20, -Moveit_Angle3_init,3,12);
		}
		else
		{
			OLED_ShowString(100,20,"+");
			OLED_ShowNumber(110,20, Moveit_Angle3_init,3,12);
		}
						//=============������=======================//	
		OLED_ShowString(0,30,"Angle4_init:");
		if( Moveit_Angle4_init<0)	
		{
			OLED_ShowString(100,30,"-");
			OLED_ShowNumber(110,30, -Moveit_Angle4_init,3,12);
		}
		else
		{
			OLED_ShowString(100,30,"+");
			OLED_ShowNumber(110,30, Moveit_Angle4_init,3,12);
		}

	}
			//=============ˢ��=======================//
		OLED_Refresh_Gram();	
} 
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
�������ܣ���APP��������
��ڲ�������
����  ֵ����
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //�Ե�ص�ѹ����ɰٷֱ���ʽ
	 Voltage_Show=(Voltage*1000-10000)/27;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //�����ٶȵ�λת��Ϊ0.01m/s��������APP��ʾ
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //���ڽ����ӡAPP���ݺ���ʾ����
	 flag_show=!flag_show;
	
	 if(PID_Send==1) 
	 {
		 //Send parameters to the APP, the APP is displayed in the debug screen
		 //���Ͳ�����APP��APP�ڵ��Խ�����ʾ
		 		printf("{C%d:%d:%d:%d:%d:%d:%d}$",
		  (int)((Moveit_Angle1+1.57f)*100), //+1.57����Ϊ���ν��治������ʾ������*100����Ϊ���ν��治��ʵ�������ĵ�������
			(int)((Moveit_Angle2+1.57f)*100),
			(int)((Moveit_Angle3+1.57f)*100),
			(int)((Moveit_Angle4+1.57f)*100),
		  (int)RC_Velocity,
			(int)Velocity_KP,
			(int)Velocity_KI);
		  PID_Send=0;	
	 }	
	 else	if(flag_show==0) 
	 {
		 //Send parameters to the APP and the APP will be displayed on the front page
		 //���Ͳ�����APP��APP����ҳ��ʾ
		 printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)gyro[2]);
	 }
	 else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //���Ͳ�����APP��APP�ڲ��ν�����ʾ
		 printf("{B%d:%d:%d}$",(int)gyro[0],(int)gyro[1],(int)gyro[2]);
	 }
}


