#include "stm32f0xx.h"
#include "config.h"
#include "setup.h"
#include "modbus-rtu.h"

static uint32_t link_cnt;
void Set_link_cnt(uint32_t n){
	link_cnt = n;
}

static uint8_t flag_i;
void Triggers_Handler(){			//���������� �������� �������� � ������ ������� ����������� ������
	
	if( ext_cmd_reg & EXT_CMD_MOTOR_START ) TriggersVariables_CurStates |= (1 << 0); //���������� � ��������� ����� EXT_CMD_MOTOR_START � �������� ext_cmd_reg
	else TriggersVariables_CurStates &= ~(1 << 0);
	
	if( ext_flags_reg & EXT_FL_EXPERIMENT_DONE ) TriggersVariables_CurStates |= (1 << 1); //���������� � ��������� ����� EXT_FL_EXPERIMENT_DONE � �������� ext_flags_reg
	else TriggersVariables_CurStates &= ~(1 << 1);
	
	if( RotationSensor_State() ) TriggersVariables_CurStates |= (1 << 2); //���������� � ��������� ������� � ������� ��������
	else TriggersVariables_CurStates &= ~(1 << 2);
	
	for( flag_i = 0; flag_i < 3; flag_i++ ){
		if( TriggersVariables_CurStates & (1 << flag_i) ){					//���� ���� ������
			
			Triggers_Reg &= ~(1 << ((2 * flag_i) + 1) );		//���������� ������� ������� ������ �����
			
			if( !( TriggersVariables_PrevStates & (1 << flag_i) ) )			//���� �� ���������� �������� ���� ��� �������
				Triggers_Reg |= (1 << (2 * flag_i) );			//������� ������� ��������� ������ �����
			
		}
		else{													//���� ���� �������
			
			Triggers_Reg &= ~(1 << (2 * flag_i) );			//���������� ������� ��������� ������ �����
			
			if( TriggersVariables_PrevStates & (1 << flag_i) )				//���� �� ���������� �������� ���� ��� ������
				Triggers_Reg |= (1 << ((2 * flag_i) + 1) );	//������� ������� ������� ������ �����
				
		}
	}
	
	TriggersVariables_PrevStates = TriggersVariables_CurStates; //����������� ������� ��������� �������� ������ � ������ ��� ������������� �� ��������� ��������
}

static uint16_t speed_reg_data;
void SetDriveSpeed(uint8_t speed){	//��������� ����������� ������� ������� ��� ��
	speed_reg_data = (uint16_t)( (1000/140) * speed );
	if (speed_reg_data && (speed_reg_data <= 1000)) AO0_Register = speed_reg_data;
}

static volatile uint16_t ADC_Data[3];
static volatile float k[3];
static volatile uint8_t kf[3] = { 2, 25, 25};

void ADC_GetData(void){				//��������� ��������� ������ � ���
uint8_t ai_ch;
	
	for(ai_ch = 0; ai_ch < 3; ai_ch++){
		ADC1->CHSELR = (1 << ai_ch);
		while (!(ADC1->ISR & ADC_ISR_ADRDY));
		ADC1->CR |= ADC_CR_ADSTART;
		while (!(ADC1->ISR & ADC_ISR_EOC));
		ADC_Data[ai_ch] = ADC1->DR;
		ADC1->CR |= ADC_CR_ADSTP;
	}
}

static uint32_t t_cnt;
void SysTick_Handler(void){			//���������� ������
	if (++t_cnt >= 200){ //1000
		t_cnt = 0;
		ext_sec_cnt++;
	}
	
	if (link_cnt) { Set_link_led(); link_cnt--; } else { Clr_link_led(); }
	
	Triggers_Handler();
	
	ADC_GetData();
  
}

static uint8_t ai;
int main(void){
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	IO_Config();
	Modbus_Config(57600);
	Modbus_set_addr(1);
	
	while(1)
	{
		/*****************���������� ���������������******************/
		/*������� �������� �������������� (HMI->PLC)*/
		if (ext_adjusted_speed_reg && (ext_adjusted_speed_reg <= 140)) 
			SetDriveSpeed(ext_adjusted_speed_reg);
		
		/*�����, ������������ �� ������ ��������� (PLC->HMI)*/
		if(flags & INT_FL_DRIVE_ERROR) ext_flags_reg |= EXT_FL_DRIVE_ERROR; //���� ������ ��
		else ext_flags_reg &= ~EXT_FL_DRIVE_ERROR;
		
		if(MotorBusy_State()) ext_flags_reg |= EXT_FL_DRIVE_RUN;			//���� ��������� ��
		else ext_flags_reg &= ~EXT_FL_DRIVE_RUN;
		
		/*�������� ������� ������ ��������������*/
		switch (Drive_State){
			
			case DRIVE_STOP:{	//��������� ��������������
				
				if(!(flags & INT_FL_FST_IMPLEMENTATION)){	//���� ���� ������ �������� � ���� ���������
					flags |= INT_FL_FST_IMPLEMENTATION;
					
					//���������� �������� ��� ����� � ��������� DRIVE_STOP
					Motor_Stop();
				}
				else{
					//��������� ��������� � ������ ��������� �� ��������
					if( !FreqConvNotBroken_State() || !SafeSensor_State() ){	//���� �������� ������ �� ��� ����� ����� �������
						Drive_State = DRIVE_ERROR;					//��������� � ��������� ������
						flags &= ~INT_FL_FST_IMPLEMENTATION;		//���������� ���� ������ �������� ��� ������������� � ��������� ���������
					}
					else{
						if (Triggers_Reg & CMD_MOT_START_RTRIG){	//���� � ������ ������ ������� � �������
							Triggers_Reg &= ~CMD_MOT_START_RTRIG;	//����� �������� ������� �������
							
							Drive_State = DRIVE_RUN;				//��������� � ��������� ������ ��.�������
							flags &= ~INT_FL_FST_IMPLEMENTATION;
						}
					}
				}
				
			} break;
			
			case DRIVE_RUN:{	//������ ��������������
				
				if(!(flags & INT_FL_FST_IMPLEMENTATION)){
					flags |= INT_FL_FST_IMPLEMENTATION;
					
					if( FreqConvNotBroken_State() && SafeSensor_State() ) //���� �� �������� � �������� ����� �������
						Motor_Start();
					else{ //����� ��������� � ��������� ��������� ��� ������������ �������� � ������
						Drive_State = DRIVE_STOP;
						flags &= ~INT_FL_FST_IMPLEMENTATION;
					}
				}
				else{
					if( (Triggers_Reg & CMD_MOT_START_FTRIG) || (Triggers_Reg & EXPRMNT_DONE_RTRIG) ||
						!FreqConvNotBroken_State() || !SafeSensor_State() ||
						(ext_flags_reg & EXT_FL_TENSION1_ERROR) || 
						(ext_flags_reg & EXT_FL_TENSION2_ERROR) || 
						(ext_flags_reg & EXT_FL_TENSION3_ERROR) ){ //���� ������ ������ ����� � ������ ����� ��.������� ��� ���������� ����������� ��� ��������� ������
						if( Triggers_Reg & CMD_MOT_START_FTRIG ) Triggers_Reg &= ~CMD_MOT_START_FTRIG;
						if( Triggers_Reg & EXPRMNT_DONE_RTRIG ) Triggers_Reg &= ~EXPRMNT_DONE_RTRIG;
						
						Drive_State = DRIVE_STOP;
						flags &= ~INT_FL_FST_IMPLEMENTATION;
					}
				}
				
			} break;
			
			case DRIVE_ERROR:{	//������ ��������������
				
				if(!(flags & INT_FL_FST_IMPLEMENTATION)){
					flags |= INT_FL_FST_IMPLEMENTATION;
				
					flags |= INT_FL_DRIVE_ERROR;
				}
				else{
					if( FreqConvNotBroken_State() && SafeSensor_State() ){
						flags &= ~INT_FL_DRIVE_ERROR;
						Drive_State = DRIVE_STOP;
						flags &= ~INT_FL_FST_IMPLEMENTATION;
					}
				}
				
			} break;
		}
		
		
		/*****************���� ���������� ������**********************/
		/*�������� � ������ ���������� ���������� ������ (PLC->HMI)*/
		if (ext_cmd_reg & EXT_CMD_START_EXPERIMENT)
		{
    adjusted_cycles	= ( (ext_adjusted_cycles_reg_hi << 16) | ext_adjusted_cycles_reg_low);	
		remaining_cycles = adjusted_cycles;
		}
		
		ext_remaining_cycles_reg_low = remaining_cycles & 0x0000FFFF;
		ext_remaining_cycles_reg_hi = (remaining_cycles >> 16)& 0x0000FFFF;
		
		current_cycles = adjusted_cycles - remaining_cycles;
		
		ext_current_cycles_reg_low = current_cycles & 0x0000FFFF;
		ext_current_cycles_reg_hi = (current_cycles >> 16) & 0x0000FFFF;
		
		// remaining_cycles = ext_adjusted_cycles_reg * 100; oleg
		// ext_remaining_cycles_reg = remaining_cycles / 100; oleg
		
		/*������������� ���������� ������ �� ������ ��������� ������ � ������� ��������*/
		if( Triggers_Reg & RotationSensor_RTRIG ){
			Triggers_Reg &= ~RotationSensor_RTRIG;
			
			if(remaining_cycles){
				remaining_cycles--;
				if(!remaining_cycles) ext_flags_reg |= EXT_FL_EXPERIMENT_DONE;
			}
		}
		if(remaining_cycles) ext_flags_reg &= ~EXT_FL_EXPERIMENT_DONE;
		
		
		/*****************�������� ��������� ��������*****************/
		//��������� ������ � ��� � �������� � ������ ��������� (PLC->HMI)
		ai = 0;
		for(ai = 0; ai < 3; ai++)
		{
			k[ai] = ( ( (float)ADC_Data[ai] * 3.3 * 3.68 ) / 4095) * kf[ai]; // * 2;
			// ������� ������������, ������� �� ����� ���
			// k[ai] = (0.000119903304693655 * k[ai]*k[ai]*k[ai]*k[ai]*k[ai]) - (0.00483746523142519 * k[ai]*k[ai]*k[ai]*k[ai]) + (0.0752636454138891 * k[ai]*k[ai]*k[ai]) - (0.538265104383006 * k[ai]*k[ai]) + (5.53516423999886 * k[ai]) - 4.95686229155284;}
		}
		
		ext_sample_tension1_reg_hi =  k[0];
		ext_sample_tension1_reg_low = ((uint32_t)(k[0]*100))%100;
		
		ext_sample_tension2_reg_hi = k[1];
		ext_sample_tension2_reg_low = ((uint32_t)(k[1]*100))%100;
		
		ext_sample_tension3_reg_hi = k[2]; 
		ext_sample_tension3_reg_low = ((uint32_t)(k[2]*100))%100;
		
		
		if( ext_cmd_reg  & EXT_CMD_TENSION1_CTRL_ON ){
			if( (ext_sample_tension1_reg_hi < ext_adjusted_low_tension_limit1) || 
				(ext_sample_tension1_reg_hi > ext_adjusted_up_tension_limit1) ){
					ext_flags_reg |= EXT_FL_TENSION1_ERROR;
				}
		}
		else ext_flags_reg &= ~EXT_FL_TENSION1_ERROR;
		
		if( ext_cmd_reg  & EXT_CMD_TENSION2_CTRL_ON ){
			if( (ext_sample_tension2_reg_hi < ext_adjusted_low_tension_limit2) || 
				(ext_sample_tension2_reg_hi > ext_adjusted_up_tension_limit2) ){
					ext_flags_reg |= EXT_FL_TENSION2_ERROR;
				}
		}
		else ext_flags_reg &= ~EXT_FL_TENSION2_ERROR;
		
		if( ext_cmd_reg  & EXT_CMD_TENSION3_CTRL_ON ){
			if( (ext_sample_tension3_reg_hi < ext_adjusted_low_tension_limit3) || 
				(ext_sample_tension3_reg_hi > ext_adjusted_up_tension_limit3) ){
					ext_flags_reg |= EXT_FL_TENSION3_ERROR;
				}
		}
		else ext_flags_reg &= ~EXT_FL_TENSION3_ERROR;
		
		/*****************��������� ������������ PLC<->HMI************/
		do_modbus(); //���������� ������������ ����� HMI � PLC
	}
}
