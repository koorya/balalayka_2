#include "stm32f0xx.h"
#include "config.h"
#include "setup.h"
#include "modbus-rtu.h"

static uint32_t link_cnt;
void Set_link_cnt(uint32_t n){
	link_cnt = n;
}

static uint8_t flag_i;
void Triggers_Handler(){			//Обработчик тригеров передних и задних фронтов добавленных флагов
	
	if( ext_cmd_reg & EXT_CMD_MOTOR_START ) TriggersVariables_CurStates |= (1 << 0); //Добавление в обработку флага EXT_CMD_MOTOR_START в регистре ext_cmd_reg
	else TriggersVariables_CurStates &= ~(1 << 0);
	
	if( ext_flags_reg & EXT_FL_EXPERIMENT_DONE ) TriggersVariables_CurStates |= (1 << 1); //Добавление в обработку флага EXT_FL_EXPERIMENT_DONE в регистре ext_flags_reg
	else TriggersVariables_CurStates &= ~(1 << 1);
	
	if( RotationSensor_State() ) TriggersVariables_CurStates |= (1 << 2); //Добавление в обработку сигнала с датчика оборотов
	else TriggersVariables_CurStates &= ~(1 << 2);
	
	for( flag_i = 0; flag_i < 3; flag_i++ ){
		if( TriggersVariables_CurStates & (1 << flag_i) ){					//Если флаг поднят
			
			Triggers_Reg &= ~(1 << ((2 * flag_i) + 1) );		//Сбрасываем триггер заднего фронта флага
			
			if( !( TriggersVariables_PrevStates & (1 << flag_i) ) )			//Если на предыдущей итерации флаг был сброшен
				Triggers_Reg |= (1 << (2 * flag_i) );			//Взводим триггер переднего фронта флага
			
		}
		else{													//Если флаг сброшен
			
			Triggers_Reg &= ~(1 << (2 * flag_i) );			//Сбрасываем триггер переднего фронта флага
			
			if( TriggersVariables_PrevStates & (1 << flag_i) )				//Если на предыдущей итерации порт был поднят
				Triggers_Reg |= (1 << ((2 * flag_i) + 1) );	//Взводим триггер заднего фронта флага
				
		}
	}
	
	TriggersVariables_PrevStates = TriggersVariables_CurStates; //Присваиваем текущее состояние регистра флагов в память для использования на следующей итерации
}

static uint16_t speed_reg_data;
void SetDriveSpeed(uint8_t speed){	//Процедура выставления опорной частоты для ПЧ
	speed_reg_data = (uint16_t)( (1000/140) * speed );
	if (speed_reg_data && (speed_reg_data <= 1000)) AO0_Register = speed_reg_data;
}

static volatile uint16_t ADC_Data[3];
static volatile float k[3];
static volatile uint8_t kf[3] = { 2, 25, 25};

void ADC_GetData(void){				//Процедура получения данных с АЦП
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
void SysTick_Handler(void){			//Глобальный таймер
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
		/*****************Управление электроприводом******************/
		/*Задание скорости электропривода (HMI->PLC)*/
		if (ext_adjusted_speed_reg && (ext_adjusted_speed_reg <= 140)) 
			SetDriveSpeed(ext_adjusted_speed_reg);
		
		/*Флаги, передаваемые на панель оператора (PLC->HMI)*/
		if(flags & INT_FL_DRIVE_ERROR) ext_flags_reg |= EXT_FL_DRIVE_ERROR; //Флаг ошибки ПЧ
		else ext_flags_reg &= ~EXT_FL_DRIVE_ERROR;
		
		if(MotorBusy_State()) ext_flags_reg |= EXT_FL_DRIVE_RUN;			//Флаг занятости ПЧ
		else ext_flags_reg &= ~EXT_FL_DRIVE_RUN;
		
		/*Конечный автомат работы электропривода*/
		switch (Drive_State){
			
			case DRIVE_STOP:{	//Остановка электропривода
				
				if(!(flags & INT_FL_FST_IMPLEMENTATION)){	//Если идет первая итерация в этом состоянии
					flags |= INT_FL_FST_IMPLEMENTATION;
					
					//Выполнение действий при входе в состояние DRIVE_STOP
					Motor_Stop();
				}
				else{
					//Настройка переходов в другие состояния по условиям
					if( !FreqConvNotBroken_State() || !SafeSensor_State() ){	//Если возникла ошибка ПЧ или дверь шкафа открыта
						Drive_State = DRIVE_ERROR;					//Переходим в состояние ошибки
						flags &= ~INT_FL_FST_IMPLEMENTATION;		//Сбрасываем флаг первой итерации для использования в следующем состоянии
					}
					else{
						if (Triggers_Reg & CMD_MOT_START_RTRIG){	//Если с панели пришла команда к запуску
							Triggers_Reg &= ~CMD_MOT_START_RTRIG;	//Сброс триггера команды запуска
							
							Drive_State = DRIVE_RUN;				//Переходим в состояние работы эл.привода
							flags &= ~INT_FL_FST_IMPLEMENTATION;
						}
					}
				}
				
			} break;
			
			case DRIVE_RUN:{	//Запуск электропривода
				
				if(!(flags & INT_FL_FST_IMPLEMENTATION)){
					flags |= INT_FL_FST_IMPLEMENTATION;
					
					if( FreqConvNotBroken_State() && SafeSensor_State() ) //Если ПЧ исправен и защитная дверь закрыта
						Motor_Start();
					else{ //Иначе переходим в состояние остановки для последующего перехода в ошибку
						Drive_State = DRIVE_STOP;
						flags &= ~INT_FL_FST_IMPLEMENTATION;
					}
				}
				else{
					if( (Triggers_Reg & CMD_MOT_START_FTRIG) || (Triggers_Reg & EXPRMNT_DONE_RTRIG) ||
						!FreqConvNotBroken_State() || !SafeSensor_State() ||
						(ext_flags_reg & EXT_FL_TENSION1_ERROR) || 
						(ext_flags_reg & EXT_FL_TENSION2_ERROR) || 
						(ext_flags_reg & EXT_FL_TENSION3_ERROR) ){ //Если пришел задний фронт с кнопки пуска эл.привода или закончился эксперимент или случилась ошибка
						if( Triggers_Reg & CMD_MOT_START_FTRIG ) Triggers_Reg &= ~CMD_MOT_START_FTRIG;
						if( Triggers_Reg & EXPRMNT_DONE_RTRIG ) Triggers_Reg &= ~EXPRMNT_DONE_RTRIG;
						
						Drive_State = DRIVE_STOP;
						flags &= ~INT_FL_FST_IMPLEMENTATION;
					}
				}
				
			} break;
			
			case DRIVE_ERROR:{	//Ошибка электропривода
				
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
		
		
		/*****************Учет пройденных циклов**********************/
		/*Передача в панель количества оставшихся циклов (PLC->HMI)*/
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
		
		/*Декрементация оставшихся циклов по приему переднего фронта с датчика оборотов*/
		if( Triggers_Reg & RotationSensor_RTRIG ){
			Triggers_Reg &= ~RotationSensor_RTRIG;
			
			if(remaining_cycles){
				remaining_cycles--;
				if(!remaining_cycles) ext_flags_reg |= EXT_FL_EXPERIMENT_DONE;
			}
		}
		if(remaining_cycles) ext_flags_reg &= ~EXT_FL_EXPERIMENT_DONE;
		
		
		/*****************Контроль натяжения образцов*****************/
		//Обработка данных с АЦП и передача в панель оператора (PLC->HMI)
		ai = 0;
		for(ai = 0; ai < 3; ai++)
		{
			k[ai] = ( ( (float)ADC_Data[ai] * 3.3 * 3.68 ) / 4095) * kf[ai]; // * 2;
			// Выпаяны стабилитроны, полином не нужен уже
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
		
		/*****************Процедура коммуникации PLC<->HMI************/
		do_modbus(); //Выполнение коммуникации между HMI и PLC
	}
}
