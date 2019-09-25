/*Инициализация портов ввода/вывода управляющей платы*/
#define DOUT0_En()					(GPIOB->BSRR = GPIO_BSRR_BS_8)
#define DOUT0_Dis()					(GPIOB->BSRR = GPIO_BSRR_BR_8)
#define DOUT1_En()					(GPIOB->BSRR = GPIO_BSRR_BS_9)
#define DOUT1_Dis()					(GPIOB->BSRR = GPIO_BSRR_BR_9)
#define DOUT2_En()					(GPIOB->BSRR = GPIO_BSRR_BS_10)
#define DOUT2_Dis()					(GPIOB->BSRR = GPIO_BSRR_BR_10)
#define DOUT3_En()					(GPIOB->BSRR = GPIO_BSRR_BS_11)
#define DOUT3_Dis()					(GPIOB->BSRR = GPIO_BSRR_BR_11)
#define Set_link_led()			(GPIOA->BSRR = GPIO_BSRR_BS_15)
#define Clr_link_led()			(GPIOA->BSRR = GPIO_BSRR_BR_15)

#define DIN0_State()				(!(GPIOB->IDR & GPIO_IDR_2))
#define DIN1_State()				(!(GPIOB->IDR & GPIO_IDR_3))
#define DIN2_State()				(!(GPIOB->IDR & GPIO_IDR_4))
#define DIN3_State()				(!(GPIOB->IDR & GPIO_IDR_5))
#define DIN4_State()				(!(GPIOB->IDR & GPIO_IDR_6))
#define DIN5_State()				(!(GPIOB->IDR & GPIO_IDR_7))

#define AO0_Register				(TIM3->CCR1) 	//(0..1000y.e. -> 0..10V)
#define AO1_Register				(TIM3->CCR2)
#define AO2_Register				(TIM3->CCR3)
#define AO3_Register				(TIM3->CCR4)

/*Присваивание портам ввода/вывода соответствий переферии стенда*/
#define Motor_Start()				(DOUT0_En())
#define Motor_Stop()				(DOUT0_Dis())

#define FreqConvNotBroken_State()	(DIN0_State())
#define MotorBusy_State()			    (DIN1_State())
#define RotationSensor_State()		(DIN2_State())
#define SafeSensor_State()			  (DIN3_State())

/*Инициализация блока управления триггерами*/
static uint16_t TriggersVariables_CurStates;	//Регистр состояний обрабатываемых переменных
static uint16_t TriggersVariables_PrevStates;	//Регистр состояний переменных на предыдущей итерации
static uint32_t Triggers_Reg;					//Регистр состояний триггеров
#define CMD_MOT_START_RTRIG			(0x0001)	//Триггер переднего фронта команды запуска привода
#define CMD_MOT_START_FTRIG			(0x0002)	//Триггер заднего фронта команды запуска привода
#define EXPRMNT_DONE_RTRIG			(0x0004)	//Триггер переднего фронта флага окончания эксперимента
#define EXPRMNT_DONE_FTRIG			(0x0008)	//Триггер заднего фронта флага окончания эксперимента
#define RotationSensor_RTRIG		(0x0010)	//Триггер переднего фронта с датчика оборотов
#define RotationSensor_FTRIG		(0x0020)	//Триггер заднего фронта с датчика оборотов

/*Локальные флаги*/
static uint16_t flags;							//Регистр внутренних флагов
#define INT_FL_FST_IMPLEMENTATION	(0x0001)	//Флаг выполнения первой итерации какого-либо состояния
#define INT_FL_DRIVE_ERROR			  (0x0002)	//Флаг ошибки электропривода

/*Флаги сетевого регистра ext_flags_reg*/
#define EXT_FL_DRIVE_ERROR			(0x0001)	//Флаг ошибки электропривода
#define EXT_FL_DRIVE_RUN			  (0x0002)	//Флаг работы электропривода
#define EXT_FL_EXPERIMENT_DONE	(0x0004)	//Флаг завершения работы стенда по выполнению необходимого кол-ва оборотов
#define EXT_FL_TENSION1_ERROR		(0x0008)	//Флаг выхода натяжения образца 1 из допустимых пределов
#define EXT_FL_TENSION2_ERROR		(0x0010)	//Флаг выхода натяжения образца 2 из допустимых пределов
#define EXT_FL_TENSION3_ERROR		(0x0020)	//Флаг выхода натяжения образца 3 из допустимых пределов

/*Флаги сетевого регистра ext_cmd_reg*/
#define EXT_CMD_MOTOR_START			  (0x0001)	//Команда на запуск электропривода
#define EXT_CMD_START_EXPERIMENT	(0x0002)	//Команда на начало учета количества выполненных оборотов
#define EXT_CMD_TENSION1_CTRL_ON	(0x0004)	//Команда начала контроля натяжения образца 1
#define EXT_CMD_TENSION2_CTRL_ON	(0x0008)	//Команда начала контроля натяжения образца 2
#define EXT_CMD_TENSION3_CTRL_ON	(0x0010)	//Команда начала контроля натяжения образца 3

static uint8_t Drive_State;

enum Enum_DriveStates{							//Перечисление состояний автомата работы электропривода
	DRIVE_STOP = 0,
	DRIVE_RUN,
	DRIVE_ERROR
};

static uint32_t adjusted_cycles;				// Целевое количество оборотов
static uint32_t remaining_cycles;				// Количество оборотов, которые осталось выполнить до остановки стенда
static uint32_t current_cycles;				  // Количество оборотов, выполнено

void IO_Config(void) //Конфигурирование портов ввода/вывода микроконтроллера
{							
	
	//		PB2..PB7 - DIN0..DIN5
	//		PB8..PB11 - DO0..DO0
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN;
	
	/*DINs*/
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
	
	/*DOUTs*/
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
	GPIOB->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0);
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11);
	
	/*LED*/
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODER15);
	GPIOA->MODER |= (GPIO_MODER_MODER15_0);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_15);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR15);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR15_0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR15);
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	/*AOUTs*/
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
	GPIOA->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
	GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
	GPIOB->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
	
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7);
	GPIOA->AFR[0] |= ( (1 << 24) | (1 << 28));
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL0 | GPIO_AFRL_AFRL1);
	GPIOB->AFR[0] |= ( (1 << 0) | (1 << 4));
	
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR7_0);
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0_0 | GPIO_OSPEEDER_OSPEEDR1_0);
	
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR1);
	
	/*Копипаста!!!*/
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	TIM3->CR1 = TIM_CR1_ARPE;
	TIM3->CR2 = 0;
	TIM3->SMCR = 0;
	TIM3->ARR = 1150;
	TIM3->PSC = (SystemCoreClock/1000000L) - 1;
	TIM3->CCMR1 = TIM_CCMR1_OC2M_2|TIM_CCMR1_OC2M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1;			// CH1,CH2 - PWM Mode 1
	TIM3->CCMR2 = TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4M_1|TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3M_1;			// CH3,CH4 - PWM Mode 1
	TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 0;
	TIM3->CCER = TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E;
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 = TIM_CR1_CEN;
	/*Конец копипасты*/
	
	/*AINs*/
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= ( GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3 );

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	ADC1->CR |= ADC_CR_ADCAL;								//Запуск автокалибровки
	while (ADC1->CR & ADC_CR_ADCAL);
	ADC1->CFGR1 &= ~ADC_CFGR1_RES;							//12-битное разрешение
	ADC1->SMPR &= ~ADC_SMPR_SMP;							//Sampling time	1.5 ADC clock cycles
	ADC1->CR |= ADC_CR_ADEN;	
}
