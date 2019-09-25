#ifndef __GPIO_LIB_H__
#define __GPIO_LIB_H__

//	GPIO_PINS
#define GPIO_PIN0				(0)
#define GPIO_PIN1				(1L)
#define GPIO_PIN2				(2L)
#define GPIO_PIN3				(3L)
#define GPIO_PIN4				(4L)
#define GPIO_PIN5				(5L)
#define GPIO_PIN6				(6L)
#define GPIO_PIN7				(7L)
#define GPIO_PIN8				(8L)
#define GPIO_PIN9				(9L)
#define GPIO_PIN10				(10L)
#define GPIO_PIN11				(11L)
#define GPIO_PIN12				(12L)
#define GPIO_PIN13				(13L)
#define GPIO_PIN14				(14L)
#define GPIO_PIN15				(15L)

//	GPIO->PUPDR
#define GPIO_PU_NOPUPD			(0)
#define GPIO_PU_PU				(1L)
#define GPIO_PU_PD				(2L)

//	GPIO->OTYPER
#define GPIO_OT_PU				(0)
#define GPIO_OT_OD				(1L)
//	GPIO->OSPEEDR
#define GPIO_SP_LOW				(0)
#define GPIO_SP_MEDIUM			(1L)
#define GPIO_SP_HIGH			(2L)
#define GPIO_SP_VERY_HIGH		(3L)
//	GPIO->AFR
#define GPIO_AF0				(0)
#define GPIO_AF1				(1L)
#define GPIO_AF2				(2L)
#define GPIO_AF3				(3L)
#define GPIO_AF4				(4L)
#define GPIO_AF5				(5L)
#define GPIO_AF6				(6L)
#define GPIO_AF7				(7L)
#define GPIO_AF8				(8L)
#define GPIO_AF9				(9L)
#define GPIO_AF10				(10L)
#define GPIO_AF11				(11L)
#define GPIO_AF12				(12L)
#define GPIO_AF13				(13L)
#define GPIO_AF14				(14L)

void GPIO_Config_IF(GPIO_TypeDef *, uint32_t pin, uint32_t pu);
void GPIO_Config_GP(GPIO_TypeDef *, uint32_t pin, uint32_t ot, uint32_t sp, uint32_t pu);
void GPIO_Config_AF(GPIO_TypeDef *, uint32_t pin, uint32_t af, uint32_t ot, uint32_t sp, uint32_t pu);
void GPIO_Config_AN(GPIO_TypeDef *, uint32_t pin, uint32_t pu);
void GPIO_Set(GPIO_TypeDef *GPIO, uint32_t pin);
void GPIO_Reset(GPIO_TypeDef *GPIO, uint32_t pin);

#endif
