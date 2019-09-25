#include "stm32f0xx.h"
#include "gpio_lib.h"

/**
  * @brief	Configure GPIO pin as input floating
  * @param	pin: pin number (0..15)
  * @param	pu: pull-up type (0-no, 1-pull-up, 2-pull-down)
  * @retval	none
  */
void GPIO_Config_IF(GPIO_TypeDef *GPIO, uint32_t pin, uint32_t pu)
{
	pin &= 0x0f;
	GPIO->MODER &= ~(3 << (pin*2));
	GPIO->PUPDR &= ~(3 << (pin*2));
	if (pu)
		GPIO->PUPDR |= (pu << (pin*2));
	
}
/**
	* @brief	Configure GPIO pin as GP output
	* @param	pin: pin number (0..15)
	* @param	ot: output type (0-pushpull, 1-open-drain)
	* @param	sp: speed (0-low, 1-medium, 2-high, 3-very high)
	* @param	pu: pull-up type (0-no, 1-pull-up, 2-pull-down)
	* @retval	none
	*/
void GPIO_Config_GP(GPIO_TypeDef *GPIO, uint32_t pin, uint32_t ot, uint32_t sp, uint32_t pu)
{
	pin &= 0x0f;
	GPIO->MODER &= ~((uint32_t)3 << (pin*2));
	GPIO->MODER |= ((uint32_t)1 << (pin*2));
	GPIO->OTYPER &= ~((uint32_t)1 << (pin));
	if (ot) 
		GPIO->OTYPER |= (ot << pin);
	GPIO->PUPDR &= ~((uint32_t)3 << (pin*2));
	if (pu)
		GPIO->PUPDR |= (pu << (pin*2));
	GPIO->OSPEEDR &= ~((uint32_t)3 << (pin*2));
	if (sp)
		GPIO->OSPEEDR |= (sp << (pin*2));
}
/**
	* @brief	Configure GPIO as AF pin
	* @param	pin: pin number (0..15)
	* @param	af:  - AF number (0..14)
	* @param	sp:	 - speed (0..3)
*/
void GPIO_Config_AF(GPIO_TypeDef *GPIO, uint32_t pin, uint32_t af, uint32_t ot, uint32_t sp, uint32_t pu)
{
	pin &= 0x0f;
	GPIO->MODER &= ~((uint32_t)3 << (pin*2));
	GPIO->MODER |= ((uint32_t)2 << (pin*2));
	if (pin < 8)
	{
		GPIO->AFR[0] &= ~((uint32_t)15 << (pin*4));
		GPIO->AFR[0] |= (af << (pin*4));
	} else
	{
		pin -= 8;
		GPIO->AFR[1] &= ~((uint32_t)15 << (pin*4));
		GPIO->AFR[1] |= (af << (pin*4));
	}
	GPIO->OTYPER &= ~((uint32_t)1 << (pin));
	if (ot) 
		GPIO->OTYPER |= (ot << pin);
	GPIO->OSPEEDR &= ~((uint32_t)3 << (pin*2));
	if (sp)
		GPIO->OSPEEDR |= (sp << (pin*2));
	GPIO->PUPDR &= ~((uint32_t)3 << (pin*2));
	if (pu)
		GPIO->PUPDR |= (pu << (pin*2));

}
/**
	* @brief	Configure GPIO as Amalog input pin
	* @param	pin: pin number (0..15)
	* @param	pu: pull-up type (0-no, 1-pull-up, 2-pull-down)
*/
void GPIO_Config_AN(GPIO_TypeDef *GPIO, uint32_t pin, uint32_t pu)
{
	pin &= 0x0f;
	GPIO->MODER |= ((uint32_t)3 << (pin*2));
	GPIO->PUPDR &= ~((uint32_t)3 << (pin*2));
	if (pu)
		GPIO->PUPDR |= (pu << (pin*2));
}
void GPIO_Set(GPIO_TypeDef *GPIO, uint32_t pin)
{
	GPIO->BSRR = ((uint32_t)1 << pin);
}
void GPIO_Reset(GPIO_TypeDef *GPIO, uint32_t pin)
{
	GPIO->BSRR = ((uint32_t)1 << (pin+16));
}
