/*
 * gpio.c
 *
 *  Created on: Feb 12, 2022
 *      Author: Mina Karam
 */


#include <stm32f446xx_gpio.h>

/* ================================================ */
/* ================ Private APIs ================== */
/* ================================================ */

static void GPIO_PeriClockControl(GPIO_Typedef_t *GPIOx, uint8_t State);

/* ================================================ */
/* ================ Public APIs =================== */
/* ================================================ */

/**================================================================
 * @Fn				- MCAL_GPIO_Init
 * @brief			- This function initialize GPIO peripherals
 * @param [in] 		- GPIOx: where x can be (A..E depending on device used) to select GPIO peripheral
 * @param [in] 		- PinConfig: pointer to a GPIO_PinConfig_t structure that contains
 * 						the configuration information for the specified GPIO PIN.
 * @param [out]		- None
 * @retval 			- None
 * Note				- None
 */
void MCAL_GPIO_Init(GPIO_Typedef_t *GPIOx,GPIO_PinConfig_t *PinConfig)
{
	/* Enable peripheral clock */
	GPIO_PeriClockControl(GPIOx, ENABLE);

	/* To Store the Register configuration */
	uint32_t REGISTER_CONFIG = (uint32_t)NULL;

	/* 1. Set GPIO pin mode configuration */
	if(PinConfig->PinMode == GPIO_MODE_ANALOG)
	{
		/* Put the Pin mode to its specific pin for the register with is 32 bit */
		REGISTER_CONFIG = (PinConfig->PinMode << (2 * PinConfig->PinNumber));

		/* Clear the 2 pin(0b11) before operations */
		GPIOx->MODER &= ~(0x3 << (2 * PinConfig->PinNumber));

		/* Setting the Mode*/
		GPIOx->MODER |= REGISTER_CONFIG;

		/*Set Register configuration to zero for next use of it */
		REGISTER_CONFIG = 0;
	}

	/* 2. Set GPIO pin Speed configuration */
	REGISTER_CONFIG = (PinConfig->PinSpeed << (2 * PinConfig->PinNumber));
	GPIOx->OSPEEDR &= ~(0x3 << (2 * PinConfig->PinNumber));
	GPIOx->OSPEEDR |= REGISTER_CONFIG;
	REGISTER_CONFIG = 0;

	/* 3. Set GPIO pin pull-up or pull-down mode configuration */
	REGISTER_CONFIG = (PinConfig->PinPuPdControl << (2 * PinConfig->PinNumber));
	GPIOx->PUPDR &= ~(0x3 << (2 * PinConfig->PinNumber));
	GPIOx->PUPDR |= REGISTER_CONFIG;
	REGISTER_CONFIG = 0;

	/* 3. Configure pin output type (push-pull or open drain) */
	REGISTER_CONFIG = (PinConfig->PinOPType << PinConfig->PinNumber);
	GPIOx->OTYPER &= ~(0x1 << PinConfig->PinNumber);
	GPIOx->OTYPER |= REGISTER_CONFIG;
	REGISTER_CONFIG = 0;

	/* 4. Configure alternate mode function */


}

/**================================================================
 * @Fn				- MCAL_GPIO_DeInit
 * @brief			- This function de-initialize GPIO peripherals
 * @param [in] 		- GPIOx: where x can be (A..E depending on device used) to select GPIO peripheral
 * @param [out]		- None
 * @retval 			- None
 * Note				- None
 */
void MCAL_GPIO_DeInit(GPIO_Typedef_t *GPIOx)
{
	if (GPIOx == GPIOA) { GPIOA_REG_RESET(); }
	else if (GPIOx == GPIOB) { GPIOB_REG_RESET(); }
	else if (GPIOx == GPIOC) { GPIOC_REG_RESET(); }
	else if (GPIOx == GPIOD) { GPIOD_REG_RESET(); }
	else if (GPIOx == GPIOE) { GPIOE_REG_RESET(); }
	else if (GPIOx == GPIOF) { GPIOF_REG_RESET(); }
	else if (GPIOx == GPIOG) { GPIOG_REG_RESET(); }
	else if (GPIOx == GPIOH) { GPIOH_REG_RESET(); }
}

/**================================================================
 * @Fn				- MCAL_GPIO_ReadPin
 * @brief			- This function reads value of input pin, on a specific port
 * @param [in] 		- GPIOx: where x can be (A..E depending on device used) to select GPIO peripheral
 * @param [in] 		- PinNumber: Set Pin Number according to @ref GPIO_PIN_NUMBERS
 * @retval 			- The input pin value
 * Note				- 0 or 1
 */
uint8_t MCAL_GPIO_ReadPin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber)
{
	uint8_t BitStatus;
	BitStatus = (uint8_t)((GPIOx->IDR >> PinNumber) & 0x1);
	return BitStatus;
}

/**================================================================
 * @Fn				- MCAL_GPIO_ReadPort
 * @brief			- This function reads value of input port
 * @param [in] 		- GPIOx: where x can be (A..E depending on device used) to select GPIO peripheral
 * @retval 			- The input port value
 * Note				- None
 */
uint16_t MCAL_GPIO_ReadPort(GPIO_Typedef_t *GPIOx)
{
	return (uint16_t)(GPIOx->IDR);
}

/**================================================================
 * @Fn				- MCAL_GPIO_WritePin
 * @brief			- This function writes value on a specific output pin
 * @param [in] 		- GPIOx: where x can be (A..E depending on device used) to select GPIO peripheral
 * @param [in] 		- PinNumber: specific the port bit to read @ref GPIO_PIN_NUMBERS
 * @param [in] 		- Value: Pin value (SET, RESET Macro)
 * @retval 			- None
 * Note				- None
 */
void MCAL_GPIO_WritePin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		//GPIOx->ODR |= PinNumber;
		//Or using BSRR Register
		GPIOx->BSRR |= (1 << PinNumber);
	}
	else
	{
		GPIOx->BSRR |= (1 << (16 + PinNumber));
	}
}

/**================================================================
 * @Fn				- MCAL_GPIO_WritePort
 * @brief			- Write on Specific PORT
 * @param [in] 		- GPIOx: where x can be (A..E depending on device used) to select GPIO peripheral
 * @param [in] 		- Value: Pin value (SET, RESET Macro)
 * @retval 			- None
 * Note				- None
 */
void MCAL_GPIO_WritePort(GPIO_Typedef_t *GPIOx, uint16_t Value)
{
	GPIOx->ODR = Value;
}

/**================================================================
 * @Fn				- MCAL_GPIO_TogglePin
 * @brief			- Toggle Specific pin
 * @param [in] 		- GPIOx: where x can be (A..E depending on device used) to select GPIO peripheral
 * @param [in] 		- PinNumber: specific the port bit to read @ref GPIO_PINS_define
 * @retval 			- None
 * Note				- None
 */
void MCAL_GPIO_TogglePin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber)
{
	GPIOx->ODR ^= (1 << PinNumber);
}

/**================================================================
 * @Fn				- MCAL_GPIO_LockPin
 * @brief			- The locking mechanism allows the IO configuration to be frozen
 * @param [in] 		- GPIOx: where x can be (A..E depending on device used) to select GPIO peripheral
 * @param [in] 		- PinNumber: specific the pin number bit
 * @retval 			- OK if pin configure is locked Or ERROR if pin is not locked (OK & ERROR are defined @ref GPIO_LOCK)
 * Note				- None
 */
uint8_t MCAL_GPIO_LockPin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber)
{

	return 0;
}

/**================================================================
 * @Fn				- GPIO_PeriClockControl
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 * @param [in] 		- GPIOx: where x can be (A..E depending on device used) to select GPIO peripheral
 * @param [in] 		- State: specific state (ENABLE, DISABLE Macros)
 * @retval 			- None
 * Note				- None
 */
static void GPIO_PeriClockControl(GPIO_Typedef_t *GPIOx, uint8_t State)
{
	if (State == ENABLE)
	{
		if(GPIOx == GPIOA){ GPIOA_PCLK_EN(); }
		else if(GPIOx == GPIOB){ GPIOB_PCLK_EN(); }
		else if(GPIOx == GPIOC){ GPIOC_PCLK_EN(); }
		else if(GPIOx == GPIOD){ GPIOD_PCLK_EN(); }
		else if(GPIOx == GPIOE){ GPIOE_PCLK_EN(); }
		else if(GPIOx == GPIOF){ GPIOF_PCLK_EN(); }
		else if(GPIOx == GPIOG){ GPIOG_PCLK_EN(); }
		else if(GPIOx == GPIOH){ GPIOH_PCLK_EN(); }
	}
	else
	{
		if(GPIOx == GPIOA){ GPIOA_PCLK_DI(); }
		else if(GPIOx == GPIOB){ GPIOB_PCLK_DI(); }
		else if(GPIOx == GPIOC){ GPIOC_PCLK_DI(); }
		else if(GPIOx == GPIOD){ GPIOD_PCLK_DI(); }
		else if(GPIOx == GPIOE){ GPIOE_PCLK_DI(); }
		else if(GPIOx == GPIOF){ GPIOF_PCLK_DI(); }
		else if(GPIOx == GPIOG){ GPIOG_PCLK_DI(); }
		else if(GPIOx == GPIOH){ GPIOH_PCLK_DI(); }
	}
}
//
//static uint32_t GPIO_GetPositionInWord(uint16_t PinNumber)
//{
//	switch(PinNumber)
//	{
//		case GPIO_PIN_0: return 0; break;
//		case GPIO_PIN_1: return 2; break;
//		case GPIO_PIN_2: return 4; break;
//		case GPIO_PIN_3: return 6; break;
//		case GPIO_PIN_4: return 8; break;
//		case GPIO_PIN_5: return 10; break;
//		case GPIO_PIN_6: return 12; break;
//		case GPIO_PIN_7: return 14; break;
//		case GPIO_PIN_8: return 16; break;
//		case GPIO_PIN_9: return 18; break;
//		case GPIO_PIN_10: return 20; break;
//		case GPIO_PIN_11: return 22; break;
//		case GPIO_PIN_12: return 24; break;
//		case GPIO_PIN_13: return 26; break;
//		case GPIO_PIN_14: return 28; break;
//		case GPIO_PIN_15: return 30; break;
//		default: break;
//	}
//	return 0;
//}
