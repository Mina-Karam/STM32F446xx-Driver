/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Feb 12, 2022
 *      Author: Mina Karam
 */


#include <stm32f446xx_gpio.h>

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
	if(PinConfig->PinMode <= GPIO_MODE_ANALOG)
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
	else
	{
		/* For Setting the interrupt mode*/

		/* Enable clock to SYSCFG (needed to set IRQ in EXTI)*/
		SYSCFG_PCLK_EN();


		/* 1. Set rising/falling edge trigger(s) */

		/* Reset it first */
		EXTI->RTSR &= ~(1 << PinConfig->PinNumber);
		EXTI->FTSR &= ~(1 << PinConfig->PinNumber);

		if(PinConfig->PinMode == GPIO_MODE_INTERRUPT_RT)
		{
			EXTI->RTSR |= (1 << PinConfig->PinNumber);
			EXTI->FTSR &= ~(1 << PinConfig->PinNumber);
		}
		else if(PinConfig->PinMode == GPIO_MODE_INTERRUPT_FT)
		{
			EXTI->FTSR |= (1 << PinConfig->PinNumber);
			EXTI->RTSR &= ~(1 << PinConfig->PinNumber);
		}
		else if(PinConfig->PinMode == GPIO_MODE_INTERRUPT_FRT)
		{
			EXTI->RTSR |= (1 << PinConfig->PinNumber);
			EXTI->FTSR |= (1 << PinConfig->PinNumber);
		}

		/* 2. Configure GPIO port selection in SYSCFG */
		uint8_t SYSCFG_EXTICR_index = PinConfig->PinNumber / 4;
		uint8_t SYSCFG_EXTICR_position = (PinConfig->PinNumber % 4) *4;

		/* Clear the four bits first */
		SYSCFG->EXTICR[SYSCFG_EXTICR_index] &= ~(0xF << SYSCFG_EXTICR_position);

		/* Set the four bits for the port */
		SYSCFG->EXTICR[SYSCFG_EXTICR_index] |= ((GPIO_BASE_TO_CODE(GPIOx) & 0xF) << SYSCFG_EXTICR_position);

		/* 3. Enable EXTI interrupt using interrupt register masking */
		EXTI->IMR |= (1 << PinConfig->PinNumber);
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

	/* 4. Configure pin output type (push-pull or open drain) */
	REGISTER_CONFIG = (PinConfig->PinOPType << PinConfig->PinNumber);
	GPIOx->OTYPER &= ~(0x1 << PinConfig->PinNumber);
	GPIOx->OTYPER |= REGISTER_CONFIG;
	REGISTER_CONFIG = 0;

	/* 5. Configure alternate mode function */
	if(PinConfig->PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t AFR_EXTICR_index = PinConfig->PinNumber / 8;
		uint8_t AFR_EXTICR_position = (PinConfig->PinNumber % 8) *4;

		GPIOx->AFR[AFR_EXTICR_index] &= ~((0xF) << AFR_EXTICR_position);
		GPIOx->AFR[AFR_EXTICR_index] |= (PinConfig->PinAltFunMode << AFR_EXTICR_position);
	}


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
 * @Fn				- MCAL_GPIO_PeriClockControl
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 * @param [in] 		- GPIOx: where x can be (A..E depending on device used) to select GPIO peripheral
 * @param [in] 		- State: specific state (ENABLE, DISABLE Macros)
 * @retval 			- None
 * Note				- None
 */
void MCAL_GPIO_PeriClockControl(GPIO_Typedef_t *GPIOx, uint8_t State)
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

/* ================================================================
 * @Fn				- MCAL_GPIO_IRQConfig
 * @brief			- Set or clear interrupt in processor NVIC
 * @param [in] 		- IRQNumber: IRQ position being configured
 * @param [in] 		- State: specific state (ENABLE, DISABLE Macros)
 * @retval 			- None
 * Note				- None
 */
void MCAL_GPIO_IRQConfig(uint8_t IRQNumber, uint8_t State)
{
	if(State == ENABLE)
	{
		NVIC_ISER->ISER[IRQNumber / 32] |= (1 << (IRQNumber % 32));
	}
	else if (State == DISABLE)
	{
		NVIC_ICER->ICER[IRQNumber / 32] |= (1 << (IRQNumber % 32));
	}
}

/* ================================================================
 * @Fn				- MCAL_GPIO_IRQPriorityConfig
 * @brief			- Set interrupt priority in processor NVIC
 * @param [in] 		- IRQNumber: IRQ position being configured
 * @param [in] 		- IRQPriority: Priority value (0 - 255)
 * @retval 			- None
 * Note				- None
 */
void MCAL_GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t shift = 8 * (IRQNumber % 4) + NO_PR_BITS_IMPLEMENTED;
	NVIC_IPR->IPR[IRQNumber / 4] |= (IRQPriority << shift);
}

/* ================================================================
 * @Fn				- MCAL_GPIO_IRQHandling
 * @brief			- Clear the pending register for set interrupt
 * @param [in] 		- PinNumber: GPIO pin number which interrupt is on
 * @retval 			- None
 * Note				- None
 */
void MCAL_GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}

