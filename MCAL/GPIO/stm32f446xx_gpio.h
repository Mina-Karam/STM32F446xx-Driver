/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Feb 12, 2022
 *      Author: Mina Karam
 */

#ifndef GPIO_STM32F446XX_GPIO_H_
#define GPIO_STM32F446XX_GPIO_H_


/* ================================================================ */
/* =========================== Includes =========================== */
/* ================================================================ */

#include "STM32F446xx.h"

/* ================================================================ */
/* ========== User type definitions (structures) ================== */
/* ================================================================ */

typedef struct
{
	uint16_t PinNumber;			/* Possible values from @GPIO_PIN_NUMBERS  			   				*/
	uint8_t PinMode;			/* Possible values from @GPIO_PIN_MODES	   		          	 		*/
	uint8_t PinSpeed;			/* Possible values from @GPIO_PIN_SPEED	   			   				*/
	uint8_t PinPuPdControl;		/* Possible values from @GPIO_PIN_PULL_UP_PULL_DOWN		   			*/
	uint8_t PinOPType;			/* Possible values from @GPIO_PIN_OUTPUT_TYPE 			   			*/
	uint8_t PinAltFunMode;		/* Has value only when @GPIO_AltFunMode is set to Alt functionality 	*/

}GPIO_PinConfig_t;

/* ================================================================ */
/* =============== Macros Configuration References ================ */
/* ================================================================ */

//@ref GPIO_PIN_NUMBERS
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

#define GPIO_PIN_ALL		((uint16_t)0xFFFF) /* All pins selected */

#define GPIO_PIN_MASK		0x0000FFFFu /* PIN mask for assert test */

/* @ref GPIO_PIN_MODES
 *
 * 00: Input (reset state)
 * 01: General purpose output mode
 * 10: Alternate function mode
 * 11: Analog mode
*/
#define GPIO_MODE_IN    				0 /* GPIO Input mode 		  	    			*/
#define GPIO_MODE_OUT   				1 /* GPIO Output mode 	  		   				*/
#define GPIO_MODE_ALTFN 				2 /* GPIO Alternate functionality 			 	*/
#define GPIO_MODE_ANALOG 				3 /* GPIO Analog mode       					*/
#define GPIO_MODE_INTERRUPT_FT			4 /* GPIO Input mode falling edge trigger     	*/
#define GPIO_MODE_INTERRUPT_RT			5 /* GPIO Input mode rising edge trigger      	*/
#define GPIO_MODE_INTERRUPT_FRT			6 /* GPIO Input mode fall-rising edge trigger 	*/

/* @ref GPIO_PIN_OUTPUT_TYPE
 *
 * 0: Output push-pull (reset state)
 * 1: Output open-drain
 */
#define GPIO_OP_TYPE_PP					0 /* GPIO Output type push-pull mode  */
#define GPIO_OP_TYPE_OD					1 /* GPIO Output type open-drain mode */

/* @ref GPIO_PIN_SPEED
 *
 * 00: Low speed
 * 01: Medium speed
 * 10: Fast speed
 * 11: High speed
 */
#define GPIO_SPEED_LOW					0 /* Output mode, Low Speed. 	*/
#define GPIO_SPEED_MEDIUM				1 /* Output mode, Medium Speed. */
#define GPIO_SPEED_FAST					2 /* Output mode, Fast Speed. 	*/
#define GPIO_SPEED_HIGH					3 /* Output mode, High Speed. 	*/

/* @ref GPIO_PIN_PULL_UP_PULL_DOWN
 *
 * 00: No pull-up, pull-down
 * 01: Pull-up
 * 10: Pull-down
 * 11: Reserved
 */
#define GPIO_PIN_NO_PUPD				0 /* GPIO configuration no pull-up, pull-down 	*/
#define GPIO_PIN_PU						1 /* GPIO configuration pull-up		      		*/
#define GPIO_PIN_PD						2 /* GPIO configuration pull-down	      		*/

/*
 * @ ref GPIO_LOCK
 */
#define GPIO_LOCK_Enabled 				1 	  /* GPIO Pin Lock Enabled 	*/
#define GPIO_LOCK_ERROR	 				0	  /* GPIO Pin Lock Error 	*/

/*
 * @ref GPIO_AltFunMode
 */
#define GPIO_AF_0				0
#define GPIO_AF_1				1
#define GPIO_AF_2				2
#define GPIO_AF_3				3
#define GPIO_AF_4				4
#define GPIO_AF_5				5
#define GPIO_AF_6				6
#define GPIO_AF_7				7
#define GPIO_AF_8				8
#define GPIO_AF_9				9
#define GPIO_AF_10				10
#define GPIO_AF_11				11
#define GPIO_AF_12				12
#define GPIO_AF_13				13
#define GPIO_AF_14				14
#define GPIO_AF_15				15

/*
 * This macro returns a code (between 0 - 7) for given GPIO base address(x)
 */
#define GPIO_BASE_TO_CODE(x)      ( (x == GPIOA)?0:\
									(x == GPIOB)?1:\
									(x == GPIOC)?2:\
									(x == GPIOD)?3:\
									(x == GPIOE)?4:\
									(x == GPIOF)?5:\
									(x == GPIOG)?6:\
									(x == GPIOH)?7:0 )
/* ================================================================ */
/* ============== APIs Supported by "MCAL GPIO DRIVER" ============ */
/* ================================================================ */

/*
 * Peripheral Clock setup
 */
void MCAL_GPIO_PeriClockControl(GPIO_Typedef_t *GPIOx, uint8_t State);

/*
 * Init and De-Init
 */
void MCAL_GPIO_Init(GPIO_Typedef_t *GPIOx,GPIO_PinConfig_t *PinConfig);
void MCAL_GPIO_DeInit(GPIO_Typedef_t *GPIOx);

/*
 * Data read and write
 */
uint8_t MCAL_GPIO_ReadPin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber);
uint16_t MCAL_GPIO_ReadPort(GPIO_Typedef_t *GPIOx);
void MCAL_GPIO_WritePin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber, uint8_t Value);
void MCAL_GPIO_WritePort(GPIO_Typedef_t *GPIOx, uint16_t Value);
void MCAL_GPIO_TogglePin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber);
uint8_t MCAL_GPIO_LockPin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void MCAL_GPIO_IRQConfig(uint8_t IRQNumber, uint8_t State);
void MCAL_GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void MCAL_GPIO_IRQHandling(uint8_t PinNumber);

#endif /* GPIO_STM32F446XX_GPIO_H_ */
