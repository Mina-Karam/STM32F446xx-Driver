/*
 * gpio.h
 *
 *  Created on: Feb 12, 2022
 *      Author: Mina Karam
 */

#ifndef GPIO_GPIO_H_
#define GPIO_GPIO_H_


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
	uint8_t PinAltFunMode;		/* Has value only when @GPIO_PIN_MODES is set to Alt functionality 	*/

}GPIO_PinConfig_t;

/* ================================================================ */
/* =============== Macros Configuration References ================ */
/* ================================================================ */

//@ref GPIO_PIN_NUMBERS
#define GPIO_PIN_0						((uint16_t)0x0001) /* Pin 0 selected */
#define GPIO_PIN_1						((uint16_t)0x0002) /* Pin 1 selected */
#define GPIO_PIN_2						((uint16_t)0x0004) /* Pin 2 selected */
#define GPIO_PIN_3						((uint16_t)0x0008) /* Pin 3 selected */
#define GPIO_PIN_4						((uint16_t)0x0010) /* Pin 4 selected */
#define GPIO_PIN_5						((uint16_t)0x0020) /* Pin 5 selected */
#define GPIO_PIN_6						((uint16_t)0x0040) /* Pin 6 selected */
#define GPIO_PIN_7						((uint16_t)0x0080) /* Pin 7 selected */
#define GPIO_PIN_8						((uint16_t)0x0100) /* Pin 8 selected */
#define GPIO_PIN_9						((uint16_t)0x0200) /* Pin 9 selected */
#define GPIO_PIN_10						((uint16_t)0x0400) /* Pin 10 selected */
#define GPIO_PIN_11						((uint16_t)0x0800) /* Pin 11 selected */
#define GPIO_PIN_12						((uint16_t)0x1000) /* Pin 12 selected */
#define GPIO_PIN_13						((uint16_t)0x2000) /* Pin 13 selected */
#define GPIO_PIN_14						((uint16_t)0x4000) /* Pin 14 selected */
#define GPIO_PIN_15						((uint16_t)0x8000) /* Pin 15 selected */
#define GPIO_PIN_ALL					((uint16_t)0xFFFF) /* All pins selected */

#define GPIO_PIN_MASK					0x0000FFFFu /* PIN mask for assert test */

/* @ref GPIO_PIN_MODES
 *
 * 00: Input (reset state)
 * 01: General purpose output mode
 * 10: Alternate function mode
 * 11: Analog mode
*/
#define GPIO_MODE_IN    				0x00u /* GPIO Input mode 		  	    */
#define GPIO_MODE_OUT   				0x01u /* GPIO Output mode 	  		   	*/
#define GPIO_MODE_ALTFN 				0x02u /* GPIO Alternate functionality 	*/
#define GPIO_MODE_ANALOG 				0x03u /* GPIO Analog mode       		*/

/* @ref GPIO_PIN_OUTPUT_TYPE
 *
 * 0: Output push-pull (reset state)
 * 1: Output open-drain
 */
#define GPIO_OP_TYPE_PP					0x00u /* GPIO Output type push-pull mode  */
#define GPIO_OP_TYPE_OD					0x01u /* GPIO Output type open-drain mode */

/* @ref GPIO_PIN_SPEED
 *
 * 00: Low speed
 * 01: Medium speed
 * 10: Fast speed
 * 11: High speed
 */
#define GPIO_SPEED_LOW					0x00u /* Output mode, Low Speed. 	*/
#define GPIO_SPEED_MEDIUM				0x01u /* Output mode, Medium Speed. */
#define GPIO_SPEED_FAST					0x02u /* Output mode, Fast Speed. 	*/
#define GPIO_SPEED_HIGH					0x03u /* Output mode, High Speed. 	*/

/* @ref GPIO_PIN_PULL_UP_PULL_DOWN
 *
 * 00: No pull-up, pull-down
 * 01: Pull-up
 * 10: Pull-down
 * 11: Reserved
 */
#define GPIO_PIN_NO_PUPD				0x00u /* GPIO configuration no pull-up, pull-down 	*/
#define GPIO_PIN_PU						0x01u /* GPIO configuration pull-up		      		*/
#define GPIO_PIN_PD						0x02u /* GPIO configuration pull-down	      		*/

//@ ref GPIO_LOCK
#define GPIO_LOCK_Enabled 				1 	  /* GPIO Pin Lock Enabled 	*/
#define GPIO_LOCK_ERROR	 				0	  /* GPIO Pin Lock Error 	*/

/* ================================================================ */
/* ============== APIs Supported by "MCAL GPIO DRIVER" ============ */
/* ================================================================ */

void MCAL_GPIO_Init(GPIO_Typedef_t *GPIOx,GPIO_PinConfig_t *PinConfig);
void MCAL_GPIO_DeInit(GPIO_Typedef_t *GPIOx);

uint8_t MCAL_GPIO_ReadPin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber);
uint8_t MCAL_GPIO_ReadPort(GPIO_Typedef_t *GPIOx);


void MCAL_GPIO_WritePin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber, uint8_t Value);
void MCAL_GPIO_WritePort(GPIO_Typedef_t *GPIOx, uint16_t Value);


void MCAL_GPIO_TogglePin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber);

uint8_t MCAL_GPIO_LockPin(GPIO_Typedef_t *GPIOx, uint16_t PinNumber);

#endif /* GPIO_GPIO_H_ */
