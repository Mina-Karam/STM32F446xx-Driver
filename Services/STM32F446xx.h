/*
 * STM32F446xx.h
 *
 *  Created on: Feb 12, 2022
 *      Author: Mina Karam
 */

#ifndef STM32F446XX_H_
#define STM32F446XX_H_

/* ================================================================ */
/* =========================== Includes =========================== */
/* ================================================================ */

#include "Platform_Types.h"

/* ================================================================ */
/* ====================== Generic Macros ========================== */
/* ================================================================ */

#define __IO 		volatile
#define __weak 		__attribute__((weak))

/* ================================================================ */
/* ========== Base Addresses of FLASH and SRAM memories =========== */
/* ================================================================ */

#define FLASH_BASE 			0x08000000U 	/* Base address of FLASH memory 	*/
#define SYSTEM_BASE			0x1FFF0000U		/* Base address of System memory 	*/
#define SRAM_BASE			0x20000000U		/* Base address of SRAM memory 		*/
#define SRAM1_BASE			SRAM_BASE		/* Base address of SRAM1 memory 	*/
#define SRAM2_BASE			0x2001C000U		/* Base address of SRAM2 memory 	*/

/* ================================================================ */
/* ====== AHBx and APBx Bus Peripheral Base Addresses============== */
/* ================================================================ */

#define PERIPH_BASE			0x40000000U		/* Base address of PERIPHERALS memory 	  */
#define APB1PERIPH_BASE		PERIPH_BASE		/* Base address of APB1 peripheral memory */
#define APB2PERIPH_BASE		0x40010000U		/* Base address of APB2 peripheral memory */
#define AHB1PERIPH_BASE		0x40020000U		/* Base address of AHB1 peripheral memory */
#define AHB2PERIPH_BASE		0x50000000U		/* Base address of AHB2 peripheral memory */

/* ================================================================ */
/* ============== Base Addresses for APB1 Peripherals ============= */
/* ================================================================ */

#define I2C1_BASE			(APB1PERIPH_BASE + 0x5400) /* I2C1 base address   */
#define I2C2_BASE			(APB1PERIPH_BASE + 0x5800) /* I2C2 base address   */
#define I2C3_BASE			(APB1PERIPH_BASE + 0x5C00) /* I2C3 base address   */
#define SPI2_BASE			(APB1PERIPH_BASE + 0x3800) /* SPI2 base address   */
#define SPI3_BASE			(APB1PERIPH_BASE + 0x3C00) /* SPI3 base address   */
#define USART2_BASE			(APB1PERIPH_BASE + 0x4400) /* USART2 base address */
#define USART3_BASE			(APB1PERIPH_BASE + 0x4800) /* USART3 base address */
#define UART4_BASE			(APB1PERIPH_BASE + 0x4C00) /* UART4 base address  */
#define UART5_BASE			(APB1PERIPH_BASE + 0x5000) /* UART5 base address  */

/* ================================================================ */
/* ============== Base Addresses for APB2 Peripherals ============= */
/* ================================================================ */

#define SPI1_BASE			(APB2PERIPH_BASE + 0x3000) /* SPI1 base address   */
#define SPI4_BASE			(APB2PERIPH_BASE + 0x3400) /* SPI4 base address   */
#define USART1_BASE			(APB2PERIPH_BASE + 0x1000) /* USART1 base address */
#define USART6_BASE			(APB2PERIPH_BASE + 0x1400) /* USART6 base address */
#define SYSCFG_BASE			(APB2PERIPH_BASE + 0x3800) /* SYSCFG base address */
#define EXTI_BASE			(APB2PERIPH_BASE + 0x3C00) /* EXTI base address   */


/* ================================================================ */
/* ============== Base Addresses for AHB1 Peripherals ============= */
/* ================================================================ */

#define GPIOA_BASE			(AHB1PERIPH_BASE + 0x0000) /* GPIOA Base Address					*/
#define GPIOB_BASE			(AHB1PERIPH_BASE + 0x0400) /* GPIOB Base Address					*/
#define GPIOC_BASE			(AHB1PERIPH_BASE + 0x0800) /* GPIOC Base Address					*/
#define GPIOD_BASE			(AHB1PERIPH_BASE + 0x0C00) /* GPIOD Base Address					*/
#define GPIOE_BASE			(AHB1PERIPH_BASE + 0x1000) /* GPIOE Base Address					*/
#define GPIOF_BASE			(AHB1PERIPH_BASE + 0x1400) /* GPIOF Base Address					*/
#define GPIOG_BASE			(AHB1PERIPH_BASE + 0x1800) /* GPIOG Base Address					*/
#define GPIOH_BASE			(AHB1PERIPH_BASE + 0x1C00) /* GPIOH Base Address					*/
#define CRC_BASE			(AHB1PERIPH_BASE + 0x3000) /* CRC base address	 					*/
#define RCC_BASE			(AHB1PERIPH_BASE + 0x3800) /* RCC base address	 					*/
#define FIR_BASE			(AHB1PERIPH_BASE + 0x3C00) /* Flash interface register base address	*/
#define DMA1_BASE			(AHB1PERIPH_BASE + 0x6000) /* DMA1 base address	 					*/
#define DMA2_BASE			(AHB2PERIPH_BASE + 0x6400) /* DMA2 base address	 					*/

/* ================================================================ */
/* ============== Base Addresses for AHB2 Peripherals ============= */
/* ================================================================ */



/* ================================================================ */
/* ================= Peripheral Registers GPIO ==================== */
/* ================================================================ */



/* ================================================================ */
/* ================= Peripheral Registers RCC  ==================== */
/* ================================================================ */



/* ================================================================ */
/* ================= Peripheral Registers EXTI ==================== */
/* ================================================================ */



/* ================================================================ */
/* ================= Peripheral Registers AFIO ==================== */
/* ================================================================ */



/* ================================================================ */
/* =================== Peripheral Instants  ======================= */
/* ================================================================ */



/* ================================================================ */
/* ===================== Clock Enable Macros ====================== */
/* ================================================================ */





#endif /* STM32F446XX_H_ */
