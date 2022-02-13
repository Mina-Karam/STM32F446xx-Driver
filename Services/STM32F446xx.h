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

#define SET 				1
#define RESET 				0
#define ENABLE 				SET
#define DISABLE 			RESET
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_SET			SET
#define FLAG_RESET			RESET

/* ================================================================ */
/* ========== Base Addresses of FLASH and SRAM memories =========== */
/* ================================================================ */

#define FLASH_BASE 			0x08000000U 	/* Base address of FLASH memory 	*/
#define SYSTEM_BASE			0x1FFF0000U		/* Base address of System memory 	*/
#define SRAM_BASE			0x20000000U		/* Base address of SRAM memory 		*/
#define SRAM1_BASE			SRAM_BASE		/* Base address of SRAM1 memory 	*/
#define SRAM2_BASE			0x2001C000U		/* Base address of SRAM2 memory 	*/

/* ================================================================ */
/* ====== AHBx and APBx Bus Peripheral Base Addresses ============= */
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

typedef struct
{
	vuint32_t MODER;  		/* GPIO port mode register, 	    			Address offset: 0x00 */
	vuint32_t OTYPER; 		/* GPIO port output type register,  			Address offset: 0x04 */
	vuint32_t OSPEEDR;  	/* GPIO port output speed register, 			Address offset: 0x08 */
	vuint32_t PUPDR; 		/* GPIO port pull-up/down register, 			Address offset: 0x0C */
	vuint32_t IDR; 			/* GPIO port input data register,    			Address offset: 0x10 */
	vuint32_t ODR; 			/* GPIO port output data register, 	 			Address offset: 0x14 */
	vuint32_t BSRR; 		/* GPIO port bit set/reset register, 			Address offset: 0x18 */
	vuint32_t LCKR; 		/* GPIO port configuration lock register,		Address offset: 0x1C */
	vuint32_t AFRL;			/* AFRL: GPIO alternate function low register,	Address offset: 0x20 */
	vuint32_t AFRH;			/* AFRH: GPIO alternate function low register,	Address offset: 0x24 */
}GPIO_Typedef_t;

/* ================================================================ */
/* ================= Peripheral Registers RCC  ==================== */
/* ================================================================ */

typedef struct
{
	vuint32_t CR;				/* RCC clock control register, 	    				Address offset: 0x00 	*/
	vuint32_t PLLCFGR;			/* RCC PLL configuration register, 	    			Address offset: 0x04 	*/
	vuint32_t CFGR;				/* RCC clock configuration register, 	   			Address offset: 0x08 	*/
	vuint32_t CIR;				/* RCC clock interrupt register, 	    			Address offset: 0x0C 	*/
	vuint32_t AHB1RSTR;			/* RCC AHB1 peripheral reset register, 	    			Address offset: 0x10 	*/
	vuint32_t AHB2RSTR;			/* RCC AHB2 peripheral reset register, 	    			Address offset: 0x14 	*/
	vuint32_t AHB3RSTR;			/* RCC AHB3 peripheral reset register, 	    			Address offset: 0x18 	*/
	uint32_t  Reserved_RCC_0;	/* RCC reserved register, 					Address offset: 0x1C 	*/
	vuint32_t APB1RSTR;			/* RCC APB1 peripheral reset register, 	    			Address offset: 0x20 	*/
	vuint32_t APB2RSTR;			/* RCC APB2 peripheral reset register, 	    			Address offset: 0x24 	*/
	uint32_t  Reserved_RCC_1[2];/* RCC reserved register, 					Address offset: 0x28-2C */
	vuint32_t AHB1ENR;			/* RCC AHB1 peripheral clock enable register, 	    		Address offset: 0x30 	*/
	vuint32_t AHB2ENR;			/* RCC AHB2 peripheral clock enable register, 	    		Address offset: 0x34 	*/
	vuint32_t AHB3ENR;			/* RCC AHB3 peripheral clock enable register, 	    		Address offset: 0x38 	*/
	uint32_t  Reserved_RCC_2;	/* RCC reserved register, 					Address offset: 0x3C 	*/
	vuint32_t APB1ENR;			/* RCC APB1 peripheral clock enable register, 	    		Address offset: 0x40 	*/
	vuint32_t APB2ENR;			/* RCC APB2 peripheral clock enable register, 	    		Address offset: 0x44 	*/
	uint32_t  Reserved_RCC_3[2];/* RCC reserved register, 					Address offset: 0x48-4C */
	vuint32_t AHB1LPENR;		/* RCC AHB1 peripheral clock enable in low power mode register,	Address offset: 0x50 	*/
	vuint32_t AHB2LPENR;		/* RCC AHB2 peripheral clock enable in low power mode register,	Address offset: 0x54 	*/
	vuint32_t AHB3LPENR;		/* RCC AHB3 peripheral clock enable in low power mode register,	Address offset: 0x58 	*/
	uint32_t  Reserved_RCC_4;	/* RCC reserved register, 					Address offset: 0x5C 	*/
	vuint32_t APB1LPENR;		/* RCC APB1 peripheral clock enable in low power mode register,	Address offset: 0x60 	*/
	vuint32_t APB2LPENR;		/* RCC APB2 peripheral clock enable in low power mode register,	Address offset: 0x64 	*/
	uint32_t  Reserved_RCC_5[2];/* RCC reserved register, 					Address offset: 0x68-6C */
	vuint32_t BDCR;				/* RCC back up domain control register, 			Address offset: 0x70 	*/
	vuint32_t CSR;				/* RCC clock control & status register, 			Address offset: 0x74 	*/
	uint32_t  Reserved_RCC_6[2];/* RCC reserved register, 					Address offset: 0x78-7C */
	vuint32_t SSCGR;			/* RCC spread spectrum clock generation register, 		Address offset: 0x80 	*/
	vuint32_t PLLI2SCFGR;		/* RCC PLLI2S configuration register, 				Address offset: 0x84 	*/
	vuint32_t PLLSAICFGR;		/* RCC PLL configuration register, 				Address offset: 0x88 	*/
	vuint32_t DCKCFGR;			/* RCC dedicated clock configuration register, 			Address offset: 0x8C 	*/
	vuint32_t CKGATENR;			/* RCC clock gated enable register, 				Address offset: 0x90 	*/
}RCC_Typedef_t;

/* ================================================================ */
/* ================= Peripheral Registers EXTI ==================== */
/* ================================================================ */



/* ================================================================ */
/* ================= Peripheral Registers AFIO ==================== */
/* ================================================================ */



/* ================================================================ */
/* =================== Peripheral Instants  ======================= */
/* ================================================================ */

#define GPIOA 			((GPIO_Typedef_t *) GPIOA_BASE)
#define GPIOB 			((GPIO_Typedef_t *) GPIOB_BASE)
#define GPIOC 			((GPIO_Typedef_t *) GPIOC_BASE)
#define GPIOD			((GPIO_Typedef_t *) GPIOD_BASE)
#define GPIOE 			((GPIO_Typedef_t *) GPIOE_BASE)
#define GPIOF 			((GPIO_Typedef_t *) GPIOF_BASE)
#define GPIOG 			((GPIO_Typedef_t *) GPIOG_BASE)
#define GPIOH 			((GPIO_Typedef_t *) GPIOH_BASE)

#define RCC				((RCC_Typedef_t*) RCC_BASE)

/* ================================================================ */
/* =========== Clock Enable/Disable/Reset Macros ================== */
/* ================================================================ */

/* ============ GPIOx peripherals ============ */

/*-*-*-*-* Enable *-*-*-*-*/
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1 << 0) ) /* GPIOA peripheral clock enabled */
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1 << 1) ) /* GPIOB peripheral clock enabled */
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1 << 2) ) /* GPIOC peripheral clock enabled */
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1 << 3) ) /* GPIOD peripheral clock enabled */
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1 << 4) ) /* GPIOE peripheral clock enabled */
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= (1 << 5) ) /* GPIOF peripheral clock enabled */
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= (1 << 6) ) /* GPIOG peripheral clock enabled */
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1 << 7) ) /* GPIOH peripheral clock enabled */

/*-*-*-*-* Disable *-*-*-*-*/
#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 0) ) /* GPIOA peripheral clock disabled */
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 1) ) /* GPIOB peripheral clock disabled */
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 2) ) /* GPIOC peripheral clock disabled */
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 3) ) /* GPIOD peripheral clock disabled */
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 4) ) /* GPIOE peripheral clock disabled */
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 5) ) /* GPIOF peripheral clock disabled */
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 6) ) /* GPIOG peripheral clock disabled */
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 7) ) /* GPIOH peripheral clock disabled */

/*-*-*-*-* Reset *-*-*-*-*/
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)



#endif /* STM32F446XX_H_ */
