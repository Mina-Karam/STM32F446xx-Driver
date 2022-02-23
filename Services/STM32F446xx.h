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

/* ==================================================================================== */
/* ============================ Cortex-M4 Specific Registers ========================== */
/* ==================================================================================== */

/* ====================================================== */
/* ========== Base Addresses of NVIC memories =========== */
/* ====================================================== */

#define NVIC_ISER0_BASE			0xE000E100U	/* Interrupt Set-Enable Registers */
#define NVIC_ICER0_BASE			0xE000E180U	/* Interrupt Clear-Enable Registers */
#define NVIC_ISPR0_BASE			0xE000E200U	/* Interrupt Set-Pending Registers */
#define NVIC_ICPR0_BASE			0xE000E280U	/* Interrupt Clear-Pending Registers */
#define NVIC_IABR0_BASE			0xE000E300U	/* Interrupt Active Bit Register */
#define NVIC_IPR0_BASE			0xE000E400U	/* Interrupt Priority Register */

#define NVIC_SYSTCK_BASE		0xE000E010U	/* SysTick Control and Status Register */

#define NVIC_NONIMPL_LOW_BITS	4
#define NO_PR_BITS_IMPLEMENTED  4 /* ARM Cortex Mx Processor number of priority bits implemented in Priority Register */

/* ==================================================================================== */
/* ======================= STM32F446RE MEMORY BASE ADDRESSES ========================== */
/* ==================================================================================== */

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

#define ADC_BASE			(APB2_BASE + 0x2000)
#define ADC1_BASE			(ADC_BASE)
#define ADC2_BASE			(ADC_BASE + 0x100)
#define ADC3_BASE			(ADC_BASE + 0x200)
#define ADC_COMM_BASE		(ADC1_BASE + 0x300)

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
#define DMA2_BASE			(AHB1PERIPH_BASE + 0x6400) /* DMA2 base address	 					*/

/* ================================================================ */
/* ============== Base Addresses for AHB2 Peripherals ============= */
/* ================================================================ */

#define SAI1_BASE			(AHB2PERIPH_BASE + 0x5C00) /* Serial audio interface 1 base address	 */
#define SAI2_BASE			(AHB2PERIPH_BASE + 0x5800) /* Serial audio interface 2 base address	 */

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
	vuint32_t AFR[2];		/* AFR[0]: GPIO alternate function low register,
						   	   AFR[1]: GPIO alternate function high register,	Address offset: 0x20-24 */
}GPIO_Typedef_t;

/* ================================================================ */
/* ================= Peripheral Registers RCC  ==================== */
/* ================================================================ */

typedef struct
{
	vuint32_t CR;				/* RCC clock control register, 	    							Address offset: 0x00 	*/
	vuint32_t PLLCFGR;			/* RCC PLL configuration register, 	    						Address offset: 0x04 	*/
	vuint32_t CFGR;				/* RCC clock configuration register, 	   						Address offset: 0x08 	*/
	vuint32_t CIR;				/* RCC clock interrupt register, 	    						Address offset: 0x0C 	*/
	vuint32_t AHB1RSTR;			/* RCC AHB1 peripheral reset register, 	    					Address offset: 0x10 	*/
	vuint32_t AHB2RSTR;			/* RCC AHB2 peripheral reset register, 	    					Address offset: 0x14 	*/
	vuint32_t AHB3RSTR;			/* RCC AHB3 peripheral reset register, 	    					Address offset: 0x18 	*/
	uint32_t  Reserved_0;	/* RCC reserved register, 										Address offset: 0x1C 	*/
	vuint32_t APB1RSTR;			/* RCC APB1 peripheral reset register, 	    					Address offset: 0x20 	*/
	vuint32_t APB2RSTR;			/* RCC APB2 peripheral reset register, 	    					Address offset: 0x24 	*/
	uint32_t  Reserved_1[2];/* RCC reserved register, 										Address offset: 0x28-2C */
	vuint32_t AHB1ENR;			/* RCC AHB1 peripheral clock enable register, 	    			Address offset: 0x30 	*/
	vuint32_t AHB2ENR;			/* RCC AHB2 peripheral clock enable register, 	    			Address offset: 0x34 	*/
	vuint32_t AHB3ENR;			/* RCC AHB3 peripheral clock enable register, 	    			Address offset: 0x38 	*/
	uint32_t  Reserved_2;	/* RCC reserved register, 										Address offset: 0x3C 	*/
	vuint32_t APB1ENR;			/* RCC APB1 peripheral clock enable register, 	    			Address offset: 0x40 	*/
	vuint32_t APB2ENR;			/* RCC APB2 peripheral clock enable register, 	    			Address offset: 0x44 	*/
	uint32_t  Reserved_3[2];/* RCC reserved register, 										Address offset: 0x48-4C */
	vuint32_t AHB1LPENR;		/* RCC AHB1 peripheral clock enable in low power mode register,	Address offset: 0x50 	*/
	vuint32_t AHB2LPENR;		/* RCC AHB2 peripheral clock enable in low power mode register,	Address offset: 0x54 	*/
	vuint32_t AHB3LPENR;		/* RCC AHB3 peripheral clock enable in low power mode register,	Address offset: 0x58 	*/
	uint32_t  Reserved_4;	/* RCC reserved register, 										Address offset: 0x5C 	*/
	vuint32_t APB1LPENR;		/* RCC APB1 peripheral clock enable in low power mode register,	Address offset: 0x60 	*/
	vuint32_t APB2LPENR;		/* RCC APB2 peripheral clock enable in low power mode register,	Address offset: 0x64 	*/
	uint32_t  Reserved_5[2];/* RCC reserved register, 										Address offset: 0x68-6C */
	vuint32_t BDCR;				/* RCC back up domain control register, 						Address offset: 0x70 	*/
	vuint32_t CSR;				/* RCC clock control & status register, 						Address offset: 0x74 	*/
	uint32_t  Reserved_6[2];/* RCC reserved register, 										Address offset: 0x78-7C */
	vuint32_t SSCGR;			/* RCC spread spectrum clock generation register, 				Address offset: 0x80 	*/
	vuint32_t PLLI2SCFGR;		/* RCC PLLI2S configuration register, 							Address offset: 0x84 	*/
	vuint32_t PLLSAICFGR;		/* RCC PLL configuration register, 								Address offset: 0x88 	*/
	vuint32_t DCKCFGR;			/* RCC dedicated clock configuration register, 					Address offset: 0x8C 	*/
	vuint32_t CKGATENR;			/* RCC clock gated enable register, 							Address offset: 0x90 	*/
}RCC_Typedef_t;

/* ================================================================ */
/* ================= Peripheral Registers EXTI ==================== */
/* ================================================================ */

typedef struct
{
	vuint32_t IMR;  	/* EXTI Interrupt mask register, 	    	Address offset: 0x00 */
	vuint32_t EMR; 		/* EXTI Event mask register,  				Address offset: 0x04 */
	vuint32_t RTSR;  	/* EXTI Rising trigger selection register, 	Address offset: 0x08 */
	vuint32_t FTSR; 	/* EXTI Falling trigger selection register, Address offset: 0x0C */
	vuint32_t SWIER; 	/* EXTI Software interrupt event register,  Address offset: 0x10 */
	vuint32_t PR; 		/* EXTI Pending register, 	 				Address offset: 0x14 */
}EXTI_Typedef_t;

/* ================================================================ */
/* ================ Peripheral Registers SYSCFG =================== */
/* ================================================================ */

typedef struct
{
	vuint32_t MEMRMP;  		/* SYSCFG Memory re-map register, 	    			Address offset: 0x00 	*/
	vuint32_t PMC; 			/* SYSCFG Peripheral mode configuration register,  	Address offset: 0x04 	*/
	vuint32_t EXTICR[4];  	/* SYSCFG External interrupt configuration register,Address offset: 0x08-14 */
	uint32_t  RESERVED_0[2];/* SYSCFG Reserved register, 						Address offset: 0x18-1C */
	vuint32_t CMPCR; 		/* SYSCFG Compensation cell control register, 	 	Address offset: 0x20 	*/
	uint32_t  RESERVED_1[2];/* SYSCFG Reserved register, 						Address offset: 0x24-28 */
	vuint32_t CFGR; 		/* SYSCFG Configuration register, 	 				Address offset: 0x2C 	*/
}STSCFG_Typedef_t;

/* ==================================================================================== */
/* ============================ Cortex-M4 Specific Registers ========================== */
/* ==================================================================================== */

/* ===================================================== */
/* ==============  NVIC ISER Registers ================= */
/* ===================================================== */
typedef struct
{
	vuint32_t ISER[8];			/* NVIC Interrupt Set-Enable Registers, 	 		Address offset: 0x00 	*/
}NVIC_ISER_Typedef_t;

/* ===================================================== */
/* ==============  NVIC ICER Registers ================= */
/* ===================================================== */
typedef struct
{
	vuint32_t ICER[8];			/* NVIC Interrupt Clear-Enable Registers, 	Address offset: 0x00 	*/
}NVIC_ICER_Typedef_t;

/* ===================================================== */
/* ==============  NVIC ISPR Registers ================= */
/* ===================================================== */
typedef struct
{
	vuint32_t ISPR[8];			/* NVIC Interrupt Set-Pending Registers, 	Address offset: 0x00 	*/
}NVIC_ISPR_Typedef_t;

/* ===================================================== */
/* ==============  NVIC ICPR Registers ================= */
/* ===================================================== */
typedef struct
{
	vuint32_t ICPR[8];			/* NVIC Interrupt Clear-Pending Registers, 	Address offset: 0x00 	*/
}NVIC_ICPR_Typedef_t;

/* ===================================================== */
/* ==============  NVIC IABR Registers ================= */
/* ===================================================== */
typedef struct
{
	vuint32_t IABR[8];			/* NVIC Interrupt Active Bit Register, 	Address offset: 0x00 	*/
}NVIC_IABR_Typedef_t;

/* ==================================================== */
/* ==============  NVIC IPR Registers ================= */
/* ==================================================== */
typedef struct
{
	vuint32_t IPR[60];			/* NVIC Interrupt Priority Register, 	 	Address offset: 0x00 	*/
}NVIC_IPR_Typedef_t;

/* ===================================================== */
/* ==============  NVIC SYSTCK Registers =============== */
/* ===================================================== */
typedef struct
{
	vuint32_t  CSR;				/* SYSTCK control and status register, 	 	Address offset: 0x00 */
	vuint32_t  RVR;				/* SYSTCK reload value register, 	 		Address offset: 0x04 */
	vuint32_t  CVR;				/* SYSTCK current value register, 	 		Address offset: 0x08 */
	vuint32_t  CALIB;			/* SYSTCK calibration value register, 	 	Address offset: 0x0C */
}NVIC_SYSTCK_Typedef_t;

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

#define RCC				((RCC_Typedef_t *) RCC_BASE)

#define EXTI			((EXTI_Typedef_t *) EXTI_BASE)

#define SYSCFG			((SYSCFG_Typedef_t *) SYSCFG_BASE)

/*-*-*-*-* NVIC For Cortex-M4 *-*-*-*-*/
#define NVIC_ISER		((NVIC_ISER_Typedef_t *)(NVIC_ISER0_BASE))
#define NVIC_ICER		((NVIC_ICER_Typedef_t *)(NVIC_ICER0_BASE))
#define NVIC_ISPR		((NVIC_ISPR_Typedef_t *)(NVIC_ISPR0_BASE))
#define NVIC_ICPR		((NVIC_ICPR_Typedef_t *)(NVIC_ICPR0_BASE))
#define NVIC_IABR		((NVIC_IABR_Typedef_t *)(NVIC_IABR0_BASE))
#define NVIC_IPR		((NVIC_IPR_Typedef_t *)(NVIC_IPR0_BASE))

#define NVIC_SYSTCK 	((NVIC_SYSTCK_Typedef_t *)(NVIC_SYSTCK_BASE))



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

/* ============ SYSCFG peripherals ============ */
/*-*-*-*-* Enable *-*-*-*-*/
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 14) )  /* SYSCFG peripheral clock enabled */

/*-*-*-*-* Disable *-*-*-*-*/
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 14) )  /* SYSCFG peripheral clock disabled */

/*-*-*-*-* Reset *-*-*-*-*/
#define SYSCFG_REG_RESET()	do{ (RCC->AHB2RSTR |= (1 << 14)); (RCC->AHB1RSTR &= ~(1 << 14)); }while(0)


/* =============================================================================== */
/* =========== IRQ(Interrupt Request) Numbers of STM32F446x MCU ================== */
/* =============================================================================== */

#define IRQ_WWDG					0   /* Window Watch-dog interrupt 				 	 						*/
#define IRQ_PVD						1   /* PVD through EXTI line detection interrupt 		  	 				*/
#define IRQ_TAMP_STAMP				2   /* Tamper and Time stamp interrupts through EXTI line					*/
#define IRQ_RTC_WKUP				3   /* RTC Wake-up interrupt through EXTI line				 				*/
#define IRQ_FLASH					4   /* Flash global interrupt 						  						*/
#define IRQ_RCC						5   /* RCC global interrupt 						 					 	*/
#define IRQ_EXTI0					6   /* EXTI Line0 interrupt 						  						*/
#define IRQ_EXTI1					7   /* EXTI Line1 interrupt 						  						*/
#define IRQ_EXTI2					8   /* EXTI Line2 interrupt 						  						*/
#define IRQ_EXTI3					9   /* EXTI Line3 interrupt 						  						*/
#define IRQ_EXTI4					10  /* EXTI Line4 interrupt 						 						*/
#define IRQ_DMA1_STREAM0			11  /* DMA1 Stream0 interrupt						  						*/
#define IRQ_DMA1_STREAM1			12  /* DMA1 Stream1 interrupt						  						*/
#define IRQ_DMA1_STREAM2			13  /* DMA1 Stream2 interrupt					 	  						*/
#define IRQ_DMA1_STREAM3			14  /* DMA1 Stream3 interrupt						 				 		*/
#define IRQ_DMA1_STREAM4			15  /* DMA1 Stream4 interrupt					 	  						*/
#define IRQ_DMA1_STREAM5			16  /* DMA1 Stream5 interrupt						  						*/
#define IRQ_DMA1_STREAM6			17  /* DMA1 Stream6 interrupt	 					  						*/
#define IRQ_ADC						18  /* ADC1, ADC2 and ADC3 global interrupts				 		 		*/
#define IRQ_CAN1_TX					19  /* CAN1 TX interrupts						  							*/
#define IRQ_CAN1_RX0				20  /* CAN1 RX0 interrupts						  							*/
#define IRQ_CAN1_RX1				21  /* CAN1 RX1 interrupts						  							*/
#define IRQ_CAN1_SCE				22  /* CAN1 SCE interrupts						  							*/
#define IRQ_EXIT5_9					23  /* EXTI Line[9:5] interrupts					  						*/
#define	IRQ_TIM1_BRK_TIM9			24  /* TIM1 Break interrupt and TIM9 global interrupt			  			*/
#define	IRQ_TIM1_UP_TIM10	  		25  /* TIM1 Update interrupt and TIM10 global interrupt			  			*/
#define IRQ_TIM1_TRG_COM_TIM11 		26  /* TIM1 Trigger and Commutation interrupts and TIM11 global interrupt	*/
#define IRQ_TIM1_CC					27  /* TIM1 Capture Compare interrupt					  					*/
#define IRQ_TIM2					28  /* TIM2 global interrupt						  						*/
#define IRQ_TIM3					29  /* TIM3 global interrupt						  						*/
#define IRQ_TIM4					30  /* TIM4 global interrupt						  						*/
#define IRQ_I2C1_EV					31  /* I2C1 event interrupt						  							*/
#define IRQ_I2C1_ER					32  /* I2C1 error interrupt	 					  							*/
#define IRQ_I2C2_EV					33  /* I2C2 event interrupt					  	  							*/
#define IRQ_I2C2_ER					34  /* I2C2 error interrupt						  							*/
#define IRQ_SPI1					35  /* SPI1 global interrupt						 						*/
#define IRQ_SPI2					36  /* SPI2 global interrupt						  						*/
#define IRQ_USART1					37  /* USART1 global interrupt						 	 					*/
#define IRQ_USART2					38  /* USART2 global interrupt						  						*/
#define IRQ_USART3					39  /* USART3 global interrupt						  						*/
#define IRQ_EXIT15_10				40  /* EXTI Line[15:10] interrupts					  						*/
#define IRQ_RTC_ALARM				41  /* RTC Alarms (A and B) through EXTI line interrupts		  			*/
#define IRQ_OTG_FS_WKUP				42  /* USB On-The-Go FS Wake-up through EXTI line interrupt		  			*/
#define IRQ_TIM8_BRK_TIM12			43  /* TIM8 Break interrupt and TIM12 global interrupt			  			*/
#define IRQ_TIM8_UP_TIM13			44  /* TIM8 Update and TIM13 global interrupt				 			 	*/
#define IRQ_TIM8_TRG_COM_TIM14		45  /* TIM8 Trigger and Commutation interrupts and TIM14 global interrupt 	*/
#define IRQ_TIM8_CC					46  /* TIM8 Capture Compare interrupt					 				 	*/
#define IRQ_DMA1_STREAM7			47  /* DMA1 Stream7 global interrupt					  					*/
#define IRQ_FMC						48  /* FMC global interrupt						  							*/
#define IRQ_SDIO					49  /* SDIO global interrupt						  						*/
#define IRQ_TIM5					50  /* TIM5 global interrupt						  						*/
#define IRQ_SPI3					51  /* SPI3 global interrupt						  						*/
#define IRQ_UART4					52  /* UART4 global interrupt						  						*/
#define IRQ_UART5					53  /* UART5 global interrupt						  						*/
#define IRQ_TIM6_DAC				54  /* TIM6 global interrupt, DAC1 and DAC2 under-run error interrupts	 	*/
#define IRQ_TIM7					55  /* TIM7 global interrupt						  						*/
#define IRQ_DMA2_STREAM0			56  /* DMA2 Stream0 global interrupt					 	 				*/
#define IRQ_DMA2_STREAM1			57  /* DMA2 Stream1 global interrupt					  					*/
#define IRQ_DMA2_STREAM2			58  /* DMA2 Stream2 global interrupt					  					*/
#define IRQ_DMA2_STREAM3			59  /* DMA2 Stream3 global interrupt					 					*/
#define IRQ_DMA2_STREAM4			60  /* DMA2 Stream4 global interrupt					  					*/
#define IRQ_CAN2_TX					63  /* CAN2 TX interrupts			 			  							*/
#define IRQ_CAN2_RX0				64  /* CAN2 RX0 interrupts						  							*/
#define IRQ_CAN2_RX1				65  /* CAN2 RX1 interrupts						  							*/
#define IRQ_CAN2_SCE				66  /* CAN2 SCE interrupts			  			  							*/
#define IRQ_CAN2_OTG				67  /* USB On The Go FS global interrupt				  					*/
#define IRQ_DMA2_STREAM5			68  /* DMA2 Stream5 global interrupt					  					*/
#define IRQ_DMA2_STREAM6			69  /* DMA2 Stream6 global interrupt					  					*/
#define IRQ_DMA2_STREAM7			70  /* DMA2 Stream7 global interrupt					  					*/
#define IRQ_USART6					71  /* USART6 global interrupt						  						*/
#define IRQ_I2C3_EV					72  /* I2C3 event interrupt 						  						*/
#define IRQ_I2C3_ER					73  /* I2C3 error interrupt 						  						*/
#define IRQ_OTG_HS_EP1_OUT			74  /* USB On The Go HS End Point 1 Out global interrupt		  			*/
#define IRQ_OTG_HS_EP1_IN			75  /* USB On The Go HS End Point 1 In global interrupt			  			*/
#define IRQ_OTG_HS_WKUP				76  /* USB On The Go HS Wake-up through EXTI interrupt			  			*/
#define IRQ_OTG_HS					77  /* USB On The Go HS global interrupt				  					*/
#define IRQ_DCMI					78  /* DCMI global interrupt						  						*/
#define IRQ_FPU						81  /* FPU global interrupt						 							*/
#define IRQ_SPI4					84  /* SPI4 global interrupt						  						*/
#define IRQ_SAI1					87  /* SAI1 global interrupt					    	  					*/
#define IRQ_SAI2					91  /* SAI2 global interrupt						  						*/
#define IRQ_QUADSPI					92  /* QuadSPI global interrupt						  						*/
#define IRQ_HDMI_CEC				93  /* HDMI-CEC global interrupt					  						*/
#define IRQ_SPDIF_RX				94  /* SPDIF-Rx global interrupt					  						*/
#define IRQ_FMPI2C1_EV				95  /* FMPI2C1 event interrupt						  						*/
#define IRQ_FMPI2C1_ER				96  /* FMPI2C1 error interrupt						  						*/

/* ================================================================= */
/* =========== Macros for all possible priority levels ============= */
/* ================================================================= */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

#endif /* STM32F446XX_H_ */
