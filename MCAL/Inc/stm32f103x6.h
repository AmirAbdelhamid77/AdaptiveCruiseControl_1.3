/*
 * STM32F103X8.h
 *
 *  Created on: Oct 8, 2024
 *      Author: Mohamed Abd El Hakeem El Said Ali
 */

#ifndef INC_STM32F103X6_H_
#define INC_STM32F103X6_H_




//-----------------------------
//Includes
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//-----------------------------


//-----------------------------
//Base addresses for Memories
#define FLASH_Memory_BASE     						0x08000000UL
#define System_Memory_BASE     						0x1FFFF000UL
#define SRAM_BASE              						0x20000000UL

#define Peripherals_BASE       						0x40000000UL

#define Cortex_M3_Internal_Peripherals_BASE         0xE0000000UL
//-----------------------------

//...............................
//NVIC_Adresses
#define NVIC_BASE									0xE000E100UL
#define NVIC_ISER0       				*(volatile unsigned long *)(NVIC_BASE+ 0x000)
#define NVIC_ISER1       				*(volatile unsigned long *)(NVIC_BASE+ 0x004)
#define NVIC_ISER2       				*(volatile unsigned long *)(NVIC_BASE+ 0x008)
#define NVIC_ICER0       				*(volatile unsigned long *)(NVIC_BASE+ 0x080)
#define NVIC_ICER1       				*(volatile unsigned long *)(NVIC_BASE+ 0x084)
#define NVIC_ICER2       				*(volatile unsigned long *)(NVIC_BASE+ 0x088)






//-----------------------------
//Base addresses for BUS Peripherals
/*Base addresses for AHB Peripherals*/
#define RCC_BASE                                  (Peripherals_BASE + 0x00021000UL)



/*Base addresses for APB2  Peripherals*/

//GPIO
//A, B fully included in LQFP48 Package
#define GPIOA_BASE      (Peripherals_BASE + 0x00010800UL)
#define GPIOB_BASE      (Peripherals_BASE + 0x00010C00UL)
//C, D partial included in LQFP48 Package
#define GPIOC_BASE      (Peripherals_BASE + 0x00011000UL)
#define GPIOD_BASE      (Peripherals_BASE + 0x00011400UL)
//E, P Not  included in LQFP48 Package
#define GPIOE_BASE      (Peripherals_BASE + 0x00011800UL)


//Alternative & External Interrupt
#define AFIO_BASE      (Peripherals_BASE + 0x00010000UL)
#define EXTI_BASE      (Peripherals_BASE + 0x00010400UL)

// Timer base addresses on APB2
#define TIM1_BASE      (Peripherals_BASE + 0x00012C00UL)


#define USART1_BASE             (Peripherals_BASE + 0x00013800UL)
//-----------------------------
//Base addresses for APB1 Peripherals
//-----------------------------
#define USART2_BASE             (Peripherals_BASE + 0x00004400UL)
#define USART3_BASE             (Peripherals_BASE + 0x00004800UL)

// Timer base addresses on APB1
#define TIM2_BASE      (Peripherals_BASE + 0x00000000UL)
#define TIM3_BASE      (Peripherals_BASE + 0x00000400UL)
#define TIM4_BASE      (Peripherals_BASE + 0x00000800UL)


#define SPI1_BASE              	0x40013000UL
#define SPI2_BASE         	    0x40003800UL


//I2C
#define I2C1_BASE				0x40005400UL
#define I2C2_BASE				0x40005800UL


//..........
//-----------------------------




//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register:RCC
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{

	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
}RCC_TypedDef;




//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register:GPIO
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{

	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;
}GPIO_TypedDef;


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register:AFIO
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{

	volatile uint32_t EVCR;
	volatile uint32_t MAPR;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED0;
	volatile uint32_t MAPR2;
}AFIO_TypedDef;


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register:EXTI
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{

	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_TypedDef;


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register:TIM
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{
    volatile uint32_t CR1;         // Control register 1                      (0x00)
    volatile uint32_t CR2;         // Control register 2                      (0x04)
    volatile uint32_t SMCR;        // Slave Mode Control register             (0x08)
    volatile uint32_t DIER;        // DMA/Interrupt Enable register           (0x0C)
    volatile uint32_t SR;          // Status register                         (0x10)
    volatile uint32_t EGR;         // Event Generation register               (0x14)
    volatile uint32_t CCMR1;       // Capture/Compare Mode register 1         (0x18)
    volatile uint32_t CCMR2;       // Capture/Compare Mode register 2         (0x1C)
    volatile uint32_t CCER;        // Capture/Compare Enable register         (0x20)
    volatile uint32_t CNT;         // Counter register                        (0x24)
    volatile uint32_t PSC;         // Prescaler register                      (0x28)
    volatile uint32_t ARR;         // Auto-Reload register                    (0x2C)
    volatile uint32_t RCR;         // Repetition Counter register (TIM1 only) (0x30)
    volatile uint32_t CCR1;        // Capture/Compare register 1              (0x34)
    volatile uint32_t CCR2;        // Capture/Compare register 2              (0x38)
    volatile uint32_t CCR3;        // Capture/Compare register 3              (0x3C)
    volatile uint32_t CCR4;        // Capture/Compare register 4              (0x40)
    volatile uint32_t BDTR;        // Break and Dead-Time register (TIM1)     (0x44)
    volatile uint32_t DCR;         // DMA Control register                    (0x48)
    volatile uint32_t DMAR;        // DMA Address for burst mode              (0x4C)
} TIM_TypeDef;


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: USART
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
} USART_TypeDef;



typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_TypeDef;



//========================================================
//		    Peripheral Registers : I2C
//========================================================

typedef struct
{
	volatile uint32_t CR1 ;					//Control Register 	1				(0x00)
	volatile uint32_t CR2 ;					//Control Register 	2				(0x04)
	volatile uint32_t OAR1 ;				//Own Address Register	1			(0x08)
	volatile uint32_t OAR2 ;				//Own Address Register	2			(0x0C)
	volatile uint32_t DR ;					//Data Register						(0x10)
	volatile uint32_t SR1 ;					//Status Register 1					(0x14)
	volatile uint32_t SR2 ;					//Status Register 2					(0x18)
	volatile uint32_t CCR ;					//Clock Control Register			(0x1C)
	volatile uint32_t TRISE ;				//Rise Time Register				(0x20)
}I2C_TypeDef;


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:GPIO
//-*-*-*-*-*-*-*-*-*-*-*

#define GPIOA               ((GPIO_TypedDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypedDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypedDef *)GPIOC_BASE)
#define GPIOD               ((GPIO_TypedDef *)GPIOD_BASE)
#define GPIOE               ((GPIO_TypedDef *)GPIOE_BASE)


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:RCC
//-*-*-*-*-*-*-*-*-*-*-*

#define RCC               ((RCC_TypedDef *)RCC_BASE)


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:AFIO
//-*-*-*-*-*-*-*-*-*-*-*

#define AFIO              ((AFIO_TypedDef *)AFIO_BASE)


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:EXTI
//-*-*-*-*-*-*-*-*-*-*-*

#define EXTI              ((EXTI_TypedDef *)EXTI_BASE)

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:TIM
//-*-*-*-*-*-*-*-*-*-*-*
#define TIM1               ((TIM_TypeDef *)TIM1_BASE)
#define TIM2               ((TIM_TypeDef *)TIM2_BASE)
#define TIM3               ((TIM_TypeDef *)TIM3_BASE)
#define TIM4               ((TIM_TypeDef *)TIM4_BASE)

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:USART
//-*-*-*-*-*-*-*-*-*-*-*

#define USART1              ((USART_TypeDef *)USART1_BASE)
#define USART2              ((USART_TypeDef *)USART2_BASE)
#define USART3              ((USART_TypeDef *)USART3_BASE)

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:SPI
//-*-*-*-*-*-*-*-*-*-*-*

#define SPI1               ((SPI_TypeDef*)SPI1_BASE)
#define SPI2               ((SPI_TypeDef*)SPI2_BASE)

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:I2C
//-*-*-*-*-*-*-*-*-*-*-*

#define I2C1               ((I2C_TypeDef*)I2C1_BASE)
#define I2C2               ((I2C_TypeDef*)I2C2_BASE)


//=====================================================================
//					Clock Enable Macros
//=====================================================================

//Enable GPIOA Clock
#define RCC_GPIOA_CLK_EN()    (RCC->APB2ENR |=1 << 2)
//Enable GPIOB Clock
#define RCC_GPIOB_CLK_EN()    (RCC->APB2ENR |=1 << 3)
//Enable GPIOC Clock
#define RCC_GPIOC_CLK_EN()    (RCC->APB2ENR |=1 << 4)
//Enable GPIOD Clock
#define RCC_GPIOD_CLK_EN()    (RCC->APB2ENR |=1 << 5)
//Enable GPIOE Clock
#define RCC_GPIOE_CLK_EN()    (RCC->APB2ENR |=1 << 6)
//Enable AFIO Clock
#define RCC_AFIO_CLK_EN()    (RCC->APB2ENR |=1 << 0)

// Enable timer clock macros (more clearly organized)
#define RCC_TIM1_CLK_EN()    (RCC->APB2ENR |= 1 << 11)  // TIM1 on APB2 bus
#define RCC_TIM2_CLK_EN()    (RCC->APB1ENR |= 1 << 0)   // TIM2 on APB1 bus
#define RCC_TIM3_CLK_EN()    (RCC->APB1ENR |= 1 << 1)   // TIM3 on APB1 bus
#define RCC_TIM4_CLK_EN()    (RCC->APB1ENR |= 1 << 2)   // TIM4 on APB1 bus

//Enable USART1 Clock
#define RCC_USART1_CLK_EN()    (RCC->APB2ENR |=1 << 14)
//Enable USART2 Clock
#define RCC_USART2_CLK_EN()    (RCC->APB1ENR |=1 << 17)
//Enable USART3 Clock
#define RCC_USART3_CLK_EN()    (RCC->APB1ENR |=1 << 18)

//Enable SPI1 Clock
#define RCC_SPI1_CLK_EN()    (RCC->APB2ENR |=1 << 12)
//Enable SPI2 Clock
#define RCC_SPI2_CLK_EN()    (RCC->APB1ENR |=1 << 14)

//Enable I2C1 Clock
#define RCC_I2C1_CLK_EN()    (RCC->APB1ENR |=1 << 21)
//Enable I2C2 Clock
#define RCC_I2C2_CLK_EN()    (RCC->APB1ENR |=1 << 22)

//======================================================================================


//===================================
//		Reset Macros :
//===================================


//===========RESET GPIO==============
//Reset GPIOA
#define RCC_GPIOA_CLK_Reset()    (RCC->APB2RSTR |=1 << 2)
//Reset GPIOB
#define RCC_GPIOB_CLK_Reset()    (RCC->APB2RSTR |=1 << 3)
//Reset GPIOC
#define RCC_GPIOC_CLK_Reset()    (RCC->APB2RSTR |=1 << 4)
//Reset GPIOD
#define RCC_GPIOD_CLK_Reset()    (RCC->APB2RSTR |=1 << 5)
//Reset GPIOE
#define RCC_GPIOE_CLK_Reset()    (RCC->APB2RSTR |=1 << 6)

//===========RESET AFIO==============
//Reset AFIO
#define RCC_AFIO_CLK_Reset()    (RCC->APB2RSTR |=1 << 0)

//===========RESET USART==============
//Reset USART1
#define RCC_USART1_CLK_Reset()    (RCC->APB2RSTR |=1 << 14)
//Reset USART2
#define RCC_USART2_CLK_Reset()    (RCC->APB1RSTR |=1 << 17)
//Reset USART3
#define RCC_USART3_CLK_Reset()    (RCC->APB1RSTR |=1 << 18)

//===========RESET SPI==============
//Reset SPI1
#define RCC_SPI1_CLK_Reset()    (RCC->APB2RSTR |=1 << 12)
//Reset SPI2
#define RCC_SPI2_CLK_Reset()    (RCC->APB1RSTR |=1 << 14)

//===========RESET I2C==============
//Reset I2C1
#define RCC_I2C1_CLK_Reset()    (RCC->APB1RSTR |=1 << 21)
//Reset I2C2
#define RCC_I2C2_CLK_Reset()    (RCC->APB1RSTR |=1 << 22)

//===========RESET TIM==============
// Reset timer macros (more clearly organized)
#define RCC_TIM1_CLK_Reset()    (RCC->APB2RSTR |= 1 << 11)
#define RCC_TIM2_CLK_Reset()    (RCC->APB1RSTR |= 1 << 0)
#define RCC_TIM3_CLK_Reset()    (RCC->APB1RSTR |= 1 << 1)
#define RCC_TIM4_CLK_Reset()    (RCC->APB1RSTR |= 1 << 2)

//======================================================================================


//NVIC IRQ Enable/Disable Macros
//Enable Interrupt Reguest
#define NVIC_IRQ6_EXTI0_Enable			(NVIC_ISER0 |= 1<<6)
#define NVIC_IRQ7_EXTI1_Enable			(NVIC_ISER0 |= 1<<7)
#define NVIC_IRQ8_EXTI2_Enable			(NVIC_ISER0 |= 1<<8)
#define NVIC_IRQ9_EXTI3_Enable			(NVIC_ISER0 |= 1<<9)
#define NVIC_IRQ10_EXTI4_Enable			(NVIC_ISER0 |= 1<<10)
#define NVIC_IRQ23_EXTI5_9_Enable		(NVIC_ISER0 |= 1<<23)
#define NVIC_IRQ40_EXTI10_15_Enable		(NVIC_ISER1 |= 1<<8)//40-32 = 8

//Disable Interrupt Reguest
#define NVIC_IRQ6_EXTI0_Disable			(NVIC_ICER0 |= 1<<6)
#define NVIC_IRQ7_EXTI1_Disable			(NVIC_ICER0 |= 1<<7)
#define NVIC_IRQ8_EXTI2_Disable			(NVIC_ICER0 |= 1<<8)
#define NVIC_IRQ9_EXTI3_Disable			(NVIC_ICER0 |= 1<<9)
#define NVIC_IRQ10_EXTI4_Disable		(NVIC_ICER0 |= 1<<10)
#define NVIC_IRQ23_EXTI5_9_Disable		(NVIC_ICER0 |= 1<<23)
#define NVIC_IRQ40_EXTI10_15_Disable	(NVIC_ICER1 |= 1<<8)//40-32 = 8

// NVIC IRQ for TIM (more clearly organized)
#define NVIC_IRQ25_TIM1_UP_Enable       (NVIC_ISER0 |= 1 << 25)
#define NVIC_IRQ26_TIM1_CC_Enable       (NVIC_ISER0 |= 1 << 26)
#define NVIC_IRQ28_TIM2_Enable          (NVIC_ISER0 |= 1 << 28)
#define NVIC_IRQ29_TIM3_Enable          (NVIC_ISER0 |= 1 << 29)
#define NVIC_IRQ30_TIM4_Enable          (NVIC_ISER0 |= 1 << 30)

#define NVIC_IRQ25_TIM1_UP_Disable      (NVIC_ICER0 |= 1 << 25)
#define NVIC_IRQ26_TIM1_CC_Disable      (NVIC_ICER0 |= 1 << 26)
#define NVIC_IRQ28_TIM2_Disable         (NVIC_ICER0 |= 1 << 28)
#define NVIC_IRQ29_TIM3_Disable         (NVIC_ICER0 |= 1 << 29)
#define NVIC_IRQ30_TIM4_Disable         (NVIC_ICER0 |= 1 << 30)

//---------------------------------------------------------------------------------
// IVT:
//---------------------------------------------------------------------------------
//EXTI
#define 	EXTI0_IRQ		6
#define 	EXTI1_IRQ		7
#define 	EXTI2_IRQ		8
#define 	EXTI3_IRQ		9
#define 	EXTI4_IRQ		10
#define 	EXTI5_IRQ		23
#define 	EXTI6_IRQ		23
#define 	EXTI7_IRQ		23
#define 	EXTI8_IRQ		23
#define 	EXTI9_IRQ		23
#define 	EXTI10_IRQ		40
#define 	EXTI11_IRQ		40
#define 	EXTI12_IRQ		40
#define 	EXTI13_IRQ		40
#define 	EXTI14_IRQ		40
#define 	EXTI15_IRQ		40

//USART
#define		USART1_IRQ		37
#define		USART2_IRQ		38
#define		USART3_IRQ		39

//SPI
#define		SPI1_IRQ		35
#define		SPI2_IRQ		36

//I2C
#define		I2C1_EV_IRQ		31
#define		I2C1_ER_IRQ		32
#define		I2C2_EV_IRQ		33
#define		I2C2_ER_IRQ		34

// Timer interrupt vector table positions
#define TIM1_UP_IRQ     25
#define TIM1_CC_IRQ     26
#define TIM2_IRQ        28
#define TIM3_IRQ        29
#define TIM4_IRQ        30

//-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-
//======================================================================================
//                              RCC Bit Definitions
//======================================================================================
//-----------------------------
// Clock Control Register (RCC_CR) bit definitions
//-----------------------------
#define RCC_CR_HSION                      (1U << 0)  // Internal High-Speed clock enable
#define RCC_CR_HSIRDY                     (1U << 1)  // Internal High-Speed clock ready flag
#define RCC_CR_HSITRIM_Pos                (3U)
#define RCC_CR_HSITRIM_Msk                (0x1FUL << RCC_CR_HSITRIM_Pos) // 0x000000F8
#define RCC_CR_HSITRIM                    RCC_CR_HSITRIM_Msk
#define RCC_CR_HSICAL_Pos                 (8U)
#define RCC_CR_HSICAL_Msk                 (0xFFUL << RCC_CR_HSICAL_Pos) // 0x0000FF00
#define RCC_CR_HSICAL                     RCC_CR_HSICAL_Msk
#define RCC_CR_HSEON                      (1U << 16) // External High-Speed clock enable
#define RCC_CR_HSERDY                     (1U << 17) // External High-Speed clock ready flag
#define RCC_CR_HSEBYP                     (1U << 18) // External High-Speed clock bypass
#define RCC_CR_CSSON                      (1U << 19) // Clock Security System enable
#define RCC_CR_PLLON                      (1U << 24) // PLL enable
#define RCC_CR_PLLRDY                     (1U << 25) // PLL ready flag

//-----------------------------
// Clock Configuration Register (RCC_CFGR) bit definitions
//-----------------------------
#define RCC_CFGR_SW_Pos                   (0U)
#define RCC_CFGR_SW_Msk                   (0x3UL << RCC_CFGR_SW_Pos) // 0x00000003
#define RCC_CFGR_SW                       RCC_CFGR_SW_Msk
#define RCC_CFGR_SW_HSI                   (0x0UL << RCC_CFGR_SW_Pos) // HSI selected as system clock
#define RCC_CFGR_SW_HSE                   (0x1UL << RCC_CFGR_SW_Pos) // HSE selected as system clock
#define RCC_CFGR_SW_PLL                   (0x2UL << RCC_CFGR_SW_Pos) // PLL selected as system clock

#define RCC_CFGR_SWS_Pos                  (2U)
#define RCC_CFGR_SWS_Msk                  (0x3UL << RCC_CFGR_SWS_Pos) // 0x0000000C
#define RCC_CFGR_SWS                      RCC_CFGR_SWS_Msk
#define RCC_CFGR_SWS_HSI                  (0x0UL << RCC_CFGR_SWS_Pos) // HSI used as system clock
#define RCC_CFGR_SWS_HSE                  (0x1UL << RCC_CFGR_SWS_Pos) // HSE used as system clock
#define RCC_CFGR_SWS_PLL                  (0x2UL << RCC_CFGR_SWS_Pos) // PLL used as system clock

#define RCC_CFGR_HPRE_Pos                 (4U)
#define RCC_CFGR_HPRE_Msk                 (0xFUL << RCC_CFGR_HPRE_Pos) // 0x000000F0
#define RCC_CFGR_HPRE                     RCC_CFGR_HPRE_Msk          // AHB prescaler

#define RCC_CFGR_PPRE1_Pos                (8U)
#define RCC_CFGR_PPRE1_Msk                (0x7UL << RCC_CFGR_PPRE1_Pos) // 0x00000700
#define RCC_CFGR_PPRE1                    RCC_CFGR_PPRE1_Msk         // APB1 prescaler

#define RCC_CFGR_PPRE2_Pos                (11U)
#define RCC_CFGR_PPRE2_Msk                (0x7UL << RCC_CFGR_PPRE2_Pos) // 0x00003800
#define RCC_CFGR_PPRE2                    RCC_CFGR_PPRE2_Msk         // APB2 prescaler

#define RCC_CFGR_PLLSRC                   (1U << 16) // PLL entry clock source
#define RCC_CFGR_PLLXTPRE                 (1U << 17) // HSE divider for PLL entry
#define RCC_CFGR_PLLMULL_Pos              (18U)
#define RCC_CFGR_PLLMULL_Msk              (0xFUL << RCC_CFGR_PLLMULL_Pos) // 0x003C0000
#define RCC_CFGR_PLLMULL                  RCC_CFGR_PLLMULL_Msk      // PLL multiplication factor

//-----------------------------
// APB2 Peripheral Clock Enable Register (RCC_APB2ENR) bit definitions
//-----------------------------
#define RCC_APB2ENR_AFIOEN                (1U << 0)  // Alternate function I/O enable
#define RCC_APB2ENR_IOPAEN                (1U << 2)  // I/O port A enable
#define RCC_APB2ENR_IOPBEN                (1U << 3)  // I/O port B enable
#define RCC_APB2ENR_IOPCEN                (1U << 4)  // I/O port C enable
#define RCC_APB2ENR_IOPDEN                (1U << 5)  // I/O port D enable
#define RCC_APB2ENR_IOPEEN                (1U << 6)  // I/O port E enable
#define RCC_APB2ENR_TIM1EN                (1U << 11) // TIM1 timer enable
#define RCC_APB2ENR_SPI1EN                (1U << 12) // SPI1 enable
#define RCC_APB2ENR_USART1EN              (1U << 14) // USART1 enable

//-----------------------------
// APB1 Peripheral Clock Enable Register (RCC_APB1ENR) bit definitions
//-----------------------------
#define RCC_APB1ENR_TIM2EN                (1U << 0)  // TIM2 timer enable
#define RCC_APB1ENR_TIM3EN                (1U << 1)  // TIM3 timer enable
#define RCC_APB1ENR_TIM4EN                (1U << 2)  // TIM4 timer enable
#define RCC_APB1ENR_WWDGEN                (1U << 11) // Window watchdog enable
#define RCC_APB1ENR_SPI2EN                (1U << 14) // SPI2 enable
#define RCC_APB1ENR_USART2EN              (1U << 17) // USART2 enable
#define RCC_APB1ENR_USART3EN              (1U << 18) // USART3 enable
#define RCC_APB1ENR_I2C1EN                (1U << 21) // I2C1 enable
#define RCC_APB1ENR_I2C2EN                (1U << 22) // I2C2 enable
#define RCC_APB1ENR_PWREN                 (1U << 28) // Power interface enable

//-----------------------------
// Flash Access Control Register (FLASH_ACR) bit definitions
//-----------------------------
#define FLASH_ACR_LATENCY_Pos             (0U)
#define FLASH_ACR_LATENCY_Msk             (0x7UL << FLASH_ACR_LATENCY_Pos) // 0x00000007
#define FLASH_ACR_LATENCY                 FLASH_ACR_LATENCY_Msk
#define FLASH_ACR_LATENCY_0               0x00000000U // Zero wait state
#define FLASH_ACR_LATENCY_1               0x00000001U // One wait state
#define FLASH_ACR_LATENCY_2               0x00000002U // Two wait states
#define FLASH_ACR_HLFCYA                  (1U << 3)  // Flash half cycle access enable
#define FLASH_ACR_PRFTBE                  (1U << 4)  // Prefetch buffer enable
#define FLASH_ACR_PRFTBS                  (1U << 5)  // Prefetch buffer status

//-----------------------------
// Flash Peripheral Base Address & Structure
//-----------------------------
#define AHBPERIPH_BASE        (Peripherals_BASE + 0x00020000U)
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x00002000)

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
} FLASH_TypeDef;

#define FLASH                 ((FLASH_TypeDef *) FLASH_R_BASE)

//-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-
//======================================================================================
//                              Generic Macros:
//======================================================================================
//-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-
#define NULL                             ((void *)0)
//-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-

//GPIO Pin Configure
//@ref GPIO_MODE_define


//Memory mapping of the different bus matrix slaves
typedef enum{
	FLASH_MEMORY,
	CAN2_PERIPH = 0X4,
	CAN1_PERIPH,
	USB_PERIPH,
	DMA2_PERIPH = 0XA,
	DMA1_PERIPH,
	SRAM_PERIPH = 0xF,
	FSMC_PERIPH = 0x11,
	Reserved1 = 0X13,
	NAND_FLASH = 0X15,
	Reserved2 = 0X17,
	CRC_PERIPH = 0X19,
	Reserved3 = 0X1B,
	Reserved4 = 0X1D,
	RESERVED5 = 0X1F,
	FLITF_PERIPH = 0X21,
	Reserved6 = 0X23,
	Reserved7 = 0X25,
	TIM11_PERIPH = 0X27,
	TIM10_PERIPH,
	TIM9_PERIPH,
	ADC3_PERIPH,
	USART1_PERIPH,
	TIM8_PERIPH,
	SPI1_PERIPH,
	TIM1_PERIPH,
	ADC2_PERIPH,
	ADC1_PERIPH,
	GPIOG_PERIPH,
	GPIOF_PERIPH,
	GPIOE_PERIPH,
	GPIOD_PERIPH,
	GPIOC_PERIPH,
	GPIOB_PERIPH,
	GPIOA_PERIPH,
	EXTI_PERIPH,
	AFIO_PERIPH,
	SysSFG = 0X45,
	Reserved8,
	TIM7_PERIPH = 0X4B,
	TIM6_PERIPH,
	TIM5_PERIPH,
	TIM4_PERIPH,
	TIM3_PERIPH,
	TIM2_PERIPH,
	WWDG = 0X53,
	IWDG = 0X55,
	WINDOW_WATCHDOG = 0X57,
	Reserved9,
	SPI3_PERIPH,
	SPI2_PERIPH,
	Reserved10,
	UART5_PERIPH,
	UART4_PERIPH,
	USART3_PERIPH,
	USART2_PERIPH,
	I2C2_PERIPH,
	I2C1_PERIPH,
	USB_FS_REG = 0x65,
	USB_FS_CAN_SRAM,
	bxCAN,
	bxCAN_STATIC,
	PWR,
	DAC,
	Reserved11 = 0X6D,
	RTC = 0X6F,
	RESERVED12 = 0X71,
	BKP = 0X73
}memory_mapping_slaves_t;

#endif /* INC_STM32F103X6_H_ */
