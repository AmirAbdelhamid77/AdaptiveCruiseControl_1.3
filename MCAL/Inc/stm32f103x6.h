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



//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*





//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:
#define RCC     			 ((RCC_TypedDef*) RCC_BASE    )


#define GPIOA                ((GPIO_TypedDef*) GPIOA_BASE )
#define GPIOB                ((GPIO_TypedDef*) GPIOB_BASE )
#define GPIOC                ((GPIO_TypedDef*) GPIOC_BASE )
#define GPIOD                ((GPIO_TypedDef*) GPIOD_BASE )
#define GPIOE                ((GPIO_TypedDef*) GPIOE_BASE )

#define AFIO                 ((AFIO_TypedDef*) AFIO_BASE  )
#define EXTI                 ((EXTI_TypedDef*) EXTI_BASE  )
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:TIM
//-*-*-*-*-*-*-*-*-*-*-*
#define TIM1               ((TIM_TypeDef *)TIM1_BASE)
#define TIM2               ((TIM_TypeDef *)TIM2_BASE)
#define TIM3               ((TIM_TypeDef *)TIM3_BASE)
#define TIM4               ((TIM_TypeDef *)TIM4_BASE)



#define USART1                ((USART_TypeDef *)USART1_BASE)
#define USART2                ((USART_TypeDef *)USART2_BASE)
#define USART3                ((USART_TypeDef *)USART3_BASE)


#define SPI1				  ((SPI_TypeDef *)SPI1_BASE)
#define SPI2				  ((SPI_TypeDef *)SPI2_BASE)

#define I2C1				  ((I2C_TypeDef *)I2C1_BASE)
#define I2C2				  ((I2C_TypeDef *)I2C2_BASE)

//-*-*-*-*-*-*-*-*-*-*-*



//-*-*-*-*-*-*-*-*-*-*-*-
//clock enable Macros:RCC_GPIO
#define GPIOA_CLOCK_ENABLE     (RCC->APB2ENR |= (1 << 2)  )
#define GPIOB_CLOCK_ENABLE     (RCC->APB2ENR |= (1 << 3)  )
#define GPIOC_CLOCK_ENABLE     (RCC->APB2ENR |= (1 << 4)  )
#define GPIOD_CLOCK_ENABLE     (RCC->APB2ENR |= (1 << 5)  )

//clock disable Macros:RCC_GPIO
#define GPIOA_CLOCK_DISABLE    (RCC->APB2ENR &= ~(1 << 2) )
#define GPIOB_CLOCK_DISABLE    (RCC->APB2ENR &= ~(1 << 3) )
#define GPIOC_CLOCK_DISABLE    (RCC->APB2ENR &= ~(1 << 4) )
#define GPIOD_CLOCK_DISABLE    (RCC->APB2ENR &= ~(1 << 5) )
// Enable timer clock macros (more clearly organized)
#define RCC_TIM1_CLK_EN()    (RCC->APB2ENR |= 1 << 11)  // TIM1 on APB2 bus
#define RCC_TIM2_CLK_EN()    (RCC->APB1ENR |= 1 << 0)   // TIM2 on APB1 bus
#define RCC_TIM3_CLK_EN()    (RCC->APB1ENR |= 1 << 1)   // TIM3 on APB1 bus
#define RCC_TIM4_CLK_EN()    (RCC->APB1ENR |= 1 << 2)   // TIM4 on APB1 bus

//===========RESET TIM==============
// Reset timer macros (more clearly organized)
#define RCC_TIM1_CLK_Reset()    (RCC->APB2RSTR |= 1 << 11)
#define RCC_TIM2_CLK_Reset()    (RCC->APB1RSTR |= 1 << 0)
#define RCC_TIM3_CLK_Reset()    (RCC->APB1RSTR |= 1 << 1)
#define RCC_TIM4_CLK_Reset()    (RCC->APB1RSTR |= 1 << 2)
//clock enable Macros:RCC_AFIO
#define AFIO_CLOCK_ENABLE      (RCC->APB2ENR |=  (1 << 0) )

//clock disable Macros:RCC_AFIO
#define AFIO_CLOCK_DISABLE     (RCC->APB2ENR &= ~(1 << 0) )


//Clock enable  Macros:USARTx.....
#define RCC_USART1_CLK_EN()	( RCC->APB2ENR |= (1<<14) )
#define RCC_USART2_CLK_EN()	( RCC->APB1ENR |= (1<<17) )
#define RCC_USART3_CLK_EN()	( RCC->APB1ENR |= (1<<18) )


//Clock Reset  Macros:USARTx.....
#define RCC_USART1_Reset()	( RCC->APB2RSTR |= (1<<14) )
#define RCC_USART2_Reset()	( RCC->APB1RSTR |= (1<<17) )
#define RCC_USART3_Reset()	( RCC->APB1RSTR |= (1<<18) )



//Clock enable  Macros:SPIx.....
#define RCC_SPI1_CLK_EN()	(RCC->APB2ENR |= 1 << 12)
#define RCC_SPI2_CLK_EN()	(RCC->APB1ENR |= 1 << 14)

//Clock disable  Macros:SPIx.....
#define RCC_SPI1_CLK_DI()	(RCC->APB2ENR &= ~(1 << 12))
#define RCC_SPI2_CLK_DI()	(RCC->APB1ENR &= ~(1 << 14))

//Clock reset  Macros:SPIx.....
#define RCC_SPI1_RESET()	(RCC->APB2RSTR |= 1 << 12)
#define RCC_SPI2_RESET()	(RCC->APB1RSTR |= 1 << 14)


//Enable I2C Clock
#define RCC_I2C1_CLK_EN()		(RCC->APB1ENR |= 1<<21)
#define RCC_I2C2_CLK_EN()		(RCC->APB1ENR |= 1<<22)

//Disable I2C Clock
#define RCC_I2C1_CLK_RESET()	(RCC->APB1RSTR |= 1<<21)
#define RCC_I2C2_CLK_RESET()	(RCC->APB1RSTR |= 1<<22)


//-*-*-*-*-*-*-*-*-*-*-*





//-*-*-*-*-*-*-*-*-*-*-*-
//Generic Macros:
//-*-*-*-*-*-*-*-*-*-*-*
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
#define TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
#define TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
#define TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
#define TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
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

//NVIC_EXTI_IRQ_ENABLE_Define
#define NVIC_IRQ6_EXTI0_ENABLE     		       (NVIC_ISER0 |= (1 << 6))
#define NVIC_IRQ7_EXTI1_ENABLE    		       (NVIC_ISER0 |= (1 << 7))
#define NVIC_IRQ8_EXTI2_ENABLE     		       (NVIC_ISER0 |= (1 << 8))
#define NVIC_IRQ9_EXTI3_ENABLE     		       (NVIC_ISER0 |= (1 << 9))
#define NVIC_IRQ10_EXTI4_ENABLE     	       (NVIC_ISER0 |= (1 << 10))
#define NVIC_IRQ23_EXTI5_9_ENABLE    	       (NVIC_ISER0 |= (1 << 23))
#define NVIC_IRQ40_EXTI10_15_ENABLE    	       (NVIC_ISER1 |= (1 << 8)) //40-32=8

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



//NVIC_IRQ_USART1
//NVIC_EXTI_IRQ_DISABLE_Define
#define NVIC_IRQ6_EXTI0_DISABLE                (NVIC_ISER0 &=  ~(1 << 6))
#define NVIC_IRQ7_EXTI1_DISABLE    		       (NVIC_ISER0 &=  ~(1 << 7))
#define NVIC_IRQ8_EXTI2_DISABLE     		   (NVIC_ISER0 &=  ~(1 << 8))
#define NVIC_IRQ9_EXTI3_DISABLE     		   (NVIC_ISER0 &=  ~(1 << 9))
#define NVIC_IRQ10_EXTI4_DISABLE     	 	   (NVIC_ISER0 &=  ~(1 << 10))
#define NVIC_IRQ23_EXTI5_9_DISABLE    	 	   (NVIC_ISER0 &=  ~(1 << 23))
#define NVIC_IRQ40_EXTI10_15_DISABLE    	   (NVIC_ISER1 &=  ~(1 << 8)) //40-32=8




//NVIC_USART_IRQ_ENABLE_Define
#define NVIC_IRQ37_USART1_Enable          	(NVIC_ISER1 |= 1<<( USART1_IRQ - 32 )) //IRQ-32
#define NVIC_IRQ38_USART2_Enable   	        (NVIC_ISER1 |= 1<<( USART2_IRQ - 32 )) //IRQ-32
#define NVIC_IRQ39_USART3_Enable   			(NVIC_ISER1 |= 1<<( USART3_IRQ - 32 )) //IRQ-32

//NVIC_USART_IRQ_DISABLE_Define
#define NVIC_IRQ37_USART1_Disable   		(NVIC_ICER1 |= 1<<( USART1_IRQ- 32 )) //IRQ-32
#define NVIC_IRQ38_USART2_Disable   		(NVIC_ICER1 |= 1<<( USART2_IRQ- 32 )) //IRQ-32
#define NVIC_IRQ39_USART3_Disable   		(NVIC_ICER1 |= 1<<( USART3_IRQ- 32 )) //IRQ-32


//NVIC_SPIx_IRQ_ENABLE_Define
#define NVIC_IRQ35_SPI1_Enable()			(NVIC_ISER1 |= 1<<(SPI1_IRQ - 32))	// SPI1
#define NVIC_IRQ36_SPI2_Enable()			(NVIC_ISER1 |= 1<<(SPI2_IRQ - 32))	// SPI2


//NVIC_SPIx_IRQ__Disable_Define
#define NVIC_IRQ35_SPI1_Disable()			(NVIC_ICER1 |= 1<<(SPI1_IRQ - 32))	// SPI1
#define NVIC_IRQ36_SPI2_Disable()			(NVIC_ICER1 |= 1<<(SPI2_IRQ - 32))	// SPI2


//NVIC_I2Cx_IRQ_Enable__Define
#define NVIC_IRQ31_I2C1_EV_Enable()			(NVIC_ISER0 |= 1<<(I2C1_EV_IRQ))
#define NVIC_IRQ32_I2C1_ER_Enable()			(NVIC_ISER1 |= 1<<(I2C1_ER_IRQ - 32))
#define NVIC_IRQ33_I2C2_EV_Enable()			(NVIC_ISER1 |= 1<<(I2C2_EV_IRQ - 32))
#define NVIC_IRQ34_I2C2_ER_Enable()			(NVIC_ISER1 |= 1<<(I2C2_ER_IRQ - 32))

//NVIC_I2Cx_IRQ_Disable__Define
#define NVIC_IRQ31_I2C1_EV_Disable()		(NVIC_ICER0 |= 1<<(I2C1_EV_IRQ))
#define NVIC_IRQ32_I2C1_ER_Disable()		(NVIC_ICER1 |= 1<<(I2C1_ER_IRQ - 32))
#define NVIC_IRQ33_I2C2_EV_Disable()		(NVIC_ICER1 |= 1<<(I2C2_EV_IRQ - 32))
#define NVIC_IRQ34_I2C2_ER_Disable()		(NVIC_ICER1 |= 1<<(I2C2_ER_IRQ - 32))






//========================================================
//========================================================
//		  			I2C Bit Definition
//========================================================
//========================================================


/*******************  Bit definition for I2C_CR1 register  ********************/
#define I2C_CR1_PE_Pos                      (0U)
#define I2C_CR1_PE_Msk                      (0x1UL << I2C_CR1_PE_Pos)           /*!< 0x00000001 */
#define I2C_CR1_PE                          I2C_CR1_PE_Msk                     /*!< Peripheral Enable */
#define I2C_CR1_SMBUS_Pos                   (1U)
#define I2C_CR1_SMBUS_Msk                   (0x1UL << I2C_CR1_SMBUS_Pos)        /*!< 0x00000002 */
#define I2C_CR1_SMBUS                       I2C_CR1_SMBUS_Msk                  /*!< SMBus Mode */
#define I2C_CR1_SMBTYPE_Pos                 (3U)
#define I2C_CR1_SMBTYPE_Msk                 (0x1UL << I2C_CR1_SMBTYPE_Pos)      /*!< 0x00000008 */
#define I2C_CR1_SMBTYPE                     I2C_CR1_SMBTYPE_Msk                /*!< SMBus Type */
#define I2C_CR1_ENARP_Pos                   (4U)
#define I2C_CR1_ENARP_Msk                   (0x1UL << I2C_CR1_ENARP_Pos)        /*!< 0x00000010 */
#define I2C_CR1_ENARP                       I2C_CR1_ENARP_Msk                  /*!< ARP Enable */
#define I2C_CR1_ENPEC_Pos                   (5U)
#define I2C_CR1_ENPEC_Msk                   (0x1UL << I2C_CR1_ENPEC_Pos)        /*!< 0x00000020 */
#define I2C_CR1_ENPEC                       I2C_CR1_ENPEC_Msk                  /*!< PEC Enable */
#define I2C_CR1_ENGC_Pos                    (6U)
#define I2C_CR1_ENGC_Msk                    (0x1UL << I2C_CR1_ENGC_Pos)         /*!< 0x00000040 */
#define I2C_CR1_ENGC                        I2C_CR1_ENGC_Msk                   /*!< General Call Enable */
#define I2C_CR1_NOSTRETCH_Pos               (7U)
#define I2C_CR1_NOSTRETCH_Msk               (0x1UL << I2C_CR1_NOSTRETCH_Pos)    /*!< 0x00000080 */
#define I2C_CR1_NOSTRETCH                   I2C_CR1_NOSTRETCH_Msk              /*!< Clock Stretching Disable (Slave mode) */
#define I2C_CR1_START_Pos                   (8U)
#define I2C_CR1_START_Msk                   (0x1UL << I2C_CR1_START_Pos)        /*!< 0x00000100 */
#define I2C_CR1_START                       I2C_CR1_START_Msk                  /*!< Start Generation */
#define I2C_CR1_STOP_Pos                    (9U)
#define I2C_CR1_STOP_Msk                    (0x1UL << I2C_CR1_STOP_Pos)         /*!< 0x00000200 */
#define I2C_CR1_STOP                        I2C_CR1_STOP_Msk                   /*!< Stop Generation */
#define I2C_CR1_ACK_Pos                     (10U)
#define I2C_CR1_ACK_Msk                     (0x1UL << I2C_CR1_ACK_Pos)          /*!< 0x00000400 */
#define I2C_CR1_ACK                         I2C_CR1_ACK_Msk                    /*!< Acknowledge Enable */
#define I2C_CR1_POS_Pos                     (11U)
#define I2C_CR1_POS_Msk                     (0x1UL << I2C_CR1_POS_Pos)          /*!< 0x00000800 */
#define I2C_CR1_POS                         I2C_CR1_POS_Msk                    /*!< Acknowledge/PEC Position (for data reception) */
#define I2C_CR1_PEC_Pos                     (12U)
#define I2C_CR1_PEC_Msk                     (0x1UL << I2C_CR1_PEC_Pos)          /*!< 0x00001000 */
#define I2C_CR1_PEC                         I2C_CR1_PEC_Msk                    /*!< Packet Error Checking */
#define I2C_CR1_ALERT_Pos                   (13U)
#define I2C_CR1_ALERT_Msk                   (0x1UL << I2C_CR1_ALERT_Pos)        /*!< 0x00002000 */
#define I2C_CR1_ALERT                       I2C_CR1_ALERT_Msk                  /*!< SMBus Alert */
#define I2C_CR1_SWRST_Pos                   (15U)
#define I2C_CR1_SWRST_Msk                   (0x1UL << I2C_CR1_SWRST_Pos)        /*!< 0x00008000 */
#define I2C_CR1_SWRST                       I2C_CR1_SWRST_Msk                  /*!< Software Reset */
/*******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_FREQ_Pos                    (0U)
#define I2C_CR2_FREQ_Msk                    (0x3FUL << I2C_CR2_FREQ_Pos)        /*!< 0x0000003F */
#define I2C_CR2_FREQ                        I2C_CR2_FREQ_Msk                   /*!< FREQ[5:0] bits (Peripheral Clock Frequency) */
#define I2C_CR2_ITERREN_Pos                 (8U)
#define I2C_CR2_ITERREN_Msk                 (0x1UL << I2C_CR2_ITERREN_Pos)      /*!< 0x00000100 */
#define I2C_CR2_ITERREN                     I2C_CR2_ITERREN_Msk                /*!< Error Interrupt Enable */
#define I2C_CR2_ITEVTEN_Pos                 (9U)
#define I2C_CR2_ITEVTEN_Msk                 (0x1UL << I2C_CR2_ITEVTEN_Pos)      /*!< 0x00000200 */
#define I2C_CR2_ITEVTEN                     I2C_CR2_ITEVTEN_Msk                /*!< Event Interrupt Enable */
#define I2C_CR2_ITBUFEN_Pos                 (10U)
#define I2C_CR2_ITBUFEN_Msk                 (0x1UL << I2C_CR2_ITBUFEN_Pos)      /*!< 0x00000400 */
#define I2C_CR2_ITBUFEN                     I2C_CR2_ITBUFEN_Msk                /*!< Buffer Interrupt Enable */
#define I2C_CR2_DMAEN_Pos                   (11U)
#define I2C_CR2_DMAEN_Msk                   (0x1UL << I2C_CR2_DMAEN_Pos)        /*!< 0x00000800 */
#define I2C_CR2_DMAEN                       I2C_CR2_DMAEN_Msk                  /*!< DMA Requests Enable */
#define I2C_CR2_LAST_Pos                    (12U)
#define I2C_CR2_LAST_Msk                    (0x1UL << I2C_CR2_LAST_Pos)         /*!< 0x00001000 */
#define I2C_CR2_LAST                        I2C_CR2_LAST_Msk                   /*!< DMA Last Transfer */
/*******************  Bit definition for I2C_OAR1 register  *******************/
#define I2C_OAR1_ADDMODE_Pos                (15U)
#define I2C_OAR1_ADDMODE_Msk                (0x1UL << I2C_OAR1_ADDMODE_Pos)      /*!< 0x00008000 */
#define I2C_OAR1_ADDMODE                    I2C_OAR1_ADDMODE_Msk               /*!< 10-bit slave address */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define I2C_OAR2_ENDUAL_Pos                 (0U)
#define I2C_OAR2_ENDUAL_Msk                 (0x1UL << I2C_OAR2_ENDUAL_Pos)      /*!< 0x00000001 */
#define I2C_OAR2_ENDUAL                     I2C_OAR2_ENDUAL_Msk                /*!< Dual addressing mode enable */
#define I2C_OAR2_ADD2_Pos                   (1U)
/*******************  Bit definition for I2C_SR1 register  ********************/
#define I2C_SR1_SB_Pos                      (0U)
#define I2C_SR1_SB_Msk                      (0x1UL << I2C_SR1_SB_Pos)           /*!< 0x00000001 */
#define I2C_SR1_SB                          I2C_SR1_SB_Msk                     /*!< Start Bit (Master mode) */
#define I2C_SR1_ADDR_Pos                    (1U)
#define I2C_SR1_ADDR_Msk                    (0x1UL << I2C_SR1_ADDR_Pos)         /*!< 0x00000002 */
#define I2C_SR1_ADDR                        I2C_SR1_ADDR_Msk                   /*!< Address sent (master mode)/matched (slave mode) */
#define I2C_SR1_BTF_Pos                     (2U)
#define I2C_SR1_BTF_Msk                     (0x1UL << I2C_SR1_BTF_Pos)          /*!< 0x00000004 */
#define I2C_SR1_BTF                         I2C_SR1_BTF_Msk                    /*!< Byte Transfer Finished */
#define I2C_SR1_ADD10_Pos                   (3U)
#define I2C_SR1_ADD10_Msk                   (0x1UL << I2C_SR1_ADD10_Pos)        /*!< 0x00000008 */
#define I2C_SR1_ADD10                       I2C_SR1_ADD10_Msk                  /*!< 10-bit header sent (Master mode) */
#define I2C_SR1_STOPF_Pos                   (4U)
#define I2C_SR1_STOPF_Msk                   (0x1UL << I2C_SR1_STOPF_Pos)        /*!< 0x00000010 */
#define I2C_SR1_STOPF                       I2C_SR1_STOPF_Msk                  /*!< Stop detection (Slave mode) */
#define I2C_SR1_RXNE_Pos                    (6U)
#define I2C_SR1_RXNE_Msk                    (0x1UL << I2C_SR1_RXNE_Pos)         /*!< 0x00000040 */
#define I2C_SR1_RXNE                        I2C_SR1_RXNE_Msk                   /*!< Data Register not Empty (receivers) */
#define I2C_SR1_TXE_Pos                     (7U)
#define I2C_SR1_TXE_Msk                     (0x1UL << I2C_SR1_TXE_Pos)          /*!< 0x00000080 */
#define I2C_SR1_TXE                         I2C_SR1_TXE_Msk                    /*!< Data Register Empty (transmitters) */
#define I2C_SR1_BERR_Pos                    (8U)
#define I2C_SR1_BERR_Msk                    (0x1UL << I2C_SR1_BERR_Pos)         /*!< 0x00000100 */
#define I2C_SR1_BERR                        I2C_SR1_BERR_Msk                   /*!< Bus Error */
#define I2C_SR1_ARLO_Pos                    (9U)
#define I2C_SR1_ARLO_Msk                    (0x1UL << I2C_SR1_ARLO_Pos)         /*!< 0x00000200 */
#define I2C_SR1_ARLO                        I2C_SR1_ARLO_Msk                   /*!< Arbitration Lost (master mode) */
#define I2C_SR1_AF_Pos                      (10U)
#define I2C_SR1_AF_Msk                      (0x1UL << I2C_SR1_AF_Pos)           /*!< 0x00000400 */
#define I2C_SR1_AF                          I2C_SR1_AF_Msk                     /*!< Acknowledge Failure */
#define I2C_SR1_OVR_Pos                     (11U)
#define I2C_SR1_OVR_Msk                     (0x1UL << I2C_SR1_OVR_Pos)          /*!< 0x00000800 */
#define I2C_SR1_OVR                         I2C_SR1_OVR_Msk                    /*!< Overrun/Underrun */
#define I2C_SR1_PECERR_Pos                  (12U)
#define I2C_SR1_PECERR_Msk                  (0x1UL << I2C_SR1_PECERR_Pos)       /*!< 0x00001000 */
#define I2C_SR1_PECERR                      I2C_SR1_PECERR_Msk                 /*!< PEC Error in reception */
#define I2C_SR1_TIMEOUT_Pos                 (14U)
#define I2C_SR1_TIMEOUT_Msk                 (0x1UL << I2C_SR1_TIMEOUT_Pos)      /*!< 0x00004000 */
#define I2C_SR1_TIMEOUT                     I2C_SR1_TIMEOUT_Msk                /*!< Timeout or Tlow Error */
#define I2C_SR1_SMBALERT_Pos                (15U)
#define I2C_SR1_SMBALERT_Msk                (0x1UL << I2C_SR1_SMBALERT_Pos)     /*!< 0x00008000 */
#define I2C_SR1_SMBALERT                    I2C_SR1_SMBALERT_Msk               /*!< SMBus Alert */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define I2C_SR2_MSL_Pos                     (0U)
#define I2C_SR2_MSL_Msk                     (0x1UL << I2C_SR2_MSL_Pos)          /*!< 0x00000001 */
#define I2C_SR2_MSL                         I2C_SR2_MSL_Msk                    /*!< Master/Slave */
#define I2C_SR2_BUSY_Pos                    (1U)
#define I2C_SR2_BUSY_Msk                    (0x1UL << I2C_SR2_BUSY_Pos)         /*!< 0x00000002 */
#define I2C_SR2_BUSY                        I2C_SR2_BUSY_Msk                   /*!< Bus Busy */
#define I2C_SR2_TRA_Pos                     (2U)
#define I2C_SR2_TRA_Msk                     (0x1UL << I2C_SR2_TRA_Pos)          /*!< 0x00000004 */
#define I2C_SR2_TRA                         I2C_SR2_TRA_Msk                    /*!< Transmitter/Receiver */
#define I2C_SR2_GENCALL_Pos                 (4U)
#define I2C_SR2_GENCALL_Msk                 (0x1UL << I2C_SR2_GENCALL_Pos)      /*!< 0x00000010 */
#define I2C_SR2_GENCALL                     I2C_SR2_GENCALL_Msk                /*!< General Call Address (Slave mode) */
#define I2C_SR2_SMBDEFAULT_Pos              (5U)
#define I2C_SR2_SMBDEFAULT_Msk              (0x1UL << I2C_SR2_SMBDEFAULT_Pos)   /*!< 0x00000020 */
#define I2C_SR2_SMBDEFAULT                  I2C_SR2_SMBDEFAULT_Msk             /*!< SMBus Device Default Address (Slave mode) */
#define I2C_SR2_SMBHOST_Pos                 (6U)
#define I2C_SR2_SMBHOST_Msk                 (0x1UL << I2C_SR2_SMBHOST_Pos)      /*!< 0x00000040 */
#define I2C_SR2_SMBHOST                     I2C_SR2_SMBHOST_Msk                /*!< SMBus Host Header (Slave mode) */
#define I2C_SR2_DUALF_Pos                   (7U)
#define I2C_SR2_DUALF_Msk                   (0x1UL << I2C_SR2_DUALF_Pos)        /*!< 0x00000080 */
#define I2C_SR2_DUALF                       I2C_SR2_DUALF_Msk                  /*!< Dual Flag (Slave mode) */
#define I2C_SR2_PEC_Pos                     (8U)
#define I2C_SR2_PEC_Msk                     (0xFFUL << I2C_SR2_PEC_Pos)         /*!< 0x0000FF00 */
#define I2C_SR2_PEC                         I2C_SR2_PEC_Msk                    /*!< Packet Error Checking Register */

/*
 * CAN_BIT_FILD.h
 *
 *  Created on: Jan 28, 2025
 *      Author: Laptix
 */

#ifndef CAN_CAN_BIT_FILD_H_
#define CAN_CAN_BIT_FILD_H_
/*!< CAN control and status registers */
/*******************  Bit definition for CAN_MCR register  ********************/
#define CAN_MCR_INRQ_Pos                     (0U)
#define CAN_MCR_INRQ_Msk                     (0x1UL << CAN_MCR_INRQ_Pos)        /*!< 0x00000001 */
#define CAN_MCR_INRQ                         CAN_MCR_INRQ_Msk                  /*!< Initialization Request */
#define CAN_MCR_SLEEP_Pos                    (1U)
#define CAN_MCR_SLEEP_Msk                    (0x1UL << CAN_MCR_SLEEP_Pos)       /*!< 0x00000002 */
#define CAN_MCR_SLEEP                        CAN_MCR_SLEEP_Msk                 /*!< Sleep Mode Request */
#define CAN_MCR_TXFP_Pos                     (2U)
#define CAN_MCR_TXFP_Msk                     (0x1UL << CAN_MCR_TXFP_Pos)        /*!< 0x00000004 */
#define CAN_MCR_TXFP                         CAN_MCR_TXFP_Msk                  /*!< Transmit FIFO Priority */
#define CAN_MCR_RFLM_Pos                     (3U)
#define CAN_MCR_RFLM_Msk                     (0x1UL << CAN_MCR_RFLM_Pos)        /*!< 0x00000008 */
#define CAN_MCR_RFLM                         CAN_MCR_RFLM_Msk                  /*!< Receive FIFO Locked Mode */
#define CAN_MCR_NART_Pos                     (4U)
#define CAN_MCR_NART_Msk                     (0x1UL << CAN_MCR_NART_Pos)        /*!< 0x00000010 */
#define CAN_MCR_NART                         CAN_MCR_NART_Msk                  /*!< No Automatic Retransmission */
#define CAN_MCR_AWUM_Pos                     (5U)
#define CAN_MCR_AWUM_Msk                     (0x1UL << CAN_MCR_AWUM_Pos)        /*!< 0x00000020 */
#define CAN_MCR_AWUM                         CAN_MCR_AWUM_Msk                  /*!< Automatic Wakeup Mode */
#define CAN_MCR_ABOM_Pos                     (6U)
#define CAN_MCR_ABOM_Msk                     (0x1UL << CAN_MCR_ABOM_Pos)        /*!< 0x00000040 */
#define CAN_MCR_ABOM                         CAN_MCR_ABOM_Msk                  /*!< Automatic Bus-Off Management */
#define CAN_MCR_TTCM_Pos                     (7U)
#define CAN_MCR_TTCM_Msk                     (0x1UL << CAN_MCR_TTCM_Pos)        /*!< 0x00000080 */
#define CAN_MCR_TTCM                         CAN_MCR_TTCM_Msk                  /*!< Time Triggered Communication Mode */
#define CAN_MCR_RESET_Pos                    (15U)
#define CAN_MCR_RESET_Msk                    (0x1UL << CAN_MCR_RESET_Pos)       /*!< 0x00008000 */
#define CAN_MCR_RESET                        CAN_MCR_RESET_Msk                 /*!< CAN software master reset */
#define CAN_MCR_DBF_Pos                      (16U)
#define CAN_MCR_DBF_Msk                      (0x1UL << CAN_MCR_DBF_Pos)         /*!< 0x00010000 */
#define CAN_MCR_DBF                          CAN_MCR_DBF_Msk                   /*!< CAN Debug freeze */

/*******************  Bit definition for CAN_MSR register  ********************/
#define CAN_MSR_INAK_Pos                     (0U)
#define CAN_MSR_INAK_Msk                     (0x1UL << CAN_MSR_INAK_Pos)        /*!< 0x00000001 */
#define CAN_MSR_INAK                         CAN_MSR_INAK_Msk                  /*!< Initialization Acknowledge */
#define CAN_MSR_SLAK_Pos                     (1U)
#define CAN_MSR_SLAK_Msk                     (0x1UL << CAN_MSR_SLAK_Pos)        /*!< 0x00000002 */
#define CAN_MSR_SLAK                         CAN_MSR_SLAK_Msk                  /*!< Sleep Acknowledge */
#define CAN_MSR_ERRI_Pos                     (2U)
#define CAN_MSR_ERRI_Msk                     (0x1UL << CAN_MSR_ERRI_Pos)        /*!< 0x00000004 */
#define CAN_MSR_ERRI                         CAN_MSR_ERRI_Msk                  /*!< Error Interrupt */
#define CAN_MSR_WKUI_Pos                     (3U)
#define CAN_MSR_WKUI_Msk                     (0x1UL << CAN_MSR_WKUI_Pos)        /*!< 0x00000008 */
#define CAN_MSR_WKUI                         CAN_MSR_WKUI_Msk                  /*!< Wakeup Interrupt */
#define CAN_MSR_SLAKI_Pos                    (4U)
#define CAN_MSR_SLAKI_Msk                    (0x1UL << CAN_MSR_SLAKI_Pos)       /*!< 0x00000010 */
#define CAN_MSR_SLAKI                        CAN_MSR_SLAKI_Msk                 /*!< Sleep Acknowledge Interrupt */
#define CAN_MSR_TXM_Pos                      (8U)
#define CAN_MSR_TXM_Msk                      (0x1UL << CAN_MSR_TXM_Pos)         /*!< 0x00000100 */
#define CAN_MSR_TXM                          CAN_MSR_TXM_Msk                   /*!< Transmit Mode */
#define CAN_MSR_RXM_Pos                      (9U)
#define CAN_MSR_RXM_Msk                      (0x1UL << CAN_MSR_RXM_Pos)         /*!< 0x00000200 */
#define CAN_MSR_RXM                          CAN_MSR_RXM_Msk                   /*!< Receive Mode */
#define CAN_MSR_SAMP_Pos                     (10U)
#define CAN_MSR_SAMP_Msk                     (0x1UL << CAN_MSR_SAMP_Pos)        /*!< 0x00000400 */
#define CAN_MSR_SAMP                         CAN_MSR_SAMP_Msk                  /*!< Last Sample Point */
#define CAN_MSR_RX_Pos                       (11U)
#define CAN_MSR_RX_Msk                       (0x1UL << CAN_MSR_RX_Pos)          /*!< 0x00000800 */
#define CAN_MSR_RX                           CAN_MSR_RX_Msk                    /*!< CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
#define CAN_TSR_RQCP0_Pos                    (0U)
#define CAN_TSR_RQCP0_Msk                    (0x1UL << CAN_TSR_RQCP0_Pos)       /*!< 0x00000001 */
#define CAN_TSR_RQCP0                        CAN_TSR_RQCP0_Msk                 /*!< Request Completed Mailbox0 */
#define CAN_TSR_TXOK0_Pos                    (1U)
#define CAN_TSR_TXOK0_Msk                    (0x1UL << CAN_TSR_TXOK0_Pos)       /*!< 0x00000002 */
#define CAN_TSR_TXOK0                        CAN_TSR_TXOK0_Msk                 /*!< Transmission OK of Mailbox0 */
#define CAN_TSR_ALST0_Pos                    (2U)
#define CAN_TSR_ALST0_Msk                    (0x1UL << CAN_TSR_ALST0_Pos)       /*!< 0x00000004 */
#define CAN_TSR_ALST0                        CAN_TSR_ALST0_Msk                 /*!< Arbitration Lost for Mailbox0 */
#define CAN_TSR_TERR0_Pos                    (3U)
#define CAN_TSR_TERR0_Msk                    (0x1UL << CAN_TSR_TERR0_Pos)       /*!< 0x00000008 */
#define CAN_TSR_TERR0                        CAN_TSR_TERR0_Msk                 /*!< Transmission Error of Mailbox0 */
#define CAN_TSR_ABRQ0_Pos                    (7U)
#define CAN_TSR_ABRQ0_Msk                    (0x1UL << CAN_TSR_ABRQ0_Pos)       /*!< 0x00000080 */
#define CAN_TSR_ABRQ0                        CAN_TSR_ABRQ0_Msk                 /*!< Abort Request for Mailbox0 */
#define CAN_TSR_RQCP1_Pos                    (8U)
#define CAN_TSR_RQCP1_Msk                    (0x1UL << CAN_TSR_RQCP1_Pos)       /*!< 0x00000100 */
#define CAN_TSR_RQCP1                        CAN_TSR_RQCP1_Msk                 /*!< Request Completed Mailbox1 */
#define CAN_TSR_TXOK1_Pos                    (9U)
#define CAN_TSR_TXOK1_Msk                    (0x1UL << CAN_TSR_TXOK1_Pos)       /*!< 0x00000200 */
#define CAN_TSR_TXOK1                        CAN_TSR_TXOK1_Msk                 /*!< Transmission OK of Mailbox1 */
#define CAN_TSR_ALST1_Pos                    (10U)
#define CAN_TSR_ALST1_Msk                    (0x1UL << CAN_TSR_ALST1_Pos)       /*!< 0x00000400 */
#define CAN_TSR_ALST1                        CAN_TSR_ALST1_Msk                 /*!< Arbitration Lost for Mailbox1 */
#define CAN_TSR_TERR1_Pos                    (11U)
#define CAN_TSR_TERR1_Msk                    (0x1UL << CAN_TSR_TERR1_Pos)       /*!< 0x00000800 */
#define CAN_TSR_TERR1                        CAN_TSR_TERR1_Msk                 /*!< Transmission Error of Mailbox1 */
#define CAN_TSR_ABRQ1_Pos                    (15U)
#define CAN_TSR_ABRQ1_Msk                    (0x1UL << CAN_TSR_ABRQ1_Pos)       /*!< 0x00008000 */
#define CAN_TSR_ABRQ1                        CAN_TSR_ABRQ1_Msk                 /*!< Abort Request for Mailbox 1 */
#define CAN_TSR_RQCP2_Pos                    (16U)
#define CAN_TSR_RQCP2_Msk                    (0x1UL << CAN_TSR_RQCP2_Pos)       /*!< 0x00010000 */
#define CAN_TSR_RQCP2                        CAN_TSR_RQCP2_Msk                 /*!< Request Completed Mailbox2 */
#define CAN_TSR_TXOK2_Pos                    (17U)
#define CAN_TSR_TXOK2_Msk                    (0x1UL << CAN_TSR_TXOK2_Pos)       /*!< 0x00020000 */
#define CAN_TSR_TXOK2                        CAN_TSR_TXOK2_Msk                 /*!< Transmission OK of Mailbox 2 */
#define CAN_TSR_ALST2_Pos                    (18U)
#define CAN_TSR_ALST2_Msk                    (0x1UL << CAN_TSR_ALST2_Pos)       /*!< 0x00040000 */
#define CAN_TSR_ALST2                        CAN_TSR_ALST2_Msk                 /*!< Arbitration Lost for mailbox 2 */
#define CAN_TSR_TERR2_Pos                    (19U)
#define CAN_TSR_TERR2_Msk                    (0x1UL << CAN_TSR_TERR2_Pos)       /*!< 0x00080000 */
#define CAN_TSR_TERR2                        CAN_TSR_TERR2_Msk                 /*!< Transmission Error of Mailbox 2 */
#define CAN_TSR_ABRQ2_Pos                    (23U)
#define CAN_TSR_ABRQ2_Msk                    (0x1UL << CAN_TSR_ABRQ2_Pos)       /*!< 0x00800000 */
#define CAN_TSR_ABRQ2                        CAN_TSR_ABRQ2_Msk                 /*!< Abort Request for Mailbox 2 */
#define CAN_TSR_CODE_Pos                     (24U)
#define CAN_TSR_CODE_Msk                     (0x3UL << CAN_TSR_CODE_Pos)        /*!< 0x03000000 */
#define CAN_TSR_CODE                         CAN_TSR_CODE_Msk                  /*!< Mailbox Code */

#define CAN_TSR_TME_Pos                      (26U)
#define CAN_TSR_TME_Msk                      (0x7UL << CAN_TSR_TME_Pos)         /*!< 0x1C000000 */
#define CAN_TSR_TME                          CAN_TSR_TME_Msk                   /*!< TME[2:0] bits */
#define CAN_TSR_TME0_Pos                     (26U)
#define CAN_TSR_TME0_Msk                     (0x1UL << CAN_TSR_TME0_Pos)        /*!< 0x04000000 */
#define CAN_TSR_TME0                         CAN_TSR_TME0_Msk                  /*!< Transmit Mailbox 0 Empty */
#define CAN_TSR_TME1_Pos                     (27U)
#define CAN_TSR_TME1_Msk                     (0x1UL << CAN_TSR_TME1_Pos)        /*!< 0x08000000 */
#define CAN_TSR_TME1                         CAN_TSR_TME1_Msk                  /*!< Transmit Mailbox 1 Empty */
#define CAN_TSR_TME2_Pos                     (28U)
#define CAN_TSR_TME2_Msk                     (0x1UL << CAN_TSR_TME2_Pos)        /*!< 0x10000000 */
#define CAN_TSR_TME2                         CAN_TSR_TME2_Msk                  /*!< Transmit Mailbox 2 Empty */

#define CAN_TSR_LOW_Pos                      (29U)
#define CAN_TSR_LOW_Msk                      (0x7UL << CAN_TSR_LOW_Pos)         /*!< 0xE0000000 */
#define CAN_TSR_LOW                          CAN_TSR_LOW_Msk                   /*!< LOW[2:0] bits */
#define CAN_TSR_LOW0_Pos                     (29U)
#define CAN_TSR_LOW0_Msk                     (0x1UL << CAN_TSR_LOW0_Pos)        /*!< 0x20000000 */
#define CAN_TSR_LOW0                         CAN_TSR_LOW0_Msk                  /*!< Lowest Priority Flag for Mailbox 0 */
#define CAN_TSR_LOW1_Pos                     (30U)
#define CAN_TSR_LOW1_Msk                     (0x1UL << CAN_TSR_LOW1_Pos)        /*!< 0x40000000 */
#define CAN_TSR_LOW1                         CAN_TSR_LOW1_Msk                  /*!< Lowest Priority Flag for Mailbox 1 */
#define CAN_TSR_LOW2_Pos                     (31U)
#define CAN_TSR_LOW2_Msk                     (0x1UL << CAN_TSR_LOW2_Pos)        /*!< 0x80000000 */
#define CAN_TSR_LOW2                         CAN_TSR_LOW2_Msk                  /*!< Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0R register  *******************/
#define CAN_RF0R_FMP0_Pos                    (0U)
#define CAN_RF0R_FMP0_Msk                    (0x3UL << CAN_RF0R_FMP0_Pos)       /*!< 0x00000003 */
#define CAN_RF0R_FMP0                        CAN_RF0R_FMP0_Msk                 /*!< FIFO 0 Message Pending */
#define CAN_RF0R_FULL0_Pos                   (3U)
#define CAN_RF0R_FULL0_Msk                   (0x1UL << CAN_RF0R_FULL0_Pos)      /*!< 0x00000008 */
#define CAN_RF0R_FULL0                       CAN_RF0R_FULL0_Msk                /*!< FIFO 0 Full */
#define CAN_RF0R_FOVR0_Pos                   (4U)
#define CAN_RF0R_FOVR0_Msk                   (0x1UL << CAN_RF0R_FOVR0_Pos)      /*!< 0x00000010 */
#define CAN_RF0R_FOVR0                       CAN_RF0R_FOVR0_Msk                /*!< FIFO 0 Overrun */
#define CAN_RF0R_RFOM0_Pos                   (5U)
#define CAN_RF0R_RFOM0_Msk                   (0x1UL << CAN_RF0R_RFOM0_Pos)      /*!< 0x00000020 */
#define CAN_RF0R_RFOM0                       CAN_RF0R_RFOM0_Msk                /*!< Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define CAN_RF1R_FMP1_Pos                    (0U)
#define CAN_RF1R_FMP1_Msk                    (0x3UL << CAN_RF1R_FMP1_Pos)       /*!< 0x00000003 */
#define CAN_RF1R_FMP1                        CAN_RF1R_FMP1_Msk                 /*!< FIFO 1 Message Pending */
#define CAN_RF1R_FULL1_Pos                   (3U)
#define CAN_RF1R_FULL1_Msk                   (0x1UL << CAN_RF1R_FULL1_Pos)      /*!< 0x00000008 */
#define CAN_RF1R_FULL1                       CAN_RF1R_FULL1_Msk                /*!< FIFO 1 Full */
#define CAN_RF1R_FOVR1_Pos                   (4U)
#define CAN_RF1R_FOVR1_Msk                   (0x1UL << CAN_RF1R_FOVR1_Pos)      /*!< 0x00000010 */
#define CAN_RF1R_FOVR1                       CAN_RF1R_FOVR1_Msk                /*!< FIFO 1 Overrun */
#define CAN_RF1R_RFOM1_Pos                   (5U)
#define CAN_RF1R_RFOM1_Msk                   (0x1UL << CAN_RF1R_RFOM1_Pos)      /*!< 0x00000020 */
#define CAN_RF1R_RFOM1                       CAN_RF1R_RFOM1_Msk                /*!< Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
#define CAN_IER_TMEIE_Pos                    (0U)
#define CAN_IER_TMEIE_Msk                    (0x1UL << CAN_IER_TMEIE_Pos)       /*!< 0x00000001 */
#define CAN_IER_TMEIE                        CAN_IER_TMEIE_Msk                 /*!< Transmit Mailbox Empty Interrupt Enable */
#define CAN_IER_FMPIE0_Pos                   (1U)
#define CAN_IER_FMPIE0_Msk                   (0x1UL << CAN_IER_FMPIE0_Pos)      /*!< 0x00000002 */
#define CAN_IER_FMPIE0                       CAN_IER_FMPIE0_Msk                /*!< FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE0_Pos                    (2U)
#define CAN_IER_FFIE0_Msk                    (0x1UL << CAN_IER_FFIE0_Pos)       /*!< 0x00000004 */
#define CAN_IER_FFIE0                        CAN_IER_FFIE0_Msk                 /*!< FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE0_Pos                   (3U)
#define CAN_IER_FOVIE0_Msk                   (0x1UL << CAN_IER_FOVIE0_Pos)      /*!< 0x00000008 */
#define CAN_IER_FOVIE0                       CAN_IER_FOVIE0_Msk                /*!< FIFO Overrun Interrupt Enable */
#define CAN_IER_FMPIE1_Pos                   (4U)
#define CAN_IER_FMPIE1_Msk                   (0x1UL << CAN_IER_FMPIE1_Pos)      /*!< 0x00000010 */
#define CAN_IER_FMPIE1                       CAN_IER_FMPIE1_Msk                /*!< FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE1_Pos                    (5U)
#define CAN_IER_FFIE1_Msk                    (0x1UL << CAN_IER_FFIE1_Pos)       /*!< 0x00000020 */
#define CAN_IER_FFIE1                        CAN_IER_FFIE1_Msk                 /*!< FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE1_Pos                   (6U)
#define CAN_IER_FOVIE1_Msk                   (0x1UL << CAN_IER_FOVIE1_Pos)      /*!< 0x00000040 */
#define CAN_IER_FOVIE1                       CAN_IER_FOVIE1_Msk                /*!< FIFO Overrun Interrupt Enable */
#define CAN_IER_EWGIE_Pos                    (8U)
#define CAN_IER_EWGIE_Msk                    (0x1UL << CAN_IER_EWGIE_Pos)       /*!< 0x00000100 */
#define CAN_IER_EWGIE                        CAN_IER_EWGIE_Msk                 /*!< Error Warning Interrupt Enable */
#define CAN_IER_EPVIE_Pos                    (9U)
#define CAN_IER_EPVIE_Msk                    (0x1UL << CAN_IER_EPVIE_Pos)       /*!< 0x00000200 */
#define CAN_IER_EPVIE                        CAN_IER_EPVIE_Msk                 /*!< Error Passive Interrupt Enable */
#define CAN_IER_BOFIE_Pos                    (10U)
#define CAN_IER_BOFIE_Msk                    (0x1UL << CAN_IER_BOFIE_Pos)       /*!< 0x00000400 */
#define CAN_IER_BOFIE                        CAN_IER_BOFIE_Msk                 /*!< Bus-Off Interrupt Enable */
#define CAN_IER_LECIE_Pos                    (11U)
#define CAN_IER_LECIE_Msk                    (0x1UL << CAN_IER_LECIE_Pos)       /*!< 0x00000800 */
#define CAN_IER_LECIE                        CAN_IER_LECIE_Msk                 /*!< Last Error Code Interrupt Enable */
#define CAN_IER_ERRIE_Pos                    (15U)
#define CAN_IER_ERRIE_Msk                    (0x1UL << CAN_IER_ERRIE_Pos)       /*!< 0x00008000 */
#define CAN_IER_ERRIE                        CAN_IER_ERRIE_Msk                 /*!< Error Interrupt Enable */
#define CAN_IER_WKUIE_Pos                    (16U)
#define CAN_IER_WKUIE_Msk                    (0x1UL << CAN_IER_WKUIE_Pos)       /*!< 0x00010000 */
#define CAN_IER_WKUIE                        CAN_IER_WKUIE_Msk                 /*!< Wakeup Interrupt Enable */
#define CAN_IER_SLKIE_Pos                    (17U)
#define CAN_IER_SLKIE_Msk                    (0x1UL << CAN_IER_SLKIE_Pos)       /*!< 0x00020000 */
#define CAN_IER_SLKIE                        CAN_IER_SLKIE_Msk                 /*!< Sleep Interrupt Enable */

/********************  Bit definition for CAN_ESR register  *******************/
#define CAN_ESR_EWGF_Pos                     (0U)
#define CAN_ESR_EWGF_Msk                     (0x1UL << CAN_ESR_EWGF_Pos)        /*!< 0x00000001 */
#define CAN_ESR_EWGF                         CAN_ESR_EWGF_Msk                  /*!< Error Warning Flag */
#define CAN_ESR_EPVF_Pos                     (1U)
#define CAN_ESR_EPVF_Msk                     (0x1UL << CAN_ESR_EPVF_Pos)        /*!< 0x00000002 */
#define CAN_ESR_EPVF                         CAN_ESR_EPVF_Msk                  /*!< Error Passive Flag */
#define CAN_ESR_BOFF_Pos                     (2U)
#define CAN_ESR_BOFF_Msk                     (0x1UL << CAN_ESR_BOFF_Pos)        /*!< 0x00000004 */
#define CAN_ESR_BOFF                         CAN_ESR_BOFF_Msk                  /*!< Bus-Off Flag */

#define CAN_ESR_LEC_Pos                      (4U)
#define CAN_ESR_LEC_Msk                      (0x7UL << CAN_ESR_LEC_Pos)         /*!< 0x00000070 */
#define CAN_ESR_LEC                          CAN_ESR_LEC_Msk                   /*!< LEC[2:0] bits (Last Error Code) */
#define CAN_ESR_LEC_0                        (0x1UL << CAN_ESR_LEC_Pos)         /*!< 0x00000010 */
#define CAN_ESR_LEC_1                        (0x2UL << CAN_ESR_LEC_Pos)         /*!< 0x00000020 */
#define CAN_ESR_LEC_2                        (0x4UL << CAN_ESR_LEC_Pos)         /*!< 0x00000040 */

#define CAN_ESR_TEC_Pos                      (16U)
#define CAN_ESR_TEC_Msk                      (0xFFUL << CAN_ESR_TEC_Pos)        /*!< 0x00FF0000 */
#define CAN_ESR_TEC                          CAN_ESR_TEC_Msk                   /*!< Least significant byte of the 9-bit Transmit Error Counter */
#define CAN_ESR_REC_Pos                      (24U)
#define CAN_ESR_REC_Msk                      (0xFFUL << CAN_ESR_REC_Pos)        /*!< 0xFF000000 */
#define CAN_ESR_REC                          CAN_ESR_REC_Msk                   /*!< Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
#define CAN_BTR_BRP_Pos                      (0U)
#define CAN_BTR_BRP_Msk                      (0x3FFUL << CAN_BTR_BRP_Pos)       /*!< 0x000003FF */
#define CAN_BTR_BRP                          CAN_BTR_BRP_Msk                   /*!<Baud Rate Prescaler */
#define CAN_BTR_TS1_Pos                      (16U)
#define CAN_BTR_TS1_Msk                      (0xFUL << CAN_BTR_TS1_Pos)         /*!< 0x000F0000 */
#define CAN_BTR_TS1                          CAN_BTR_TS1_Msk                   /*!<Time Segment 1 */
#define CAN_BTR_TS1_0                        (0x1UL << CAN_BTR_TS1_Pos)         /*!< 0x00010000 */
#define CAN_BTR_TS1_1                        (0x2UL << CAN_BTR_TS1_Pos)         /*!< 0x00020000 */
#define CAN_BTR_TS1_2                        (0x4UL << CAN_BTR_TS1_Pos)         /*!< 0x00040000 */
#define CAN_BTR_TS1_3                        (0x8UL << CAN_BTR_TS1_Pos)         /*!< 0x00080000 */
#define CAN_BTR_TS2_Pos                      (20U)
#define CAN_BTR_TS2_Msk                      (0x7UL << CAN_BTR_TS2_Pos)         /*!< 0x00700000 */
#define CAN_BTR_TS2                          CAN_BTR_TS2_Msk                   /*!<Time Segment 2 */
#define CAN_BTR_TS2_0                        (0x1UL << CAN_BTR_TS2_Pos)         /*!< 0x00100000 */
#define CAN_BTR_TS2_1                        (0x2UL << CAN_BTR_TS2_Pos)         /*!< 0x00200000 */
#define CAN_BTR_TS2_2                        (0x4UL << CAN_BTR_TS2_Pos)         /*!< 0x00400000 */
#define CAN_BTR_SJW_Pos                      (24U)
#define CAN_BTR_SJW_Msk                      (0x3UL << CAN_BTR_SJW_Pos)         /*!< 0x03000000 */
#define CAN_BTR_SJW                          CAN_BTR_SJW_Msk                   /*!<Resynchronization Jump Width */
#define CAN_BTR_SJW_0                        (0x1UL << CAN_BTR_SJW_Pos)         /*!< 0x01000000 */
#define CAN_BTR_SJW_1                        (0x2UL << CAN_BTR_SJW_Pos)         /*!< 0x02000000 */
#define CAN_BTR_LBKM_Pos                     (30U)
#define CAN_BTR_LBKM_Msk                     (0x1UL << CAN_BTR_LBKM_Pos)        /*!< 0x40000000 */
#define CAN_BTR_LBKM                         CAN_BTR_LBKM_Msk                  /*!<Loop Back Mode (Debug) */
#define CAN_BTR_SILM_Pos                     (31U)
#define CAN_BTR_SILM_Msk                     (0x1UL << CAN_BTR_SILM_Pos)        /*!< 0x80000000 */
#define CAN_BTR_SILM                         CAN_BTR_SILM_Msk                  /*!<Silent Mode */

/*!< Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define CAN_TI0R_TXRQ_Pos                    (0U)
#define CAN_TI0R_TXRQ_Msk                    (0x1UL << CAN_TI0R_TXRQ_Pos)       /*!< 0x00000001 */
#define CAN_TI0R_TXRQ                        CAN_TI0R_TXRQ_Msk                 /*!< Transmit Mailbox Request */
#define CAN_TI0R_RTR_Pos                     (1U)
#define CAN_TI0R_RTR_Msk                     (0x1UL << CAN_TI0R_RTR_Pos)        /*!< 0x00000002 */
#define CAN_TI0R_RTR                         CAN_TI0R_RTR_Msk                  /*!< Remote Transmission Request */
#define CAN_TI0R_IDE_Pos                     (2U)
#define CAN_TI0R_IDE_Msk                     (0x1UL << CAN_TI0R_IDE_Pos)        /*!< 0x00000004 */
#define CAN_TI0R_IDE                         CAN_TI0R_IDE_Msk                  /*!< Identifier Extension */
#define CAN_TI0R_EXID_Pos                    (3U)
#define CAN_TI0R_EXID_Msk                    (0x3FFFFUL << CAN_TI0R_EXID_Pos)   /*!< 0x001FFFF8 */
#define CAN_TI0R_EXID                        CAN_TI0R_EXID_Msk                 /*!< Extended Identifier */
#define CAN_TI0R_STID_Pos                    (21U)
#define CAN_TI0R_STID_Msk                    (0x7FFUL << CAN_TI0R_STID_Pos)     /*!< 0xFFE00000 */
#define CAN_TI0R_STID                        CAN_TI0R_STID_Msk                 /*!< Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
#define CAN_TDT0R_DLC_Pos                    (0U)
#define CAN_TDT0R_DLC_Msk                    (0xFUL << CAN_TDT0R_DLC_Pos)       /*!< 0x0000000F */
#define CAN_TDT0R_DLC                        CAN_TDT0R_DLC_Msk                 /*!< Data Length Code */
#define CAN_TDT0R_TGT_Pos                    (8U)
#define CAN_TDT0R_TGT_Msk                    (0x1UL << CAN_TDT0R_TGT_Pos)       /*!< 0x00000100 */
#define CAN_TDT0R_TGT                        CAN_TDT0R_TGT_Msk                 /*!< Transmit Global Time */
#define CAN_TDT0R_TIME_Pos                   (16U)
#define CAN_TDT0R_TIME_Msk                   (0xFFFFUL << CAN_TDT0R_TIME_Pos)   /*!< 0xFFFF0000 */
#define CAN_TDT0R_TIME                       CAN_TDT0R_TIME_Msk                /*!< Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
#define CAN_TDL0R_DATA0_Pos                  (0U)
#define CAN_TDL0R_DATA0_Msk                  (0xFFUL << CAN_TDL0R_DATA0_Pos)    /*!< 0x000000FF */
#define CAN_TDL0R_DATA0                      CAN_TDL0R_DATA0_Msk               /*!< Data byte 0 */
#define CAN_TDL0R_DATA1_Pos                  (8U)
#define CAN_TDL0R_DATA1_Msk                  (0xFFUL << CAN_TDL0R_DATA1_Pos)    /*!< 0x0000FF00 */
#define CAN_TDL0R_DATA1                      CAN_TDL0R_DATA1_Msk               /*!< Data byte 1 */
#define CAN_TDL0R_DATA2_Pos                  (16U)
#define CAN_TDL0R_DATA2_Msk                  (0xFFUL << CAN_TDL0R_DATA2_Pos)    /*!< 0x00FF0000 */
#define CAN_TDL0R_DATA2                      CAN_TDL0R_DATA2_Msk               /*!< Data byte 2 */
#define CAN_TDL0R_DATA3_Pos                  (24U)
#define CAN_TDL0R_DATA3_Msk                  (0xFFUL << CAN_TDL0R_DATA3_Pos)    /*!< 0xFF000000 */
#define CAN_TDL0R_DATA3                      CAN_TDL0R_DATA3_Msk               /*!< Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
#define CAN_TDH0R_DATA4_Pos                  (0U)
#define CAN_TDH0R_DATA4_Msk                  (0xFFUL << CAN_TDH0R_DATA4_Pos)    /*!< 0x000000FF */
#define CAN_TDH0R_DATA4                      CAN_TDH0R_DATA4_Msk               /*!< Data byte 4 */
#define CAN_TDH0R_DATA5_Pos                  (8U)
#define CAN_TDH0R_DATA5_Msk                  (0xFFUL << CAN_TDH0R_DATA5_Pos)    /*!< 0x0000FF00 */
#define CAN_TDH0R_DATA5                      CAN_TDH0R_DATA5_Msk               /*!< Data byte 5 */
#define CAN_TDH0R_DATA6_Pos                  (16U)
#define CAN_TDH0R_DATA6_Msk                  (0xFFUL << CAN_TDH0R_DATA6_Pos)    /*!< 0x00FF0000 */
#define CAN_TDH0R_DATA6                      CAN_TDH0R_DATA6_Msk               /*!< Data byte 6 */
#define CAN_TDH0R_DATA7_Pos                  (24U)
#define CAN_TDH0R_DATA7_Msk                  (0xFFUL << CAN_TDH0R_DATA7_Pos)    /*!< 0xFF000000 */
#define CAN_TDH0R_DATA7                      CAN_TDH0R_DATA7_Msk               /*!< Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
#define CAN_TI1R_TXRQ_Pos                    (0U)
#define CAN_TI1R_TXRQ_Msk                    (0x1UL << CAN_TI1R_TXRQ_Pos)       /*!< 0x00000001 */
#define CAN_TI1R_TXRQ                        CAN_TI1R_TXRQ_Msk                 /*!< Transmit Mailbox Request */
#define CAN_TI1R_RTR_Pos                     (1U)
#define CAN_TI1R_RTR_Msk                     (0x1UL << CAN_TI1R_RTR_Pos)        /*!< 0x00000002 */
#define CAN_TI1R_RTR                         CAN_TI1R_RTR_Msk                  /*!< Remote Transmission Request */
#define CAN_TI1R_IDE_Pos                     (2U)
#define CAN_TI1R_IDE_Msk                     (0x1UL << CAN_TI1R_IDE_Pos)        /*!< 0x00000004 */
#define CAN_TI1R_IDE                         CAN_TI1R_IDE_Msk                  /*!< Identifier Extension */
#define CAN_TI1R_EXID_Pos                    (3U)
#define CAN_TI1R_EXID_Msk                    (0x3FFFFUL << CAN_TI1R_EXID_Pos)   /*!< 0x001FFFF8 */
#define CAN_TI1R_EXID                        CAN_TI1R_EXID_Msk                 /*!< Extended Identifier */
#define CAN_TI1R_STID_Pos                    (21U)
#define CAN_TI1R_STID_Msk                    (0x7FFUL << CAN_TI1R_STID_Pos)     /*!< 0xFFE00000 */
#define CAN_TI1R_STID                        CAN_TI1R_STID_Msk                 /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
#define CAN_TDT1R_DLC_Pos                    (0U)
#define CAN_TDT1R_DLC_Msk                    (0xFUL << CAN_TDT1R_DLC_Pos)       /*!< 0x0000000F */
#define CAN_TDT1R_DLC                        CAN_TDT1R_DLC_Msk                 /*!< Data Length Code */
#define CAN_TDT1R_TGT_Pos                    (8U)
#define CAN_TDT1R_TGT_Msk                    (0x1UL << CAN_TDT1R_TGT_Pos)       /*!< 0x00000100 */
#define CAN_TDT1R_TGT                        CAN_TDT1R_TGT_Msk                 /*!< Transmit Global Time */
#define CAN_TDT1R_TIME_Pos                   (16U)
#define CAN_TDT1R_TIME_Msk                   (0xFFFFUL << CAN_TDT1R_TIME_Pos)   /*!< 0xFFFF0000 */
#define CAN_TDT1R_TIME                       CAN_TDT1R_TIME_Msk                /*!< Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
#define CAN_TDL1R_DATA0_Pos                  (0U)
#define CAN_TDL1R_DATA0_Msk                  (0xFFUL << CAN_TDL1R_DATA0_Pos)    /*!< 0x000000FF */
#define CAN_TDL1R_DATA0                      CAN_TDL1R_DATA0_Msk               /*!< Data byte 0 */
#define CAN_TDL1R_DATA1_Pos                  (8U)
#define CAN_TDL1R_DATA1_Msk                  (0xFFUL << CAN_TDL1R_DATA1_Pos)    /*!< 0x0000FF00 */
#define CAN_TDL1R_DATA1                      CAN_TDL1R_DATA1_Msk               /*!< Data byte 1 */
#define CAN_TDL1R_DATA2_Pos                  (16U)
#define CAN_TDL1R_DATA2_Msk                  (0xFFUL << CAN_TDL1R_DATA2_Pos)    /*!< 0x00FF0000 */
#define CAN_TDL1R_DATA2                      CAN_TDL1R_DATA2_Msk               /*!< Data byte 2 */
#define CAN_TDL1R_DATA3_Pos                  (24U)
#define CAN_TDL1R_DATA3_Msk                  (0xFFUL << CAN_TDL1R_DATA3_Pos)    /*!< 0xFF000000 */
#define CAN_TDL1R_DATA3                      CAN_TDL1R_DATA3_Msk               /*!< Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
#define CAN_TDH1R_DATA4_Pos                  (0U)
#define CAN_TDH1R_DATA4_Msk                  (0xFFUL << CAN_TDH1R_DATA4_Pos)    /*!< 0x000000FF */
#define CAN_TDH1R_DATA4                      CAN_TDH1R_DATA4_Msk               /*!< Data byte 4 */
#define CAN_TDH1R_DATA5_Pos                  (8U)
#define CAN_TDH1R_DATA5_Msk                  (0xFFUL << CAN_TDH1R_DATA5_Pos)    /*!< 0x0000FF00 */
#define CAN_TDH1R_DATA5                      CAN_TDH1R_DATA5_Msk               /*!< Data byte 5 */
#define CAN_TDH1R_DATA6_Pos                  (16U)
#define CAN_TDH1R_DATA6_Msk                  (0xFFUL << CAN_TDH1R_DATA6_Pos)    /*!< 0x00FF0000 */
#define CAN_TDH1R_DATA6                      CAN_TDH1R_DATA6_Msk               /*!< Data byte 6 */
#define CAN_TDH1R_DATA7_Pos                  (24U)
#define CAN_TDH1R_DATA7_Msk                  (0xFFUL << CAN_TDH1R_DATA7_Pos)    /*!< 0xFF000000 */
#define CAN_TDH1R_DATA7                      CAN_TDH1R_DATA7_Msk               /*!< Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
#define CAN_TI2R_TXRQ_Pos                    (0U)
#define CAN_TI2R_TXRQ_Msk                    (0x1UL << CAN_TI2R_TXRQ_Pos)       /*!< 0x00000001 */
#define CAN_TI2R_TXRQ                        CAN_TI2R_TXRQ_Msk                 /*!< Transmit Mailbox Request */
#define CAN_TI2R_RTR_Pos                     (1U)
#define CAN_TI2R_RTR_Msk                     (0x1UL << CAN_TI2R_RTR_Pos)        /*!< 0x00000002 */
#define CAN_TI2R_RTR                         CAN_TI2R_RTR_Msk                  /*!< Remote Transmission Request */
#define CAN_TI2R_IDE_Pos                     (2U)
#define CAN_TI2R_IDE_Msk                     (0x1UL << CAN_TI2R_IDE_Pos)        /*!< 0x00000004 */
#define CAN_TI2R_IDE                         CAN_TI2R_IDE_Msk                  /*!< Identifier Extension */
#define CAN_TI2R_EXID_Pos                    (3U)
#define CAN_TI2R_EXID_Msk                    (0x3FFFFUL << CAN_TI2R_EXID_Pos)   /*!< 0x001FFFF8 */
#define CAN_TI2R_EXID                        CAN_TI2R_EXID_Msk                 /*!< Extended identifier */
#define CAN_TI2R_STID_Pos                    (21U)
#define CAN_TI2R_STID_Msk                    (0x7FFUL << CAN_TI2R_STID_Pos)     /*!< 0xFFE00000 */
#define CAN_TI2R_STID                        CAN_TI2R_STID_Msk                 /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/
#define CAN_TDT2R_DLC_Pos                    (0U)
#define CAN_TDT2R_DLC_Msk                    (0xFUL << CAN_TDT2R_DLC_Pos)       /*!< 0x0000000F */
#define CAN_TDT2R_DLC                        CAN_TDT2R_DLC_Msk                 /*!< Data Length Code */
#define CAN_TDT2R_TGT_Pos                    (8U)
#define CAN_TDT2R_TGT_Msk                    (0x1UL << CAN_TDT2R_TGT_Pos)       /*!< 0x00000100 */
#define CAN_TDT2R_TGT                        CAN_TDT2R_TGT_Msk                 /*!< Transmit Global Time */
#define CAN_TDT2R_TIME_Pos                   (16U)
#define CAN_TDT2R_TIME_Msk                   (0xFFFFUL << CAN_TDT2R_TIME_Pos)   /*!< 0xFFFF0000 */
#define CAN_TDT2R_TIME                       CAN_TDT2R_TIME_Msk                /*!< Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
#define CAN_TDL2R_DATA0_Pos                  (0U)
#define CAN_TDL2R_DATA0_Msk                  (0xFFUL << CAN_TDL2R_DATA0_Pos)    /*!< 0x000000FF */
#define CAN_TDL2R_DATA0                      CAN_TDL2R_DATA0_Msk               /*!< Data byte 0 */
#define CAN_TDL2R_DATA1_Pos                  (8U)
#define CAN_TDL2R_DATA1_Msk                  (0xFFUL << CAN_TDL2R_DATA1_Pos)    /*!< 0x0000FF00 */
#define CAN_TDL2R_DATA1                      CAN_TDL2R_DATA1_Msk               /*!< Data byte 1 */
#define CAN_TDL2R_DATA2_Pos                  (16U)
#define CAN_TDL2R_DATA2_Msk                  (0xFFUL << CAN_TDL2R_DATA2_Pos)    /*!< 0x00FF0000 */
#define CAN_TDL2R_DATA2                      CAN_TDL2R_DATA2_Msk               /*!< Data byte 2 */
#define CAN_TDL2R_DATA3_Pos                  (24U)
#define CAN_TDL2R_DATA3_Msk                  (0xFFUL << CAN_TDL2R_DATA3_Pos)    /*!< 0xFF000000 */
#define CAN_TDL2R_DATA3                      CAN_TDL2R_DATA3_Msk               /*!< Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
#define CAN_TDH2R_DATA4_Pos                  (0U)
#define CAN_TDH2R_DATA4_Msk                  (0xFFUL << CAN_TDH2R_DATA4_Pos)    /*!< 0x000000FF */
#define CAN_TDH2R_DATA4                      CAN_TDH2R_DATA4_Msk               /*!< Data byte 4 */
#define CAN_TDH2R_DATA5_Pos                  (8U)
#define CAN_TDH2R_DATA5_Msk                  (0xFFUL << CAN_TDH2R_DATA5_Pos)    /*!< 0x0000FF00 */
#define CAN_TDH2R_DATA5                      CAN_TDH2R_DATA5_Msk               /*!< Data byte 5 */
#define CAN_TDH2R_DATA6_Pos                  (16U)
#define CAN_TDH2R_DATA6_Msk                  (0xFFUL << CAN_TDH2R_DATA6_Pos)    /*!< 0x00FF0000 */
#define CAN_TDH2R_DATA6                      CAN_TDH2R_DATA6_Msk               /*!< Data byte 6 */
#define CAN_TDH2R_DATA7_Pos                  (24U)
#define CAN_TDH2R_DATA7_Msk                  (0xFFUL << CAN_TDH2R_DATA7_Pos)    /*!< 0xFF000000 */
#define CAN_TDH2R_DATA7                      CAN_TDH2R_DATA7_Msk               /*!< Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
#define CAN_RI0R_RTR_Pos                     (1U)
#define CAN_RI0R_RTR_Msk                     (0x1UL << CAN_RI0R_RTR_Pos)        /*!< 0x00000002 */
#define CAN_RI0R_RTR                         CAN_RI0R_RTR_Msk                  /*!< Remote Transmission Request */
#define CAN_RI0R_IDE_Pos                     (2U)
#define CAN_RI0R_IDE_Msk                     (0x1UL << CAN_RI0R_IDE_Pos)        /*!< 0x00000004 */
#define CAN_RI0R_IDE                         CAN_RI0R_IDE_Msk                  /*!< Identifier Extension */
#define CAN_RI0R_EXID_Pos                    (3U)
#define CAN_RI0R_EXID_Msk                    (0x3FFFFUL << CAN_RI0R_EXID_Pos)   /*!< 0x001FFFF8 */
#define CAN_RI0R_EXID                        CAN_RI0R_EXID_Msk                 /*!< Extended Identifier */
#define CAN_RI0R_STID_Pos                    (21U)
#define CAN_RI0R_STID_Msk                    (0x7FFUL << CAN_RI0R_STID_Pos)     /*!< 0xFFE00000 */
#define CAN_RI0R_STID                        CAN_RI0R_STID_Msk                 /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
#define CAN_RDT0R_DLC_Pos                    (0U)
#define CAN_RDT0R_DLC_Msk                    (0xFUL << CAN_RDT0R_DLC_Pos)       /*!< 0x0000000F */
#define CAN_RDT0R_DLC                        CAN_RDT0R_DLC_Msk                 /*!< Data Length Code */
#define CAN_RDT0R_FMI_Pos                    (8U)
#define CAN_RDT0R_FMI_Msk                    (0xFFUL << CAN_RDT0R_FMI_Pos)      /*!< 0x0000FF00 */
#define CAN_RDT0R_FMI                        CAN_RDT0R_FMI_Msk                 /*!< Filter Match Index */
#define CAN_RDT0R_TIME_Pos                   (16U)
#define CAN_RDT0R_TIME_Msk                   (0xFFFFUL << CAN_RDT0R_TIME_Pos)   /*!< 0xFFFF0000 */
#define CAN_RDT0R_TIME                       CAN_RDT0R_TIME_Msk                /*!< Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
#define CAN_RDL0R_DATA0_Pos                  (0U)
#define CAN_RDL0R_DATA0_Msk                  (0xFFUL << CAN_RDL0R_DATA0_Pos)    /*!< 0x000000FF */
#define CAN_RDL0R_DATA0                      CAN_RDL0R_DATA0_Msk               /*!< Data byte 0 */
#define CAN_RDL0R_DATA1_Pos                  (8U)
#define CAN_RDL0R_DATA1_Msk                  (0xFFUL << CAN_RDL0R_DATA1_Pos)    /*!< 0x0000FF00 */
#define CAN_RDL0R_DATA1                      CAN_RDL0R_DATA1_Msk               /*!< Data byte 1 */
#define CAN_RDL0R_DATA2_Pos                  (16U)
#define CAN_RDL0R_DATA2_Msk                  (0xFFUL << CAN_RDL0R_DATA2_Pos)    /*!< 0x00FF0000 */
#define CAN_RDL0R_DATA2                      CAN_RDL0R_DATA2_Msk               /*!< Data byte 2 */
#define CAN_RDL0R_DATA3_Pos                  (24U)
#define CAN_RDL0R_DATA3_Msk                  (0xFFUL << CAN_RDL0R_DATA3_Pos)    /*!< 0xFF000000 */
#define CAN_RDL0R_DATA3                      CAN_RDL0R_DATA3_Msk               /*!< Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
#define CAN_RDH0R_DATA4_Pos                  (0U)
#define CAN_RDH0R_DATA4_Msk                  (0xFFUL << CAN_RDH0R_DATA4_Pos)    /*!< 0x000000FF */
#define CAN_RDH0R_DATA4                      CAN_RDH0R_DATA4_Msk               /*!< Data byte 4 */
#define CAN_RDH0R_DATA5_Pos                  (8U)
#define CAN_RDH0R_DATA5_Msk                  (0xFFUL << CAN_RDH0R_DATA5_Pos)    /*!< 0x0000FF00 */
#define CAN_RDH0R_DATA5                      CAN_RDH0R_DATA5_Msk               /*!< Data byte 5 */
#define CAN_RDH0R_DATA6_Pos                  (16U)
#define CAN_RDH0R_DATA6_Msk                  (0xFFUL << CAN_RDH0R_DATA6_Pos)    /*!< 0x00FF0000 */
#define CAN_RDH0R_DATA6                      CAN_RDH0R_DATA6_Msk               /*!< Data byte 6 */
#define CAN_RDH0R_DATA7_Pos                  (24U)
#define CAN_RDH0R_DATA7_Msk                  (0xFFUL << CAN_RDH0R_DATA7_Pos)    /*!< 0xFF000000 */
#define CAN_RDH0R_DATA7                      CAN_RDH0R_DATA7_Msk               /*!< Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
#define CAN_RI1R_RTR_Pos                     (1U)
#define CAN_RI1R_RTR_Msk                     (0x1UL << CAN_RI1R_RTR_Pos)        /*!< 0x00000002 */
#define CAN_RI1R_RTR                         CAN_RI1R_RTR_Msk                  /*!< Remote Transmission Request */
#define CAN_RI1R_IDE_Pos                     (2U)
#define CAN_RI1R_IDE_Msk                     (0x1UL << CAN_RI1R_IDE_Pos)        /*!< 0x00000004 */
#define CAN_RI1R_IDE                         CAN_RI1R_IDE_Msk                  /*!< Identifier Extension */
#define CAN_RI1R_EXID_Pos                    (3U)
#define CAN_RI1R_EXID_Msk                    (0x3FFFFUL << CAN_RI1R_EXID_Pos)   /*!< 0x001FFFF8 */
#define CAN_RI1R_EXID                        CAN_RI1R_EXID_Msk                 /*!< Extended identifier */
#define CAN_RI1R_STID_Pos                    (21U)
#define CAN_RI1R_STID_Msk                    (0x7FFUL << CAN_RI1R_STID_Pos)     /*!< 0xFFE00000 */
#define CAN_RI1R_STID                        CAN_RI1R_STID_Msk                 /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
#define CAN_RDT1R_DLC_Pos                    (0U)
#define CAN_RDT1R_DLC_Msk                    (0xFUL << CAN_RDT1R_DLC_Pos)       /*!< 0x0000000F */
#define CAN_RDT1R_DLC                        CAN_RDT1R_DLC_Msk                 /*!< Data Length Code */
#define CAN_RDT1R_FMI_Pos                    (8U)
#define CAN_RDT1R_FMI_Msk                    (0xFFUL << CAN_RDT1R_FMI_Pos)      /*!< 0x0000FF00 */
#define CAN_RDT1R_FMI                        CAN_RDT1R_FMI_Msk                 /*!< Filter Match Index */
#define CAN_RDT1R_TIME_Pos                   (16U)
#define CAN_RDT1R_TIME_Msk                   (0xFFFFUL << CAN_RDT1R_TIME_Pos)   /*!< 0xFFFF0000 */
#define CAN_RDT1R_TIME                       CAN_RDT1R_TIME_Msk                /*!< Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
#define CAN_RDL1R_DATA0_Pos                  (0U)
#define CAN_RDL1R_DATA0_Msk                  (0xFFUL << CAN_RDL1R_DATA0_Pos)    /*!< 0x000000FF */
#define CAN_RDL1R_DATA0                      CAN_RDL1R_DATA0_Msk               /*!< Data byte 0 */
#define CAN_RDL1R_DATA1_Pos                  (8U)
#define CAN_RDL1R_DATA1_Msk                  (0xFFUL << CAN_RDL1R_DATA1_Pos)    /*!< 0x0000FF00 */
#define CAN_RDL1R_DATA1                      CAN_RDL1R_DATA1_Msk               /*!< Data byte 1 */
#define CAN_RDL1R_DATA2_Pos                  (16U)
#define CAN_RDL1R_DATA2_Msk                  (0xFFUL << CAN_RDL1R_DATA2_Pos)    /*!< 0x00FF0000 */
#define CAN_RDL1R_DATA2                      CAN_RDL1R_DATA2_Msk               /*!< Data byte 2 */
#define CAN_RDL1R_DATA3_Pos                  (24U)
#define CAN_RDL1R_DATA3_Msk                  (0xFFUL << CAN_RDL1R_DATA3_Pos)    /*!< 0xFF000000 */
#define CAN_RDL1R_DATA3                      CAN_RDL1R_DATA3_Msk               /*!< Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
#define CAN_RDH1R_DATA4_Pos                  (0U)
#define CAN_RDH1R_DATA4_Msk                  (0xFFUL << CAN_RDH1R_DATA4_Pos)    /*!< 0x000000FF */
#define CAN_RDH1R_DATA4                      CAN_RDH1R_DATA4_Msk               /*!< Data byte 4 */
#define CAN_RDH1R_DATA5_Pos                  (8U)
#define CAN_RDH1R_DATA5_Msk                  (0xFFUL << CAN_RDH1R_DATA5_Pos)    /*!< 0x0000FF00 */
#define CAN_RDH1R_DATA5                      CAN_RDH1R_DATA5_Msk               /*!< Data byte 5 */
#define CAN_RDH1R_DATA6_Pos                  (16U)
#define CAN_RDH1R_DATA6_Msk                  (0xFFUL << CAN_RDH1R_DATA6_Pos)    /*!< 0x00FF0000 */
#define CAN_RDH1R_DATA6                      CAN_RDH1R_DATA6_Msk               /*!< Data byte 6 */
#define CAN_RDH1R_DATA7_Pos                  (24U)
#define CAN_RDH1R_DATA7_Msk                  (0xFFUL << CAN_RDH1R_DATA7_Pos)    /*!< 0xFF000000 */
#define CAN_RDH1R_DATA7                      CAN_RDH1R_DATA7_Msk               /*!< Data byte 7 */

/*!< CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define CAN_FMR_FINIT_Pos                    (0U)
#define CAN_FMR_FINIT_Msk                    (0x1UL << CAN_FMR_FINIT_Pos)       /*!< 0x00000001 */
#define CAN_FMR_FINIT                        CAN_FMR_FINIT_Msk                 /*!< Filter Init Mode */
#define CAN_FMR_CAN2SB_Pos                   (8U)
#define CAN_FMR_CAN2SB_Msk                   (0x3FUL << CAN_FMR_CAN2SB_Pos)     /*!< 0x00003F00 */
#define CAN_FMR_CAN2SB                       CAN_FMR_CAN2SB_Msk                /*!< CAN2 start bank */

/*******************  Bit definition for CAN_FM1R register  *******************/
#define CAN_FM1R_FBM_Pos                     (0U)
#define CAN_FM1R_FBM_Msk                     (0x3FFFUL << CAN_FM1R_FBM_Pos)     /*!< 0x00003FFF */
#define CAN_FM1R_FBM                         CAN_FM1R_FBM_Msk                  /*!< Filter Mode */
#define CAN_FM1R_FBM0_Pos                    (0U)
#define CAN_FM1R_FBM0_Msk                    (0x1UL << CAN_FM1R_FBM0_Pos)       /*!< 0x00000001 */
#define CAN_FM1R_FBM0                        CAN_FM1R_FBM0_Msk                 /*!< Filter Init Mode for filter 0 */
#define CAN_FM1R_FBM1_Pos                    (1U)
#define CAN_FM1R_FBM1_Msk                    (0x1UL << CAN_FM1R_FBM1_Pos)       /*!< 0x00000002 */
#define CAN_FM1R_FBM1                        CAN_FM1R_FBM1_Msk                 /*!< Filter Init Mode for filter 1 */
#define CAN_FM1R_FBM2_Pos                    (2U)
#define CAN_FM1R_FBM2_Msk                    (0x1UL << CAN_FM1R_FBM2_Pos)       /*!< 0x00000004 */
#define CAN_FM1R_FBM2                        CAN_FM1R_FBM2_Msk                 /*!< Filter Init Mode for filter 2 */
#define CAN_FM1R_FBM3_Pos                    (3U)
#define CAN_FM1R_FBM3_Msk                    (0x1UL << CAN_FM1R_FBM3_Pos)       /*!< 0x00000008 */
#define CAN_FM1R_FBM3                        CAN_FM1R_FBM3_Msk                 /*!< Filter Init Mode for filter 3 */
#define CAN_FM1R_FBM4_Pos                    (4U)
#define CAN_FM1R_FBM4_Msk                    (0x1UL << CAN_FM1R_FBM4_Pos)       /*!< 0x00000010 */
#define CAN_FM1R_FBM4                        CAN_FM1R_FBM4_Msk                 /*!< Filter Init Mode for filter 4 */
#define CAN_FM1R_FBM5_Pos                    (5U)
#define CAN_FM1R_FBM5_Msk                    (0x1UL << CAN_FM1R_FBM5_Pos)       /*!< 0x00000020 */
#define CAN_FM1R_FBM5                        CAN_FM1R_FBM5_Msk                 /*!< Filter Init Mode for filter 5 */
#define CAN_FM1R_FBM6_Pos                    (6U)
#define CAN_FM1R_FBM6_Msk                    (0x1UL << CAN_FM1R_FBM6_Pos)       /*!< 0x00000040 */
#define CAN_FM1R_FBM6                        CAN_FM1R_FBM6_Msk                 /*!< Filter Init Mode for filter 6 */
#define CAN_FM1R_FBM7_Pos                    (7U)
#define CAN_FM1R_FBM7_Msk                    (0x1UL << CAN_FM1R_FBM7_Pos)       /*!< 0x00000080 */
#define CAN_FM1R_FBM7                        CAN_FM1R_FBM7_Msk                 /*!< Filter Init Mode for filter 7 */
#define CAN_FM1R_FBM8_Pos                    (8U)
#define CAN_FM1R_FBM8_Msk                    (0x1UL << CAN_FM1R_FBM8_Pos)       /*!< 0x00000100 */
#define CAN_FM1R_FBM8                        CAN_FM1R_FBM8_Msk                 /*!< Filter Init Mode for filter 8 */
#define CAN_FM1R_FBM9_Pos                    (9U)
#define CAN_FM1R_FBM9_Msk                    (0x1UL << CAN_FM1R_FBM9_Pos)       /*!< 0x00000200 */
#define CAN_FM1R_FBM9                        CAN_FM1R_FBM9_Msk                 /*!< Filter Init Mode for filter 9 */
#define CAN_FM1R_FBM10_Pos                   (10U)
#define CAN_FM1R_FBM10_Msk                   (0x1UL << CAN_FM1R_FBM10_Pos)      /*!< 0x00000400 */
#define CAN_FM1R_FBM10                       CAN_FM1R_FBM10_Msk                /*!< Filter Init Mode for filter 10 */
#define CAN_FM1R_FBM11_Pos                   (11U)
#define CAN_FM1R_FBM11_Msk                   (0x1UL << CAN_FM1R_FBM11_Pos)      /*!< 0x00000800 */
#define CAN_FM1R_FBM11                       CAN_FM1R_FBM11_Msk                /*!< Filter Init Mode for filter 11 */
#define CAN_FM1R_FBM12_Pos                   (12U)
#define CAN_FM1R_FBM12_Msk                   (0x1UL << CAN_FM1R_FBM12_Pos)      /*!< 0x00001000 */
#define CAN_FM1R_FBM12                       CAN_FM1R_FBM12_Msk                /*!< Filter Init Mode for filter 12 */
#define CAN_FM1R_FBM13_Pos                   (13U)
#define CAN_FM1R_FBM13_Msk                   (0x1UL << CAN_FM1R_FBM13_Pos)      /*!< 0x00002000 */
#define CAN_FM1R_FBM13                       CAN_FM1R_FBM13_Msk                /*!< Filter Init Mode for filter 13 */

/*******************  Bit definition for CAN_FS1R register  *******************/
#define CAN_FS1R_FSC_Pos                     (0U)
#define CAN_FS1R_FSC_Msk                     (0x3FFFUL << CAN_FS1R_FSC_Pos)     /*!< 0x00003FFF */
#define CAN_FS1R_FSC                         CAN_FS1R_FSC_Msk                  /*!< Filter Scale Configuration */
#define CAN_FS1R_FSC0_Pos                    (0U)
#define CAN_FS1R_FSC0_Msk                    (0x1UL << CAN_FS1R_FSC0_Pos)       /*!< 0x00000001 */
#define CAN_FS1R_FSC0                        CAN_FS1R_FSC0_Msk                 /*!< Filter Scale Configuration for filter 0 */
#define CAN_FS1R_FSC1_Pos                    (1U)
#define CAN_FS1R_FSC1_Msk                    (0x1UL << CAN_FS1R_FSC1_Pos)       /*!< 0x00000002 */
#define CAN_FS1R_FSC1                        CAN_FS1R_FSC1_Msk                 /*!< Filter Scale Configuration for filter 1 */
#define CAN_FS1R_FSC2_Pos                    (2U)
#define CAN_FS1R_FSC2_Msk                    (0x1UL << CAN_FS1R_FSC2_Pos)       /*!< 0x00000004 */
#define CAN_FS1R_FSC2                        CAN_FS1R_FSC2_Msk                 /*!< Filter Scale Configuration for filter 2 */
#define CAN_FS1R_FSC3_Pos                    (3U)
#define CAN_FS1R_FSC3_Msk                    (0x1UL << CAN_FS1R_FSC3_Pos)       /*!< 0x00000008 */
#define CAN_FS1R_FSC3                        CAN_FS1R_FSC3_Msk                 /*!< Filter Scale Configuration for filter 3 */
#define CAN_FS1R_FSC4_Pos                    (4U)
#define CAN_FS1R_FSC4_Msk                    (0x1UL << CAN_FS1R_FSC4_Pos)       /*!< 0x00000010 */
#define CAN_FS1R_FSC4                        CAN_FS1R_FSC4_Msk                 /*!< Filter Scale Configuration for filter 4 */
#define CAN_FS1R_FSC5_Pos                    (5U)
#define CAN_FS1R_FSC5_Msk                    (0x1UL << CAN_FS1R_FSC5_Pos)       /*!< 0x00000020 */
#define CAN_FS1R_FSC5                        CAN_FS1R_FSC5_Msk                 /*!< Filter Scale Configuration for filter 5 */
#define CAN_FS1R_FSC6_Pos                    (6U)
#define CAN_FS1R_FSC6_Msk                    (0x1UL << CAN_FS1R_FSC6_Pos)       /*!< 0x00000040 */
#define CAN_FS1R_FSC6                        CAN_FS1R_FSC6_Msk                 /*!< Filter Scale Configuration for filter 6 */
#define CAN_FS1R_FSC7_Pos                    (7U)
#define CAN_FS1R_FSC7_Msk                    (0x1UL << CAN_FS1R_FSC7_Pos)       /*!< 0x00000080 */
#define CAN_FS1R_FSC7                        CAN_FS1R_FSC7_Msk                 /*!< Filter Scale Configuration for filter 7 */
#define CAN_FS1R_FSC8_Pos                    (8U)
#define CAN_FS1R_FSC8_Msk                    (0x1UL << CAN_FS1R_FSC8_Pos)       /*!< 0x00000100 */
#define CAN_FS1R_FSC8                        CAN_FS1R_FSC8_Msk                 /*!< Filter Scale Configuration for filter 8 */
#define CAN_FS1R_FSC9_Pos                    (9U)
#define CAN_FS1R_FSC9_Msk                    (0x1UL << CAN_FS1R_FSC9_Pos)       /*!< 0x00000200 */
#define CAN_FS1R_FSC9                        CAN_FS1R_FSC9_Msk                 /*!< Filter Scale Configuration for filter 9 */
#define CAN_FS1R_FSC10_Pos                   (10U)
#define CAN_FS1R_FSC10_Msk                   (0x1UL << CAN_FS1R_FSC10_Pos)      /*!< 0x00000400 */
#define CAN_FS1R_FSC10                       CAN_FS1R_FSC10_Msk                /*!< Filter Scale Configuration for filter 10 */
#define CAN_FS1R_FSC11_Pos                   (11U)
#define CAN_FS1R_FSC11_Msk                   (0x1UL << CAN_FS1R_FSC11_Pos)      /*!< 0x00000800 */
#define CAN_FS1R_FSC11                       CAN_FS1R_FSC11_Msk                /*!< Filter Scale Configuration for filter 11 */
#define CAN_FS1R_FSC12_Pos                   (12U)
#define CAN_FS1R_FSC12_Msk                   (0x1UL << CAN_FS1R_FSC12_Pos)      /*!< 0x00001000 */
#define CAN_FS1R_FSC12                       CAN_FS1R_FSC12_Msk                /*!< Filter Scale Configuration for filter 12 */
#define CAN_FS1R_FSC13_Pos                   (13U)
#define CAN_FS1R_FSC13_Msk                   (0x1UL << CAN_FS1R_FSC13_Pos)      /*!< 0x00002000 */
#define CAN_FS1R_FSC13                       CAN_FS1R_FSC13_Msk                /*!< Filter Scale Configuration for filter 13 */

/******************  Bit definition for CAN_FFA1R register  *******************/
#define CAN_FFA1R_FFA_Pos                    (0U)
#define CAN_FFA1R_FFA_Msk                    (0x3FFFUL << CAN_FFA1R_FFA_Pos)    /*!< 0x00003FFF */
#define CAN_FFA1R_FFA                        CAN_FFA1R_FFA_Msk                 /*!< Filter FIFO Assignment */
#define CAN_FFA1R_FFA0_Pos                   (0U)
#define CAN_FFA1R_FFA0_Msk                   (0x1UL << CAN_FFA1R_FFA0_Pos)      /*!< 0x00000001 */
#define CAN_FFA1R_FFA0                       CAN_FFA1R_FFA0_Msk                /*!< Filter FIFO Assignment for filter 0 */
#define CAN_FFA1R_FFA1_Pos                   (1U)
#define CAN_FFA1R_FFA1_Msk                   (0x1UL << CAN_FFA1R_FFA1_Pos)      /*!< 0x00000002 */
#define CAN_FFA1R_FFA1                       CAN_FFA1R_FFA1_Msk                /*!< Filter FIFO Assignment for filter 1 */
#define CAN_FFA1R_FFA2_Pos                   (2U)
#define CAN_FFA1R_FFA2_Msk                   (0x1UL << CAN_FFA1R_FFA2_Pos)      /*!< 0x00000004 */
#define CAN_FFA1R_FFA2                       CAN_FFA1R_FFA2_Msk                /*!< Filter FIFO Assignment for filter 2 */
#define CAN_FFA1R_FFA3_Pos                   (3U)
#define CAN_FFA1R_FFA3_Msk                   (0x1UL << CAN_FFA1R_FFA3_Pos)      /*!< 0x00000008 */
#define CAN_FFA1R_FFA3                       CAN_FFA1R_FFA3_Msk                /*!< Filter FIFO Assignment for filter 3 */
#define CAN_FFA1R_FFA4_Pos                   (4U)
#define CAN_FFA1R_FFA4_Msk                   (0x1UL << CAN_FFA1R_FFA4_Pos)      /*!< 0x00000010 */
#define CAN_FFA1R_FFA4                       CAN_FFA1R_FFA4_Msk                /*!< Filter FIFO Assignment for filter 4 */
#define CAN_FFA1R_FFA5_Pos                   (5U)
#define CAN_FFA1R_FFA5_Msk                   (0x1UL << CAN_FFA1R_FFA5_Pos)      /*!< 0x00000020 */
#define CAN_FFA1R_FFA5                       CAN_FFA1R_FFA5_Msk                /*!< Filter FIFO Assignment for filter 5 */
#define CAN_FFA1R_FFA6_Pos                   (6U)
#define CAN_FFA1R_FFA6_Msk                   (0x1UL << CAN_FFA1R_FFA6_Pos)      /*!< 0x00000040 */
#define CAN_FFA1R_FFA6                       CAN_FFA1R_FFA6_Msk                /*!< Filter FIFO Assignment for filter 6 */
#define CAN_FFA1R_FFA7_Pos                   (7U)
#define CAN_FFA1R_FFA7_Msk                   (0x1UL << CAN_FFA1R_FFA7_Pos)      /*!< 0x00000080 */
#define CAN_FFA1R_FFA7                       CAN_FFA1R_FFA7_Msk                /*!< Filter FIFO Assignment for filter 7 */
#define CAN_FFA1R_FFA8_Pos                   (8U)
#define CAN_FFA1R_FFA8_Msk                   (0x1UL << CAN_FFA1R_FFA8_Pos)      /*!< 0x00000100 */
#define CAN_FFA1R_FFA8                       CAN_FFA1R_FFA8_Msk                /*!< Filter FIFO Assignment for filter 8 */
#define CAN_FFA1R_FFA9_Pos                   (9U)
#define CAN_FFA1R_FFA9_Msk                   (0x1UL << CAN_FFA1R_FFA9_Pos)      /*!< 0x00000200 */
#define CAN_FFA1R_FFA9                       CAN_FFA1R_FFA9_Msk                /*!< Filter FIFO Assignment for filter 9 */
#define CAN_FFA1R_FFA10_Pos                  (10U)
#define CAN_FFA1R_FFA10_Msk                  (0x1UL << CAN_FFA1R_FFA10_Pos)     /*!< 0x00000400 */
#define CAN_FFA1R_FFA10                      CAN_FFA1R_FFA10_Msk               /*!< Filter FIFO Assignment for filter 10 */
#define CAN_FFA1R_FFA11_Pos                  (11U)
#define CAN_FFA1R_FFA11_Msk                  (0x1UL << CAN_FFA1R_FFA11_Pos)     /*!< 0x00000800 */
#define CAN_FFA1R_FFA11                      CAN_FFA1R_FFA11_Msk               /*!< Filter FIFO Assignment for filter 11 */
#define CAN_FFA1R_FFA12_Pos                  (12U)
#define CAN_FFA1R_FFA12_Msk                  (0x1UL << CAN_FFA1R_FFA12_Pos)     /*!< 0x00001000 */
#define CAN_FFA1R_FFA12                      CAN_FFA1R_FFA12_Msk               /*!< Filter FIFO Assignment for filter 12 */
#define CAN_FFA1R_FFA13_Pos                  (13U)
#define CAN_FFA1R_FFA13_Msk                  (0x1UL << CAN_FFA1R_FFA13_Pos)     /*!< 0x00002000 */
#define CAN_FFA1R_FFA13                      CAN_FFA1R_FFA13_Msk               /*!< Filter FIFO Assignment for filter 13 */

/*******************  Bit definition for CAN_FA1R register  *******************/
#define CAN_FA1R_FACT_Pos                    (0U)
#define CAN_FA1R_FACT_Msk                    (0x3FFFUL << CAN_FA1R_FACT_Pos)    /*!< 0x00003FFF */
#define CAN_FA1R_FACT                        CAN_FA1R_FACT_Msk                 /*!< Filter Active */
#define CAN_FA1R_FACT0_Pos                   (0U)
#define CAN_FA1R_FACT0_Msk                   (0x1UL << CAN_FA1R_FACT0_Pos)      /*!< 0x00000001 */
#define CAN_FA1R_FACT0                       CAN_FA1R_FACT0_Msk                /*!< Filter 0 Active */
#define CAN_FA1R_FACT1_Pos                   (1U)
#define CAN_FA1R_FACT1_Msk                   (0x1UL << CAN_FA1R_FACT1_Pos)      /*!< 0x00000002 */
#define CAN_FA1R_FACT1                       CAN_FA1R_FACT1_Msk                /*!< Filter 1 Active */
#define CAN_FA1R_FACT2_Pos                   (2U)
#define CAN_FA1R_FACT2_Msk                   (0x1UL << CAN_FA1R_FACT2_Pos)      /*!< 0x00000004 */
#define CAN_FA1R_FACT2                       CAN_FA1R_FACT2_Msk                /*!< Filter 2 Active */
#define CAN_FA1R_FACT3_Pos                   (3U)
#define CAN_FA1R_FACT3_Msk                   (0x1UL << CAN_FA1R_FACT3_Pos)      /*!< 0x00000008 */
#define CAN_FA1R_FACT3                       CAN_FA1R_FACT3_Msk                /*!< Filter 3 Active */
#define CAN_FA1R_FACT4_Pos                   (4U)
#define CAN_FA1R_FACT4_Msk                   (0x1UL << CAN_FA1R_FACT4_Pos)      /*!< 0x00000010 */
#define CAN_FA1R_FACT4                       CAN_FA1R_FACT4_Msk                /*!< Filter 4 Active */
#define CAN_FA1R_FACT5_Pos                   (5U)
#define CAN_FA1R_FACT5_Msk                   (0x1UL << CAN_FA1R_FACT5_Pos)      /*!< 0x00000020 */
#define CAN_FA1R_FACT5                       CAN_FA1R_FACT5_Msk                /*!< Filter 5 Active */
#define CAN_FA1R_FACT6_Pos                   (6U)
#define CAN_FA1R_FACT6_Msk                   (0x1UL << CAN_FA1R_FACT6_Pos)      /*!< 0x00000040 */
#define CAN_FA1R_FACT6                       CAN_FA1R_FACT6_Msk                /*!< Filter 6 Active */
#define CAN_FA1R_FACT7_Pos                   (7U)
#define CAN_FA1R_FACT7_Msk                   (0x1UL << CAN_FA1R_FACT7_Pos)      /*!< 0x00000080 */
#define CAN_FA1R_FACT7                       CAN_FA1R_FACT7_Msk                /*!< Filter 7 Active */
#define CAN_FA1R_FACT8_Pos                   (8U)
#define CAN_FA1R_FACT8_Msk                   (0x1UL << CAN_FA1R_FACT8_Pos)      /*!< 0x00000100 */
#define CAN_FA1R_FACT8                       CAN_FA1R_FACT8_Msk                /*!< Filter 8 Active */
#define CAN_FA1R_FACT9_Pos                   (9U)
#define CAN_FA1R_FACT9_Msk                   (0x1UL << CAN_FA1R_FACT9_Pos)      /*!< 0x00000200 */
#define CAN_FA1R_FACT9                       CAN_FA1R_FACT9_Msk                /*!< Filter 9 Active */
#define CAN_FA1R_FACT10_Pos                  (10U)
#define CAN_FA1R_FACT10_Msk                  (0x1UL << CAN_FA1R_FACT10_Pos)     /*!< 0x00000400 */
#define CAN_FA1R_FACT10                      CAN_FA1R_FACT10_Msk               /*!< Filter 10 Active */
#define CAN_FA1R_FACT11_Pos                  (11U)
#define CAN_FA1R_FACT11_Msk                  (0x1UL << CAN_FA1R_FACT11_Pos)     /*!< 0x00000800 */
#define CAN_FA1R_FACT11                      CAN_FA1R_FACT11_Msk               /*!< Filter 11 Active */
#define CAN_FA1R_FACT12_Pos                  (12U)
#define CAN_FA1R_FACT12_Msk                  (0x1UL << CAN_FA1R_FACT12_Pos)     /*!< 0x00001000 */
#define CAN_FA1R_FACT12                      CAN_FA1R_FACT12_Msk               /*!< Filter 12 Active */
#define CAN_FA1R_FACT13_Pos                  (13U)
#define CAN_FA1R_FACT13_Msk                  (0x1UL << CAN_FA1R_FACT13_Pos)     /*!< 0x00002000 */
#define CAN_FA1R_FACT13                      CAN_FA1R_FACT13_Msk               /*!< Filter 13 Active */

/*******************  Bit definition for CAN_F0R1 register  *******************/
#define CAN_F0R1_FB0_Pos                     (0U)
#define CAN_F0R1_FB0_Msk                     (0x1UL << CAN_F0R1_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F0R1_FB0                         CAN_F0R1_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F0R1_FB1_Pos                     (1U)
#define CAN_F0R1_FB1_Msk                     (0x1UL << CAN_F0R1_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F0R1_FB1                         CAN_F0R1_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F0R1_FB2_Pos                     (2U)
#define CAN_F0R1_FB2_Msk                     (0x1UL << CAN_F0R1_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F0R1_FB2                         CAN_F0R1_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F0R1_FB3_Pos                     (3U)
#define CAN_F0R1_FB3_Msk                     (0x1UL << CAN_F0R1_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F0R1_FB3                         CAN_F0R1_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F0R1_FB4_Pos                     (4U)
#define CAN_F0R1_FB4_Msk                     (0x1UL << CAN_F0R1_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F0R1_FB4                         CAN_F0R1_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F0R1_FB5_Pos                     (5U)
#define CAN_F0R1_FB5_Msk                     (0x1UL << CAN_F0R1_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F0R1_FB5                         CAN_F0R1_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F0R1_FB6_Pos                     (6U)
#define CAN_F0R1_FB6_Msk                     (0x1UL << CAN_F0R1_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F0R1_FB6                         CAN_F0R1_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F0R1_FB7_Pos                     (7U)
#define CAN_F0R1_FB7_Msk                     (0x1UL << CAN_F0R1_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F0R1_FB7                         CAN_F0R1_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F0R1_FB8_Pos                     (8U)
#define CAN_F0R1_FB8_Msk                     (0x1UL << CAN_F0R1_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F0R1_FB8                         CAN_F0R1_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F0R1_FB9_Pos                     (9U)
#define CAN_F0R1_FB9_Msk                     (0x1UL << CAN_F0R1_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F0R1_FB9                         CAN_F0R1_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F0R1_FB10_Pos                    (10U)
#define CAN_F0R1_FB10_Msk                    (0x1UL << CAN_F0R1_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F0R1_FB10                        CAN_F0R1_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F0R1_FB11_Pos                    (11U)
#define CAN_F0R1_FB11_Msk                    (0x1UL << CAN_F0R1_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F0R1_FB11                        CAN_F0R1_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F0R1_FB12_Pos                    (12U)
#define CAN_F0R1_FB12_Msk                    (0x1UL << CAN_F0R1_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F0R1_FB12                        CAN_F0R1_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F0R1_FB13_Pos                    (13U)
#define CAN_F0R1_FB13_Msk                    (0x1UL << CAN_F0R1_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F0R1_FB13                        CAN_F0R1_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F0R1_FB14_Pos                    (14U)
#define CAN_F0R1_FB14_Msk                    (0x1UL << CAN_F0R1_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F0R1_FB14                        CAN_F0R1_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F0R1_FB15_Pos                    (15U)
#define CAN_F0R1_FB15_Msk                    (0x1UL << CAN_F0R1_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F0R1_FB15                        CAN_F0R1_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F0R1_FB16_Pos                    (16U)
#define CAN_F0R1_FB16_Msk                    (0x1UL << CAN_F0R1_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F0R1_FB16                        CAN_F0R1_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F0R1_FB17_Pos                    (17U)
#define CAN_F0R1_FB17_Msk                    (0x1UL << CAN_F0R1_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F0R1_FB17                        CAN_F0R1_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F0R1_FB18_Pos                    (18U)
#define CAN_F0R1_FB18_Msk                    (0x1UL << CAN_F0R1_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F0R1_FB18                        CAN_F0R1_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F0R1_FB19_Pos                    (19U)
#define CAN_F0R1_FB19_Msk                    (0x1UL << CAN_F0R1_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F0R1_FB19                        CAN_F0R1_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F0R1_FB20_Pos                    (20U)
#define CAN_F0R1_FB20_Msk                    (0x1UL << CAN_F0R1_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F0R1_FB20                        CAN_F0R1_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F0R1_FB21_Pos                    (21U)
#define CAN_F0R1_FB21_Msk                    (0x1UL << CAN_F0R1_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F0R1_FB21                        CAN_F0R1_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F0R1_FB22_Pos                    (22U)
#define CAN_F0R1_FB22_Msk                    (0x1UL << CAN_F0R1_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F0R1_FB22                        CAN_F0R1_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F0R1_FB23_Pos                    (23U)
#define CAN_F0R1_FB23_Msk                    (0x1UL << CAN_F0R1_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F0R1_FB23                        CAN_F0R1_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F0R1_FB24_Pos                    (24U)
#define CAN_F0R1_FB24_Msk                    (0x1UL << CAN_F0R1_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F0R1_FB24                        CAN_F0R1_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F0R1_FB25_Pos                    (25U)
#define CAN_F0R1_FB25_Msk                    (0x1UL << CAN_F0R1_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F0R1_FB25                        CAN_F0R1_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F0R1_FB26_Pos                    (26U)
#define CAN_F0R1_FB26_Msk                    (0x1UL << CAN_F0R1_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F0R1_FB26                        CAN_F0R1_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F0R1_FB27_Pos                    (27U)
#define CAN_F0R1_FB27_Msk                    (0x1UL << CAN_F0R1_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F0R1_FB27                        CAN_F0R1_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F0R1_FB28_Pos                    (28U)
#define CAN_F0R1_FB28_Msk                    (0x1UL << CAN_F0R1_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F0R1_FB28                        CAN_F0R1_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F0R1_FB29_Pos                    (29U)
#define CAN_F0R1_FB29_Msk                    (0x1UL << CAN_F0R1_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F0R1_FB29                        CAN_F0R1_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F0R1_FB30_Pos                    (30U)
#define CAN_F0R1_FB30_Msk                    (0x1UL << CAN_F0R1_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F0R1_FB30                        CAN_F0R1_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F0R1_FB31_Pos                    (31U)
#define CAN_F0R1_FB31_Msk                    (0x1UL << CAN_F0R1_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F0R1_FB31                        CAN_F0R1_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F1R1 register  *******************/
#define CAN_F1R1_FB0_Pos                     (0U)
#define CAN_F1R1_FB0_Msk                     (0x1UL << CAN_F1R1_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F1R1_FB0                         CAN_F1R1_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F1R1_FB1_Pos                     (1U)
#define CAN_F1R1_FB1_Msk                     (0x1UL << CAN_F1R1_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F1R1_FB1                         CAN_F1R1_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F1R1_FB2_Pos                     (2U)
#define CAN_F1R1_FB2_Msk                     (0x1UL << CAN_F1R1_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F1R1_FB2                         CAN_F1R1_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F1R1_FB3_Pos                     (3U)
#define CAN_F1R1_FB3_Msk                     (0x1UL << CAN_F1R1_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F1R1_FB3                         CAN_F1R1_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F1R1_FB4_Pos                     (4U)
#define CAN_F1R1_FB4_Msk                     (0x1UL << CAN_F1R1_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F1R1_FB4                         CAN_F1R1_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F1R1_FB5_Pos                     (5U)
#define CAN_F1R1_FB5_Msk                     (0x1UL << CAN_F1R1_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F1R1_FB5                         CAN_F1R1_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F1R1_FB6_Pos                     (6U)
#define CAN_F1R1_FB6_Msk                     (0x1UL << CAN_F1R1_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F1R1_FB6                         CAN_F1R1_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F1R1_FB7_Pos                     (7U)
#define CAN_F1R1_FB7_Msk                     (0x1UL << CAN_F1R1_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F1R1_FB7                         CAN_F1R1_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F1R1_FB8_Pos                     (8U)
#define CAN_F1R1_FB8_Msk                     (0x1UL << CAN_F1R1_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F1R1_FB8                         CAN_F1R1_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F1R1_FB9_Pos                     (9U)
#define CAN_F1R1_FB9_Msk                     (0x1UL << CAN_F1R1_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F1R1_FB9                         CAN_F1R1_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F1R1_FB10_Pos                    (10U)
#define CAN_F1R1_FB10_Msk                    (0x1UL << CAN_F1R1_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F1R1_FB10                        CAN_F1R1_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F1R1_FB11_Pos                    (11U)
#define CAN_F1R1_FB11_Msk                    (0x1UL << CAN_F1R1_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F1R1_FB11                        CAN_F1R1_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F1R1_FB12_Pos                    (12U)
#define CAN_F1R1_FB12_Msk                    (0x1UL << CAN_F1R1_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F1R1_FB12                        CAN_F1R1_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F1R1_FB13_Pos                    (13U)
#define CAN_F1R1_FB13_Msk                    (0x1UL << CAN_F1R1_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F1R1_FB13                        CAN_F1R1_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F1R1_FB14_Pos                    (14U)
#define CAN_F1R1_FB14_Msk                    (0x1UL << CAN_F1R1_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F1R1_FB14                        CAN_F1R1_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F1R1_FB15_Pos                    (15U)
#define CAN_F1R1_FB15_Msk                    (0x1UL << CAN_F1R1_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F1R1_FB15                        CAN_F1R1_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F1R1_FB16_Pos                    (16U)
#define CAN_F1R1_FB16_Msk                    (0x1UL << CAN_F1R1_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F1R1_FB16                        CAN_F1R1_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F1R1_FB17_Pos                    (17U)
#define CAN_F1R1_FB17_Msk                    (0x1UL << CAN_F1R1_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F1R1_FB17                        CAN_F1R1_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F1R1_FB18_Pos                    (18U)
#define CAN_F1R1_FB18_Msk                    (0x1UL << CAN_F1R1_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F1R1_FB18                        CAN_F1R1_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F1R1_FB19_Pos                    (19U)
#define CAN_F1R1_FB19_Msk                    (0x1UL << CAN_F1R1_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F1R1_FB19                        CAN_F1R1_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F1R1_FB20_Pos                    (20U)
#define CAN_F1R1_FB20_Msk                    (0x1UL << CAN_F1R1_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F1R1_FB20                        CAN_F1R1_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F1R1_FB21_Pos                    (21U)
#define CAN_F1R1_FB21_Msk                    (0x1UL << CAN_F1R1_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F1R1_FB21                        CAN_F1R1_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F1R1_FB22_Pos                    (22U)
#define CAN_F1R1_FB22_Msk                    (0x1UL << CAN_F1R1_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F1R1_FB22                        CAN_F1R1_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F1R1_FB23_Pos                    (23U)
#define CAN_F1R1_FB23_Msk                    (0x1UL << CAN_F1R1_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F1R1_FB23                        CAN_F1R1_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F1R1_FB24_Pos                    (24U)
#define CAN_F1R1_FB24_Msk                    (0x1UL << CAN_F1R1_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F1R1_FB24                        CAN_F1R1_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F1R1_FB25_Pos                    (25U)
#define CAN_F1R1_FB25_Msk                    (0x1UL << CAN_F1R1_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F1R1_FB25                        CAN_F1R1_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F1R1_FB26_Pos                    (26U)
#define CAN_F1R1_FB26_Msk                    (0x1UL << CAN_F1R1_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F1R1_FB26                        CAN_F1R1_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F1R1_FB27_Pos                    (27U)
#define CAN_F1R1_FB27_Msk                    (0x1UL << CAN_F1R1_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F1R1_FB27                        CAN_F1R1_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F1R1_FB28_Pos                    (28U)
#define CAN_F1R1_FB28_Msk                    (0x1UL << CAN_F1R1_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F1R1_FB28                        CAN_F1R1_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F1R1_FB29_Pos                    (29U)
#define CAN_F1R1_FB29_Msk                    (0x1UL << CAN_F1R1_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F1R1_FB29                        CAN_F1R1_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F1R1_FB30_Pos                    (30U)
#define CAN_F1R1_FB30_Msk                    (0x1UL << CAN_F1R1_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F1R1_FB30                        CAN_F1R1_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F1R1_FB31_Pos                    (31U)
#define CAN_F1R1_FB31_Msk                    (0x1UL << CAN_F1R1_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F1R1_FB31                        CAN_F1R1_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F2R1 register  *******************/
#define CAN_F2R1_FB0_Pos                     (0U)
#define CAN_F2R1_FB0_Msk                     (0x1UL << CAN_F2R1_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F2R1_FB0                         CAN_F2R1_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F2R1_FB1_Pos                     (1U)
#define CAN_F2R1_FB1_Msk                     (0x1UL << CAN_F2R1_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F2R1_FB1                         CAN_F2R1_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F2R1_FB2_Pos                     (2U)
#define CAN_F2R1_FB2_Msk                     (0x1UL << CAN_F2R1_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F2R1_FB2                         CAN_F2R1_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F2R1_FB3_Pos                     (3U)
#define CAN_F2R1_FB3_Msk                     (0x1UL << CAN_F2R1_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F2R1_FB3                         CAN_F2R1_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F2R1_FB4_Pos                     (4U)
#define CAN_F2R1_FB4_Msk                     (0x1UL << CAN_F2R1_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F2R1_FB4                         CAN_F2R1_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F2R1_FB5_Pos                     (5U)
#define CAN_F2R1_FB5_Msk                     (0x1UL << CAN_F2R1_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F2R1_FB5                         CAN_F2R1_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F2R1_FB6_Pos                     (6U)
#define CAN_F2R1_FB6_Msk                     (0x1UL << CAN_F2R1_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F2R1_FB6                         CAN_F2R1_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F2R1_FB7_Pos                     (7U)
#define CAN_F2R1_FB7_Msk                     (0x1UL << CAN_F2R1_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F2R1_FB7                         CAN_F2R1_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F2R1_FB8_Pos                     (8U)
#define CAN_F2R1_FB8_Msk                     (0x1UL << CAN_F2R1_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F2R1_FB8                         CAN_F2R1_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F2R1_FB9_Pos                     (9U)
#define CAN_F2R1_FB9_Msk                     (0x1UL << CAN_F2R1_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F2R1_FB9                         CAN_F2R1_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F2R1_FB10_Pos                    (10U)
#define CAN_F2R1_FB10_Msk                    (0x1UL << CAN_F2R1_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F2R1_FB10                        CAN_F2R1_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F2R1_FB11_Pos                    (11U)
#define CAN_F2R1_FB11_Msk                    (0x1UL << CAN_F2R1_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F2R1_FB11                        CAN_F2R1_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F2R1_FB12_Pos                    (12U)
#define CAN_F2R1_FB12_Msk                    (0x1UL << CAN_F2R1_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F2R1_FB12                        CAN_F2R1_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F2R1_FB13_Pos                    (13U)
#define CAN_F2R1_FB13_Msk                    (0x1UL << CAN_F2R1_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F2R1_FB13                        CAN_F2R1_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F2R1_FB14_Pos                    (14U)
#define CAN_F2R1_FB14_Msk                    (0x1UL << CAN_F2R1_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F2R1_FB14                        CAN_F2R1_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F2R1_FB15_Pos                    (15U)
#define CAN_F2R1_FB15_Msk                    (0x1UL << CAN_F2R1_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F2R1_FB15                        CAN_F2R1_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F2R1_FB16_Pos                    (16U)
#define CAN_F2R1_FB16_Msk                    (0x1UL << CAN_F2R1_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F2R1_FB16                        CAN_F2R1_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F2R1_FB17_Pos                    (17U)
#define CAN_F2R1_FB17_Msk                    (0x1UL << CAN_F2R1_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F2R1_FB17                        CAN_F2R1_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F2R1_FB18_Pos                    (18U)
#define CAN_F2R1_FB18_Msk                    (0x1UL << CAN_F2R1_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F2R1_FB18                        CAN_F2R1_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F2R1_FB19_Pos                    (19U)
#define CAN_F2R1_FB19_Msk                    (0x1UL << CAN_F2R1_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F2R1_FB19                        CAN_F2R1_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F2R1_FB20_Pos                    (20U)
#define CAN_F2R1_FB20_Msk                    (0x1UL << CAN_F2R1_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F2R1_FB20                        CAN_F2R1_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F2R1_FB21_Pos                    (21U)
#define CAN_F2R1_FB21_Msk                    (0x1UL << CAN_F2R1_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F2R1_FB21                        CAN_F2R1_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F2R1_FB22_Pos                    (22U)
#define CAN_F2R1_FB22_Msk                    (0x1UL << CAN_F2R1_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F2R1_FB22                        CAN_F2R1_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F2R1_FB23_Pos                    (23U)
#define CAN_F2R1_FB23_Msk                    (0x1UL << CAN_F2R1_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F2R1_FB23                        CAN_F2R1_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F2R1_FB24_Pos                    (24U)
#define CAN_F2R1_FB24_Msk                    (0x1UL << CAN_F2R1_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F2R1_FB24                        CAN_F2R1_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F2R1_FB25_Pos                    (25U)
#define CAN_F2R1_FB25_Msk                    (0x1UL << CAN_F2R1_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F2R1_FB25                        CAN_F2R1_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F2R1_FB26_Pos                    (26U)
#define CAN_F2R1_FB26_Msk                    (0x1UL << CAN_F2R1_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F2R1_FB26                        CAN_F2R1_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F2R1_FB27_Pos                    (27U)
#define CAN_F2R1_FB27_Msk                    (0x1UL << CAN_F2R1_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F2R1_FB27                        CAN_F2R1_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F2R1_FB28_Pos                    (28U)
#define CAN_F2R1_FB28_Msk                    (0x1UL << CAN_F2R1_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F2R1_FB28                        CAN_F2R1_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F2R1_FB29_Pos                    (29U)
#define CAN_F2R1_FB29_Msk                    (0x1UL << CAN_F2R1_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F2R1_FB29                        CAN_F2R1_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F2R1_FB30_Pos                    (30U)
#define CAN_F2R1_FB30_Msk                    (0x1UL << CAN_F2R1_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F2R1_FB30                        CAN_F2R1_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F2R1_FB31_Pos                    (31U)
#define CAN_F2R1_FB31_Msk                    (0x1UL << CAN_F2R1_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F2R1_FB31                        CAN_F2R1_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F3R1 register  *******************/
#define CAN_F3R1_FB0_Pos                     (0U)
#define CAN_F3R1_FB0_Msk                     (0x1UL << CAN_F3R1_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F3R1_FB0                         CAN_F3R1_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F3R1_FB1_Pos                     (1U)
#define CAN_F3R1_FB1_Msk                     (0x1UL << CAN_F3R1_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F3R1_FB1                         CAN_F3R1_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F3R1_FB2_Pos                     (2U)
#define CAN_F3R1_FB2_Msk                     (0x1UL << CAN_F3R1_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F3R1_FB2                         CAN_F3R1_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F3R1_FB3_Pos                     (3U)
#define CAN_F3R1_FB3_Msk                     (0x1UL << CAN_F3R1_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F3R1_FB3                         CAN_F3R1_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F3R1_FB4_Pos                     (4U)
#define CAN_F3R1_FB4_Msk                     (0x1UL << CAN_F3R1_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F3R1_FB4                         CAN_F3R1_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F3R1_FB5_Pos                     (5U)
#define CAN_F3R1_FB5_Msk                     (0x1UL << CAN_F3R1_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F3R1_FB5                         CAN_F3R1_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F3R1_FB6_Pos                     (6U)
#define CAN_F3R1_FB6_Msk                     (0x1UL << CAN_F3R1_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F3R1_FB6                         CAN_F3R1_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F3R1_FB7_Pos                     (7U)
#define CAN_F3R1_FB7_Msk                     (0x1UL << CAN_F3R1_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F3R1_FB7                         CAN_F3R1_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F3R1_FB8_Pos                     (8U)
#define CAN_F3R1_FB8_Msk                     (0x1UL << CAN_F3R1_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F3R1_FB8                         CAN_F3R1_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F3R1_FB9_Pos                     (9U)
#define CAN_F3R1_FB9_Msk                     (0x1UL << CAN_F3R1_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F3R1_FB9                         CAN_F3R1_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F3R1_FB10_Pos                    (10U)
#define CAN_F3R1_FB10_Msk                    (0x1UL << CAN_F3R1_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F3R1_FB10                        CAN_F3R1_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F3R1_FB11_Pos                    (11U)
#define CAN_F3R1_FB11_Msk                    (0x1UL << CAN_F3R1_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F3R1_FB11                        CAN_F3R1_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F3R1_FB12_Pos                    (12U)
#define CAN_F3R1_FB12_Msk                    (0x1UL << CAN_F3R1_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F3R1_FB12                        CAN_F3R1_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F3R1_FB13_Pos                    (13U)
#define CAN_F3R1_FB13_Msk                    (0x1UL << CAN_F3R1_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F3R1_FB13                        CAN_F3R1_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F3R1_FB14_Pos                    (14U)
#define CAN_F3R1_FB14_Msk                    (0x1UL << CAN_F3R1_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F3R1_FB14                        CAN_F3R1_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F3R1_FB15_Pos                    (15U)
#define CAN_F3R1_FB15_Msk                    (0x1UL << CAN_F3R1_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F3R1_FB15                        CAN_F3R1_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F3R1_FB16_Pos                    (16U)
#define CAN_F3R1_FB16_Msk                    (0x1UL << CAN_F3R1_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F3R1_FB16                        CAN_F3R1_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F3R1_FB17_Pos                    (17U)
#define CAN_F3R1_FB17_Msk                    (0x1UL << CAN_F3R1_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F3R1_FB17                        CAN_F3R1_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F3R1_FB18_Pos                    (18U)
#define CAN_F3R1_FB18_Msk                    (0x1UL << CAN_F3R1_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F3R1_FB18                        CAN_F3R1_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F3R1_FB19_Pos                    (19U)
#define CAN_F3R1_FB19_Msk                    (0x1UL << CAN_F3R1_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F3R1_FB19                        CAN_F3R1_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F3R1_FB20_Pos                    (20U)
#define CAN_F3R1_FB20_Msk                    (0x1UL << CAN_F3R1_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F3R1_FB20                        CAN_F3R1_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F3R1_FB21_Pos                    (21U)
#define CAN_F3R1_FB21_Msk                    (0x1UL << CAN_F3R1_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F3R1_FB21                        CAN_F3R1_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F3R1_FB22_Pos                    (22U)
#define CAN_F3R1_FB22_Msk                    (0x1UL << CAN_F3R1_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F3R1_FB22                        CAN_F3R1_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F3R1_FB23_Pos                    (23U)
#define CAN_F3R1_FB23_Msk                    (0x1UL << CAN_F3R1_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F3R1_FB23                        CAN_F3R1_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F3R1_FB24_Pos                    (24U)
#define CAN_F3R1_FB24_Msk                    (0x1UL << CAN_F3R1_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F3R1_FB24                        CAN_F3R1_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F3R1_FB25_Pos                    (25U)
#define CAN_F3R1_FB25_Msk                    (0x1UL << CAN_F3R1_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F3R1_FB25                        CAN_F3R1_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F3R1_FB26_Pos                    (26U)
#define CAN_F3R1_FB26_Msk                    (0x1UL << CAN_F3R1_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F3R1_FB26                        CAN_F3R1_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F3R1_FB27_Pos                    (27U)
#define CAN_F3R1_FB27_Msk                    (0x1UL << CAN_F3R1_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F3R1_FB27                        CAN_F3R1_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F3R1_FB28_Pos                    (28U)
#define CAN_F3R1_FB28_Msk                    (0x1UL << CAN_F3R1_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F3R1_FB28                        CAN_F3R1_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F3R1_FB29_Pos                    (29U)
#define CAN_F3R1_FB29_Msk                    (0x1UL << CAN_F3R1_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F3R1_FB29                        CAN_F3R1_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F3R1_FB30_Pos                    (30U)
#define CAN_F3R1_FB30_Msk                    (0x1UL << CAN_F3R1_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F3R1_FB30                        CAN_F3R1_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F3R1_FB31_Pos                    (31U)
#define CAN_F3R1_FB31_Msk                    (0x1UL << CAN_F3R1_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F3R1_FB31                        CAN_F3R1_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F4R1 register  *******************/
#define CAN_F4R1_FB0_Pos                     (0U)
#define CAN_F4R1_FB0_Msk                     (0x1UL << CAN_F4R1_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F4R1_FB0                         CAN_F4R1_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F4R1_FB1_Pos                     (1U)
#define CAN_F4R1_FB1_Msk                     (0x1UL << CAN_F4R1_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F4R1_FB1                         CAN_F4R1_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F4R1_FB2_Pos                     (2U)
#define CAN_F4R1_FB2_Msk                     (0x1UL << CAN_F4R1_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F4R1_FB2                         CAN_F4R1_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F4R1_FB3_Pos                     (3U)
#define CAN_F4R1_FB3_Msk                     (0x1UL << CAN_F4R1_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F4R1_FB3                         CAN_F4R1_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F4R1_FB4_Pos                     (4U)
#define CAN_F4R1_FB4_Msk                     (0x1UL << CAN_F4R1_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F4R1_FB4                         CAN_F4R1_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F4R1_FB5_Pos                     (5U)
#define CAN_F4R1_FB5_Msk                     (0x1UL << CAN_F4R1_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F4R1_FB5                         CAN_F4R1_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F4R1_FB6_Pos                     (6U)
#define CAN_F4R1_FB6_Msk                     (0x1UL << CAN_F4R1_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F4R1_FB6                         CAN_F4R1_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F4R1_FB7_Pos                     (7U)
#define CAN_F4R1_FB7_Msk                     (0x1UL << CAN_F4R1_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F4R1_FB7                         CAN_F4R1_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F4R1_FB8_Pos                     (8U)
#define CAN_F4R1_FB8_Msk                     (0x1UL << CAN_F4R1_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F4R1_FB8                         CAN_F4R1_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F4R1_FB9_Pos                     (9U)
#define CAN_F4R1_FB9_Msk                     (0x1UL << CAN_F4R1_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F4R1_FB9                         CAN_F4R1_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F4R1_FB10_Pos                    (10U)
#define CAN_F4R1_FB10_Msk                    (0x1UL << CAN_F4R1_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F4R1_FB10                        CAN_F4R1_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F4R1_FB11_Pos                    (11U)
#define CAN_F4R1_FB11_Msk                    (0x1UL << CAN_F4R1_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F4R1_FB11                        CAN_F4R1_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F4R1_FB12_Pos                    (12U)
#define CAN_F4R1_FB12_Msk                    (0x1UL << CAN_F4R1_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F4R1_FB12                        CAN_F4R1_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F4R1_FB13_Pos                    (13U)
#define CAN_F4R1_FB13_Msk                    (0x1UL << CAN_F4R1_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F4R1_FB13                        CAN_F4R1_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F4R1_FB14_Pos                    (14U)
#define CAN_F4R1_FB14_Msk                    (0x1UL << CAN_F4R1_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F4R1_FB14                        CAN_F4R1_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F4R1_FB15_Pos                    (15U)
#define CAN_F4R1_FB15_Msk                    (0x1UL << CAN_F4R1_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F4R1_FB15                        CAN_F4R1_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F4R1_FB16_Pos                    (16U)
#define CAN_F4R1_FB16_Msk                    (0x1UL << CAN_F4R1_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F4R1_FB16                        CAN_F4R1_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F4R1_FB17_Pos                    (17U)
#define CAN_F4R1_FB17_Msk                    (0x1UL << CAN_F4R1_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F4R1_FB17                        CAN_F4R1_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F4R1_FB18_Pos                    (18U)
#define CAN_F4R1_FB18_Msk                    (0x1UL << CAN_F4R1_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F4R1_FB18                        CAN_F4R1_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F4R1_FB19_Pos                    (19U)
#define CAN_F4R1_FB19_Msk                    (0x1UL << CAN_F4R1_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F4R1_FB19                        CAN_F4R1_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F4R1_FB20_Pos                    (20U)
#define CAN_F4R1_FB20_Msk                    (0x1UL << CAN_F4R1_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F4R1_FB20                        CAN_F4R1_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F4R1_FB21_Pos                    (21U)
#define CAN_F4R1_FB21_Msk                    (0x1UL << CAN_F4R1_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F4R1_FB21                        CAN_F4R1_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F4R1_FB22_Pos                    (22U)
#define CAN_F4R1_FB22_Msk                    (0x1UL << CAN_F4R1_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F4R1_FB22                        CAN_F4R1_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F4R1_FB23_Pos                    (23U)
#define CAN_F4R1_FB23_Msk                    (0x1UL << CAN_F4R1_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F4R1_FB23                        CAN_F4R1_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F4R1_FB24_Pos                    (24U)
#define CAN_F4R1_FB24_Msk                    (0x1UL << CAN_F4R1_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F4R1_FB24                        CAN_F4R1_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F4R1_FB25_Pos                    (25U)
#define CAN_F4R1_FB25_Msk                    (0x1UL << CAN_F4R1_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F4R1_FB25                        CAN_F4R1_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F4R1_FB26_Pos                    (26U)
#define CAN_F4R1_FB26_Msk                    (0x1UL << CAN_F4R1_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F4R1_FB26                        CAN_F4R1_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F4R1_FB27_Pos                    (27U)
#define CAN_F4R1_FB27_Msk                    (0x1UL << CAN_F4R1_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F4R1_FB27                        CAN_F4R1_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F4R1_FB28_Pos                    (28U)
#define CAN_F4R1_FB28_Msk                    (0x1UL << CAN_F4R1_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F4R1_FB28                        CAN_F4R1_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F4R1_FB29_Pos                    (29U)
#define CAN_F4R1_FB29_Msk                    (0x1UL << CAN_F4R1_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F4R1_FB29                        CAN_F4R1_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F4R1_FB30_Pos                    (30U)
#define CAN_F4R1_FB30_Msk                    (0x1UL << CAN_F4R1_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F4R1_FB30                        CAN_F4R1_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F4R1_FB31_Pos                    (31U)
#define CAN_F4R1_FB31_Msk                    (0x1UL << CAN_F4R1_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F4R1_FB31                        CAN_F4R1_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F5R1 register  *******************/
#define CAN_F5R1_FB0_Pos                     (0U)
#define CAN_F5R1_FB0_Msk                     (0x1UL << CAN_F5R1_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F5R1_FB0                         CAN_F5R1_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F5R1_FB1_Pos                     (1U)
#define CAN_F5R1_FB1_Msk                     (0x1UL << CAN_F5R1_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F5R1_FB1                         CAN_F5R1_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F5R1_FB2_Pos                     (2U)
#define CAN_F5R1_FB2_Msk                     (0x1UL << CAN_F5R1_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F5R1_FB2                         CAN_F5R1_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F5R1_FB3_Pos                     (3U)
#define CAN_F5R1_FB3_Msk                     (0x1UL << CAN_F5R1_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F5R1_FB3                         CAN_F5R1_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F5R1_FB4_Pos                     (4U)
#define CAN_F5R1_FB4_Msk                     (0x1UL << CAN_F5R1_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F5R1_FB4                         CAN_F5R1_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F5R1_FB5_Pos                     (5U)
#define CAN_F5R1_FB5_Msk                     (0x1UL << CAN_F5R1_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F5R1_FB5                         CAN_F5R1_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F5R1_FB6_Pos                     (6U)
#define CAN_F5R1_FB6_Msk                     (0x1UL << CAN_F5R1_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F5R1_FB6                         CAN_F5R1_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F5R1_FB7_Pos                     (7U)
#define CAN_F5R1_FB7_Msk                     (0x1UL << CAN_F5R1_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F5R1_FB7                         CAN_F5R1_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F5R1_FB8_Pos                     (8U)
#define CAN_F5R1_FB8_Msk                     (0x1UL << CAN_F5R1_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F5R1_FB8                         CAN_F5R1_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F5R1_FB9_Pos                     (9U)
#define CAN_F5R1_FB9_Msk                     (0x1UL << CAN_F5R1_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F5R1_FB9                         CAN_F5R1_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F5R1_FB10_Pos                    (10U)
#define CAN_F5R1_FB10_Msk                    (0x1UL << CAN_F5R1_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F5R1_FB10                        CAN_F5R1_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F5R1_FB11_Pos                    (11U)
#define CAN_F5R1_FB11_Msk                    (0x1UL << CAN_F5R1_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F5R1_FB11                        CAN_F5R1_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F5R1_FB12_Pos                    (12U)
#define CAN_F5R1_FB12_Msk                    (0x1UL << CAN_F5R1_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F5R1_FB12                        CAN_F5R1_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F5R1_FB13_Pos                    (13U)
#define CAN_F5R1_FB13_Msk                    (0x1UL << CAN_F5R1_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F5R1_FB13                        CAN_F5R1_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F5R1_FB14_Pos                    (14U)
#define CAN_F5R1_FB14_Msk                    (0x1UL << CAN_F5R1_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F5R1_FB14                        CAN_F5R1_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F5R1_FB15_Pos                    (15U)
#define CAN_F5R1_FB15_Msk                    (0x1UL << CAN_F5R1_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F5R1_FB15                        CAN_F5R1_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F5R1_FB16_Pos                    (16U)
#define CAN_F5R1_FB16_Msk                    (0x1UL << CAN_F5R1_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F5R1_FB16                        CAN_F5R1_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F5R1_FB17_Pos                    (17U)
#define CAN_F5R1_FB17_Msk                    (0x1UL << CAN_F5R1_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F5R1_FB17                        CAN_F5R1_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F5R1_FB18_Pos                    (18U)
#define CAN_F5R1_FB18_Msk                    (0x1UL << CAN_F5R1_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F5R1_FB18                        CAN_F5R1_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F5R1_FB19_Pos                    (19U)
#define CAN_F5R1_FB19_Msk                    (0x1UL << CAN_F5R1_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F5R1_FB19                        CAN_F5R1_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F5R1_FB20_Pos                    (20U)
#define CAN_F5R1_FB20_Msk                    (0x1UL << CAN_F5R1_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F5R1_FB20                        CAN_F5R1_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F5R1_FB21_Pos                    (21U)
#define CAN_F5R1_FB21_Msk                    (0x1UL << CAN_F5R1_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F5R1_FB21                        CAN_F5R1_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F5R1_FB22_Pos                    (22U)
#define CAN_F5R1_FB22_Msk                    (0x1UL << CAN_F5R1_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F5R1_FB22                        CAN_F5R1_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F5R1_FB23_Pos                    (23U)
#define CAN_F5R1_FB23_Msk                    (0x1UL << CAN_F5R1_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F5R1_FB23                        CAN_F5R1_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F5R1_FB24_Pos                    (24U)
#define CAN_F5R1_FB24_Msk                    (0x1UL << CAN_F5R1_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F5R1_FB24                        CAN_F5R1_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F5R1_FB25_Pos                    (25U)
#define CAN_F5R1_FB25_Msk                    (0x1UL << CAN_F5R1_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F5R1_FB25                        CAN_F5R1_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F5R1_FB26_Pos                    (26U)
#define CAN_F5R1_FB26_Msk                    (0x1UL << CAN_F5R1_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F5R1_FB26                        CAN_F5R1_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F5R1_FB27_Pos                    (27U)
#define CAN_F5R1_FB27_Msk                    (0x1UL << CAN_F5R1_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F5R1_FB27                        CAN_F5R1_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F5R1_FB28_Pos                    (28U)
#define CAN_F5R1_FB28_Msk                    (0x1UL << CAN_F5R1_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F5R1_FB28                        CAN_F5R1_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F5R1_FB29_Pos                    (29U)
#define CAN_F5R1_FB29_Msk                    (0x1UL << CAN_F5R1_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F5R1_FB29                        CAN_F5R1_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F5R1_FB30_Pos                    (30U)
#define CAN_F5R1_FB30_Msk                    (0x1UL << CAN_F5R1_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F5R1_FB30                        CAN_F5R1_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F5R1_FB31_Pos                    (31U)
#define CAN_F5R1_FB31_Msk                    (0x1UL << CAN_F5R1_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F5R1_FB31                        CAN_F5R1_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F6R1 register  *******************/
#define CAN_F6R1_FB0_Pos                     (0U)
#define CAN_F6R1_FB0_Msk                     (0x1UL << CAN_F6R1_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F6R1_FB0                         CAN_F6R1_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F6R1_FB1_Pos                     (1U)
#define CAN_F6R1_FB1_Msk                     (0x1UL << CAN_F6R1_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F6R1_FB1                         CAN_F6R1_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F6R1_FB2_Pos                     (2U)
#define CAN_F6R1_FB2_Msk                     (0x1UL << CAN_F6R1_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F6R1_FB2                         CAN_F6R1_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F6R1_FB3_Pos                     (3U)
#define CAN_F6R1_FB3_Msk                     (0x1UL << CAN_F6R1_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F6R1_FB3                         CAN_F6R1_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F6R1_FB4_Pos                     (4U)
#define CAN_F6R1_FB4_Msk                     (0x1UL << CAN_F6R1_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F6R1_FB4                         CAN_F6R1_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F6R1_FB5_Pos                     (5U)
#define CAN_F6R1_FB5_Msk                     (0x1UL << CAN_F6R1_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F6R1_FB5                         CAN_F6R1_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F6R1_FB6_Pos                     (6U)
#define CAN_F6R1_FB6_Msk                     (0x1UL << CAN_F6R1_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F6R1_FB6                         CAN_F6R1_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F6R1_FB7_Pos                     (7U)
#define CAN_F6R1_FB7_Msk                     (0x1UL << CAN_F6R1_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F6R1_FB7                         CAN_F6R1_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F6R1_FB8_Pos                     (8U)
#define CAN_F6R1_FB8_Msk                     (0x1UL << CAN_F6R1_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F6R1_FB8                         CAN_F6R1_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F6R1_FB9_Pos                     (9U)
#define CAN_F6R1_FB9_Msk                     (0x1UL << CAN_F6R1_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F6R1_FB9                         CAN_F6R1_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F6R1_FB10_Pos                    (10U)
#define CAN_F6R1_FB10_Msk                    (0x1UL << CAN_F6R1_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F6R1_FB10                        CAN_F6R1_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F6R1_FB11_Pos                    (11U)
#define CAN_F6R1_FB11_Msk                    (0x1UL << CAN_F6R1_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F6R1_FB11                        CAN_F6R1_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F6R1_FB12_Pos                    (12U)
#define CAN_F6R1_FB12_Msk                    (0x1UL << CAN_F6R1_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F6R1_FB12                        CAN_F6R1_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F6R1_FB13_Pos                    (13U)
#define CAN_F6R1_FB13_Msk                    (0x1UL << CAN_F6R1_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F6R1_FB13                        CAN_F6R1_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F6R1_FB14_Pos                    (14U)
#define CAN_F6R1_FB14_Msk                    (0x1UL << CAN_F6R1_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F6R1_FB14                        CAN_F6R1_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F6R1_FB15_Pos                    (15U)
#define CAN_F6R1_FB15_Msk                    (0x1UL << CAN_F6R1_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F6R1_FB15                        CAN_F6R1_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F6R1_FB16_Pos                    (16U)
#define CAN_F6R1_FB16_Msk                    (0x1UL << CAN_F6R1_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F6R1_FB16                        CAN_F6R1_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F6R1_FB17_Pos                    (17U)
#define CAN_F6R1_FB17_Msk                    (0x1UL << CAN_F6R1_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F6R1_FB17                        CAN_F6R1_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F6R1_FB18_Pos                    (18U)
#define CAN_F6R1_FB18_Msk                    (0x1UL << CAN_F6R1_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F6R1_FB18                        CAN_F6R1_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F6R1_FB19_Pos                    (19U)
#define CAN_F6R1_FB19_Msk                    (0x1UL << CAN_F6R1_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F6R1_FB19                        CAN_F6R1_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F6R1_FB20_Pos                    (20U)
#define CAN_F6R1_FB20_Msk                    (0x1UL << CAN_F6R1_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F6R1_FB20                        CAN_F6R1_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F6R1_FB21_Pos                    (21U)
#define CAN_F6R1_FB21_Msk                    (0x1UL << CAN_F6R1_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F6R1_FB21                        CAN_F6R1_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F6R1_FB22_Pos                    (22U)
#define CAN_F6R1_FB22_Msk                    (0x1UL << CAN_F6R1_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F6R1_FB22                        CAN_F6R1_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F6R1_FB23_Pos                    (23U)
#define CAN_F6R1_FB23_Msk                    (0x1UL << CAN_F6R1_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F6R1_FB23                        CAN_F6R1_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F6R1_FB24_Pos                    (24U)
#define CAN_F6R1_FB24_Msk                    (0x1UL << CAN_F6R1_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F6R1_FB24                        CAN_F6R1_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F6R1_FB25_Pos                    (25U)
#define CAN_F6R1_FB25_Msk                    (0x1UL << CAN_F6R1_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F6R1_FB25                        CAN_F6R1_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F6R1_FB26_Pos                    (26U)
#define CAN_F6R1_FB26_Msk                    (0x1UL << CAN_F6R1_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F6R1_FB26                        CAN_F6R1_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F6R1_FB27_Pos                    (27U)
#define CAN_F6R1_FB27_Msk                    (0x1UL << CAN_F6R1_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F6R1_FB27                        CAN_F6R1_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F6R1_FB28_Pos                    (28U)
#define CAN_F6R1_FB28_Msk                    (0x1UL << CAN_F6R1_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F6R1_FB28                        CAN_F6R1_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F6R1_FB29_Pos                    (29U)
#define CAN_F6R1_FB29_Msk                    (0x1UL << CAN_F6R1_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F6R1_FB29                        CAN_F6R1_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F6R1_FB30_Pos                    (30U)
#define CAN_F6R1_FB30_Msk                    (0x1UL << CAN_F6R1_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F6R1_FB30                        CAN_F6R1_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F6R1_FB31_Pos                    (31U)
#define CAN_F6R1_FB31_Msk                    (0x1UL << CAN_F6R1_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F6R1_FB31                        CAN_F6R1_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F7R1 register  *******************/
#define CAN_F7R1_FB0_Pos                     (0U)
#define CAN_F7R1_FB0_Msk                     (0x1UL << CAN_F7R1_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F7R1_FB0                         CAN_F7R1_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F7R1_FB1_Pos                     (1U)
#define CAN_F7R1_FB1_Msk                     (0x1UL << CAN_F7R1_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F7R1_FB1                         CAN_F7R1_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F7R1_FB2_Pos                     (2U)
#define CAN_F7R1_FB2_Msk                     (0x1UL << CAN_F7R1_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F7R1_FB2                         CAN_F7R1_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F7R1_FB3_Pos                     (3U)
#define CAN_F7R1_FB3_Msk                     (0x1UL << CAN_F7R1_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F7R1_FB3                         CAN_F7R1_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F7R1_FB4_Pos                     (4U)
#define CAN_F7R1_FB4_Msk                     (0x1UL << CAN_F7R1_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F7R1_FB4                         CAN_F7R1_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F7R1_FB5_Pos                     (5U)
#define CAN_F7R1_FB5_Msk                     (0x1UL << CAN_F7R1_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F7R1_FB5                         CAN_F7R1_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F7R1_FB6_Pos                     (6U)
#define CAN_F7R1_FB6_Msk                     (0x1UL << CAN_F7R1_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F7R1_FB6                         CAN_F7R1_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F7R1_FB7_Pos                     (7U)
#define CAN_F7R1_FB7_Msk                     (0x1UL << CAN_F7R1_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F7R1_FB7                         CAN_F7R1_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F7R1_FB8_Pos                     (8U)
#define CAN_F7R1_FB8_Msk                     (0x1UL << CAN_F7R1_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F7R1_FB8                         CAN_F7R1_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F7R1_FB9_Pos                     (9U)
#define CAN_F7R1_FB9_Msk                     (0x1UL << CAN_F7R1_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F7R1_FB9                         CAN_F7R1_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F7R1_FB10_Pos                    (10U)
#define CAN_F7R1_FB10_Msk                    (0x1UL << CAN_F7R1_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F7R1_FB10                        CAN_F7R1_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F7R1_FB11_Pos                    (11U)
#define CAN_F7R1_FB11_Msk                    (0x1UL << CAN_F7R1_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F7R1_FB11                        CAN_F7R1_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F7R1_FB12_Pos                    (12U)
#define CAN_F7R1_FB12_Msk                    (0x1UL << CAN_F7R1_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F7R1_FB12                        CAN_F7R1_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F7R1_FB13_Pos                    (13U)
#define CAN_F7R1_FB13_Msk                    (0x1UL << CAN_F7R1_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F7R1_FB13                        CAN_F7R1_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F7R1_FB14_Pos                    (14U)
#define CAN_F7R1_FB14_Msk                    (0x1UL << CAN_F7R1_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F7R1_FB14                        CAN_F7R1_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F7R1_FB15_Pos                    (15U)
#define CAN_F7R1_FB15_Msk                    (0x1UL << CAN_F7R1_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F7R1_FB15                        CAN_F7R1_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F7R1_FB16_Pos                    (16U)
#define CAN_F7R1_FB16_Msk                    (0x1UL << CAN_F7R1_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F7R1_FB16                        CAN_F7R1_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F7R1_FB17_Pos                    (17U)
#define CAN_F7R1_FB17_Msk                    (0x1UL << CAN_F7R1_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F7R1_FB17                        CAN_F7R1_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F7R1_FB18_Pos                    (18U)
#define CAN_F7R1_FB18_Msk                    (0x1UL << CAN_F7R1_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F7R1_FB18                        CAN_F7R1_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F7R1_FB19_Pos                    (19U)
#define CAN_F7R1_FB19_Msk                    (0x1UL << CAN_F7R1_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F7R1_FB19                        CAN_F7R1_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F7R1_FB20_Pos                    (20U)
#define CAN_F7R1_FB20_Msk                    (0x1UL << CAN_F7R1_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F7R1_FB20                        CAN_F7R1_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F7R1_FB21_Pos                    (21U)
#define CAN_F7R1_FB21_Msk                    (0x1UL << CAN_F7R1_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F7R1_FB21                        CAN_F7R1_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F7R1_FB22_Pos                    (22U)
#define CAN_F7R1_FB22_Msk                    (0x1UL << CAN_F7R1_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F7R1_FB22                        CAN_F7R1_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F7R1_FB23_Pos                    (23U)
#define CAN_F7R1_FB23_Msk                    (0x1UL << CAN_F7R1_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F7R1_FB23                        CAN_F7R1_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F7R1_FB24_Pos                    (24U)
#define CAN_F7R1_FB24_Msk                    (0x1UL << CAN_F7R1_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F7R1_FB24                        CAN_F7R1_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F7R1_FB25_Pos                    (25U)
#define CAN_F7R1_FB25_Msk                    (0x1UL << CAN_F7R1_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F7R1_FB25                        CAN_F7R1_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F7R1_FB26_Pos                    (26U)
#define CAN_F7R1_FB26_Msk                    (0x1UL << CAN_F7R1_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F7R1_FB26                        CAN_F7R1_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F7R1_FB27_Pos                    (27U)
#define CAN_F7R1_FB27_Msk                    (0x1UL << CAN_F7R1_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F7R1_FB27                        CAN_F7R1_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F7R1_FB28_Pos                    (28U)
#define CAN_F7R1_FB28_Msk                    (0x1UL << CAN_F7R1_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F7R1_FB28                        CAN_F7R1_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F7R1_FB29_Pos                    (29U)
#define CAN_F7R1_FB29_Msk                    (0x1UL << CAN_F7R1_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F7R1_FB29                        CAN_F7R1_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F7R1_FB30_Pos                    (30U)
#define CAN_F7R1_FB30_Msk                    (0x1UL << CAN_F7R1_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F7R1_FB30                        CAN_F7R1_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F7R1_FB31_Pos                    (31U)
#define CAN_F7R1_FB31_Msk                    (0x1UL << CAN_F7R1_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F7R1_FB31                        CAN_F7R1_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F8R1 register  *******************/
#define CAN_F8R1_FB0_Pos                     (0U)
#define CAN_F8R1_FB0_Msk                     (0x1UL << CAN_F8R1_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F8R1_FB0                         CAN_F8R1_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F8R1_FB1_Pos                     (1U)
#define CAN_F8R1_FB1_Msk                     (0x1UL << CAN_F8R1_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F8R1_FB1                         CAN_F8R1_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F8R1_FB2_Pos                     (2U)
#define CAN_F8R1_FB2_Msk                     (0x1UL << CAN_F8R1_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F8R1_FB2                         CAN_F8R1_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F8R1_FB3_Pos                     (3U)
#define CAN_F8R1_FB3_Msk                     (0x1UL << CAN_F8R1_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F8R1_FB3                         CAN_F8R1_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F8R1_FB4_Pos                     (4U)
#define CAN_F8R1_FB4_Msk                     (0x1UL << CAN_F8R1_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F8R1_FB4                         CAN_F8R1_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F8R1_FB5_Pos                     (5U)
#define CAN_F8R1_FB5_Msk                     (0x1UL << CAN_F8R1_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F8R1_FB5                         CAN_F8R1_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F8R1_FB6_Pos                     (6U)
#define CAN_F8R1_FB6_Msk                     (0x1UL << CAN_F8R1_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F8R1_FB6                         CAN_F8R1_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F8R1_FB7_Pos                     (7U)
#define CAN_F8R1_FB7_Msk                     (0x1UL << CAN_F8R1_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F8R1_FB7                         CAN_F8R1_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F8R1_FB8_Pos                     (8U)
#define CAN_F8R1_FB8_Msk                     (0x1UL << CAN_F8R1_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F8R1_FB8                         CAN_F8R1_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F8R1_FB9_Pos                     (9U)
#define CAN_F8R1_FB9_Msk                     (0x1UL << CAN_F8R1_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F8R1_FB9                         CAN_F8R1_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F8R1_FB10_Pos                    (10U)
#define CAN_F8R1_FB10_Msk                    (0x1UL << CAN_F8R1_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F8R1_FB10                        CAN_F8R1_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F8R1_FB11_Pos                    (11U)
#define CAN_F8R1_FB11_Msk                    (0x1UL << CAN_F8R1_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F8R1_FB11                        CAN_F8R1_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F8R1_FB12_Pos                    (12U)
#define CAN_F8R1_FB12_Msk                    (0x1UL << CAN_F8R1_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F8R1_FB12                        CAN_F8R1_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F8R1_FB13_Pos                    (13U)
#define CAN_F8R1_FB13_Msk                    (0x1UL << CAN_F8R1_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F8R1_FB13                        CAN_F8R1_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F8R1_FB14_Pos                    (14U)
#define CAN_F8R1_FB14_Msk                    (0x1UL << CAN_F8R1_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F8R1_FB14                        CAN_F8R1_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F8R1_FB15_Pos                    (15U)
#define CAN_F8R1_FB15_Msk                    (0x1UL << CAN_F8R1_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F8R1_FB15                        CAN_F8R1_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F8R1_FB16_Pos                    (16U)
#define CAN_F8R1_FB16_Msk                    (0x1UL << CAN_F8R1_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F8R1_FB16                        CAN_F8R1_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F8R1_FB17_Pos                    (17U)
#define CAN_F8R1_FB17_Msk                    (0x1UL << CAN_F8R1_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F8R1_FB17                        CAN_F8R1_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F8R1_FB18_Pos                    (18U)
#define CAN_F8R1_FB18_Msk                    (0x1UL << CAN_F8R1_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F8R1_FB18                        CAN_F8R1_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F8R1_FB19_Pos                    (19U)
#define CAN_F8R1_FB19_Msk                    (0x1UL << CAN_F8R1_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F8R1_FB19                        CAN_F8R1_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F8R1_FB20_Pos                    (20U)
#define CAN_F8R1_FB20_Msk                    (0x1UL << CAN_F8R1_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F8R1_FB20                        CAN_F8R1_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F8R1_FB21_Pos                    (21U)
#define CAN_F8R1_FB21_Msk                    (0x1UL << CAN_F8R1_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F8R1_FB21                        CAN_F8R1_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F8R1_FB22_Pos                    (22U)
#define CAN_F8R1_FB22_Msk                    (0x1UL << CAN_F8R1_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F8R1_FB22                        CAN_F8R1_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F8R1_FB23_Pos                    (23U)
#define CAN_F8R1_FB23_Msk                    (0x1UL << CAN_F8R1_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F8R1_FB23                        CAN_F8R1_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F8R1_FB24_Pos                    (24U)
#define CAN_F8R1_FB24_Msk                    (0x1UL << CAN_F8R1_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F8R1_FB24                        CAN_F8R1_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F8R1_FB25_Pos                    (25U)
#define CAN_F8R1_FB25_Msk                    (0x1UL << CAN_F8R1_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F8R1_FB25                        CAN_F8R1_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F8R1_FB26_Pos                    (26U)
#define CAN_F8R1_FB26_Msk                    (0x1UL << CAN_F8R1_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F8R1_FB26                        CAN_F8R1_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F8R1_FB27_Pos                    (27U)
#define CAN_F8R1_FB27_Msk                    (0x1UL << CAN_F8R1_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F8R1_FB27                        CAN_F8R1_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F8R1_FB28_Pos                    (28U)
#define CAN_F8R1_FB28_Msk                    (0x1UL << CAN_F8R1_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F8R1_FB28                        CAN_F8R1_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F8R1_FB29_Pos                    (29U)
#define CAN_F8R1_FB29_Msk                    (0x1UL << CAN_F8R1_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F8R1_FB29                        CAN_F8R1_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F8R1_FB30_Pos                    (30U)
#define CAN_F8R1_FB30_Msk                    (0x1UL << CAN_F8R1_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F8R1_FB30                        CAN_F8R1_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F8R1_FB31_Pos                    (31U)
#define CAN_F8R1_FB31_Msk                    (0x1UL << CAN_F8R1_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F8R1_FB31                        CAN_F8R1_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F9R1 register  *******************/
#define CAN_F9R1_FB0_Pos                     (0U)
#define CAN_F9R1_FB0_Msk                     (0x1UL << CAN_F9R1_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F9R1_FB0                         CAN_F9R1_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F9R1_FB1_Pos                     (1U)
#define CAN_F9R1_FB1_Msk                     (0x1UL << CAN_F9R1_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F9R1_FB1                         CAN_F9R1_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F9R1_FB2_Pos                     (2U)
#define CAN_F9R1_FB2_Msk                     (0x1UL << CAN_F9R1_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F9R1_FB2                         CAN_F9R1_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F9R1_FB3_Pos                     (3U)
#define CAN_F9R1_FB3_Msk                     (0x1UL << CAN_F9R1_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F9R1_FB3                         CAN_F9R1_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F9R1_FB4_Pos                     (4U)
#define CAN_F9R1_FB4_Msk                     (0x1UL << CAN_F9R1_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F9R1_FB4                         CAN_F9R1_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F9R1_FB5_Pos                     (5U)
#define CAN_F9R1_FB5_Msk                     (0x1UL << CAN_F9R1_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F9R1_FB5                         CAN_F9R1_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F9R1_FB6_Pos                     (6U)
#define CAN_F9R1_FB6_Msk                     (0x1UL << CAN_F9R1_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F9R1_FB6                         CAN_F9R1_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F9R1_FB7_Pos                     (7U)
#define CAN_F9R1_FB7_Msk                     (0x1UL << CAN_F9R1_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F9R1_FB7                         CAN_F9R1_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F9R1_FB8_Pos                     (8U)
#define CAN_F9R1_FB8_Msk                     (0x1UL << CAN_F9R1_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F9R1_FB8                         CAN_F9R1_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F9R1_FB9_Pos                     (9U)
#define CAN_F9R1_FB9_Msk                     (0x1UL << CAN_F9R1_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F9R1_FB9                         CAN_F9R1_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F9R1_FB10_Pos                    (10U)
#define CAN_F9R1_FB10_Msk                    (0x1UL << CAN_F9R1_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F9R1_FB10                        CAN_F9R1_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F9R1_FB11_Pos                    (11U)
#define CAN_F9R1_FB11_Msk                    (0x1UL << CAN_F9R1_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F9R1_FB11                        CAN_F9R1_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F9R1_FB12_Pos                    (12U)
#define CAN_F9R1_FB12_Msk                    (0x1UL << CAN_F9R1_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F9R1_FB12                        CAN_F9R1_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F9R1_FB13_Pos                    (13U)
#define CAN_F9R1_FB13_Msk                    (0x1UL << CAN_F9R1_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F9R1_FB13                        CAN_F9R1_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F9R1_FB14_Pos                    (14U)
#define CAN_F9R1_FB14_Msk                    (0x1UL << CAN_F9R1_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F9R1_FB14                        CAN_F9R1_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F9R1_FB15_Pos                    (15U)
#define CAN_F9R1_FB15_Msk                    (0x1UL << CAN_F9R1_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F9R1_FB15                        CAN_F9R1_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F9R1_FB16_Pos                    (16U)
#define CAN_F9R1_FB16_Msk                    (0x1UL << CAN_F9R1_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F9R1_FB16                        CAN_F9R1_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F9R1_FB17_Pos                    (17U)
#define CAN_F9R1_FB17_Msk                    (0x1UL << CAN_F9R1_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F9R1_FB17                        CAN_F9R1_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F9R1_FB18_Pos                    (18U)
#define CAN_F9R1_FB18_Msk                    (0x1UL << CAN_F9R1_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F9R1_FB18                        CAN_F9R1_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F9R1_FB19_Pos                    (19U)
#define CAN_F9R1_FB19_Msk                    (0x1UL << CAN_F9R1_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F9R1_FB19                        CAN_F9R1_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F9R1_FB20_Pos                    (20U)
#define CAN_F9R1_FB20_Msk                    (0x1UL << CAN_F9R1_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F9R1_FB20                        CAN_F9R1_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F9R1_FB21_Pos                    (21U)
#define CAN_F9R1_FB21_Msk                    (0x1UL << CAN_F9R1_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F9R1_FB21                        CAN_F9R1_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F9R1_FB22_Pos                    (22U)
#define CAN_F9R1_FB22_Msk                    (0x1UL << CAN_F9R1_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F9R1_FB22                        CAN_F9R1_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F9R1_FB23_Pos                    (23U)
#define CAN_F9R1_FB23_Msk                    (0x1UL << CAN_F9R1_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F9R1_FB23                        CAN_F9R1_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F9R1_FB24_Pos                    (24U)
#define CAN_F9R1_FB24_Msk                    (0x1UL << CAN_F9R1_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F9R1_FB24                        CAN_F9R1_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F9R1_FB25_Pos                    (25U)
#define CAN_F9R1_FB25_Msk                    (0x1UL << CAN_F9R1_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F9R1_FB25                        CAN_F9R1_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F9R1_FB26_Pos                    (26U)
#define CAN_F9R1_FB26_Msk                    (0x1UL << CAN_F9R1_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F9R1_FB26                        CAN_F9R1_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F9R1_FB27_Pos                    (27U)
#define CAN_F9R1_FB27_Msk                    (0x1UL << CAN_F9R1_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F9R1_FB27                        CAN_F9R1_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F9R1_FB28_Pos                    (28U)
#define CAN_F9R1_FB28_Msk                    (0x1UL << CAN_F9R1_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F9R1_FB28                        CAN_F9R1_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F9R1_FB29_Pos                    (29U)
#define CAN_F9R1_FB29_Msk                    (0x1UL << CAN_F9R1_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F9R1_FB29                        CAN_F9R1_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F9R1_FB30_Pos                    (30U)
#define CAN_F9R1_FB30_Msk                    (0x1UL << CAN_F9R1_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F9R1_FB30                        CAN_F9R1_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F9R1_FB31_Pos                    (31U)
#define CAN_F9R1_FB31_Msk                    (0x1UL << CAN_F9R1_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F9R1_FB31                        CAN_F9R1_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F10R1 register  ******************/
#define CAN_F10R1_FB0_Pos                    (0U)
#define CAN_F10R1_FB0_Msk                    (0x1UL << CAN_F10R1_FB0_Pos)       /*!< 0x00000001 */
#define CAN_F10R1_FB0                        CAN_F10R1_FB0_Msk                 /*!< Filter bit 0 */
#define CAN_F10R1_FB1_Pos                    (1U)
#define CAN_F10R1_FB1_Msk                    (0x1UL << CAN_F10R1_FB1_Pos)       /*!< 0x00000002 */
#define CAN_F10R1_FB1                        CAN_F10R1_FB1_Msk                 /*!< Filter bit 1 */
#define CAN_F10R1_FB2_Pos                    (2U)
#define CAN_F10R1_FB2_Msk                    (0x1UL << CAN_F10R1_FB2_Pos)       /*!< 0x00000004 */
#define CAN_F10R1_FB2                        CAN_F10R1_FB2_Msk                 /*!< Filter bit 2 */
#define CAN_F10R1_FB3_Pos                    (3U)
#define CAN_F10R1_FB3_Msk                    (0x1UL << CAN_F10R1_FB3_Pos)       /*!< 0x00000008 */
#define CAN_F10R1_FB3                        CAN_F10R1_FB3_Msk                 /*!< Filter bit 3 */
#define CAN_F10R1_FB4_Pos                    (4U)
#define CAN_F10R1_FB4_Msk                    (0x1UL << CAN_F10R1_FB4_Pos)       /*!< 0x00000010 */
#define CAN_F10R1_FB4                        CAN_F10R1_FB4_Msk                 /*!< Filter bit 4 */
#define CAN_F10R1_FB5_Pos                    (5U)
#define CAN_F10R1_FB5_Msk                    (0x1UL << CAN_F10R1_FB5_Pos)       /*!< 0x00000020 */
#define CAN_F10R1_FB5                        CAN_F10R1_FB5_Msk                 /*!< Filter bit 5 */
#define CAN_F10R1_FB6_Pos                    (6U)
#define CAN_F10R1_FB6_Msk                    (0x1UL << CAN_F10R1_FB6_Pos)       /*!< 0x00000040 */
#define CAN_F10R1_FB6                        CAN_F10R1_FB6_Msk                 /*!< Filter bit 6 */
#define CAN_F10R1_FB7_Pos                    (7U)
#define CAN_F10R1_FB7_Msk                    (0x1UL << CAN_F10R1_FB7_Pos)       /*!< 0x00000080 */
#define CAN_F10R1_FB7                        CAN_F10R1_FB7_Msk                 /*!< Filter bit 7 */
#define CAN_F10R1_FB8_Pos                    (8U)
#define CAN_F10R1_FB8_Msk                    (0x1UL << CAN_F10R1_FB8_Pos)       /*!< 0x00000100 */
#define CAN_F10R1_FB8                        CAN_F10R1_FB8_Msk                 /*!< Filter bit 8 */
#define CAN_F10R1_FB9_Pos                    (9U)
#define CAN_F10R1_FB9_Msk                    (0x1UL << CAN_F10R1_FB9_Pos)       /*!< 0x00000200 */
#define CAN_F10R1_FB9                        CAN_F10R1_FB9_Msk                 /*!< Filter bit 9 */
#define CAN_F10R1_FB10_Pos                   (10U)
#define CAN_F10R1_FB10_Msk                   (0x1UL << CAN_F10R1_FB10_Pos)      /*!< 0x00000400 */
#define CAN_F10R1_FB10                       CAN_F10R1_FB10_Msk                /*!< Filter bit 10 */
#define CAN_F10R1_FB11_Pos                   (11U)
#define CAN_F10R1_FB11_Msk                   (0x1UL << CAN_F10R1_FB11_Pos)      /*!< 0x00000800 */
#define CAN_F10R1_FB11                       CAN_F10R1_FB11_Msk                /*!< Filter bit 11 */
#define CAN_F10R1_FB12_Pos                   (12U)
#define CAN_F10R1_FB12_Msk                   (0x1UL << CAN_F10R1_FB12_Pos)      /*!< 0x00001000 */
#define CAN_F10R1_FB12                       CAN_F10R1_FB12_Msk                /*!< Filter bit 12 */
#define CAN_F10R1_FB13_Pos                   (13U)
#define CAN_F10R1_FB13_Msk                   (0x1UL << CAN_F10R1_FB13_Pos)      /*!< 0x00002000 */
#define CAN_F10R1_FB13                       CAN_F10R1_FB13_Msk                /*!< Filter bit 13 */
#define CAN_F10R1_FB14_Pos                   (14U)
#define CAN_F10R1_FB14_Msk                   (0x1UL << CAN_F10R1_FB14_Pos)      /*!< 0x00004000 */
#define CAN_F10R1_FB14                       CAN_F10R1_FB14_Msk                /*!< Filter bit 14 */
#define CAN_F10R1_FB15_Pos                   (15U)
#define CAN_F10R1_FB15_Msk                   (0x1UL << CAN_F10R1_FB15_Pos)      /*!< 0x00008000 */
#define CAN_F10R1_FB15                       CAN_F10R1_FB15_Msk                /*!< Filter bit 15 */
#define CAN_F10R1_FB16_Pos                   (16U)
#define CAN_F10R1_FB16_Msk                   (0x1UL << CAN_F10R1_FB16_Pos)      /*!< 0x00010000 */
#define CAN_F10R1_FB16                       CAN_F10R1_FB16_Msk                /*!< Filter bit 16 */
#define CAN_F10R1_FB17_Pos                   (17U)
#define CAN_F10R1_FB17_Msk                   (0x1UL << CAN_F10R1_FB17_Pos)      /*!< 0x00020000 */
#define CAN_F10R1_FB17                       CAN_F10R1_FB17_Msk                /*!< Filter bit 17 */
#define CAN_F10R1_FB18_Pos                   (18U)
#define CAN_F10R1_FB18_Msk                   (0x1UL << CAN_F10R1_FB18_Pos)      /*!< 0x00040000 */
#define CAN_F10R1_FB18                       CAN_F10R1_FB18_Msk                /*!< Filter bit 18 */
#define CAN_F10R1_FB19_Pos                   (19U)
#define CAN_F10R1_FB19_Msk                   (0x1UL << CAN_F10R1_FB19_Pos)      /*!< 0x00080000 */
#define CAN_F10R1_FB19                       CAN_F10R1_FB19_Msk                /*!< Filter bit 19 */
#define CAN_F10R1_FB20_Pos                   (20U)
#define CAN_F10R1_FB20_Msk                   (0x1UL << CAN_F10R1_FB20_Pos)      /*!< 0x00100000 */
#define CAN_F10R1_FB20                       CAN_F10R1_FB20_Msk                /*!< Filter bit 20 */
#define CAN_F10R1_FB21_Pos                   (21U)
#define CAN_F10R1_FB21_Msk                   (0x1UL << CAN_F10R1_FB21_Pos)      /*!< 0x00200000 */
#define CAN_F10R1_FB21                       CAN_F10R1_FB21_Msk                /*!< Filter bit 21 */
#define CAN_F10R1_FB22_Pos                   (22U)
#define CAN_F10R1_FB22_Msk                   (0x1UL << CAN_F10R1_FB22_Pos)      /*!< 0x00400000 */
#define CAN_F10R1_FB22                       CAN_F10R1_FB22_Msk                /*!< Filter bit 22 */
#define CAN_F10R1_FB23_Pos                   (23U)
#define CAN_F10R1_FB23_Msk                   (0x1UL << CAN_F10R1_FB23_Pos)      /*!< 0x00800000 */
#define CAN_F10R1_FB23                       CAN_F10R1_FB23_Msk                /*!< Filter bit 23 */
#define CAN_F10R1_FB24_Pos                   (24U)
#define CAN_F10R1_FB24_Msk                   (0x1UL << CAN_F10R1_FB24_Pos)      /*!< 0x01000000 */
#define CAN_F10R1_FB24                       CAN_F10R1_FB24_Msk                /*!< Filter bit 24 */
#define CAN_F10R1_FB25_Pos                   (25U)
#define CAN_F10R1_FB25_Msk                   (0x1UL << CAN_F10R1_FB25_Pos)      /*!< 0x02000000 */
#define CAN_F10R1_FB25                       CAN_F10R1_FB25_Msk                /*!< Filter bit 25 */
#define CAN_F10R1_FB26_Pos                   (26U)
#define CAN_F10R1_FB26_Msk                   (0x1UL << CAN_F10R1_FB26_Pos)      /*!< 0x04000000 */
#define CAN_F10R1_FB26                       CAN_F10R1_FB26_Msk                /*!< Filter bit 26 */
#define CAN_F10R1_FB27_Pos                   (27U)
#define CAN_F10R1_FB27_Msk                   (0x1UL << CAN_F10R1_FB27_Pos)      /*!< 0x08000000 */
#define CAN_F10R1_FB27                       CAN_F10R1_FB27_Msk                /*!< Filter bit 27 */
#define CAN_F10R1_FB28_Pos                   (28U)
#define CAN_F10R1_FB28_Msk                   (0x1UL << CAN_F10R1_FB28_Pos)      /*!< 0x10000000 */
#define CAN_F10R1_FB28                       CAN_F10R1_FB28_Msk                /*!< Filter bit 28 */
#define CAN_F10R1_FB29_Pos                   (29U)
#define CAN_F10R1_FB29_Msk                   (0x1UL << CAN_F10R1_FB29_Pos)      /*!< 0x20000000 */
#define CAN_F10R1_FB29                       CAN_F10R1_FB29_Msk                /*!< Filter bit 29 */
#define CAN_F10R1_FB30_Pos                   (30U)
#define CAN_F10R1_FB30_Msk                   (0x1UL << CAN_F10R1_FB30_Pos)      /*!< 0x40000000 */
#define CAN_F10R1_FB30                       CAN_F10R1_FB30_Msk                /*!< Filter bit 30 */
#define CAN_F10R1_FB31_Pos                   (31U)
#define CAN_F10R1_FB31_Msk                   (0x1UL << CAN_F10R1_FB31_Pos)      /*!< 0x80000000 */
#define CAN_F10R1_FB31                       CAN_F10R1_FB31_Msk                /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F11R1 register  ******************/
#define CAN_F11R1_FB0_Pos                    (0U)
#define CAN_F11R1_FB0_Msk                    (0x1UL << CAN_F11R1_FB0_Pos)       /*!< 0x00000001 */
#define CAN_F11R1_FB0                        CAN_F11R1_FB0_Msk                 /*!< Filter bit 0 */
#define CAN_F11R1_FB1_Pos                    (1U)
#define CAN_F11R1_FB1_Msk                    (0x1UL << CAN_F11R1_FB1_Pos)       /*!< 0x00000002 */
#define CAN_F11R1_FB1                        CAN_F11R1_FB1_Msk                 /*!< Filter bit 1 */
#define CAN_F11R1_FB2_Pos                    (2U)
#define CAN_F11R1_FB2_Msk                    (0x1UL << CAN_F11R1_FB2_Pos)       /*!< 0x00000004 */
#define CAN_F11R1_FB2                        CAN_F11R1_FB2_Msk                 /*!< Filter bit 2 */
#define CAN_F11R1_FB3_Pos                    (3U)
#define CAN_F11R1_FB3_Msk                    (0x1UL << CAN_F11R1_FB3_Pos)       /*!< 0x00000008 */
#define CAN_F11R1_FB3                        CAN_F11R1_FB3_Msk                 /*!< Filter bit 3 */
#define CAN_F11R1_FB4_Pos                    (4U)
#define CAN_F11R1_FB4_Msk                    (0x1UL << CAN_F11R1_FB4_Pos)       /*!< 0x00000010 */
#define CAN_F11R1_FB4                        CAN_F11R1_FB4_Msk                 /*!< Filter bit 4 */
#define CAN_F11R1_FB5_Pos                    (5U)
#define CAN_F11R1_FB5_Msk                    (0x1UL << CAN_F11R1_FB5_Pos)       /*!< 0x00000020 */
#define CAN_F11R1_FB5                        CAN_F11R1_FB5_Msk                 /*!< Filter bit 5 */
#define CAN_F11R1_FB6_Pos                    (6U)
#define CAN_F11R1_FB6_Msk                    (0x1UL << CAN_F11R1_FB6_Pos)       /*!< 0x00000040 */
#define CAN_F11R1_FB6                        CAN_F11R1_FB6_Msk                 /*!< Filter bit 6 */
#define CAN_F11R1_FB7_Pos                    (7U)
#define CAN_F11R1_FB7_Msk                    (0x1UL << CAN_F11R1_FB7_Pos)       /*!< 0x00000080 */
#define CAN_F11R1_FB7                        CAN_F11R1_FB7_Msk                 /*!< Filter bit 7 */
#define CAN_F11R1_FB8_Pos                    (8U)
#define CAN_F11R1_FB8_Msk                    (0x1UL << CAN_F11R1_FB8_Pos)       /*!< 0x00000100 */
#define CAN_F11R1_FB8                        CAN_F11R1_FB8_Msk                 /*!< Filter bit 8 */
#define CAN_F11R1_FB9_Pos                    (9U)
#define CAN_F11R1_FB9_Msk                    (0x1UL << CAN_F11R1_FB9_Pos)       /*!< 0x00000200 */
#define CAN_F11R1_FB9                        CAN_F11R1_FB9_Msk                 /*!< Filter bit 9 */
#define CAN_F11R1_FB10_Pos                   (10U)
#define CAN_F11R1_FB10_Msk                   (0x1UL << CAN_F11R1_FB10_Pos)      /*!< 0x00000400 */
#define CAN_F11R1_FB10                       CAN_F11R1_FB10_Msk                /*!< Filter bit 10 */
#define CAN_F11R1_FB11_Pos                   (11U)
#define CAN_F11R1_FB11_Msk                   (0x1UL << CAN_F11R1_FB11_Pos)      /*!< 0x00000800 */
#define CAN_F11R1_FB11                       CAN_F11R1_FB11_Msk                /*!< Filter bit 11 */
#define CAN_F11R1_FB12_Pos                   (12U)
#define CAN_F11R1_FB12_Msk                   (0x1UL << CAN_F11R1_FB12_Pos)      /*!< 0x00001000 */
#define CAN_F11R1_FB12                       CAN_F11R1_FB12_Msk                /*!< Filter bit 12 */
#define CAN_F11R1_FB13_Pos                   (13U)
#define CAN_F11R1_FB13_Msk                   (0x1UL << CAN_F11R1_FB13_Pos)      /*!< 0x00002000 */
#define CAN_F11R1_FB13                       CAN_F11R1_FB13_Msk                /*!< Filter bit 13 */
#define CAN_F11R1_FB14_Pos                   (14U)
#define CAN_F11R1_FB14_Msk                   (0x1UL << CAN_F11R1_FB14_Pos)      /*!< 0x00004000 */
#define CAN_F11R1_FB14                       CAN_F11R1_FB14_Msk                /*!< Filter bit 14 */
#define CAN_F11R1_FB15_Pos                   (15U)
#define CAN_F11R1_FB15_Msk                   (0x1UL << CAN_F11R1_FB15_Pos)      /*!< 0x00008000 */
#define CAN_F11R1_FB15                       CAN_F11R1_FB15_Msk                /*!< Filter bit 15 */
#define CAN_F11R1_FB16_Pos                   (16U)
#define CAN_F11R1_FB16_Msk                   (0x1UL << CAN_F11R1_FB16_Pos)      /*!< 0x00010000 */
#define CAN_F11R1_FB16                       CAN_F11R1_FB16_Msk                /*!< Filter bit 16 */
#define CAN_F11R1_FB17_Pos                   (17U)
#define CAN_F11R1_FB17_Msk                   (0x1UL << CAN_F11R1_FB17_Pos)      /*!< 0x00020000 */
#define CAN_F11R1_FB17                       CAN_F11R1_FB17_Msk                /*!< Filter bit 17 */
#define CAN_F11R1_FB18_Pos                   (18U)
#define CAN_F11R1_FB18_Msk                   (0x1UL << CAN_F11R1_FB18_Pos)      /*!< 0x00040000 */
#define CAN_F11R1_FB18                       CAN_F11R1_FB18_Msk                /*!< Filter bit 18 */
#define CAN_F11R1_FB19_Pos                   (19U)
#define CAN_F11R1_FB19_Msk                   (0x1UL << CAN_F11R1_FB19_Pos)      /*!< 0x00080000 */
#define CAN_F11R1_FB19                       CAN_F11R1_FB19_Msk                /*!< Filter bit 19 */
#define CAN_F11R1_FB20_Pos                   (20U)
#define CAN_F11R1_FB20_Msk                   (0x1UL << CAN_F11R1_FB20_Pos)      /*!< 0x00100000 */
#define CAN_F11R1_FB20                       CAN_F11R1_FB20_Msk                /*!< Filter bit 20 */
#define CAN_F11R1_FB21_Pos                   (21U)
#define CAN_F11R1_FB21_Msk                   (0x1UL << CAN_F11R1_FB21_Pos)      /*!< 0x00200000 */
#define CAN_F11R1_FB21                       CAN_F11R1_FB21_Msk                /*!< Filter bit 21 */
#define CAN_F11R1_FB22_Pos                   (22U)
#define CAN_F11R1_FB22_Msk                   (0x1UL << CAN_F11R1_FB22_Pos)      /*!< 0x00400000 */
#define CAN_F11R1_FB22                       CAN_F11R1_FB22_Msk                /*!< Filter bit 22 */
#define CAN_F11R1_FB23_Pos                   (23U)
#define CAN_F11R1_FB23_Msk                   (0x1UL << CAN_F11R1_FB23_Pos)      /*!< 0x00800000 */
#define CAN_F11R1_FB23                       CAN_F11R1_FB23_Msk                /*!< Filter bit 23 */
#define CAN_F11R1_FB24_Pos                   (24U)
#define CAN_F11R1_FB24_Msk                   (0x1UL << CAN_F11R1_FB24_Pos)      /*!< 0x01000000 */
#define CAN_F11R1_FB24                       CAN_F11R1_FB24_Msk                /*!< Filter bit 24 */
#define CAN_F11R1_FB25_Pos                   (25U)
#define CAN_F11R1_FB25_Msk                   (0x1UL << CAN_F11R1_FB25_Pos)      /*!< 0x02000000 */
#define CAN_F11R1_FB25                       CAN_F11R1_FB25_Msk                /*!< Filter bit 25 */
#define CAN_F11R1_FB26_Pos                   (26U)
#define CAN_F11R1_FB26_Msk                   (0x1UL << CAN_F11R1_FB26_Pos)      /*!< 0x04000000 */
#define CAN_F11R1_FB26                       CAN_F11R1_FB26_Msk                /*!< Filter bit 26 */
#define CAN_F11R1_FB27_Pos                   (27U)
#define CAN_F11R1_FB27_Msk                   (0x1UL << CAN_F11R1_FB27_Pos)      /*!< 0x08000000 */
#define CAN_F11R1_FB27                       CAN_F11R1_FB27_Msk                /*!< Filter bit 27 */
#define CAN_F11R1_FB28_Pos                   (28U)
#define CAN_F11R1_FB28_Msk                   (0x1UL << CAN_F11R1_FB28_Pos)      /*!< 0x10000000 */
#define CAN_F11R1_FB28                       CAN_F11R1_FB28_Msk                /*!< Filter bit 28 */
#define CAN_F11R1_FB29_Pos                   (29U)
#define CAN_F11R1_FB29_Msk                   (0x1UL << CAN_F11R1_FB29_Pos)      /*!< 0x20000000 */
#define CAN_F11R1_FB29                       CAN_F11R1_FB29_Msk                /*!< Filter bit 29 */
#define CAN_F11R1_FB30_Pos                   (30U)
#define CAN_F11R1_FB30_Msk                   (0x1UL << CAN_F11R1_FB30_Pos)      /*!< 0x40000000 */
#define CAN_F11R1_FB30                       CAN_F11R1_FB30_Msk                /*!< Filter bit 30 */
#define CAN_F11R1_FB31_Pos                   (31U)
#define CAN_F11R1_FB31_Msk                   (0x1UL << CAN_F11R1_FB31_Pos)      /*!< 0x80000000 */
#define CAN_F11R1_FB31                       CAN_F11R1_FB31_Msk                /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F12R1 register  ******************/
#define CAN_F12R1_FB0_Pos                    (0U)
#define CAN_F12R1_FB0_Msk                    (0x1UL << CAN_F12R1_FB0_Pos)       /*!< 0x00000001 */
#define CAN_F12R1_FB0                        CAN_F12R1_FB0_Msk                 /*!< Filter bit 0 */
#define CAN_F12R1_FB1_Pos                    (1U)
#define CAN_F12R1_FB1_Msk                    (0x1UL << CAN_F12R1_FB1_Pos)       /*!< 0x00000002 */
#define CAN_F12R1_FB1                        CAN_F12R1_FB1_Msk                 /*!< Filter bit 1 */
#define CAN_F12R1_FB2_Pos                    (2U)
#define CAN_F12R1_FB2_Msk                    (0x1UL << CAN_F12R1_FB2_Pos)       /*!< 0x00000004 */
#define CAN_F12R1_FB2                        CAN_F12R1_FB2_Msk                 /*!< Filter bit 2 */
#define CAN_F12R1_FB3_Pos                    (3U)
#define CAN_F12R1_FB3_Msk                    (0x1UL << CAN_F12R1_FB3_Pos)       /*!< 0x00000008 */
#define CAN_F12R1_FB3                        CAN_F12R1_FB3_Msk                 /*!< Filter bit 3 */
#define CAN_F12R1_FB4_Pos                    (4U)
#define CAN_F12R1_FB4_Msk                    (0x1UL << CAN_F12R1_FB4_Pos)       /*!< 0x00000010 */
#define CAN_F12R1_FB4                        CAN_F12R1_FB4_Msk                 /*!< Filter bit 4 */
#define CAN_F12R1_FB5_Pos                    (5U)
#define CAN_F12R1_FB5_Msk                    (0x1UL << CAN_F12R1_FB5_Pos)       /*!< 0x00000020 */
#define CAN_F12R1_FB5                        CAN_F12R1_FB5_Msk                 /*!< Filter bit 5 */
#define CAN_F12R1_FB6_Pos                    (6U)
#define CAN_F12R1_FB6_Msk                    (0x1UL << CAN_F12R1_FB6_Pos)       /*!< 0x00000040 */
#define CAN_F12R1_FB6                        CAN_F12R1_FB6_Msk                 /*!< Filter bit 6 */
#define CAN_F12R1_FB7_Pos                    (7U)
#define CAN_F12R1_FB7_Msk                    (0x1UL << CAN_F12R1_FB7_Pos)       /*!< 0x00000080 */
#define CAN_F12R1_FB7                        CAN_F12R1_FB7_Msk                 /*!< Filter bit 7 */
#define CAN_F12R1_FB8_Pos                    (8U)
#define CAN_F12R1_FB8_Msk                    (0x1UL << CAN_F12R1_FB8_Pos)       /*!< 0x00000100 */
#define CAN_F12R1_FB8                        CAN_F12R1_FB8_Msk                 /*!< Filter bit 8 */
#define CAN_F12R1_FB9_Pos                    (9U)
#define CAN_F12R1_FB9_Msk                    (0x1UL << CAN_F12R1_FB9_Pos)       /*!< 0x00000200 */
#define CAN_F12R1_FB9                        CAN_F12R1_FB9_Msk                 /*!< Filter bit 9 */
#define CAN_F12R1_FB10_Pos                   (10U)
#define CAN_F12R1_FB10_Msk                   (0x1UL << CAN_F12R1_FB10_Pos)      /*!< 0x00000400 */
#define CAN_F12R1_FB10                       CAN_F12R1_FB10_Msk                /*!< Filter bit 10 */
#define CAN_F12R1_FB11_Pos                   (11U)
#define CAN_F12R1_FB11_Msk                   (0x1UL << CAN_F12R1_FB11_Pos)      /*!< 0x00000800 */
#define CAN_F12R1_FB11                       CAN_F12R1_FB11_Msk                /*!< Filter bit 11 */
#define CAN_F12R1_FB12_Pos                   (12U)
#define CAN_F12R1_FB12_Msk                   (0x1UL << CAN_F12R1_FB12_Pos)      /*!< 0x00001000 */
#define CAN_F12R1_FB12                       CAN_F12R1_FB12_Msk                /*!< Filter bit 12 */
#define CAN_F12R1_FB13_Pos                   (13U)
#define CAN_F12R1_FB13_Msk                   (0x1UL << CAN_F12R1_FB13_Pos)      /*!< 0x00002000 */
#define CAN_F12R1_FB13                       CAN_F12R1_FB13_Msk                /*!< Filter bit 13 */
#define CAN_F12R1_FB14_Pos                   (14U)
#define CAN_F12R1_FB14_Msk                   (0x1UL << CAN_F12R1_FB14_Pos)      /*!< 0x00004000 */
#define CAN_F12R1_FB14                       CAN_F12R1_FB14_Msk                /*!< Filter bit 14 */
#define CAN_F12R1_FB15_Pos                   (15U)
#define CAN_F12R1_FB15_Msk                   (0x1UL << CAN_F12R1_FB15_Pos)      /*!< 0x00008000 */
#define CAN_F12R1_FB15                       CAN_F12R1_FB15_Msk                /*!< Filter bit 15 */
#define CAN_F12R1_FB16_Pos                   (16U)
#define CAN_F12R1_FB16_Msk                   (0x1UL << CAN_F12R1_FB16_Pos)      /*!< 0x00010000 */
#define CAN_F12R1_FB16                       CAN_F12R1_FB16_Msk                /*!< Filter bit 16 */
#define CAN_F12R1_FB17_Pos                   (17U)
#define CAN_F12R1_FB17_Msk                   (0x1UL << CAN_F12R1_FB17_Pos)      /*!< 0x00020000 */
#define CAN_F12R1_FB17                       CAN_F12R1_FB17_Msk                /*!< Filter bit 17 */
#define CAN_F12R1_FB18_Pos                   (18U)
#define CAN_F12R1_FB18_Msk                   (0x1UL << CAN_F12R1_FB18_Pos)      /*!< 0x00040000 */
#define CAN_F12R1_FB18                       CAN_F12R1_FB18_Msk                /*!< Filter bit 18 */
#define CAN_F12R1_FB19_Pos                   (19U)
#define CAN_F12R1_FB19_Msk                   (0x1UL << CAN_F12R1_FB19_Pos)      /*!< 0x00080000 */
#define CAN_F12R1_FB19                       CAN_F12R1_FB19_Msk                /*!< Filter bit 19 */
#define CAN_F12R1_FB20_Pos                   (20U)
#define CAN_F12R1_FB20_Msk                   (0x1UL << CAN_F12R1_FB20_Pos)      /*!< 0x00100000 */
#define CAN_F12R1_FB20                       CAN_F12R1_FB20_Msk                /*!< Filter bit 20 */
#define CAN_F12R1_FB21_Pos                   (21U)
#define CAN_F12R1_FB21_Msk                   (0x1UL << CAN_F12R1_FB21_Pos)      /*!< 0x00200000 */
#define CAN_F12R1_FB21                       CAN_F12R1_FB21_Msk                /*!< Filter bit 21 */
#define CAN_F12R1_FB22_Pos                   (22U)
#define CAN_F12R1_FB22_Msk                   (0x1UL << CAN_F12R1_FB22_Pos)      /*!< 0x00400000 */
#define CAN_F12R1_FB22                       CAN_F12R1_FB22_Msk                /*!< Filter bit 22 */
#define CAN_F12R1_FB23_Pos                   (23U)
#define CAN_F12R1_FB23_Msk                   (0x1UL << CAN_F12R1_FB23_Pos)      /*!< 0x00800000 */
#define CAN_F12R1_FB23                       CAN_F12R1_FB23_Msk                /*!< Filter bit 23 */
#define CAN_F12R1_FB24_Pos                   (24U)
#define CAN_F12R1_FB24_Msk                   (0x1UL << CAN_F12R1_FB24_Pos)      /*!< 0x01000000 */
#define CAN_F12R1_FB24                       CAN_F12R1_FB24_Msk                /*!< Filter bit 24 */
#define CAN_F12R1_FB25_Pos                   (25U)
#define CAN_F12R1_FB25_Msk                   (0x1UL << CAN_F12R1_FB25_Pos)      /*!< 0x02000000 */
#define CAN_F12R1_FB25                       CAN_F12R1_FB25_Msk                /*!< Filter bit 25 */
#define CAN_F12R1_FB26_Pos                   (26U)
#define CAN_F12R1_FB26_Msk                   (0x1UL << CAN_F12R1_FB26_Pos)      /*!< 0x04000000 */
#define CAN_F12R1_FB26                       CAN_F12R1_FB26_Msk                /*!< Filter bit 26 */
#define CAN_F12R1_FB27_Pos                   (27U)
#define CAN_F12R1_FB27_Msk                   (0x1UL << CAN_F12R1_FB27_Pos)      /*!< 0x08000000 */
#define CAN_F12R1_FB27                       CAN_F12R1_FB27_Msk                /*!< Filter bit 27 */
#define CAN_F12R1_FB28_Pos                   (28U)
#define CAN_F12R1_FB28_Msk                   (0x1UL << CAN_F12R1_FB28_Pos)      /*!< 0x10000000 */
#define CAN_F12R1_FB28                       CAN_F12R1_FB28_Msk                /*!< Filter bit 28 */
#define CAN_F12R1_FB29_Pos                   (29U)
#define CAN_F12R1_FB29_Msk                   (0x1UL << CAN_F12R1_FB29_Pos)      /*!< 0x20000000 */
#define CAN_F12R1_FB29                       CAN_F12R1_FB29_Msk                /*!< Filter bit 29 */
#define CAN_F12R1_FB30_Pos                   (30U)
#define CAN_F12R1_FB30_Msk                   (0x1UL << CAN_F12R1_FB30_Pos)      /*!< 0x40000000 */
#define CAN_F12R1_FB30                       CAN_F12R1_FB30_Msk                /*!< Filter bit 30 */
#define CAN_F12R1_FB31_Pos                   (31U)
#define CAN_F12R1_FB31_Msk                   (0x1UL << CAN_F12R1_FB31_Pos)      /*!< 0x80000000 */
#define CAN_F12R1_FB31                       CAN_F12R1_FB31_Msk                /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F13R1 register  ******************/
#define CAN_F13R1_FB0_Pos                    (0U)
#define CAN_F13R1_FB0_Msk                    (0x1UL << CAN_F13R1_FB0_Pos)       /*!< 0x00000001 */
#define CAN_F13R1_FB0                        CAN_F13R1_FB0_Msk                 /*!< Filter bit 0 */
#define CAN_F13R1_FB1_Pos                    (1U)
#define CAN_F13R1_FB1_Msk                    (0x1UL << CAN_F13R1_FB1_Pos)       /*!< 0x00000002 */
#define CAN_F13R1_FB1                        CAN_F13R1_FB1_Msk                 /*!< Filter bit 1 */
#define CAN_F13R1_FB2_Pos                    (2U)
#define CAN_F13R1_FB2_Msk                    (0x1UL << CAN_F13R1_FB2_Pos)       /*!< 0x00000004 */
#define CAN_F13R1_FB2                        CAN_F13R1_FB2_Msk                 /*!< Filter bit 2 */
#define CAN_F13R1_FB3_Pos                    (3U)
#define CAN_F13R1_FB3_Msk                    (0x1UL << CAN_F13R1_FB3_Pos)       /*!< 0x00000008 */
#define CAN_F13R1_FB3                        CAN_F13R1_FB3_Msk                 /*!< Filter bit 3 */
#define CAN_F13R1_FB4_Pos                    (4U)
#define CAN_F13R1_FB4_Msk                    (0x1UL << CAN_F13R1_FB4_Pos)       /*!< 0x00000010 */
#define CAN_F13R1_FB4                        CAN_F13R1_FB4_Msk                 /*!< Filter bit 4 */
#define CAN_F13R1_FB5_Pos                    (5U)
#define CAN_F13R1_FB5_Msk                    (0x1UL << CAN_F13R1_FB5_Pos)       /*!< 0x00000020 */
#define CAN_F13R1_FB5                        CAN_F13R1_FB5_Msk                 /*!< Filter bit 5 */
#define CAN_F13R1_FB6_Pos                    (6U)
#define CAN_F13R1_FB6_Msk                    (0x1UL << CAN_F13R1_FB6_Pos)       /*!< 0x00000040 */
#define CAN_F13R1_FB6                        CAN_F13R1_FB6_Msk                 /*!< Filter bit 6 */
#define CAN_F13R1_FB7_Pos                    (7U)
#define CAN_F13R1_FB7_Msk                    (0x1UL << CAN_F13R1_FB7_Pos)       /*!< 0x00000080 */
#define CAN_F13R1_FB7                        CAN_F13R1_FB7_Msk                 /*!< Filter bit 7 */
#define CAN_F13R1_FB8_Pos                    (8U)
#define CAN_F13R1_FB8_Msk                    (0x1UL << CAN_F13R1_FB8_Pos)       /*!< 0x00000100 */
#define CAN_F13R1_FB8                        CAN_F13R1_FB8_Msk                 /*!< Filter bit 8 */
#define CAN_F13R1_FB9_Pos                    (9U)
#define CAN_F13R1_FB9_Msk                    (0x1UL << CAN_F13R1_FB9_Pos)       /*!< 0x00000200 */
#define CAN_F13R1_FB9                        CAN_F13R1_FB9_Msk                 /*!< Filter bit 9 */
#define CAN_F13R1_FB10_Pos                   (10U)
#define CAN_F13R1_FB10_Msk                   (0x1UL << CAN_F13R1_FB10_Pos)      /*!< 0x00000400 */
#define CAN_F13R1_FB10                       CAN_F13R1_FB10_Msk                /*!< Filter bit 10 */
#define CAN_F13R1_FB11_Pos                   (11U)
#define CAN_F13R1_FB11_Msk                   (0x1UL << CAN_F13R1_FB11_Pos)      /*!< 0x00000800 */
#define CAN_F13R1_FB11                       CAN_F13R1_FB11_Msk                /*!< Filter bit 11 */
#define CAN_F13R1_FB12_Pos                   (12U)
#define CAN_F13R1_FB12_Msk                   (0x1UL << CAN_F13R1_FB12_Pos)      /*!< 0x00001000 */
#define CAN_F13R1_FB12                       CAN_F13R1_FB12_Msk                /*!< Filter bit 12 */
#define CAN_F13R1_FB13_Pos                   (13U)
#define CAN_F13R1_FB13_Msk                   (0x1UL << CAN_F13R1_FB13_Pos)      /*!< 0x00002000 */
#define CAN_F13R1_FB13                       CAN_F13R1_FB13_Msk                /*!< Filter bit 13 */
#define CAN_F13R1_FB14_Pos                   (14U)
#define CAN_F13R1_FB14_Msk                   (0x1UL << CAN_F13R1_FB14_Pos)      /*!< 0x00004000 */
#define CAN_F13R1_FB14                       CAN_F13R1_FB14_Msk                /*!< Filter bit 14 */
#define CAN_F13R1_FB15_Pos                   (15U)
#define CAN_F13R1_FB15_Msk                   (0x1UL << CAN_F13R1_FB15_Pos)      /*!< 0x00008000 */
#define CAN_F13R1_FB15                       CAN_F13R1_FB15_Msk                /*!< Filter bit 15 */
#define CAN_F13R1_FB16_Pos                   (16U)
#define CAN_F13R1_FB16_Msk                   (0x1UL << CAN_F13R1_FB16_Pos)      /*!< 0x00010000 */
#define CAN_F13R1_FB16                       CAN_F13R1_FB16_Msk                /*!< Filter bit 16 */
#define CAN_F13R1_FB17_Pos                   (17U)
#define CAN_F13R1_FB17_Msk                   (0x1UL << CAN_F13R1_FB17_Pos)      /*!< 0x00020000 */
#define CAN_F13R1_FB17                       CAN_F13R1_FB17_Msk                /*!< Filter bit 17 */
#define CAN_F13R1_FB18_Pos                   (18U)
#define CAN_F13R1_FB18_Msk                   (0x1UL << CAN_F13R1_FB18_Pos)      /*!< 0x00040000 */
#define CAN_F13R1_FB18                       CAN_F13R1_FB18_Msk                /*!< Filter bit 18 */
#define CAN_F13R1_FB19_Pos                   (19U)
#define CAN_F13R1_FB19_Msk                   (0x1UL << CAN_F13R1_FB19_Pos)      /*!< 0x00080000 */
#define CAN_F13R1_FB19                       CAN_F13R1_FB19_Msk                /*!< Filter bit 19 */
#define CAN_F13R1_FB20_Pos                   (20U)
#define CAN_F13R1_FB20_Msk                   (0x1UL << CAN_F13R1_FB20_Pos)      /*!< 0x00100000 */
#define CAN_F13R1_FB20                       CAN_F13R1_FB20_Msk                /*!< Filter bit 20 */
#define CAN_F13R1_FB21_Pos                   (21U)
#define CAN_F13R1_FB21_Msk                   (0x1UL << CAN_F13R1_FB21_Pos)      /*!< 0x00200000 */
#define CAN_F13R1_FB21                       CAN_F13R1_FB21_Msk                /*!< Filter bit 21 */
#define CAN_F13R1_FB22_Pos                   (22U)
#define CAN_F13R1_FB22_Msk                   (0x1UL << CAN_F13R1_FB22_Pos)      /*!< 0x00400000 */
#define CAN_F13R1_FB22                       CAN_F13R1_FB22_Msk                /*!< Filter bit 22 */
#define CAN_F13R1_FB23_Pos                   (23U)
#define CAN_F13R1_FB23_Msk                   (0x1UL << CAN_F13R1_FB23_Pos)      /*!< 0x00800000 */
#define CAN_F13R1_FB23                       CAN_F13R1_FB23_Msk                /*!< Filter bit 23 */
#define CAN_F13R1_FB24_Pos                   (24U)
#define CAN_F13R1_FB24_Msk                   (0x1UL << CAN_F13R1_FB24_Pos)      /*!< 0x01000000 */
#define CAN_F13R1_FB24                       CAN_F13R1_FB24_Msk                /*!< Filter bit 24 */
#define CAN_F13R1_FB25_Pos                   (25U)
#define CAN_F13R1_FB25_Msk                   (0x1UL << CAN_F13R1_FB25_Pos)      /*!< 0x02000000 */
#define CAN_F13R1_FB25                       CAN_F13R1_FB25_Msk                /*!< Filter bit 25 */
#define CAN_F13R1_FB26_Pos                   (26U)
#define CAN_F13R1_FB26_Msk                   (0x1UL << CAN_F13R1_FB26_Pos)      /*!< 0x04000000 */
#define CAN_F13R1_FB26                       CAN_F13R1_FB26_Msk                /*!< Filter bit 26 */
#define CAN_F13R1_FB27_Pos                   (27U)
#define CAN_F13R1_FB27_Msk                   (0x1UL << CAN_F13R1_FB27_Pos)      /*!< 0x08000000 */
#define CAN_F13R1_FB27                       CAN_F13R1_FB27_Msk                /*!< Filter bit 27 */
#define CAN_F13R1_FB28_Pos                   (28U)
#define CAN_F13R1_FB28_Msk                   (0x1UL << CAN_F13R1_FB28_Pos)      /*!< 0x10000000 */
#define CAN_F13R1_FB28                       CAN_F13R1_FB28_Msk                /*!< Filter bit 28 */
#define CAN_F13R1_FB29_Pos                   (29U)
#define CAN_F13R1_FB29_Msk                   (0x1UL << CAN_F13R1_FB29_Pos)      /*!< 0x20000000 */
#define CAN_F13R1_FB29                       CAN_F13R1_FB29_Msk                /*!< Filter bit 29 */
#define CAN_F13R1_FB30_Pos                   (30U)
#define CAN_F13R1_FB30_Msk                   (0x1UL << CAN_F13R1_FB30_Pos)      /*!< 0x40000000 */
#define CAN_F13R1_FB30                       CAN_F13R1_FB30_Msk                /*!< Filter bit 30 */
#define CAN_F13R1_FB31_Pos                   (31U)
#define CAN_F13R1_FB31_Msk                   (0x1UL << CAN_F13R1_FB31_Pos)      /*!< 0x80000000 */
#define CAN_F13R1_FB31                       CAN_F13R1_FB31_Msk                /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F0R2 register  *******************/
#define CAN_F0R2_FB0_Pos                     (0U)
#define CAN_F0R2_FB0_Msk                     (0x1UL << CAN_F0R2_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F0R2_FB0                         CAN_F0R2_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F0R2_FB1_Pos                     (1U)
#define CAN_F0R2_FB1_Msk                     (0x1UL << CAN_F0R2_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F0R2_FB1                         CAN_F0R2_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F0R2_FB2_Pos                     (2U)
#define CAN_F0R2_FB2_Msk                     (0x1UL << CAN_F0R2_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F0R2_FB2                         CAN_F0R2_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F0R2_FB3_Pos                     (3U)
#define CAN_F0R2_FB3_Msk                     (0x1UL << CAN_F0R2_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F0R2_FB3                         CAN_F0R2_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F0R2_FB4_Pos                     (4U)
#define CAN_F0R2_FB4_Msk                     (0x1UL << CAN_F0R2_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F0R2_FB4                         CAN_F0R2_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F0R2_FB5_Pos                     (5U)
#define CAN_F0R2_FB5_Msk                     (0x1UL << CAN_F0R2_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F0R2_FB5                         CAN_F0R2_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F0R2_FB6_Pos                     (6U)
#define CAN_F0R2_FB6_Msk                     (0x1UL << CAN_F0R2_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F0R2_FB6                         CAN_F0R2_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F0R2_FB7_Pos                     (7U)
#define CAN_F0R2_FB7_Msk                     (0x1UL << CAN_F0R2_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F0R2_FB7                         CAN_F0R2_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F0R2_FB8_Pos                     (8U)
#define CAN_F0R2_FB8_Msk                     (0x1UL << CAN_F0R2_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F0R2_FB8                         CAN_F0R2_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F0R2_FB9_Pos                     (9U)
#define CAN_F0R2_FB9_Msk                     (0x1UL << CAN_F0R2_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F0R2_FB9                         CAN_F0R2_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F0R2_FB10_Pos                    (10U)
#define CAN_F0R2_FB10_Msk                    (0x1UL << CAN_F0R2_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F0R2_FB10                        CAN_F0R2_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F0R2_FB11_Pos                    (11U)
#define CAN_F0R2_FB11_Msk                    (0x1UL << CAN_F0R2_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F0R2_FB11                        CAN_F0R2_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F0R2_FB12_Pos                    (12U)
#define CAN_F0R2_FB12_Msk                    (0x1UL << CAN_F0R2_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F0R2_FB12                        CAN_F0R2_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F0R2_FB13_Pos                    (13U)
#define CAN_F0R2_FB13_Msk                    (0x1UL << CAN_F0R2_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F0R2_FB13                        CAN_F0R2_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F0R2_FB14_Pos                    (14U)
#define CAN_F0R2_FB14_Msk                    (0x1UL << CAN_F0R2_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F0R2_FB14                        CAN_F0R2_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F0R2_FB15_Pos                    (15U)
#define CAN_F0R2_FB15_Msk                    (0x1UL << CAN_F0R2_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F0R2_FB15                        CAN_F0R2_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F0R2_FB16_Pos                    (16U)
#define CAN_F0R2_FB16_Msk                    (0x1UL << CAN_F0R2_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F0R2_FB16                        CAN_F0R2_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F0R2_FB17_Pos                    (17U)
#define CAN_F0R2_FB17_Msk                    (0x1UL << CAN_F0R2_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F0R2_FB17                        CAN_F0R2_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F0R2_FB18_Pos                    (18U)
#define CAN_F0R2_FB18_Msk                    (0x1UL << CAN_F0R2_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F0R2_FB18                        CAN_F0R2_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F0R2_FB19_Pos                    (19U)
#define CAN_F0R2_FB19_Msk                    (0x1UL << CAN_F0R2_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F0R2_FB19                        CAN_F0R2_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F0R2_FB20_Pos                    (20U)
#define CAN_F0R2_FB20_Msk                    (0x1UL << CAN_F0R2_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F0R2_FB20                        CAN_F0R2_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F0R2_FB21_Pos                    (21U)
#define CAN_F0R2_FB21_Msk                    (0x1UL << CAN_F0R2_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F0R2_FB21                        CAN_F0R2_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F0R2_FB22_Pos                    (22U)
#define CAN_F0R2_FB22_Msk                    (0x1UL << CAN_F0R2_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F0R2_FB22                        CAN_F0R2_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F0R2_FB23_Pos                    (23U)
#define CAN_F0R2_FB23_Msk                    (0x1UL << CAN_F0R2_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F0R2_FB23                        CAN_F0R2_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F0R2_FB24_Pos                    (24U)
#define CAN_F0R2_FB24_Msk                    (0x1UL << CAN_F0R2_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F0R2_FB24                        CAN_F0R2_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F0R2_FB25_Pos                    (25U)
#define CAN_F0R2_FB25_Msk                    (0x1UL << CAN_F0R2_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F0R2_FB25                        CAN_F0R2_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F0R2_FB26_Pos                    (26U)
#define CAN_F0R2_FB26_Msk                    (0x1UL << CAN_F0R2_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F0R2_FB26                        CAN_F0R2_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F0R2_FB27_Pos                    (27U)
#define CAN_F0R2_FB27_Msk                    (0x1UL << CAN_F0R2_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F0R2_FB27                        CAN_F0R2_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F0R2_FB28_Pos                    (28U)
#define CAN_F0R2_FB28_Msk                    (0x1UL << CAN_F0R2_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F0R2_FB28                        CAN_F0R2_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F0R2_FB29_Pos                    (29U)
#define CAN_F0R2_FB29_Msk                    (0x1UL << CAN_F0R2_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F0R2_FB29                        CAN_F0R2_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F0R2_FB30_Pos                    (30U)
#define CAN_F0R2_FB30_Msk                    (0x1UL << CAN_F0R2_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F0R2_FB30                        CAN_F0R2_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F0R2_FB31_Pos                    (31U)
#define CAN_F0R2_FB31_Msk                    (0x1UL << CAN_F0R2_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F0R2_FB31                        CAN_F0R2_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F1R2 register  *******************/
#define CAN_F1R2_FB0_Pos                     (0U)
#define CAN_F1R2_FB0_Msk                     (0x1UL << CAN_F1R2_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F1R2_FB0                         CAN_F1R2_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F1R2_FB1_Pos                     (1U)
#define CAN_F1R2_FB1_Msk                     (0x1UL << CAN_F1R2_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F1R2_FB1                         CAN_F1R2_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F1R2_FB2_Pos                     (2U)
#define CAN_F1R2_FB2_Msk                     (0x1UL << CAN_F1R2_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F1R2_FB2                         CAN_F1R2_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F1R2_FB3_Pos                     (3U)
#define CAN_F1R2_FB3_Msk                     (0x1UL << CAN_F1R2_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F1R2_FB3                         CAN_F1R2_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F1R2_FB4_Pos                     (4U)
#define CAN_F1R2_FB4_Msk                     (0x1UL << CAN_F1R2_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F1R2_FB4                         CAN_F1R2_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F1R2_FB5_Pos                     (5U)
#define CAN_F1R2_FB5_Msk                     (0x1UL << CAN_F1R2_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F1R2_FB5                         CAN_F1R2_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F1R2_FB6_Pos                     (6U)
#define CAN_F1R2_FB6_Msk                     (0x1UL << CAN_F1R2_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F1R2_FB6                         CAN_F1R2_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F1R2_FB7_Pos                     (7U)
#define CAN_F1R2_FB7_Msk                     (0x1UL << CAN_F1R2_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F1R2_FB7                         CAN_F1R2_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F1R2_FB8_Pos                     (8U)
#define CAN_F1R2_FB8_Msk                     (0x1UL << CAN_F1R2_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F1R2_FB8                         CAN_F1R2_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F1R2_FB9_Pos                     (9U)
#define CAN_F1R2_FB9_Msk                     (0x1UL << CAN_F1R2_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F1R2_FB9                         CAN_F1R2_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F1R2_FB10_Pos                    (10U)
#define CAN_F1R2_FB10_Msk                    (0x1UL << CAN_F1R2_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F1R2_FB10                        CAN_F1R2_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F1R2_FB11_Pos                    (11U)
#define CAN_F1R2_FB11_Msk                    (0x1UL << CAN_F1R2_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F1R2_FB11                        CAN_F1R2_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F1R2_FB12_Pos                    (12U)
#define CAN_F1R2_FB12_Msk                    (0x1UL << CAN_F1R2_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F1R2_FB12                        CAN_F1R2_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F1R2_FB13_Pos                    (13U)
#define CAN_F1R2_FB13_Msk                    (0x1UL << CAN_F1R2_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F1R2_FB13                        CAN_F1R2_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F1R2_FB14_Pos                    (14U)
#define CAN_F1R2_FB14_Msk                    (0x1UL << CAN_F1R2_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F1R2_FB14                        CAN_F1R2_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F1R2_FB15_Pos                    (15U)
#define CAN_F1R2_FB15_Msk                    (0x1UL << CAN_F1R2_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F1R2_FB15                        CAN_F1R2_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F1R2_FB16_Pos                    (16U)
#define CAN_F1R2_FB16_Msk                    (0x1UL << CAN_F1R2_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F1R2_FB16                        CAN_F1R2_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F1R2_FB17_Pos                    (17U)
#define CAN_F1R2_FB17_Msk                    (0x1UL << CAN_F1R2_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F1R2_FB17                        CAN_F1R2_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F1R2_FB18_Pos                    (18U)
#define CAN_F1R2_FB18_Msk                    (0x1UL << CAN_F1R2_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F1R2_FB18                        CAN_F1R2_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F1R2_FB19_Pos                    (19U)
#define CAN_F1R2_FB19_Msk                    (0x1UL << CAN_F1R2_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F1R2_FB19                        CAN_F1R2_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F1R2_FB20_Pos                    (20U)
#define CAN_F1R2_FB20_Msk                    (0x1UL << CAN_F1R2_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F1R2_FB20                        CAN_F1R2_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F1R2_FB21_Pos                    (21U)
#define CAN_F1R2_FB21_Msk                    (0x1UL << CAN_F1R2_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F1R2_FB21                        CAN_F1R2_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F1R2_FB22_Pos                    (22U)
#define CAN_F1R2_FB22_Msk                    (0x1UL << CAN_F1R2_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F1R2_FB22                        CAN_F1R2_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F1R2_FB23_Pos                    (23U)
#define CAN_F1R2_FB23_Msk                    (0x1UL << CAN_F1R2_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F1R2_FB23                        CAN_F1R2_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F1R2_FB24_Pos                    (24U)
#define CAN_F1R2_FB24_Msk                    (0x1UL << CAN_F1R2_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F1R2_FB24                        CAN_F1R2_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F1R2_FB25_Pos                    (25U)
#define CAN_F1R2_FB25_Msk                    (0x1UL << CAN_F1R2_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F1R2_FB25                        CAN_F1R2_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F1R2_FB26_Pos                    (26U)
#define CAN_F1R2_FB26_Msk                    (0x1UL << CAN_F1R2_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F1R2_FB26                        CAN_F1R2_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F1R2_FB27_Pos                    (27U)
#define CAN_F1R2_FB27_Msk                    (0x1UL << CAN_F1R2_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F1R2_FB27                        CAN_F1R2_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F1R2_FB28_Pos                    (28U)
#define CAN_F1R2_FB28_Msk                    (0x1UL << CAN_F1R2_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F1R2_FB28                        CAN_F1R2_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F1R2_FB29_Pos                    (29U)
#define CAN_F1R2_FB29_Msk                    (0x1UL << CAN_F1R2_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F1R2_FB29                        CAN_F1R2_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F1R2_FB30_Pos                    (30U)
#define CAN_F1R2_FB30_Msk                    (0x1UL << CAN_F1R2_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F1R2_FB30                        CAN_F1R2_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F1R2_FB31_Pos                    (31U)
#define CAN_F1R2_FB31_Msk                    (0x1UL << CAN_F1R2_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F1R2_FB31                        CAN_F1R2_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F2R2 register  *******************/
#define CAN_F2R2_FB0_Pos                     (0U)
#define CAN_F2R2_FB0_Msk                     (0x1UL << CAN_F2R2_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F2R2_FB0                         CAN_F2R2_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F2R2_FB1_Pos                     (1U)
#define CAN_F2R2_FB1_Msk                     (0x1UL << CAN_F2R2_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F2R2_FB1                         CAN_F2R2_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F2R2_FB2_Pos                     (2U)
#define CAN_F2R2_FB2_Msk                     (0x1UL << CAN_F2R2_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F2R2_FB2                         CAN_F2R2_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F2R2_FB3_Pos                     (3U)
#define CAN_F2R2_FB3_Msk                     (0x1UL << CAN_F2R2_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F2R2_FB3                         CAN_F2R2_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F2R2_FB4_Pos                     (4U)
#define CAN_F2R2_FB4_Msk                     (0x1UL << CAN_F2R2_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F2R2_FB4                         CAN_F2R2_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F2R2_FB5_Pos                     (5U)
#define CAN_F2R2_FB5_Msk                     (0x1UL << CAN_F2R2_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F2R2_FB5                         CAN_F2R2_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F2R2_FB6_Pos                     (6U)
#define CAN_F2R2_FB6_Msk                     (0x1UL << CAN_F2R2_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F2R2_FB6                         CAN_F2R2_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F2R2_FB7_Pos                     (7U)
#define CAN_F2R2_FB7_Msk                     (0x1UL << CAN_F2R2_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F2R2_FB7                         CAN_F2R2_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F2R2_FB8_Pos                     (8U)
#define CAN_F2R2_FB8_Msk                     (0x1UL << CAN_F2R2_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F2R2_FB8                         CAN_F2R2_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F2R2_FB9_Pos                     (9U)
#define CAN_F2R2_FB9_Msk                     (0x1UL << CAN_F2R2_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F2R2_FB9                         CAN_F2R2_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F2R2_FB10_Pos                    (10U)
#define CAN_F2R2_FB10_Msk                    (0x1UL << CAN_F2R2_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F2R2_FB10                        CAN_F2R2_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F2R2_FB11_Pos                    (11U)
#define CAN_F2R2_FB11_Msk                    (0x1UL << CAN_F2R2_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F2R2_FB11                        CAN_F2R2_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F2R2_FB12_Pos                    (12U)
#define CAN_F2R2_FB12_Msk                    (0x1UL << CAN_F2R2_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F2R2_FB12                        CAN_F2R2_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F2R2_FB13_Pos                    (13U)
#define CAN_F2R2_FB13_Msk                    (0x1UL << CAN_F2R2_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F2R2_FB13                        CAN_F2R2_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F2R2_FB14_Pos                    (14U)
#define CAN_F2R2_FB14_Msk                    (0x1UL << CAN_F2R2_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F2R2_FB14                        CAN_F2R2_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F2R2_FB15_Pos                    (15U)
#define CAN_F2R2_FB15_Msk                    (0x1UL << CAN_F2R2_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F2R2_FB15                        CAN_F2R2_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F2R2_FB16_Pos                    (16U)
#define CAN_F2R2_FB16_Msk                    (0x1UL << CAN_F2R2_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F2R2_FB16                        CAN_F2R2_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F2R2_FB17_Pos                    (17U)
#define CAN_F2R2_FB17_Msk                    (0x1UL << CAN_F2R2_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F2R2_FB17                        CAN_F2R2_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F2R2_FB18_Pos                    (18U)
#define CAN_F2R2_FB18_Msk                    (0x1UL << CAN_F2R2_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F2R2_FB18                        CAN_F2R2_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F2R2_FB19_Pos                    (19U)
#define CAN_F2R2_FB19_Msk                    (0x1UL << CAN_F2R2_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F2R2_FB19                        CAN_F2R2_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F2R2_FB20_Pos                    (20U)
#define CAN_F2R2_FB20_Msk                    (0x1UL << CAN_F2R2_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F2R2_FB20                        CAN_F2R2_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F2R2_FB21_Pos                    (21U)
#define CAN_F2R2_FB21_Msk                    (0x1UL << CAN_F2R2_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F2R2_FB21                        CAN_F2R2_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F2R2_FB22_Pos                    (22U)
#define CAN_F2R2_FB22_Msk                    (0x1UL << CAN_F2R2_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F2R2_FB22                        CAN_F2R2_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F2R2_FB23_Pos                    (23U)
#define CAN_F2R2_FB23_Msk                    (0x1UL << CAN_F2R2_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F2R2_FB23                        CAN_F2R2_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F2R2_FB24_Pos                    (24U)
#define CAN_F2R2_FB24_Msk                    (0x1UL << CAN_F2R2_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F2R2_FB24                        CAN_F2R2_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F2R2_FB25_Pos                    (25U)
#define CAN_F2R2_FB25_Msk                    (0x1UL << CAN_F2R2_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F2R2_FB25                        CAN_F2R2_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F2R2_FB26_Pos                    (26U)
#define CAN_F2R2_FB26_Msk                    (0x1UL << CAN_F2R2_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F2R2_FB26                        CAN_F2R2_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F2R2_FB27_Pos                    (27U)
#define CAN_F2R2_FB27_Msk                    (0x1UL << CAN_F2R2_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F2R2_FB27                        CAN_F2R2_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F2R2_FB28_Pos                    (28U)
#define CAN_F2R2_FB28_Msk                    (0x1UL << CAN_F2R2_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F2R2_FB28                        CAN_F2R2_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F2R2_FB29_Pos                    (29U)
#define CAN_F2R2_FB29_Msk                    (0x1UL << CAN_F2R2_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F2R2_FB29                        CAN_F2R2_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F2R2_FB30_Pos                    (30U)
#define CAN_F2R2_FB30_Msk                    (0x1UL << CAN_F2R2_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F2R2_FB30                        CAN_F2R2_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F2R2_FB31_Pos                    (31U)
#define CAN_F2R2_FB31_Msk                    (0x1UL << CAN_F2R2_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F2R2_FB31                        CAN_F2R2_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F3R2 register  *******************/
#define CAN_F3R2_FB0_Pos                     (0U)
#define CAN_F3R2_FB0_Msk                     (0x1UL << CAN_F3R2_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F3R2_FB0                         CAN_F3R2_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F3R2_FB1_Pos                     (1U)
#define CAN_F3R2_FB1_Msk                     (0x1UL << CAN_F3R2_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F3R2_FB1                         CAN_F3R2_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F3R2_FB2_Pos                     (2U)
#define CAN_F3R2_FB2_Msk                     (0x1UL << CAN_F3R2_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F3R2_FB2                         CAN_F3R2_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F3R2_FB3_Pos                     (3U)
#define CAN_F3R2_FB3_Msk                     (0x1UL << CAN_F3R2_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F3R2_FB3                         CAN_F3R2_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F3R2_FB4_Pos                     (4U)
#define CAN_F3R2_FB4_Msk                     (0x1UL << CAN_F3R2_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F3R2_FB4                         CAN_F3R2_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F3R2_FB5_Pos                     (5U)
#define CAN_F3R2_FB5_Msk                     (0x1UL << CAN_F3R2_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F3R2_FB5                         CAN_F3R2_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F3R2_FB6_Pos                     (6U)
#define CAN_F3R2_FB6_Msk                     (0x1UL << CAN_F3R2_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F3R2_FB6                         CAN_F3R2_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F3R2_FB7_Pos                     (7U)
#define CAN_F3R2_FB7_Msk                     (0x1UL << CAN_F3R2_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F3R2_FB7                         CAN_F3R2_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F3R2_FB8_Pos                     (8U)
#define CAN_F3R2_FB8_Msk                     (0x1UL << CAN_F3R2_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F3R2_FB8                         CAN_F3R2_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F3R2_FB9_Pos                     (9U)
#define CAN_F3R2_FB9_Msk                     (0x1UL << CAN_F3R2_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F3R2_FB9                         CAN_F3R2_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F3R2_FB10_Pos                    (10U)
#define CAN_F3R2_FB10_Msk                    (0x1UL << CAN_F3R2_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F3R2_FB10                        CAN_F3R2_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F3R2_FB11_Pos                    (11U)
#define CAN_F3R2_FB11_Msk                    (0x1UL << CAN_F3R2_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F3R2_FB11                        CAN_F3R2_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F3R2_FB12_Pos                    (12U)
#define CAN_F3R2_FB12_Msk                    (0x1UL << CAN_F3R2_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F3R2_FB12                        CAN_F3R2_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F3R2_FB13_Pos                    (13U)
#define CAN_F3R2_FB13_Msk                    (0x1UL << CAN_F3R2_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F3R2_FB13                        CAN_F3R2_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F3R2_FB14_Pos                    (14U)
#define CAN_F3R2_FB14_Msk                    (0x1UL << CAN_F3R2_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F3R2_FB14                        CAN_F3R2_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F3R2_FB15_Pos                    (15U)
#define CAN_F3R2_FB15_Msk                    (0x1UL << CAN_F3R2_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F3R2_FB15                        CAN_F3R2_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F3R2_FB16_Pos                    (16U)
#define CAN_F3R2_FB16_Msk                    (0x1UL << CAN_F3R2_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F3R2_FB16                        CAN_F3R2_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F3R2_FB17_Pos                    (17U)
#define CAN_F3R2_FB17_Msk                    (0x1UL << CAN_F3R2_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F3R2_FB17                        CAN_F3R2_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F3R2_FB18_Pos                    (18U)
#define CAN_F3R2_FB18_Msk                    (0x1UL << CAN_F3R2_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F3R2_FB18                        CAN_F3R2_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F3R2_FB19_Pos                    (19U)
#define CAN_F3R2_FB19_Msk                    (0x1UL << CAN_F3R2_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F3R2_FB19                        CAN_F3R2_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F3R2_FB20_Pos                    (20U)
#define CAN_F3R2_FB20_Msk                    (0x1UL << CAN_F3R2_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F3R2_FB20                        CAN_F3R2_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F3R2_FB21_Pos                    (21U)
#define CAN_F3R2_FB21_Msk                    (0x1UL << CAN_F3R2_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F3R2_FB21                        CAN_F3R2_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F3R2_FB22_Pos                    (22U)
#define CAN_F3R2_FB22_Msk                    (0x1UL << CAN_F3R2_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F3R2_FB22                        CAN_F3R2_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F3R2_FB23_Pos                    (23U)
#define CAN_F3R2_FB23_Msk                    (0x1UL << CAN_F3R2_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F3R2_FB23                        CAN_F3R2_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F3R2_FB24_Pos                    (24U)
#define CAN_F3R2_FB24_Msk                    (0x1UL << CAN_F3R2_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F3R2_FB24                        CAN_F3R2_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F3R2_FB25_Pos                    (25U)
#define CAN_F3R2_FB25_Msk                    (0x1UL << CAN_F3R2_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F3R2_FB25                        CAN_F3R2_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F3R2_FB26_Pos                    (26U)
#define CAN_F3R2_FB26_Msk                    (0x1UL << CAN_F3R2_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F3R2_FB26                        CAN_F3R2_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F3R2_FB27_Pos                    (27U)
#define CAN_F3R2_FB27_Msk                    (0x1UL << CAN_F3R2_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F3R2_FB27                        CAN_F3R2_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F3R2_FB28_Pos                    (28U)
#define CAN_F3R2_FB28_Msk                    (0x1UL << CAN_F3R2_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F3R2_FB28                        CAN_F3R2_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F3R2_FB29_Pos                    (29U)
#define CAN_F3R2_FB29_Msk                    (0x1UL << CAN_F3R2_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F3R2_FB29                        CAN_F3R2_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F3R2_FB30_Pos                    (30U)
#define CAN_F3R2_FB30_Msk                    (0x1UL << CAN_F3R2_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F3R2_FB30                        CAN_F3R2_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F3R2_FB31_Pos                    (31U)
#define CAN_F3R2_FB31_Msk                    (0x1UL << CAN_F3R2_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F3R2_FB31                        CAN_F3R2_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F4R2 register  *******************/
#define CAN_F4R2_FB0_Pos                     (0U)
#define CAN_F4R2_FB0_Msk                     (0x1UL << CAN_F4R2_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F4R2_FB0                         CAN_F4R2_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F4R2_FB1_Pos                     (1U)
#define CAN_F4R2_FB1_Msk                     (0x1UL << CAN_F4R2_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F4R2_FB1                         CAN_F4R2_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F4R2_FB2_Pos                     (2U)
#define CAN_F4R2_FB2_Msk                     (0x1UL << CAN_F4R2_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F4R2_FB2                         CAN_F4R2_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F4R2_FB3_Pos                     (3U)
#define CAN_F4R2_FB3_Msk                     (0x1UL << CAN_F4R2_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F4R2_FB3                         CAN_F4R2_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F4R2_FB4_Pos                     (4U)
#define CAN_F4R2_FB4_Msk                     (0x1UL << CAN_F4R2_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F4R2_FB4                         CAN_F4R2_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F4R2_FB5_Pos                     (5U)
#define CAN_F4R2_FB5_Msk                     (0x1UL << CAN_F4R2_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F4R2_FB5                         CAN_F4R2_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F4R2_FB6_Pos                     (6U)
#define CAN_F4R2_FB6_Msk                     (0x1UL << CAN_F4R2_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F4R2_FB6                         CAN_F4R2_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F4R2_FB7_Pos                     (7U)
#define CAN_F4R2_FB7_Msk                     (0x1UL << CAN_F4R2_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F4R2_FB7                         CAN_F4R2_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F4R2_FB8_Pos                     (8U)
#define CAN_F4R2_FB8_Msk                     (0x1UL << CAN_F4R2_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F4R2_FB8                         CAN_F4R2_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F4R2_FB9_Pos                     (9U)
#define CAN_F4R2_FB9_Msk                     (0x1UL << CAN_F4R2_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F4R2_FB9                         CAN_F4R2_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F4R2_FB10_Pos                    (10U)
#define CAN_F4R2_FB10_Msk                    (0x1UL << CAN_F4R2_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F4R2_FB10                        CAN_F4R2_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F4R2_FB11_Pos                    (11U)
#define CAN_F4R2_FB11_Msk                    (0x1UL << CAN_F4R2_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F4R2_FB11                        CAN_F4R2_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F4R2_FB12_Pos                    (12U)
#define CAN_F4R2_FB12_Msk                    (0x1UL << CAN_F4R2_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F4R2_FB12                        CAN_F4R2_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F4R2_FB13_Pos                    (13U)
#define CAN_F4R2_FB13_Msk                    (0x1UL << CAN_F4R2_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F4R2_FB13                        CAN_F4R2_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F4R2_FB14_Pos                    (14U)
#define CAN_F4R2_FB14_Msk                    (0x1UL << CAN_F4R2_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F4R2_FB14                        CAN_F4R2_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F4R2_FB15_Pos                    (15U)
#define CAN_F4R2_FB15_Msk                    (0x1UL << CAN_F4R2_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F4R2_FB15                        CAN_F4R2_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F4R2_FB16_Pos                    (16U)
#define CAN_F4R2_FB16_Msk                    (0x1UL << CAN_F4R2_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F4R2_FB16                        CAN_F4R2_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F4R2_FB17_Pos                    (17U)
#define CAN_F4R2_FB17_Msk                    (0x1UL << CAN_F4R2_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F4R2_FB17                        CAN_F4R2_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F4R2_FB18_Pos                    (18U)
#define CAN_F4R2_FB18_Msk                    (0x1UL << CAN_F4R2_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F4R2_FB18                        CAN_F4R2_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F4R2_FB19_Pos                    (19U)
#define CAN_F4R2_FB19_Msk                    (0x1UL << CAN_F4R2_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F4R2_FB19                        CAN_F4R2_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F4R2_FB20_Pos                    (20U)
#define CAN_F4R2_FB20_Msk                    (0x1UL << CAN_F4R2_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F4R2_FB20                        CAN_F4R2_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F4R2_FB21_Pos                    (21U)
#define CAN_F4R2_FB21_Msk                    (0x1UL << CAN_F4R2_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F4R2_FB21                        CAN_F4R2_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F4R2_FB22_Pos                    (22U)
#define CAN_F4R2_FB22_Msk                    (0x1UL << CAN_F4R2_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F4R2_FB22                        CAN_F4R2_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F4R2_FB23_Pos                    (23U)
#define CAN_F4R2_FB23_Msk                    (0x1UL << CAN_F4R2_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F4R2_FB23                        CAN_F4R2_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F4R2_FB24_Pos                    (24U)
#define CAN_F4R2_FB24_Msk                    (0x1UL << CAN_F4R2_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F4R2_FB24                        CAN_F4R2_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F4R2_FB25_Pos                    (25U)
#define CAN_F4R2_FB25_Msk                    (0x1UL << CAN_F4R2_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F4R2_FB25                        CAN_F4R2_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F4R2_FB26_Pos                    (26U)
#define CAN_F4R2_FB26_Msk                    (0x1UL << CAN_F4R2_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F4R2_FB26                        CAN_F4R2_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F4R2_FB27_Pos                    (27U)
#define CAN_F4R2_FB27_Msk                    (0x1UL << CAN_F4R2_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F4R2_FB27                        CAN_F4R2_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F4R2_FB28_Pos                    (28U)
#define CAN_F4R2_FB28_Msk                    (0x1UL << CAN_F4R2_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F4R2_FB28                        CAN_F4R2_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F4R2_FB29_Pos                    (29U)
#define CAN_F4R2_FB29_Msk                    (0x1UL << CAN_F4R2_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F4R2_FB29                        CAN_F4R2_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F4R2_FB30_Pos                    (30U)
#define CAN_F4R2_FB30_Msk                    (0x1UL << CAN_F4R2_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F4R2_FB30                        CAN_F4R2_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F4R2_FB31_Pos                    (31U)
#define CAN_F4R2_FB31_Msk                    (0x1UL << CAN_F4R2_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F4R2_FB31                        CAN_F4R2_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F5R2 register  *******************/
#define CAN_F5R2_FB0_Pos                     (0U)
#define CAN_F5R2_FB0_Msk                     (0x1UL << CAN_F5R2_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F5R2_FB0                         CAN_F5R2_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F5R2_FB1_Pos                     (1U)
#define CAN_F5R2_FB1_Msk                     (0x1UL << CAN_F5R2_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F5R2_FB1                         CAN_F5R2_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F5R2_FB2_Pos                     (2U)
#define CAN_F5R2_FB2_Msk                     (0x1UL << CAN_F5R2_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F5R2_FB2                         CAN_F5R2_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F5R2_FB3_Pos                     (3U)
#define CAN_F5R2_FB3_Msk                     (0x1UL << CAN_F5R2_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F5R2_FB3                         CAN_F5R2_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F5R2_FB4_Pos                     (4U)
#define CAN_F5R2_FB4_Msk                     (0x1UL << CAN_F5R2_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F5R2_FB4                         CAN_F5R2_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F5R2_FB5_Pos                     (5U)
#define CAN_F5R2_FB5_Msk                     (0x1UL << CAN_F5R2_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F5R2_FB5                         CAN_F5R2_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F5R2_FB6_Pos                     (6U)
#define CAN_F5R2_FB6_Msk                     (0x1UL << CAN_F5R2_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F5R2_FB6                         CAN_F5R2_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F5R2_FB7_Pos                     (7U)
#define CAN_F5R2_FB7_Msk                     (0x1UL << CAN_F5R2_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F5R2_FB7                         CAN_F5R2_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F5R2_FB8_Pos                     (8U)
#define CAN_F5R2_FB8_Msk                     (0x1UL << CAN_F5R2_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F5R2_FB8                         CAN_F5R2_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F5R2_FB9_Pos                     (9U)
#define CAN_F5R2_FB9_Msk                     (0x1UL << CAN_F5R2_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F5R2_FB9                         CAN_F5R2_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F5R2_FB10_Pos                    (10U)
#define CAN_F5R2_FB10_Msk                    (0x1UL << CAN_F5R2_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F5R2_FB10                        CAN_F5R2_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F5R2_FB11_Pos                    (11U)
#define CAN_F5R2_FB11_Msk                    (0x1UL << CAN_F5R2_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F5R2_FB11                        CAN_F5R2_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F5R2_FB12_Pos                    (12U)
#define CAN_F5R2_FB12_Msk                    (0x1UL << CAN_F5R2_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F5R2_FB12                        CAN_F5R2_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F5R2_FB13_Pos                    (13U)
#define CAN_F5R2_FB13_Msk                    (0x1UL << CAN_F5R2_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F5R2_FB13                        CAN_F5R2_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F5R2_FB14_Pos                    (14U)
#define CAN_F5R2_FB14_Msk                    (0x1UL << CAN_F5R2_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F5R2_FB14                        CAN_F5R2_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F5R2_FB15_Pos                    (15U)
#define CAN_F5R2_FB15_Msk                    (0x1UL << CAN_F5R2_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F5R2_FB15                        CAN_F5R2_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F5R2_FB16_Pos                    (16U)
#define CAN_F5R2_FB16_Msk                    (0x1UL << CAN_F5R2_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F5R2_FB16                        CAN_F5R2_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F5R2_FB17_Pos                    (17U)
#define CAN_F5R2_FB17_Msk                    (0x1UL << CAN_F5R2_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F5R2_FB17                        CAN_F5R2_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F5R2_FB18_Pos                    (18U)
#define CAN_F5R2_FB18_Msk                    (0x1UL << CAN_F5R2_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F5R2_FB18                        CAN_F5R2_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F5R2_FB19_Pos                    (19U)
#define CAN_F5R2_FB19_Msk                    (0x1UL << CAN_F5R2_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F5R2_FB19                        CAN_F5R2_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F5R2_FB20_Pos                    (20U)
#define CAN_F5R2_FB20_Msk                    (0x1UL << CAN_F5R2_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F5R2_FB20                        CAN_F5R2_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F5R2_FB21_Pos                    (21U)
#define CAN_F5R2_FB21_Msk                    (0x1UL << CAN_F5R2_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F5R2_FB21                        CAN_F5R2_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F5R2_FB22_Pos                    (22U)
#define CAN_F5R2_FB22_Msk                    (0x1UL << CAN_F5R2_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F5R2_FB22                        CAN_F5R2_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F5R2_FB23_Pos                    (23U)
#define CAN_F5R2_FB23_Msk                    (0x1UL << CAN_F5R2_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F5R2_FB23                        CAN_F5R2_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F5R2_FB24_Pos                    (24U)
#define CAN_F5R2_FB24_Msk                    (0x1UL << CAN_F5R2_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F5R2_FB24                        CAN_F5R2_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F5R2_FB25_Pos                    (25U)
#define CAN_F5R2_FB25_Msk                    (0x1UL << CAN_F5R2_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F5R2_FB25                        CAN_F5R2_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F5R2_FB26_Pos                    (26U)
#define CAN_F5R2_FB26_Msk                    (0x1UL << CAN_F5R2_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F5R2_FB26                        CAN_F5R2_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F5R2_FB27_Pos                    (27U)
#define CAN_F5R2_FB27_Msk                    (0x1UL << CAN_F5R2_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F5R2_FB27                        CAN_F5R2_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F5R2_FB28_Pos                    (28U)
#define CAN_F5R2_FB28_Msk                    (0x1UL << CAN_F5R2_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F5R2_FB28                        CAN_F5R2_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F5R2_FB29_Pos                    (29U)
#define CAN_F5R2_FB29_Msk                    (0x1UL << CAN_F5R2_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F5R2_FB29                        CAN_F5R2_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F5R2_FB30_Pos                    (30U)
#define CAN_F5R2_FB30_Msk                    (0x1UL << CAN_F5R2_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F5R2_FB30                        CAN_F5R2_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F5R2_FB31_Pos                    (31U)
#define CAN_F5R2_FB31_Msk                    (0x1UL << CAN_F5R2_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F5R2_FB31                        CAN_F5R2_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F6R2 register  *******************/
#define CAN_F6R2_FB0_Pos                     (0U)
#define CAN_F6R2_FB0_Msk                     (0x1UL << CAN_F6R2_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F6R2_FB0                         CAN_F6R2_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F6R2_FB1_Pos                     (1U)
#define CAN_F6R2_FB1_Msk                     (0x1UL << CAN_F6R2_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F6R2_FB1                         CAN_F6R2_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F6R2_FB2_Pos                     (2U)
#define CAN_F6R2_FB2_Msk                     (0x1UL << CAN_F6R2_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F6R2_FB2                         CAN_F6R2_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F6R2_FB3_Pos                     (3U)
#define CAN_F6R2_FB3_Msk                     (0x1UL << CAN_F6R2_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F6R2_FB3                         CAN_F6R2_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F6R2_FB4_Pos                     (4U)
#define CAN_F6R2_FB4_Msk                     (0x1UL << CAN_F6R2_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F6R2_FB4                         CAN_F6R2_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F6R2_FB5_Pos                     (5U)
#define CAN_F6R2_FB5_Msk                     (0x1UL << CAN_F6R2_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F6R2_FB5                         CAN_F6R2_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F6R2_FB6_Pos                     (6U)
#define CAN_F6R2_FB6_Msk                     (0x1UL << CAN_F6R2_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F6R2_FB6                         CAN_F6R2_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F6R2_FB7_Pos                     (7U)
#define CAN_F6R2_FB7_Msk                     (0x1UL << CAN_F6R2_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F6R2_FB7                         CAN_F6R2_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F6R2_FB8_Pos                     (8U)
#define CAN_F6R2_FB8_Msk                     (0x1UL << CAN_F6R2_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F6R2_FB8                         CAN_F6R2_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F6R2_FB9_Pos                     (9U)
#define CAN_F6R2_FB9_Msk                     (0x1UL << CAN_F6R2_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F6R2_FB9                         CAN_F6R2_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F6R2_FB10_Pos                    (10U)
#define CAN_F6R2_FB10_Msk                    (0x1UL << CAN_F6R2_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F6R2_FB10                        CAN_F6R2_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F6R2_FB11_Pos                    (11U)
#define CAN_F6R2_FB11_Msk                    (0x1UL << CAN_F6R2_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F6R2_FB11                        CAN_F6R2_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F6R2_FB12_Pos                    (12U)
#define CAN_F6R2_FB12_Msk                    (0x1UL << CAN_F6R2_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F6R2_FB12                        CAN_F6R2_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F6R2_FB13_Pos                    (13U)
#define CAN_F6R2_FB13_Msk                    (0x1UL << CAN_F6R2_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F6R2_FB13                        CAN_F6R2_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F6R2_FB14_Pos                    (14U)
#define CAN_F6R2_FB14_Msk                    (0x1UL << CAN_F6R2_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F6R2_FB14                        CAN_F6R2_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F6R2_FB15_Pos                    (15U)
#define CAN_F6R2_FB15_Msk                    (0x1UL << CAN_F6R2_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F6R2_FB15                        CAN_F6R2_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F6R2_FB16_Pos                    (16U)
#define CAN_F6R2_FB16_Msk                    (0x1UL << CAN_F6R2_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F6R2_FB16                        CAN_F6R2_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F6R2_FB17_Pos                    (17U)
#define CAN_F6R2_FB17_Msk                    (0x1UL << CAN_F6R2_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F6R2_FB17                        CAN_F6R2_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F6R2_FB18_Pos                    (18U)
#define CAN_F6R2_FB18_Msk                    (0x1UL << CAN_F6R2_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F6R2_FB18                        CAN_F6R2_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F6R2_FB19_Pos                    (19U)
#define CAN_F6R2_FB19_Msk                    (0x1UL << CAN_F6R2_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F6R2_FB19                        CAN_F6R2_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F6R2_FB20_Pos                    (20U)
#define CAN_F6R2_FB20_Msk                    (0x1UL << CAN_F6R2_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F6R2_FB20                        CAN_F6R2_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F6R2_FB21_Pos                    (21U)
#define CAN_F6R2_FB21_Msk                    (0x1UL << CAN_F6R2_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F6R2_FB21                        CAN_F6R2_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F6R2_FB22_Pos                    (22U)
#define CAN_F6R2_FB22_Msk                    (0x1UL << CAN_F6R2_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F6R2_FB22                        CAN_F6R2_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F6R2_FB23_Pos                    (23U)
#define CAN_F6R2_FB23_Msk                    (0x1UL << CAN_F6R2_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F6R2_FB23                        CAN_F6R2_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F6R2_FB24_Pos                    (24U)
#define CAN_F6R2_FB24_Msk                    (0x1UL << CAN_F6R2_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F6R2_FB24                        CAN_F6R2_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F6R2_FB25_Pos                    (25U)
#define CAN_F6R2_FB25_Msk                    (0x1UL << CAN_F6R2_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F6R2_FB25                        CAN_F6R2_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F6R2_FB26_Pos                    (26U)
#define CAN_F6R2_FB26_Msk                    (0x1UL << CAN_F6R2_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F6R2_FB26                        CAN_F6R2_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F6R2_FB27_Pos                    (27U)
#define CAN_F6R2_FB27_Msk                    (0x1UL << CAN_F6R2_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F6R2_FB27                        CAN_F6R2_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F6R2_FB28_Pos                    (28U)
#define CAN_F6R2_FB28_Msk                    (0x1UL << CAN_F6R2_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F6R2_FB28                        CAN_F6R2_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F6R2_FB29_Pos                    (29U)
#define CAN_F6R2_FB29_Msk                    (0x1UL << CAN_F6R2_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F6R2_FB29                        CAN_F6R2_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F6R2_FB30_Pos                    (30U)
#define CAN_F6R2_FB30_Msk                    (0x1UL << CAN_F6R2_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F6R2_FB30                        CAN_F6R2_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F6R2_FB31_Pos                    (31U)
#define CAN_F6R2_FB31_Msk                    (0x1UL << CAN_F6R2_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F6R2_FB31                        CAN_F6R2_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F7R2 register  *******************/
#define CAN_F7R2_FB0_Pos                     (0U)
#define CAN_F7R2_FB0_Msk                     (0x1UL << CAN_F7R2_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F7R2_FB0                         CAN_F7R2_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F7R2_FB1_Pos                     (1U)
#define CAN_F7R2_FB1_Msk                     (0x1UL << CAN_F7R2_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F7R2_FB1                         CAN_F7R2_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F7R2_FB2_Pos                     (2U)
#define CAN_F7R2_FB2_Msk                     (0x1UL << CAN_F7R2_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F7R2_FB2                         CAN_F7R2_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F7R2_FB3_Pos                     (3U)
#define CAN_F7R2_FB3_Msk                     (0x1UL << CAN_F7R2_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F7R2_FB3                         CAN_F7R2_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F7R2_FB4_Pos                     (4U)
#define CAN_F7R2_FB4_Msk                     (0x1UL << CAN_F7R2_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F7R2_FB4                         CAN_F7R2_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F7R2_FB5_Pos                     (5U)
#define CAN_F7R2_FB5_Msk                     (0x1UL << CAN_F7R2_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F7R2_FB5                         CAN_F7R2_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F7R2_FB6_Pos                     (6U)
#define CAN_F7R2_FB6_Msk                     (0x1UL << CAN_F7R2_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F7R2_FB6                         CAN_F7R2_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F7R2_FB7_Pos                     (7U)
#define CAN_F7R2_FB7_Msk                     (0x1UL << CAN_F7R2_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F7R2_FB7                         CAN_F7R2_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F7R2_FB8_Pos                     (8U)
#define CAN_F7R2_FB8_Msk                     (0x1UL << CAN_F7R2_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F7R2_FB8                         CAN_F7R2_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F7R2_FB9_Pos                     (9U)
#define CAN_F7R2_FB9_Msk                     (0x1UL << CAN_F7R2_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F7R2_FB9                         CAN_F7R2_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F7R2_FB10_Pos                    (10U)
#define CAN_F7R2_FB10_Msk                    (0x1UL << CAN_F7R2_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F7R2_FB10                        CAN_F7R2_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F7R2_FB11_Pos                    (11U)
#define CAN_F7R2_FB11_Msk                    (0x1UL << CAN_F7R2_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F7R2_FB11                        CAN_F7R2_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F7R2_FB12_Pos                    (12U)
#define CAN_F7R2_FB12_Msk                    (0x1UL << CAN_F7R2_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F7R2_FB12                        CAN_F7R2_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F7R2_FB13_Pos                    (13U)
#define CAN_F7R2_FB13_Msk                    (0x1UL << CAN_F7R2_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F7R2_FB13                        CAN_F7R2_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F7R2_FB14_Pos                    (14U)
#define CAN_F7R2_FB14_Msk                    (0x1UL << CAN_F7R2_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F7R2_FB14                        CAN_F7R2_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F7R2_FB15_Pos                    (15U)
#define CAN_F7R2_FB15_Msk                    (0x1UL << CAN_F7R2_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F7R2_FB15                        CAN_F7R2_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F7R2_FB16_Pos                    (16U)
#define CAN_F7R2_FB16_Msk                    (0x1UL << CAN_F7R2_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F7R2_FB16                        CAN_F7R2_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F7R2_FB17_Pos                    (17U)
#define CAN_F7R2_FB17_Msk                    (0x1UL << CAN_F7R2_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F7R2_FB17                        CAN_F7R2_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F7R2_FB18_Pos                    (18U)
#define CAN_F7R2_FB18_Msk                    (0x1UL << CAN_F7R2_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F7R2_FB18                        CAN_F7R2_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F7R2_FB19_Pos                    (19U)
#define CAN_F7R2_FB19_Msk                    (0x1UL << CAN_F7R2_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F7R2_FB19                        CAN_F7R2_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F7R2_FB20_Pos                    (20U)
#define CAN_F7R2_FB20_Msk                    (0x1UL << CAN_F7R2_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F7R2_FB20                        CAN_F7R2_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F7R2_FB21_Pos                    (21U)
#define CAN_F7R2_FB21_Msk                    (0x1UL << CAN_F7R2_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F7R2_FB21                        CAN_F7R2_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F7R2_FB22_Pos                    (22U)
#define CAN_F7R2_FB22_Msk                    (0x1UL << CAN_F7R2_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F7R2_FB22                        CAN_F7R2_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F7R2_FB23_Pos                    (23U)
#define CAN_F7R2_FB23_Msk                    (0x1UL << CAN_F7R2_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F7R2_FB23                        CAN_F7R2_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F7R2_FB24_Pos                    (24U)
#define CAN_F7R2_FB24_Msk                    (0x1UL << CAN_F7R2_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F7R2_FB24                        CAN_F7R2_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F7R2_FB25_Pos                    (25U)
#define CAN_F7R2_FB25_Msk                    (0x1UL << CAN_F7R2_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F7R2_FB25                        CAN_F7R2_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F7R2_FB26_Pos                    (26U)
#define CAN_F7R2_FB26_Msk                    (0x1UL << CAN_F7R2_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F7R2_FB26                        CAN_F7R2_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F7R2_FB27_Pos                    (27U)
#define CAN_F7R2_FB27_Msk                    (0x1UL << CAN_F7R2_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F7R2_FB27                        CAN_F7R2_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F7R2_FB28_Pos                    (28U)
#define CAN_F7R2_FB28_Msk                    (0x1UL << CAN_F7R2_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F7R2_FB28                        CAN_F7R2_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F7R2_FB29_Pos                    (29U)
#define CAN_F7R2_FB29_Msk                    (0x1UL << CAN_F7R2_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F7R2_FB29                        CAN_F7R2_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F7R2_FB30_Pos                    (30U)
#define CAN_F7R2_FB30_Msk                    (0x1UL << CAN_F7R2_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F7R2_FB30                        CAN_F7R2_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F7R2_FB31_Pos                    (31U)
#define CAN_F7R2_FB31_Msk                    (0x1UL << CAN_F7R2_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F7R2_FB31                        CAN_F7R2_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F8R2 register  *******************/
#define CAN_F8R2_FB0_Pos                     (0U)
#define CAN_F8R2_FB0_Msk                     (0x1UL << CAN_F8R2_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F8R2_FB0                         CAN_F8R2_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F8R2_FB1_Pos                     (1U)
#define CAN_F8R2_FB1_Msk                     (0x1UL << CAN_F8R2_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F8R2_FB1                         CAN_F8R2_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F8R2_FB2_Pos                     (2U)
#define CAN_F8R2_FB2_Msk                     (0x1UL << CAN_F8R2_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F8R2_FB2                         CAN_F8R2_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F8R2_FB3_Pos                     (3U)
#define CAN_F8R2_FB3_Msk                     (0x1UL << CAN_F8R2_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F8R2_FB3                         CAN_F8R2_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F8R2_FB4_Pos                     (4U)
#define CAN_F8R2_FB4_Msk                     (0x1UL << CAN_F8R2_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F8R2_FB4                         CAN_F8R2_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F8R2_FB5_Pos                     (5U)
#define CAN_F8R2_FB5_Msk                     (0x1UL << CAN_F8R2_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F8R2_FB5                         CAN_F8R2_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F8R2_FB6_Pos                     (6U)
#define CAN_F8R2_FB6_Msk                     (0x1UL << CAN_F8R2_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F8R2_FB6                         CAN_F8R2_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F8R2_FB7_Pos                     (7U)
#define CAN_F8R2_FB7_Msk                     (0x1UL << CAN_F8R2_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F8R2_FB7                         CAN_F8R2_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F8R2_FB8_Pos                     (8U)
#define CAN_F8R2_FB8_Msk                     (0x1UL << CAN_F8R2_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F8R2_FB8                         CAN_F8R2_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F8R2_FB9_Pos                     (9U)
#define CAN_F8R2_FB9_Msk                     (0x1UL << CAN_F8R2_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F8R2_FB9                         CAN_F8R2_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F8R2_FB10_Pos                    (10U)
#define CAN_F8R2_FB10_Msk                    (0x1UL << CAN_F8R2_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F8R2_FB10                        CAN_F8R2_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F8R2_FB11_Pos                    (11U)
#define CAN_F8R2_FB11_Msk                    (0x1UL << CAN_F8R2_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F8R2_FB11                        CAN_F8R2_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F8R2_FB12_Pos                    (12U)
#define CAN_F8R2_FB12_Msk                    (0x1UL << CAN_F8R2_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F8R2_FB12                        CAN_F8R2_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F8R2_FB13_Pos                    (13U)
#define CAN_F8R2_FB13_Msk                    (0x1UL << CAN_F8R2_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F8R2_FB13                        CAN_F8R2_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F8R2_FB14_Pos                    (14U)
#define CAN_F8R2_FB14_Msk                    (0x1UL << CAN_F8R2_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F8R2_FB14                        CAN_F8R2_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F8R2_FB15_Pos                    (15U)
#define CAN_F8R2_FB15_Msk                    (0x1UL << CAN_F8R2_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F8R2_FB15                        CAN_F8R2_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F8R2_FB16_Pos                    (16U)
#define CAN_F8R2_FB16_Msk                    (0x1UL << CAN_F8R2_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F8R2_FB16                        CAN_F8R2_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F8R2_FB17_Pos                    (17U)
#define CAN_F8R2_FB17_Msk                    (0x1UL << CAN_F8R2_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F8R2_FB17                        CAN_F8R2_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F8R2_FB18_Pos                    (18U)
#define CAN_F8R2_FB18_Msk                    (0x1UL << CAN_F8R2_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F8R2_FB18                        CAN_F8R2_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F8R2_FB19_Pos                    (19U)
#define CAN_F8R2_FB19_Msk                    (0x1UL << CAN_F8R2_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F8R2_FB19                        CAN_F8R2_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F8R2_FB20_Pos                    (20U)
#define CAN_F8R2_FB20_Msk                    (0x1UL << CAN_F8R2_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F8R2_FB20                        CAN_F8R2_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F8R2_FB21_Pos                    (21U)
#define CAN_F8R2_FB21_Msk                    (0x1UL << CAN_F8R2_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F8R2_FB21                        CAN_F8R2_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F8R2_FB22_Pos                    (22U)
#define CAN_F8R2_FB22_Msk                    (0x1UL << CAN_F8R2_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F8R2_FB22                        CAN_F8R2_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F8R2_FB23_Pos                    (23U)
#define CAN_F8R2_FB23_Msk                    (0x1UL << CAN_F8R2_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F8R2_FB23                        CAN_F8R2_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F8R2_FB24_Pos                    (24U)
#define CAN_F8R2_FB24_Msk                    (0x1UL << CAN_F8R2_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F8R2_FB24                        CAN_F8R2_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F8R2_FB25_Pos                    (25U)
#define CAN_F8R2_FB25_Msk                    (0x1UL << CAN_F8R2_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F8R2_FB25                        CAN_F8R2_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F8R2_FB26_Pos                    (26U)
#define CAN_F8R2_FB26_Msk                    (0x1UL << CAN_F8R2_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F8R2_FB26                        CAN_F8R2_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F8R2_FB27_Pos                    (27U)
#define CAN_F8R2_FB27_Msk                    (0x1UL << CAN_F8R2_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F8R2_FB27                        CAN_F8R2_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F8R2_FB28_Pos                    (28U)
#define CAN_F8R2_FB28_Msk                    (0x1UL << CAN_F8R2_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F8R2_FB28                        CAN_F8R2_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F8R2_FB29_Pos                    (29U)
#define CAN_F8R2_FB29_Msk                    (0x1UL << CAN_F8R2_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F8R2_FB29                        CAN_F8R2_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F8R2_FB30_Pos                    (30U)
#define CAN_F8R2_FB30_Msk                    (0x1UL << CAN_F8R2_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F8R2_FB30                        CAN_F8R2_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F8R2_FB31_Pos                    (31U)
#define CAN_F8R2_FB31_Msk                    (0x1UL << CAN_F8R2_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F8R2_FB31                        CAN_F8R2_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F9R2 register  *******************/
#define CAN_F9R2_FB0_Pos                     (0U)
#define CAN_F9R2_FB0_Msk                     (0x1UL << CAN_F9R2_FB0_Pos)        /*!< 0x00000001 */
#define CAN_F9R2_FB0                         CAN_F9R2_FB0_Msk                  /*!< Filter bit 0 */
#define CAN_F9R2_FB1_Pos                     (1U)
#define CAN_F9R2_FB1_Msk                     (0x1UL << CAN_F9R2_FB1_Pos)        /*!< 0x00000002 */
#define CAN_F9R2_FB1                         CAN_F9R2_FB1_Msk                  /*!< Filter bit 1 */
#define CAN_F9R2_FB2_Pos                     (2U)
#define CAN_F9R2_FB2_Msk                     (0x1UL << CAN_F9R2_FB2_Pos)        /*!< 0x00000004 */
#define CAN_F9R2_FB2                         CAN_F9R2_FB2_Msk                  /*!< Filter bit 2 */
#define CAN_F9R2_FB3_Pos                     (3U)
#define CAN_F9R2_FB3_Msk                     (0x1UL << CAN_F9R2_FB3_Pos)        /*!< 0x00000008 */
#define CAN_F9R2_FB3                         CAN_F9R2_FB3_Msk                  /*!< Filter bit 3 */
#define CAN_F9R2_FB4_Pos                     (4U)
#define CAN_F9R2_FB4_Msk                     (0x1UL << CAN_F9R2_FB4_Pos)        /*!< 0x00000010 */
#define CAN_F9R2_FB4                         CAN_F9R2_FB4_Msk                  /*!< Filter bit 4 */
#define CAN_F9R2_FB5_Pos                     (5U)
#define CAN_F9R2_FB5_Msk                     (0x1UL << CAN_F9R2_FB5_Pos)        /*!< 0x00000020 */
#define CAN_F9R2_FB5                         CAN_F9R2_FB5_Msk                  /*!< Filter bit 5 */
#define CAN_F9R2_FB6_Pos                     (6U)
#define CAN_F9R2_FB6_Msk                     (0x1UL << CAN_F9R2_FB6_Pos)        /*!< 0x00000040 */
#define CAN_F9R2_FB6                         CAN_F9R2_FB6_Msk                  /*!< Filter bit 6 */
#define CAN_F9R2_FB7_Pos                     (7U)
#define CAN_F9R2_FB7_Msk                     (0x1UL << CAN_F9R2_FB7_Pos)        /*!< 0x00000080 */
#define CAN_F9R2_FB7                         CAN_F9R2_FB7_Msk                  /*!< Filter bit 7 */
#define CAN_F9R2_FB8_Pos                     (8U)
#define CAN_F9R2_FB8_Msk                     (0x1UL << CAN_F9R2_FB8_Pos)        /*!< 0x00000100 */
#define CAN_F9R2_FB8                         CAN_F9R2_FB8_Msk                  /*!< Filter bit 8 */
#define CAN_F9R2_FB9_Pos                     (9U)
#define CAN_F9R2_FB9_Msk                     (0x1UL << CAN_F9R2_FB9_Pos)        /*!< 0x00000200 */
#define CAN_F9R2_FB9                         CAN_F9R2_FB9_Msk                  /*!< Filter bit 9 */
#define CAN_F9R2_FB10_Pos                    (10U)
#define CAN_F9R2_FB10_Msk                    (0x1UL << CAN_F9R2_FB10_Pos)       /*!< 0x00000400 */
#define CAN_F9R2_FB10                        CAN_F9R2_FB10_Msk                 /*!< Filter bit 10 */
#define CAN_F9R2_FB11_Pos                    (11U)
#define CAN_F9R2_FB11_Msk                    (0x1UL << CAN_F9R2_FB11_Pos)       /*!< 0x00000800 */
#define CAN_F9R2_FB11                        CAN_F9R2_FB11_Msk                 /*!< Filter bit 11 */
#define CAN_F9R2_FB12_Pos                    (12U)
#define CAN_F9R2_FB12_Msk                    (0x1UL << CAN_F9R2_FB12_Pos)       /*!< 0x00001000 */
#define CAN_F9R2_FB12                        CAN_F9R2_FB12_Msk                 /*!< Filter bit 12 */
#define CAN_F9R2_FB13_Pos                    (13U)
#define CAN_F9R2_FB13_Msk                    (0x1UL << CAN_F9R2_FB13_Pos)       /*!< 0x00002000 */
#define CAN_F9R2_FB13                        CAN_F9R2_FB13_Msk                 /*!< Filter bit 13 */
#define CAN_F9R2_FB14_Pos                    (14U)
#define CAN_F9R2_FB14_Msk                    (0x1UL << CAN_F9R2_FB14_Pos)       /*!< 0x00004000 */
#define CAN_F9R2_FB14                        CAN_F9R2_FB14_Msk                 /*!< Filter bit 14 */
#define CAN_F9R2_FB15_Pos                    (15U)
#define CAN_F9R2_FB15_Msk                    (0x1UL << CAN_F9R2_FB15_Pos)       /*!< 0x00008000 */
#define CAN_F9R2_FB15                        CAN_F9R2_FB15_Msk                 /*!< Filter bit 15 */
#define CAN_F9R2_FB16_Pos                    (16U)
#define CAN_F9R2_FB16_Msk                    (0x1UL << CAN_F9R2_FB16_Pos)       /*!< 0x00010000 */
#define CAN_F9R2_FB16                        CAN_F9R2_FB16_Msk                 /*!< Filter bit 16 */
#define CAN_F9R2_FB17_Pos                    (17U)
#define CAN_F9R2_FB17_Msk                    (0x1UL << CAN_F9R2_FB17_Pos)       /*!< 0x00020000 */
#define CAN_F9R2_FB17                        CAN_F9R2_FB17_Msk                 /*!< Filter bit 17 */
#define CAN_F9R2_FB18_Pos                    (18U)
#define CAN_F9R2_FB18_Msk                    (0x1UL << CAN_F9R2_FB18_Pos)       /*!< 0x00040000 */
#define CAN_F9R2_FB18                        CAN_F9R2_FB18_Msk                 /*!< Filter bit 18 */
#define CAN_F9R2_FB19_Pos                    (19U)
#define CAN_F9R2_FB19_Msk                    (0x1UL << CAN_F9R2_FB19_Pos)       /*!< 0x00080000 */
#define CAN_F9R2_FB19                        CAN_F9R2_FB19_Msk                 /*!< Filter bit 19 */
#define CAN_F9R2_FB20_Pos                    (20U)
#define CAN_F9R2_FB20_Msk                    (0x1UL << CAN_F9R2_FB20_Pos)       /*!< 0x00100000 */
#define CAN_F9R2_FB20                        CAN_F9R2_FB20_Msk                 /*!< Filter bit 20 */
#define CAN_F9R2_FB21_Pos                    (21U)
#define CAN_F9R2_FB21_Msk                    (0x1UL << CAN_F9R2_FB21_Pos)       /*!< 0x00200000 */
#define CAN_F9R2_FB21                        CAN_F9R2_FB21_Msk                 /*!< Filter bit 21 */
#define CAN_F9R2_FB22_Pos                    (22U)
#define CAN_F9R2_FB22_Msk                    (0x1UL << CAN_F9R2_FB22_Pos)       /*!< 0x00400000 */
#define CAN_F9R2_FB22                        CAN_F9R2_FB22_Msk                 /*!< Filter bit 22 */
#define CAN_F9R2_FB23_Pos                    (23U)
#define CAN_F9R2_FB23_Msk                    (0x1UL << CAN_F9R2_FB23_Pos)       /*!< 0x00800000 */
#define CAN_F9R2_FB23                        CAN_F9R2_FB23_Msk                 /*!< Filter bit 23 */
#define CAN_F9R2_FB24_Pos                    (24U)
#define CAN_F9R2_FB24_Msk                    (0x1UL << CAN_F9R2_FB24_Pos)       /*!< 0x01000000 */
#define CAN_F9R2_FB24                        CAN_F9R2_FB24_Msk                 /*!< Filter bit 24 */
#define CAN_F9R2_FB25_Pos                    (25U)
#define CAN_F9R2_FB25_Msk                    (0x1UL << CAN_F9R2_FB25_Pos)       /*!< 0x02000000 */
#define CAN_F9R2_FB25                        CAN_F9R2_FB25_Msk                 /*!< Filter bit 25 */
#define CAN_F9R2_FB26_Pos                    (26U)
#define CAN_F9R2_FB26_Msk                    (0x1UL << CAN_F9R2_FB26_Pos)       /*!< 0x04000000 */
#define CAN_F9R2_FB26                        CAN_F9R2_FB26_Msk                 /*!< Filter bit 26 */
#define CAN_F9R2_FB27_Pos                    (27U)
#define CAN_F9R2_FB27_Msk                    (0x1UL << CAN_F9R2_FB27_Pos)       /*!< 0x08000000 */
#define CAN_F9R2_FB27                        CAN_F9R2_FB27_Msk                 /*!< Filter bit 27 */
#define CAN_F9R2_FB28_Pos                    (28U)
#define CAN_F9R2_FB28_Msk                    (0x1UL << CAN_F9R2_FB28_Pos)       /*!< 0x10000000 */
#define CAN_F9R2_FB28                        CAN_F9R2_FB28_Msk                 /*!< Filter bit 28 */
#define CAN_F9R2_FB29_Pos                    (29U)
#define CAN_F9R2_FB29_Msk                    (0x1UL << CAN_F9R2_FB29_Pos)       /*!< 0x20000000 */
#define CAN_F9R2_FB29                        CAN_F9R2_FB29_Msk                 /*!< Filter bit 29 */
#define CAN_F9R2_FB30_Pos                    (30U)
#define CAN_F9R2_FB30_Msk                    (0x1UL << CAN_F9R2_FB30_Pos)       /*!< 0x40000000 */
#define CAN_F9R2_FB30                        CAN_F9R2_FB30_Msk                 /*!< Filter bit 30 */
#define CAN_F9R2_FB31_Pos                    (31U)
#define CAN_F9R2_FB31_Msk                    (0x1UL << CAN_F9R2_FB31_Pos)       /*!< 0x80000000 */
#define CAN_F9R2_FB31                        CAN_F9R2_FB31_Msk                 /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F10R2 register  ******************/
#define CAN_F10R2_FB0_Pos                    (0U)
#define CAN_F10R2_FB0_Msk                    (0x1UL << CAN_F10R2_FB0_Pos)       /*!< 0x00000001 */
#define CAN_F10R2_FB0                        CAN_F10R2_FB0_Msk                 /*!< Filter bit 0 */
#define CAN_F10R2_FB1_Pos                    (1U)
#define CAN_F10R2_FB1_Msk                    (0x1UL << CAN_F10R2_FB1_Pos)       /*!< 0x00000002 */
#define CAN_F10R2_FB1                        CAN_F10R2_FB1_Msk                 /*!< Filter bit 1 */
#define CAN_F10R2_FB2_Pos                    (2U)
#define CAN_F10R2_FB2_Msk                    (0x1UL << CAN_F10R2_FB2_Pos)       /*!< 0x00000004 */
#define CAN_F10R2_FB2                        CAN_F10R2_FB2_Msk                 /*!< Filter bit 2 */
#define CAN_F10R2_FB3_Pos                    (3U)
#define CAN_F10R2_FB3_Msk                    (0x1UL << CAN_F10R2_FB3_Pos)       /*!< 0x00000008 */
#define CAN_F10R2_FB3                        CAN_F10R2_FB3_Msk                 /*!< Filter bit 3 */
#define CAN_F10R2_FB4_Pos                    (4U)
#define CAN_F10R2_FB4_Msk                    (0x1UL << CAN_F10R2_FB4_Pos)       /*!< 0x00000010 */
#define CAN_F10R2_FB4                        CAN_F10R2_FB4_Msk                 /*!< Filter bit 4 */
#define CAN_F10R2_FB5_Pos                    (5U)
#define CAN_F10R2_FB5_Msk                    (0x1UL << CAN_F10R2_FB5_Pos)       /*!< 0x00000020 */
#define CAN_F10R2_FB5                        CAN_F10R2_FB5_Msk                 /*!< Filter bit 5 */
#define CAN_F10R2_FB6_Pos                    (6U)
#define CAN_F10R2_FB6_Msk                    (0x1UL << CAN_F10R2_FB6_Pos)       /*!< 0x00000040 */
#define CAN_F10R2_FB6                        CAN_F10R2_FB6_Msk                 /*!< Filter bit 6 */
#define CAN_F10R2_FB7_Pos                    (7U)
#define CAN_F10R2_FB7_Msk                    (0x1UL << CAN_F10R2_FB7_Pos)       /*!< 0x00000080 */
#define CAN_F10R2_FB7                        CAN_F10R2_FB7_Msk                 /*!< Filter bit 7 */
#define CAN_F10R2_FB8_Pos                    (8U)
#define CAN_F10R2_FB8_Msk                    (0x1UL << CAN_F10R2_FB8_Pos)       /*!< 0x00000100 */
#define CAN_F10R2_FB8                        CAN_F10R2_FB8_Msk                 /*!< Filter bit 8 */
#define CAN_F10R2_FB9_Pos                    (9U)
#define CAN_F10R2_FB9_Msk                    (0x1UL << CAN_F10R2_FB9_Pos)       /*!< 0x00000200 */
#define CAN_F10R2_FB9                        CAN_F10R2_FB9_Msk                 /*!< Filter bit 9 */
#define CAN_F10R2_FB10_Pos                   (10U)
#define CAN_F10R2_FB10_Msk                   (0x1UL << CAN_F10R2_FB10_Pos)      /*!< 0x00000400 */
#define CAN_F10R2_FB10                       CAN_F10R2_FB10_Msk                /*!< Filter bit 10 */
#define CAN_F10R2_FB11_Pos                   (11U)
#define CAN_F10R2_FB11_Msk                   (0x1UL << CAN_F10R2_FB11_Pos)      /*!< 0x00000800 */
#define CAN_F10R2_FB11                       CAN_F10R2_FB11_Msk                /*!< Filter bit 11 */
#define CAN_F10R2_FB12_Pos                   (12U)
#define CAN_F10R2_FB12_Msk                   (0x1UL << CAN_F10R2_FB12_Pos)      /*!< 0x00001000 */
#define CAN_F10R2_FB12                       CAN_F10R2_FB12_Msk                /*!< Filter bit 12 */
#define CAN_F10R2_FB13_Pos                   (13U)
#define CAN_F10R2_FB13_Msk                   (0x1UL << CAN_F10R2_FB13_Pos)      /*!< 0x00002000 */
#define CAN_F10R2_FB13                       CAN_F10R2_FB13_Msk                /*!< Filter bit 13 */
#define CAN_F10R2_FB14_Pos                   (14U)
#define CAN_F10R2_FB14_Msk                   (0x1UL << CAN_F10R2_FB14_Pos)      /*!< 0x00004000 */
#define CAN_F10R2_FB14                       CAN_F10R2_FB14_Msk                /*!< Filter bit 14 */
#define CAN_F10R2_FB15_Pos                   (15U)
#define CAN_F10R2_FB15_Msk                   (0x1UL << CAN_F10R2_FB15_Pos)      /*!< 0x00008000 */
#define CAN_F10R2_FB15                       CAN_F10R2_FB15_Msk                /*!< Filter bit 15 */
#define CAN_F10R2_FB16_Pos                   (16U)
#define CAN_F10R2_FB16_Msk                   (0x1UL << CAN_F10R2_FB16_Pos)      /*!< 0x00010000 */
#define CAN_F10R2_FB16                       CAN_F10R2_FB16_Msk                /*!< Filter bit 16 */
#define CAN_F10R2_FB17_Pos                   (17U)
#define CAN_F10R2_FB17_Msk                   (0x1UL << CAN_F10R2_FB17_Pos)      /*!< 0x00020000 */
#define CAN_F10R2_FB17                       CAN_F10R2_FB17_Msk                /*!< Filter bit 17 */
#define CAN_F10R2_FB18_Pos                   (18U)
#define CAN_F10R2_FB18_Msk                   (0x1UL << CAN_F10R2_FB18_Pos)      /*!< 0x00040000 */
#define CAN_F10R2_FB18                       CAN_F10R2_FB18_Msk                /*!< Filter bit 18 */
#define CAN_F10R2_FB19_Pos                   (19U)
#define CAN_F10R2_FB19_Msk                   (0x1UL << CAN_F10R2_FB19_Pos)      /*!< 0x00080000 */
#define CAN_F10R2_FB19                       CAN_F10R2_FB19_Msk                /*!< Filter bit 19 */
#define CAN_F10R2_FB20_Pos                   (20U)
#define CAN_F10R2_FB20_Msk                   (0x1UL << CAN_F10R2_FB20_Pos)      /*!< 0x00100000 */
#define CAN_F10R2_FB20                       CAN_F10R2_FB20_Msk                /*!< Filter bit 20 */
#define CAN_F10R2_FB21_Pos                   (21U)
#define CAN_F10R2_FB21_Msk                   (0x1UL << CAN_F10R2_FB21_Pos)      /*!< 0x00200000 */
#define CAN_F10R2_FB21                       CAN_F10R2_FB21_Msk                /*!< Filter bit 21 */
#define CAN_F10R2_FB22_Pos                   (22U)
#define CAN_F10R2_FB22_Msk                   (0x1UL << CAN_F10R2_FB22_Pos)      /*!< 0x00400000 */
#define CAN_F10R2_FB22                       CAN_F10R2_FB22_Msk                /*!< Filter bit 22 */
#define CAN_F10R2_FB23_Pos                   (23U)
#define CAN_F10R2_FB23_Msk                   (0x1UL << CAN_F10R2_FB23_Pos)      /*!< 0x00800000 */
#define CAN_F10R2_FB23                       CAN_F10R2_FB23_Msk                /*!< Filter bit 23 */
#define CAN_F10R2_FB24_Pos                   (24U)
#define CAN_F10R2_FB24_Msk                   (0x1UL << CAN_F10R2_FB24_Pos)      /*!< 0x01000000 */
#define CAN_F10R2_FB24                       CAN_F10R2_FB24_Msk                /*!< Filter bit 24 */
#define CAN_F10R2_FB25_Pos                   (25U)
#define CAN_F10R2_FB25_Msk                   (0x1UL << CAN_F10R2_FB25_Pos)      /*!< 0x02000000 */
#define CAN_F10R2_FB25                       CAN_F10R2_FB25_Msk                /*!< Filter bit 25 */
#define CAN_F10R2_FB26_Pos                   (26U)
#define CAN_F10R2_FB26_Msk                   (0x1UL << CAN_F10R2_FB26_Pos)      /*!< 0x04000000 */
#define CAN_F10R2_FB26                       CAN_F10R2_FB26_Msk                /*!< Filter bit 26 */
#define CAN_F10R2_FB27_Pos                   (27U)
#define CAN_F10R2_FB27_Msk                   (0x1UL << CAN_F10R2_FB27_Pos)      /*!< 0x08000000 */
#define CAN_F10R2_FB27                       CAN_F10R2_FB27_Msk                /*!< Filter bit 27 */
#define CAN_F10R2_FB28_Pos                   (28U)
#define CAN_F10R2_FB28_Msk                   (0x1UL << CAN_F10R2_FB28_Pos)      /*!< 0x10000000 */
#define CAN_F10R2_FB28                       CAN_F10R2_FB28_Msk                /*!< Filter bit 28 */
#define CAN_F10R2_FB29_Pos                   (29U)
#define CAN_F10R2_FB29_Msk                   (0x1UL << CAN_F10R2_FB29_Pos)      /*!< 0x20000000 */
#define CAN_F10R2_FB29                       CAN_F10R2_FB29_Msk                /*!< Filter bit 29 */
#define CAN_F10R2_FB30_Pos                   (30U)
#define CAN_F10R2_FB30_Msk                   (0x1UL << CAN_F10R2_FB30_Pos)      /*!< 0x40000000 */
#define CAN_F10R2_FB30                       CAN_F10R2_FB30_Msk                /*!< Filter bit 30 */
#define CAN_F10R2_FB31_Pos                   (31U)
#define CAN_F10R2_FB31_Msk                   (0x1UL << CAN_F10R2_FB31_Pos)      /*!< 0x80000000 */
#define CAN_F10R2_FB31                       CAN_F10R2_FB31_Msk                /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F11R2 register  ******************/
#define CAN_F11R2_FB0_Pos                    (0U)
#define CAN_F11R2_FB0_Msk                    (0x1UL << CAN_F11R2_FB0_Pos)       /*!< 0x00000001 */
#define CAN_F11R2_FB0                        CAN_F11R2_FB0_Msk                 /*!< Filter bit 0 */
#define CAN_F11R2_FB1_Pos                    (1U)
#define CAN_F11R2_FB1_Msk                    (0x1UL << CAN_F11R2_FB1_Pos)       /*!< 0x00000002 */
#define CAN_F11R2_FB1                        CAN_F11R2_FB1_Msk                 /*!< Filter bit 1 */
#define CAN_F11R2_FB2_Pos                    (2U)
#define CAN_F11R2_FB2_Msk                    (0x1UL << CAN_F11R2_FB2_Pos)       /*!< 0x00000004 */
#define CAN_F11R2_FB2                        CAN_F11R2_FB2_Msk                 /*!< Filter bit 2 */
#define CAN_F11R2_FB3_Pos                    (3U)
#define CAN_F11R2_FB3_Msk                    (0x1UL << CAN_F11R2_FB3_Pos)       /*!< 0x00000008 */
#define CAN_F11R2_FB3                        CAN_F11R2_FB3_Msk                 /*!< Filter bit 3 */
#define CAN_F11R2_FB4_Pos                    (4U)
#define CAN_F11R2_FB4_Msk                    (0x1UL << CAN_F11R2_FB4_Pos)       /*!< 0x00000010 */
#define CAN_F11R2_FB4                        CAN_F11R2_FB4_Msk                 /*!< Filter bit 4 */
#define CAN_F11R2_FB5_Pos                    (5U)
#define CAN_F11R2_FB5_Msk                    (0x1UL << CAN_F11R2_FB5_Pos)       /*!< 0x00000020 */
#define CAN_F11R2_FB5                        CAN_F11R2_FB5_Msk                 /*!< Filter bit 5 */
#define CAN_F11R2_FB6_Pos                    (6U)
#define CAN_F11R2_FB6_Msk                    (0x1UL << CAN_F11R2_FB6_Pos)       /*!< 0x00000040 */
#define CAN_F11R2_FB6                        CAN_F11R2_FB6_Msk                 /*!< Filter bit 6 */
#define CAN_F11R2_FB7_Pos                    (7U)
#define CAN_F11R2_FB7_Msk                    (0x1UL << CAN_F11R2_FB7_Pos)       /*!< 0x00000080 */
#define CAN_F11R2_FB7                        CAN_F11R2_FB7_Msk                 /*!< Filter bit 7 */
#define CAN_F11R2_FB8_Pos                    (8U)
#define CAN_F11R2_FB8_Msk                    (0x1UL << CAN_F11R2_FB8_Pos)       /*!< 0x00000100 */
#define CAN_F11R2_FB8                        CAN_F11R2_FB8_Msk                 /*!< Filter bit 8 */
#define CAN_F11R2_FB9_Pos                    (9U)
#define CAN_F11R2_FB9_Msk                    (0x1UL << CAN_F11R2_FB9_Pos)       /*!< 0x00000200 */
#define CAN_F11R2_FB9                        CAN_F11R2_FB9_Msk                 /*!< Filter bit 9 */
#define CAN_F11R2_FB10_Pos                   (10U)
#define CAN_F11R2_FB10_Msk                   (0x1UL << CAN_F11R2_FB10_Pos)      /*!< 0x00000400 */
#define CAN_F11R2_FB10                       CAN_F11R2_FB10_Msk                /*!< Filter bit 10 */
#define CAN_F11R2_FB11_Pos                   (11U)
#define CAN_F11R2_FB11_Msk                   (0x1UL << CAN_F11R2_FB11_Pos)      /*!< 0x00000800 */
#define CAN_F11R2_FB11                       CAN_F11R2_FB11_Msk                /*!< Filter bit 11 */
#define CAN_F11R2_FB12_Pos                   (12U)
#define CAN_F11R2_FB12_Msk                   (0x1UL << CAN_F11R2_FB12_Pos)      /*!< 0x00001000 */
#define CAN_F11R2_FB12                       CAN_F11R2_FB12_Msk                /*!< Filter bit 12 */
#define CAN_F11R2_FB13_Pos                   (13U)
#define CAN_F11R2_FB13_Msk                   (0x1UL << CAN_F11R2_FB13_Pos)      /*!< 0x00002000 */
#define CAN_F11R2_FB13                       CAN_F11R2_FB13_Msk                /*!< Filter bit 13 */
#define CAN_F11R2_FB14_Pos                   (14U)
#define CAN_F11R2_FB14_Msk                   (0x1UL << CAN_F11R2_FB14_Pos)      /*!< 0x00004000 */
#define CAN_F11R2_FB14                       CAN_F11R2_FB14_Msk                /*!< Filter bit 14 */
#define CAN_F11R2_FB15_Pos                   (15U)
#define CAN_F11R2_FB15_Msk                   (0x1UL << CAN_F11R2_FB15_Pos)      /*!< 0x00008000 */
#define CAN_F11R2_FB15                       CAN_F11R2_FB15_Msk                /*!< Filter bit 15 */
#define CAN_F11R2_FB16_Pos                   (16U)
#define CAN_F11R2_FB16_Msk                   (0x1UL << CAN_F11R2_FB16_Pos)      /*!< 0x00010000 */
#define CAN_F11R2_FB16                       CAN_F11R2_FB16_Msk                /*!< Filter bit 16 */
#define CAN_F11R2_FB17_Pos                   (17U)
#define CAN_F11R2_FB17_Msk                   (0x1UL << CAN_F11R2_FB17_Pos)      /*!< 0x00020000 */
#define CAN_F11R2_FB17                       CAN_F11R2_FB17_Msk                /*!< Filter bit 17 */
#define CAN_F11R2_FB18_Pos                   (18U)
#define CAN_F11R2_FB18_Msk                   (0x1UL << CAN_F11R2_FB18_Pos)      /*!< 0x00040000 */
#define CAN_F11R2_FB18                       CAN_F11R2_FB18_Msk                /*!< Filter bit 18 */
#define CAN_F11R2_FB19_Pos                   (19U)
#define CAN_F11R2_FB19_Msk                   (0x1UL << CAN_F11R2_FB19_Pos)      /*!< 0x00080000 */
#define CAN_F11R2_FB19                       CAN_F11R2_FB19_Msk                /*!< Filter bit 19 */
#define CAN_F11R2_FB20_Pos                   (20U)
#define CAN_F11R2_FB20_Msk                   (0x1UL << CAN_F11R2_FB20_Pos)      /*!< 0x00100000 */
#define CAN_F11R2_FB20                       CAN_F11R2_FB20_Msk                /*!< Filter bit 20 */
#define CAN_F11R2_FB21_Pos                   (21U)
#define CAN_F11R2_FB21_Msk                   (0x1UL << CAN_F11R2_FB21_Pos)      /*!< 0x00200000 */
#define CAN_F11R2_FB21                       CAN_F11R2_FB21_Msk                /*!< Filter bit 21 */
#define CAN_F11R2_FB22_Pos                   (22U)
#define CAN_F11R2_FB22_Msk                   (0x1UL << CAN_F11R2_FB22_Pos)      /*!< 0x00400000 */
#define CAN_F11R2_FB22                       CAN_F11R2_FB22_Msk                /*!< Filter bit 22 */
#define CAN_F11R2_FB23_Pos                   (23U)
#define CAN_F11R2_FB23_Msk                   (0x1UL << CAN_F11R2_FB23_Pos)      /*!< 0x00800000 */
#define CAN_F11R2_FB23                       CAN_F11R2_FB23_Msk                /*!< Filter bit 23 */
#define CAN_F11R2_FB24_Pos                   (24U)
#define CAN_F11R2_FB24_Msk                   (0x1UL << CAN_F11R2_FB24_Pos)      /*!< 0x01000000 */
#define CAN_F11R2_FB24                       CAN_F11R2_FB24_Msk                /*!< Filter bit 24 */
#define CAN_F11R2_FB25_Pos                   (25U)
#define CAN_F11R2_FB25_Msk                   (0x1UL << CAN_F11R2_FB25_Pos)      /*!< 0x02000000 */
#define CAN_F11R2_FB25                       CAN_F11R2_FB25_Msk                /*!< Filter bit 25 */
#define CAN_F11R2_FB26_Pos                   (26U)
#define CAN_F11R2_FB26_Msk                   (0x1UL << CAN_F11R2_FB26_Pos)      /*!< 0x04000000 */
#define CAN_F11R2_FB26                       CAN_F11R2_FB26_Msk                /*!< Filter bit 26 */
#define CAN_F11R2_FB27_Pos                   (27U)
#define CAN_F11R2_FB27_Msk                   (0x1UL << CAN_F11R2_FB27_Pos)      /*!< 0x08000000 */
#define CAN_F11R2_FB27                       CAN_F11R2_FB27_Msk                /*!< Filter bit 27 */
#define CAN_F11R2_FB28_Pos                   (28U)
#define CAN_F11R2_FB28_Msk                   (0x1UL << CAN_F11R2_FB28_Pos)      /*!< 0x10000000 */
#define CAN_F11R2_FB28                       CAN_F11R2_FB28_Msk                /*!< Filter bit 28 */
#define CAN_F11R2_FB29_Pos                   (29U)
#define CAN_F11R2_FB29_Msk                   (0x1UL << CAN_F11R2_FB29_Pos)      /*!< 0x20000000 */
#define CAN_F11R2_FB29                       CAN_F11R2_FB29_Msk                /*!< Filter bit 29 */
#define CAN_F11R2_FB30_Pos                   (30U)
#define CAN_F11R2_FB30_Msk                   (0x1UL << CAN_F11R2_FB30_Pos)      /*!< 0x40000000 */
#define CAN_F11R2_FB30                       CAN_F11R2_FB30_Msk                /*!< Filter bit 30 */
#define CAN_F11R2_FB31_Pos                   (31U)
#define CAN_F11R2_FB31_Msk                   (0x1UL << CAN_F11R2_FB31_Pos)      /*!< 0x80000000 */
#define CAN_F11R2_FB31                       CAN_F11R2_FB31_Msk                /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F12R2 register  ******************/
#define CAN_F12R2_FB0_Pos                    (0U)
#define CAN_F12R2_FB0_Msk                    (0x1UL << CAN_F12R2_FB0_Pos)       /*!< 0x00000001 */
#define CAN_F12R2_FB0                        CAN_F12R2_FB0_Msk                 /*!< Filter bit 0 */
#define CAN_F12R2_FB1_Pos                    (1U)
#define CAN_F12R2_FB1_Msk                    (0x1UL << CAN_F12R2_FB1_Pos)       /*!< 0x00000002 */
#define CAN_F12R2_FB1                        CAN_F12R2_FB1_Msk                 /*!< Filter bit 1 */
#define CAN_F12R2_FB2_Pos                    (2U)
#define CAN_F12R2_FB2_Msk                    (0x1UL << CAN_F12R2_FB2_Pos)       /*!< 0x00000004 */
#define CAN_F12R2_FB2                        CAN_F12R2_FB2_Msk                 /*!< Filter bit 2 */
#define CAN_F12R2_FB3_Pos                    (3U)
#define CAN_F12R2_FB3_Msk                    (0x1UL << CAN_F12R2_FB3_Pos)       /*!< 0x00000008 */
#define CAN_F12R2_FB3                        CAN_F12R2_FB3_Msk                 /*!< Filter bit 3 */
#define CAN_F12R2_FB4_Pos                    (4U)
#define CAN_F12R2_FB4_Msk                    (0x1UL << CAN_F12R2_FB4_Pos)       /*!< 0x00000010 */
#define CAN_F12R2_FB4                        CAN_F12R2_FB4_Msk                 /*!< Filter bit 4 */
#define CAN_F12R2_FB5_Pos                    (5U)
#define CAN_F12R2_FB5_Msk                    (0x1UL << CAN_F12R2_FB5_Pos)       /*!< 0x00000020 */
#define CAN_F12R2_FB5                        CAN_F12R2_FB5_Msk                 /*!< Filter bit 5 */
#define CAN_F12R2_FB6_Pos                    (6U)
#define CAN_F12R2_FB6_Msk                    (0x1UL << CAN_F12R2_FB6_Pos)       /*!< 0x00000040 */
#define CAN_F12R2_FB6                        CAN_F12R2_FB6_Msk                 /*!< Filter bit 6 */
#define CAN_F12R2_FB7_Pos                    (7U)
#define CAN_F12R2_FB7_Msk                    (0x1UL << CAN_F12R2_FB7_Pos)       /*!< 0x00000080 */
#define CAN_F12R2_FB7                        CAN_F12R2_FB7_Msk                 /*!< Filter bit 7 */
#define CAN_F12R2_FB8_Pos                    (8U)
#define CAN_F12R2_FB8_Msk                    (0x1UL << CAN_F12R2_FB8_Pos)       /*!< 0x00000100 */
#define CAN_F12R2_FB8                        CAN_F12R2_FB8_Msk                 /*!< Filter bit 8 */
#define CAN_F12R2_FB9_Pos                    (9U)
#define CAN_F12R2_FB9_Msk                    (0x1UL << CAN_F12R2_FB9_Pos)       /*!< 0x00000200 */
#define CAN_F12R2_FB9                        CAN_F12R2_FB9_Msk                 /*!< Filter bit 9 */
#define CAN_F12R2_FB10_Pos                   (10U)
#define CAN_F12R2_FB10_Msk                   (0x1UL << CAN_F12R2_FB10_Pos)      /*!< 0x00000400 */
#define CAN_F12R2_FB10                       CAN_F12R2_FB10_Msk                /*!< Filter bit 10 */
#define CAN_F12R2_FB11_Pos                   (11U)
#define CAN_F12R2_FB11_Msk                   (0x1UL << CAN_F12R2_FB11_Pos)      /*!< 0x00000800 */
#define CAN_F12R2_FB11                       CAN_F12R2_FB11_Msk                /*!< Filter bit 11 */
#define CAN_F12R2_FB12_Pos                   (12U)
#define CAN_F12R2_FB12_Msk                   (0x1UL << CAN_F12R2_FB12_Pos)      /*!< 0x00001000 */
#define CAN_F12R2_FB12                       CAN_F12R2_FB12_Msk                /*!< Filter bit 12 */
#define CAN_F12R2_FB13_Pos                   (13U)
#define CAN_F12R2_FB13_Msk                   (0x1UL << CAN_F12R2_FB13_Pos)      /*!< 0x00002000 */
#define CAN_F12R2_FB13                       CAN_F12R2_FB13_Msk                /*!< Filter bit 13 */
#define CAN_F12R2_FB14_Pos                   (14U)
#define CAN_F12R2_FB14_Msk                   (0x1UL << CAN_F12R2_FB14_Pos)      /*!< 0x00004000 */
#define CAN_F12R2_FB14                       CAN_F12R2_FB14_Msk                /*!< Filter bit 14 */
#define CAN_F12R2_FB15_Pos                   (15U)
#define CAN_F12R2_FB15_Msk                   (0x1UL << CAN_F12R2_FB15_Pos)      /*!< 0x00008000 */
#define CAN_F12R2_FB15                       CAN_F12R2_FB15_Msk                /*!< Filter bit 15 */
#define CAN_F12R2_FB16_Pos                   (16U)
#define CAN_F12R2_FB16_Msk                   (0x1UL << CAN_F12R2_FB16_Pos)      /*!< 0x00010000 */
#define CAN_F12R2_FB16                       CAN_F12R2_FB16_Msk                /*!< Filter bit 16 */
#define CAN_F12R2_FB17_Pos                   (17U)
#define CAN_F12R2_FB17_Msk                   (0x1UL << CAN_F12R2_FB17_Pos)      /*!< 0x00020000 */
#define CAN_F12R2_FB17                       CAN_F12R2_FB17_Msk                /*!< Filter bit 17 */
#define CAN_F12R2_FB18_Pos                   (18U)
#define CAN_F12R2_FB18_Msk                   (0x1UL << CAN_F12R2_FB18_Pos)      /*!< 0x00040000 */
#define CAN_F12R2_FB18                       CAN_F12R2_FB18_Msk                /*!< Filter bit 18 */
#define CAN_F12R2_FB19_Pos                   (19U)
#define CAN_F12R2_FB19_Msk                   (0x1UL << CAN_F12R2_FB19_Pos)      /*!< 0x00080000 */
#define CAN_F12R2_FB19                       CAN_F12R2_FB19_Msk                /*!< Filter bit 19 */
#define CAN_F12R2_FB20_Pos                   (20U)
#define CAN_F12R2_FB20_Msk                   (0x1UL << CAN_F12R2_FB20_Pos)      /*!< 0x00100000 */
#define CAN_F12R2_FB20                       CAN_F12R2_FB20_Msk                /*!< Filter bit 20 */
#define CAN_F12R2_FB21_Pos                   (21U)
#define CAN_F12R2_FB21_Msk                   (0x1UL << CAN_F12R2_FB21_Pos)      /*!< 0x00200000 */
#define CAN_F12R2_FB21                       CAN_F12R2_FB21_Msk                /*!< Filter bit 21 */
#define CAN_F12R2_FB22_Pos                   (22U)
#define CAN_F12R2_FB22_Msk                   (0x1UL << CAN_F12R2_FB22_Pos)      /*!< 0x00400000 */
#define CAN_F12R2_FB22                       CAN_F12R2_FB22_Msk                /*!< Filter bit 22 */
#define CAN_F12R2_FB23_Pos                   (23U)
#define CAN_F12R2_FB23_Msk                   (0x1UL << CAN_F12R2_FB23_Pos)      /*!< 0x00800000 */
#define CAN_F12R2_FB23                       CAN_F12R2_FB23_Msk                /*!< Filter bit 23 */
#define CAN_F12R2_FB24_Pos                   (24U)
#define CAN_F12R2_FB24_Msk                   (0x1UL << CAN_F12R2_FB24_Pos)      /*!< 0x01000000 */
#define CAN_F12R2_FB24                       CAN_F12R2_FB24_Msk                /*!< Filter bit 24 */
#define CAN_F12R2_FB25_Pos                   (25U)
#define CAN_F12R2_FB25_Msk                   (0x1UL << CAN_F12R2_FB25_Pos)      /*!< 0x02000000 */
#define CAN_F12R2_FB25                       CAN_F12R2_FB25_Msk                /*!< Filter bit 25 */
#define CAN_F12R2_FB26_Pos                   (26U)
#define CAN_F12R2_FB26_Msk                   (0x1UL << CAN_F12R2_FB26_Pos)      /*!< 0x04000000 */
#define CAN_F12R2_FB26                       CAN_F12R2_FB26_Msk                /*!< Filter bit 26 */
#define CAN_F12R2_FB27_Pos                   (27U)
#define CAN_F12R2_FB27_Msk                   (0x1UL << CAN_F12R2_FB27_Pos)      /*!< 0x08000000 */
#define CAN_F12R2_FB27                       CAN_F12R2_FB27_Msk                /*!< Filter bit 27 */
#define CAN_F12R2_FB28_Pos                   (28U)
#define CAN_F12R2_FB28_Msk                   (0x1UL << CAN_F12R2_FB28_Pos)      /*!< 0x10000000 */
#define CAN_F12R2_FB28                       CAN_F12R2_FB28_Msk                /*!< Filter bit 28 */
#define CAN_F12R2_FB29_Pos                   (29U)
#define CAN_F12R2_FB29_Msk                   (0x1UL << CAN_F12R2_FB29_Pos)      /*!< 0x20000000 */
#define CAN_F12R2_FB29                       CAN_F12R2_FB29_Msk                /*!< Filter bit 29 */
#define CAN_F12R2_FB30_Pos                   (30U)
#define CAN_F12R2_FB30_Msk                   (0x1UL << CAN_F12R2_FB30_Pos)      /*!< 0x40000000 */
#define CAN_F12R2_FB30                       CAN_F12R2_FB30_Msk                /*!< Filter bit 30 */
#define CAN_F12R2_FB31_Pos                   (31U)
#define CAN_F12R2_FB31_Msk                   (0x1UL << CAN_F12R2_FB31_Pos)      /*!< 0x80000000 */
#define CAN_F12R2_FB31                       CAN_F12R2_FB31_Msk                /*!< Filter bit 31 */

/*******************  Bit definition for CAN_F13R2 register  ******************/
#define CAN_F13R2_FB0_Pos                    (0U)
#define CAN_F13R2_FB0_Msk                    (0x1UL << CAN_F13R2_FB0_Pos)       /*!< 0x00000001 */
#define CAN_F13R2_FB0                        CAN_F13R2_FB0_Msk                 /*!< Filter bit 0 */
#define CAN_F13R2_FB1_Pos                    (1U)
#define CAN_F13R2_FB1_Msk                    (0x1UL << CAN_F13R2_FB1_Pos)       /*!< 0x00000002 */
#define CAN_F13R2_FB1                        CAN_F13R2_FB1_Msk                 /*!< Filter bit 1 */
#define CAN_F13R2_FB2_Pos                    (2U)
#define CAN_F13R2_FB2_Msk                    (0x1UL << CAN_F13R2_FB2_Pos)       /*!< 0x00000004 */
#define CAN_F13R2_FB2                        CAN_F13R2_FB2_Msk                 /*!< Filter bit 2 */
#define CAN_F13R2_FB3_Pos                    (3U)
#define CAN_F13R2_FB3_Msk                    (0x1UL << CAN_F13R2_FB3_Pos)       /*!< 0x00000008 */
#define CAN_F13R2_FB3                        CAN_F13R2_FB3_Msk                 /*!< Filter bit 3 */
#define CAN_F13R2_FB4_Pos                    (4U)
#define CAN_F13R2_FB4_Msk                    (0x1UL << CAN_F13R2_FB4_Pos)       /*!< 0x00000010 */
#define CAN_F13R2_FB4                        CAN_F13R2_FB4_Msk                 /*!< Filter bit 4 */
#define CAN_F13R2_FB5_Pos                    (5U)
#define CAN_F13R2_FB5_Msk                    (0x1UL << CAN_F13R2_FB5_Pos)       /*!< 0x00000020 */
#define CAN_F13R2_FB5                        CAN_F13R2_FB5_Msk                 /*!< Filter bit 5 */
#define CAN_F13R2_FB6_Pos                    (6U)
#define CAN_F13R2_FB6_Msk                    (0x1UL << CAN_F13R2_FB6_Pos)       /*!< 0x00000040 */
#define CAN_F13R2_FB6                        CAN_F13R2_FB6_Msk                 /*!< Filter bit 6 */
#define CAN_F13R2_FB7_Pos                    (7U)
#define CAN_F13R2_FB7_Msk                    (0x1UL << CAN_F13R2_FB7_Pos)       /*!< 0x00000080 */
#define CAN_F13R2_FB7                        CAN_F13R2_FB7_Msk                 /*!< Filter bit 7 */
#define CAN_F13R2_FB8_Pos                    (8U)
#define CAN_F13R2_FB8_Msk                    (0x1UL << CAN_F13R2_FB8_Pos)       /*!< 0x00000100 */
#define CAN_F13R2_FB8                        CAN_F13R2_FB8_Msk                 /*!< Filter bit 8 */
#define CAN_F13R2_FB9_Pos                    (9U)
#define CAN_F13R2_FB9_Msk                    (0x1UL << CAN_F13R2_FB9_Pos)       /*!< 0x00000200 */
#define CAN_F13R2_FB9                        CAN_F13R2_FB9_Msk                 /*!< Filter bit 9 */
#define CAN_F13R2_FB10_Pos                   (10U)
#define CAN_F13R2_FB10_Msk                   (0x1UL << CAN_F13R2_FB10_Pos)      /*!< 0x00000400 */
#define CAN_F13R2_FB10                       CAN_F13R2_FB10_Msk                /*!< Filter bit 10 */
#define CAN_F13R2_FB11_Pos                   (11U)
#define CAN_F13R2_FB11_Msk                   (0x1UL << CAN_F13R2_FB11_Pos)      /*!< 0x00000800 */
#define CAN_F13R2_FB11                       CAN_F13R2_FB11_Msk                /*!< Filter bit 11 */
#define CAN_F13R2_FB12_Pos                   (12U)
#define CAN_F13R2_FB12_Msk                   (0x1UL << CAN_F13R2_FB12_Pos)      /*!< 0x00001000 */
#define CAN_F13R2_FB12                       CAN_F13R2_FB12_Msk                /*!< Filter bit 12 */
#define CAN_F13R2_FB13_Pos                   (13U)
#define CAN_F13R2_FB13_Msk                   (0x1UL << CAN_F13R2_FB13_Pos)      /*!< 0x00002000 */
#define CAN_F13R2_FB13                       CAN_F13R2_FB13_Msk                /*!< Filter bit 13 */
#define CAN_F13R2_FB14_Pos                   (14U)
#define CAN_F13R2_FB14_Msk                   (0x1UL << CAN_F13R2_FB14_Pos)      /*!< 0x00004000 */
#define CAN_F13R2_FB14                       CAN_F13R2_FB14_Msk                /*!< Filter bit 14 */
#define CAN_F13R2_FB15_Pos                   (15U)
#define CAN_F13R2_FB15_Msk                   (0x1UL << CAN_F13R2_FB15_Pos)      /*!< 0x00008000 */
#define CAN_F13R2_FB15                       CAN_F13R2_FB15_Msk                /*!< Filter bit 15 */
#define CAN_F13R2_FB16_Pos                   (16U)
#define CAN_F13R2_FB16_Msk                   (0x1UL << CAN_F13R2_FB16_Pos)      /*!< 0x00010000 */
#define CAN_F13R2_FB16                       CAN_F13R2_FB16_Msk                /*!< Filter bit 16 */
#define CAN_F13R2_FB17_Pos                   (17U)
#define CAN_F13R2_FB17_Msk                   (0x1UL << CAN_F13R2_FB17_Pos)      /*!< 0x00020000 */
#define CAN_F13R2_FB17                       CAN_F13R2_FB17_Msk                /*!< Filter bit 17 */
#define CAN_F13R2_FB18_Pos                   (18U)
#define CAN_F13R2_FB18_Msk                   (0x1UL << CAN_F13R2_FB18_Pos)      /*!< 0x00040000 */
#define CAN_F13R2_FB18                       CAN_F13R2_FB18_Msk                /*!< Filter bit 18 */
#define CAN_F13R2_FB19_Pos                   (19U)
#define CAN_F13R2_FB19_Msk                   (0x1UL << CAN_F13R2_FB19_Pos)      /*!< 0x00080000 */
#define CAN_F13R2_FB19                       CAN_F13R2_FB19_Msk                /*!< Filter bit 19 */
#define CAN_F13R2_FB20_Pos                   (20U)
#define CAN_F13R2_FB20_Msk                   (0x1UL << CAN_F13R2_FB20_Pos)      /*!< 0x00100000 */
#define CAN_F13R2_FB20                       CAN_F13R2_FB20_Msk                /*!< Filter bit 20 */
#define CAN_F13R2_FB21_Pos                   (21U)
#define CAN_F13R2_FB21_Msk                   (0x1UL << CAN_F13R2_FB21_Pos)      /*!< 0x00200000 */
#define CAN_F13R2_FB21                       CAN_F13R2_FB21_Msk                /*!< Filter bit 21 */
#define CAN_F13R2_FB22_Pos                   (22U)
#define CAN_F13R2_FB22_Msk                   (0x1UL << CAN_F13R2_FB22_Pos)      /*!< 0x00400000 */
#define CAN_F13R2_FB22                       CAN_F13R2_FB22_Msk                /*!< Filter bit 22 */
#define CAN_F13R2_FB23_Pos                   (23U)
#define CAN_F13R2_FB23_Msk                   (0x1UL << CAN_F13R2_FB23_Pos)      /*!< 0x00800000 */
#define CAN_F13R2_FB23                       CAN_F13R2_FB23_Msk                /*!< Filter bit 23 */
#define CAN_F13R2_FB24_Pos                   (24U)
#define CAN_F13R2_FB24_Msk                   (0x1UL << CAN_F13R2_FB24_Pos)      /*!< 0x01000000 */
#define CAN_F13R2_FB24                       CAN_F13R2_FB24_Msk                /*!< Filter bit 24 */
#define CAN_F13R2_FB25_Pos                   (25U)
#define CAN_F13R2_FB25_Msk                   (0x1UL << CAN_F13R2_FB25_Pos)      /*!< 0x02000000 */
#define CAN_F13R2_FB25                       CAN_F13R2_FB25_Msk                /*!< Filter bit 25 */
#define CAN_F13R2_FB26_Pos                   (26U)
#define CAN_F13R2_FB26_Msk                   (0x1UL << CAN_F13R2_FB26_Pos)      /*!< 0x04000000 */
#define CAN_F13R2_FB26                       CAN_F13R2_FB26_Msk                /*!< Filter bit 26 */
#define CAN_F13R2_FB27_Pos                   (27U)
#define CAN_F13R2_FB27_Msk                   (0x1UL << CAN_F13R2_FB27_Pos)      /*!< 0x08000000 */
#define CAN_F13R2_FB27                       CAN_F13R2_FB27_Msk                /*!< Filter bit 27 */
#define CAN_F13R2_FB28_Pos                   (28U)
#define CAN_F13R2_FB28_Msk                   (0x1UL << CAN_F13R2_FB28_Pos)      /*!< 0x10000000 */
#define CAN_F13R2_FB28                       CAN_F13R2_FB28_Msk                /*!< Filter bit 28 */
#define CAN_F13R2_FB29_Pos                   (29U)
#define CAN_F13R2_FB29_Msk                   (0x1UL << CAN_F13R2_FB29_Pos)      /*!< 0x20000000 */
#define CAN_F13R2_FB29                       CAN_F13R2_FB29_Msk                /*!< Filter bit 29 */
#define CAN_F13R2_FB30_Pos                   (30U)
#define CAN_F13R2_FB30_Msk                   (0x1UL << CAN_F13R2_FB30_Pos)      /*!< 0x40000000 */
#define CAN_F13R2_FB30                       CAN_F13R2_FB30_Msk                /*!< Filter bit 30 */
#define CAN_F13R2_FB31_Pos                   (31U)
#define CAN_F13R2_FB31_Msk                   (0x1UL << CAN_F13R2_FB31_Pos)      /*!< 0x80000000 */
#define CAN_F13R2_FB31                       CAN_F13R2_FB31_Msk                /*!< Filter bit 31 */

/******************************************************************************/


#endif /* CAN_CAN_BIT_FILD_H_ */





#endif  /* INC_STM32F103X6_H_ */
