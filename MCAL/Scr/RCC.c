/*
 * RCC.c
 *
 *  Created on: Apr 9, 2024
 *      Author: hassa
 */






#include "RCC.h"

/*Set and cleared by software to control the division factor of the APB High speed clock (PCLK2).
0xx: HCLK not divided
100: HCLK divided by 2
101: HCLK divided by 4
110: HCLK divided by 8
111: HCLK divided by 16*/

const	uint8_t	APbrescTable[8U] = 	{0,0,0,0,1,2,3,4};

/*Set and cleared by software to control AHB clock division factor.
0xxx: SYSCLK not divided
1000: SYSCLK divided by 2
1001: SYSCLK divided by 4
1010: SYSCLK divided by 8
1011: SYSCLK divided by 16
1100: SYSCLK divided by 64
1101: SYSCLK divided by 128
1110: SYSCLK divided by 256
1111: SYSCLK divided by 512*/
const	uint16_t	AHBrescTable[16U] = 	{0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};


uint32_t	MCAL_RCC_GETSYSCLKfreq(void)
{
	uint32_t SystemClock = 0;
	uint8_t CLKSRC = (RCC->CFGR >> 2) & 0b11; // SWS bits

	switch (CLKSRC) {
		case 0: // HSI used as system clock
			SystemClock = HSI_RC_CLK;
			break;
		case 1: // HSE used as system clock
			// Assume HSE_VALUE is defined correctly elsewhere (e.g., in stm32f1xx_hal_conf.h or similar)
			// Or use the define from RCC.h for now, but make sure it matches your crystal
			SystemClock = HSE_CLK;
			break;
		case 2: // PLL used as system clock
		{
			uint32_t pllmul = ((RCC->CFGR >> 18) & 0b1111) + 2; // PLLMUL bits + 2
			// Adjust multiplier for values >= 16 (represented differently)
			if (pllmul > 16) pllmul = 16; // Max multiplier is 16 for F103

			uint32_t pllsrc = (RCC->CFGR >> 16) & 0b1; // PLLSRC bit

			if (pllsrc == 0) { // HSI/2 selected as PLL source
				SystemClock = (HSI_RC_CLK / 2) * pllmul;
			} else { // HSE selected as PLL source
				uint8_t hse_divider = ((RCC->CFGR >> 17) & 0b1) + 1; // PLLXTPRE bit + 1 (1 or 2)
				SystemClock = (HSE_CLK / hse_divider) * pllmul;
			}
			break;
		}
		default:
			// Should not happen, default to HSI
			SystemClock = HSI_RC_CLK;
			break;
	}
	return SystemClock;
}

uint32_t	MCAL_RCC_GETHCLKfreq(void){
	//Bits 7:4 HPRE[3:0]: AHB prescaler
	return MCAL_RCC_GETSYSCLKfreq() >> AHBrescTable[(RCC->CFGR >> 4)&0xF];

}

uint32_t	MCAL_RCC_GETPCLK1Freq(void){
	//Bits 10:8 PPRE1[2:0]: APB Low-speed prescaler (APB1)
	return	MCAL_RCC_GETHCLKfreq() >> APbrescTable[(RCC->CFGR >> 8) & 0b111]  ;

}
uint32_t 	MCAL_RCC_GETPCLK2Freq(void){
	//Bits 13:11 PPRE2[2:0]: APB high-speed prescaler (APB2)
	return	MCAL_RCC_GETHCLKfreq() >> APbrescTable[(RCC->CFGR >> 11) & 0b111]  ;

}
