/*
 * stm32f103x6_exti_driver.c
 *
 *  Created on: Oct 19, 2024
 *      Author: Mohamed Hakeem
 */


#include <stm32_F103C6_EXTI_driver.h>

void (* GP_FuncCallBack[16] ) (void);


#define EXTI_GPIOx(x)      (x == GPIOA) ? 0 : \
						   (x == GPIOB) ? 1 : \
						   (x == GPIOC) ? 2 : \
						   (x == GPIOD) ? 3 : 0







/*
 * ===============================================
 * APIs Supported by "MCAL EXTI DRIVER"
 * ===============================================
 **/



/**================================================================
 * @Fn-MCAL_EXTI_DEInit
 * @brief -This Func is responsible for reseting the EXTI Peripheral [Registers] & NVIC Controller
 * @param [in] - 	none
 * @retval -none
 * Note-.....
 */

void NVIC__EXTI_IRQ_ENABLE(EXTI_PINCONFIG_t* EXTI_Config)
{
	EXTI->IMR |= (1 << EXTI_Config->EXTI_Map_Pin.EXTI_Line_Number);
	
	// Use if-else instead of switch for groups that share the same IRQ number
	if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI0_IRQ)
	{
		NVIC_IRQ6_EXTI0_Enable;
	}
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI1_IRQ)
	{
		NVIC_IRQ7_EXTI1_Enable;
	}
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI2_IRQ)
	{
		NVIC_IRQ8_EXTI2_Enable;
	}
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI3_IRQ)
	{
		NVIC_IRQ9_EXTI3_Enable;
	}
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI4_IRQ)
	{
		NVIC_IRQ10_EXTI4_Enable;
	}
	// EXTI5 through EXTI9 share the same IRQ number
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI5_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI6_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI7_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI8_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI9_IRQ)
	{
		NVIC_IRQ23_EXTI5_9_Enable;
	}
	// EXTI10 through EXTI15 share the same IRQ number
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI10_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI11_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI12_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI13_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI14_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI15_IRQ)
	{
		NVIC_IRQ40_EXTI10_15_Enable;
	}
}


/**================================================================
 * @Fn-MCAL_EXTI_DEInit
 * @brief -This Func is responsible for reseting the EXTI Peripheral [Registers] & NVIC Controller
 * @param [in] - 	none
 * @retval -none
 * Note-.....
 */


void NVIC__EXTI_IRQ_DISABLE(EXTI_PINCONFIG_t* EXTI_Config)
{
	EXTI->IMR &= ~(1 << EXTI_Config->EXTI_Map_Pin.EXTI_Line_Number);
	
	// Use if-else instead of switch for groups that share the same IRQ number
	if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI0_IRQ)
	{
		NVIC_IRQ6_EXTI0_Disable;
	}
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI1_IRQ)
	{
		NVIC_IRQ7_EXTI1_Disable;
	}
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI2_IRQ)
	{
		NVIC_IRQ8_EXTI2_Disable;
	}
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI3_IRQ)
	{
		NVIC_IRQ9_EXTI3_Disable;
	}
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI4_IRQ)
	{
		NVIC_IRQ10_EXTI4_Disable;
	}
	// EXTI5 through EXTI9 share the same IRQ number
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI5_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI6_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI7_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI8_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI9_IRQ)
	{
		NVIC_IRQ23_EXTI5_9_Disable;
	}
	// EXTI10 through EXTI15 share the same IRQ number
	else if(EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI10_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI11_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI12_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI13_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI14_IRQ ||
			EXTI_Config->EXTI_Map_Pin.IVT_EXTI_IRQn == EXTI15_IRQ)
	{
		NVIC_IRQ40_EXTI10_15_Disable;
	}
}




/**================================================================
 * @Fn-MCAL_EXTI_DEInit
 * @brief -This Func is responsible for reseting the EXTI Peripheral [Registers] & NVIC Controller
 * @param [in] - 	none
 * @retval -none
 * Note-.....
 */
void MCAL_EXTI_DEInit(void)
{
	//Reset EXTI_Registers
	EXTI->IMR  = 0X00000000;
	EXTI->EMR  = 0X00000000;
	EXTI->RTSR = 0x00000000;
	EXTI->FTSR = 0X00000000;
	EXTI->SWIER = 0X00000000;
	EXTI->PR = 0XFFFFFFFF;


	//Reset NVIC Controller
	NVIC_IRQ6_EXTI0_Disable;
	NVIC_IRQ7_EXTI1_Disable;
	NVIC_IRQ8_EXTI2_Disable;
	NVIC_IRQ9_EXTI3_Disable;
	NVIC_IRQ10_EXTI4_Disable;
	NVIC_IRQ23_EXTI5_9_Disable;
	NVIC_IRQ40_EXTI10_15_Disable;

}



/**================================================================
 * @Fn-MCAL_EXTI_Init
 * @brief -This Func is responsible for reseting the EXTI Peripheral [Registers] & NVIC Controller
 * @param [in] - EXTI_PINCONFIG_t* EXTI_Config
 * @retval -none
 * Note-.....
 */
void MCAL_EXTI_Init(EXTI_PINCONFIG_t* EXTI_Config)
{

	// Configure GPIO PIN As FLOATING Input PIN
	GPIO_PinConfig_t GPIO_EXTI_Config;
	GPIO_EXTI_Config.GPIO_PinNumber=EXTI_Config->EXTI_Map_Pin.EXTI_Pin_Number;
	GPIO_EXTI_Config.GPIO_Mode=GPIO_Mode_Inp_Floating;
	GPIO_EXTI_Config.GPIO_Speed= GPIO_Speed_Reset_State;
	MCAL_GPIO_Init(EXTI_Config->EXTI_Map_Pin.GPIOX,&GPIO_EXTI_Config);
	MCAL_GPIO_Enable_ClocK(EXTI_Config->EXTI_Map_Pin.GPIOX);

	// Mapping Between GPIO AND EXTI by AFIO



	uint8_t AFIO_EXTICR_Index=EXTI_Config->EXTI_Map_Pin.EXTI_Line_Number/4;
	uint8_t EXTIx_Position= (EXTI_Config->EXTI_Map_Pin.EXTI_Line_Number % 4)*4;
	AFIO->EXTICR[AFIO_EXTICR_Index] &= ~ (0xF <<EXTIx_Position) ;
	AFIO->EXTICR[AFIO_EXTICR_Index] |= ((EXTI_GPIOx(EXTI_Config->EXTI_Map_Pin.GPIOX) & 0xF ) << EXTIx_Position);

	EXTI->RTSR &= ~(1 << EXTI_Config->EXTI_Map_Pin.EXTI_Line_Number);
	EXTI->FTSR &= ~(1 << EXTI_Config->EXTI_Map_Pin.EXTI_Line_Number);

	//Configuration of EXTI
	if(EXTI_Config->EXTI_Trigger_State == EXTI_Rising_Trigger)
	{
		EXTI->RTSR |= (1 << EXTI_Config->EXTI_Map_Pin.EXTI_Line_Number);
	}
	else if(EXTI_Config->EXTI_Trigger_State == EXTI_Falling_Trigger)
	{
		EXTI->FTSR |= (1 << EXTI_Config->EXTI_Map_Pin.EXTI_Line_Number);
	}
	else if(EXTI_Config->EXTI_Trigger_State == EXTI_Rising_Falling_Trigger)
	{
		EXTI->RTSR |= (1 << EXTI_Config->EXTI_Map_Pin.EXTI_Line_Number);
		EXTI->FTSR |= (1 << EXTI_Config->EXTI_Map_Pin.EXTI_Line_Number);
	}

	//CallBack Function.
	GP_FuncCallBack[EXTI_Config->EXTI_Map_Pin.EXTI_Line_Number]= EXTI_Config->PF_IRQ_CALLBACK;

	//Enable OR Disable EXTIx & NVIC
	if(EXTI_Config->IRQ_EN == EXTI_IRQ_ENABLE)
	{
		NVIC__EXTI_IRQ_ENABLE(EXTI_Config);

	}
	else if(EXTI_Config->IRQ_EN == EXTI_IRQ_DISABLE)
	{
		NVIC__EXTI_IRQ_DISABLE(EXTI_Config);
	}

}


void EXTI0_IRQHandler(void)
{
	GP_FuncCallBack[0]();
	EXTI->PR |= (1 << 0);

}

void EXTI1_IRQHandler(void)
{
	GP_FuncCallBack[1]();
	EXTI->PR |= (1 << 1);

}

void EXTI2_IRQHandler(void)
{
	GP_FuncCallBack[2]();
	EXTI->PR |= (1 << 2);


}

void EXTI3_IRQHandler(void)
{
	GP_FuncCallBack[3]();
	EXTI->PR |= (1 << 3);

}

void EXTI4_IRQHandler(void)
{
	GP_FuncCallBack[4]();
	EXTI->PR |= (1 << 4);

}


void EXTI9_5_IRQHandler(void)
{
	if(EXTI->PR & (1<<5)) { GP_FuncCallBack[5]();   EXTI->PR |= (1 << 5); }
	if(EXTI->PR & (1<<6)) { GP_FuncCallBack[6]();   EXTI->PR |= (1 << 6); }
	if(EXTI->PR & (1<<7)) { GP_FuncCallBack[7]();   EXTI->PR |= (1 << 7); }
	if(EXTI->PR & (1<<8)) { GP_FuncCallBack[8]();   EXTI->PR |= (1 << 8); }
	if(EXTI->PR & (1<<9)) { GP_FuncCallBack[9]();   EXTI->PR |= (1 << 9); }
}


void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR & (1<<10)) { GP_FuncCallBack[10]();   EXTI->PR |= (1 << 10); }
	if(EXTI->PR & (1<<11)) { GP_FuncCallBack[11]();   EXTI->PR |= (1 << 11); }
	if(EXTI->PR & (1<<12)) { GP_FuncCallBack[12]();   EXTI->PR |= (1 << 12); }
	if(EXTI->PR & (1<<13)) { GP_FuncCallBack[13]();   EXTI->PR |= (1 << 13); }
	if(EXTI->PR & (1<<14)) { GP_FuncCallBack[14]();   EXTI->PR |= (1 << 14); }
	if(EXTI->PR & (1<<15)) { GP_FuncCallBack[15]();   EXTI->PR |= (1 << 15); }

}
