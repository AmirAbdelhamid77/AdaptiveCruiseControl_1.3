/*
 * stm32_F103C8T6_timer_driver.h
 * Auther: Amir Abdelhamid
 */

#ifndef INC_STM32_F103C6_TIMER_DRIVER_H_
#define INC_STM32_F103C6_TIMER_DRIVER_H_

//-----------------------------
// Includes
//-----------------------------
#include "stm32f103x6.h"
#include "stm32_F103C6_gpio_driver.h"

//-----------------------------
// User type definitions (structures)
//-----------------------------

typedef struct
{
    TIM_TypeDef* TIMx;              /* Specifies the Timer instance */
    
    uint16_t Prescaler;             /* Specifies the prescaler value used to divide the TIM clock.
                                      This parameter can be a value between 0x0000 and 0xFFFF */
    
    uint16_t CounterMode;           /* Specifies the counter mode.
                                      This parameter can be a value of @ref TIM_Counter_Mode */
    
    uint32_t Period;                /* Specifies the period value to be loaded into the active
                                      Auto-Reload Register at the next update event.
                                      This parameter can be a number between 0x0000 and 0xFFFF */
    
    uint16_t ClockDivision;         /* Specifies the clock division.
                                      This parameter can be a value of @ref TIM_Clock_Division */
    
    uint8_t RepetitionCounter;      /* Specifies the repetition counter value. For advanced timers only.
                                      This parameter must be a number between 0x00 and 0xFF */
    
} TIM_TimeBase_Config_t;

typedef struct
{
    uint16_t OCMode;                /* Specifies the output compare mode.
                                      This parameter can be a value of @ref TIM_Output_Compare_Mode */
    
    uint16_t Pulse;                 /* Specifies the pulse value to be loaded into the Capture Compare Register.
                                      This parameter can be a number between 0x0000 and 0xFFFF */
    
    uint16_t OCPolarity;            /* Specifies the output polarity.
                                      This parameter can be a value of @ref TIM_Output_Compare_Polarity */
    
    uint16_t OCNPolarity;           /* Specifies the complementary output polarity. For advanced timers only.
                                      This parameter can be a value of @ref TIM_Output_Compare_N_Polarity */
    
    uint16_t OCIdleState;           /* Specifies the idle state of the output compare pin. For advanced timers only.
                                      This parameter can be a value of @ref TIM_Output_Compare_Idle_State */
    
    uint16_t OCNIdleState;          /* Specifies the idle state of the complementary output compare pin. For advanced timers only.
                                      This parameter can be a value of @ref TIM_Output_Compare_N_Idle_State */
} TIM_OC_Config_t;

typedef struct
{
    uint16_t InputChannel;          /* Specifies TIM input channel
                                      This parameter can be a value of @ref TIM_Channel */
    
    uint16_t ICPolarity;            /* Specifies the active edge of the input signal.
                                      This parameter can be a value of @ref TIM_Input_Capture_Polarity */
    
    uint16_t ICSelection;           /* Specifies the input.
                                      This parameter can be a value of @ref TIM_Input_Capture_Selection */
    
    uint16_t ICPrescaler;           /* Specifies the Input Capture Prescaler.
                                      This parameter can be a value of @ref TIM_Input_Capture_Prescaler */
    
    uint16_t ICFilter;              /* Specifies the input capture filter.
                                      This parameter can be a number between 0x0 and 0xF */
} TIM_IC_Config_t;

//-----------------------------
// Macros Configuration References
//-----------------------------

// @ref TIM_Counter_Mode
#define TIM_COUNTERMODE_UP                 0x0000 /* Counter used as up-counter */
#define TIM_COUNTERMODE_DOWN               0x0010 /* Counter used as down-counter */
#define TIM_COUNTERMODE_CENTERALIGNED1     0x0020 /* Center-aligned mode 1 */
#define TIM_COUNTERMODE_CENTERALIGNED2     0x0040 /* Center-aligned mode 2 */
#define TIM_COUNTERMODE_CENTERALIGNED3     0x0060 /* Center-aligned mode 3 */

// @ref TIM_Clock_Division
#define TIM_CLOCKDIVISION_DIV1             0x0000 /* Clock division: tDTS=tCK_INT */
#define TIM_CLOCKDIVISION_DIV2             0x0100 /* Clock division: tDTS=2*tCK_INT */
#define TIM_CLOCKDIVISION_DIV4             0x0200 /* Clock division: tDTS=4*tCK_INT */

// @ref TIM_Output_Compare_Mode
#define TIM_OCMODE_TIMING                  0x0000 /* Frozen - No output compare active */
#define TIM_OCMODE_ACTIVE                  0x0010 /* Active - OCx is active when TIMx_CNT=TIMx_CCRx */
#define TIM_OCMODE_INACTIVE                0x0020 /* Inactive - OCx is inactive when TIMx_CNT=TIMx_CCRx */
#define TIM_OCMODE_TOGGLE                  0x0030 /* Toggle - OCx toggles when TIMx_CNT=TIMx_CCRx */
#define TIM_OCMODE_PWM1                    0x0060 /* PWM mode 1 */
#define TIM_OCMODE_PWM2                    0x0070 /* PWM mode 2 */

// @ref TIM_Output_Compare_Polarity
#define TIM_OCPOLARITY_HIGH                0x0000 /* Output compare active high */
#define TIM_OCPOLARITY_LOW                 0x0002 /* Output compare active low */

// @ref TIM_Output_Compare_N_Polarity
#define TIM_OCNPOLARITY_HIGH               0x0000 /* Complementary output compare active high */
#define TIM_OCNPOLARITY_LOW                0x0008 /* Complementary output compare active low */

// @ref TIM_Output_Compare_Idle_State
#define TIM_OCIDLESTATE_SET                0x0100 /* Output compare idle state is set */
#define TIM_OCIDLESTATE_RESET              0x0000 /* Output compare idle state is reset */

// @ref TIM_Output_Compare_N_Idle_State
#define TIM_OCNIDLESTATE_SET               0x0200 /* Complementary output compare idle state is set */
#define TIM_OCNIDLESTATE_RESET             0x0000 /* Complementary output compare idle state is reset */

// @ref TIM_Channel
#define TIM_CHANNEL_1                      0x0000 /* Capture/compare channel 1 */
#define TIM_CHANNEL_2                      0x0004 /* Capture/compare channel 2 */
#define TIM_CHANNEL_3                      0x0008 /* Capture/compare channel 3 */
#define TIM_CHANNEL_4                      0x000C /* Capture/compare channel 4 */

// @ref TIM_Input_Capture_Polarity
#define TIM_ICPOLARITY_RISING              0x0000 /* Input capture on rising edge */
#define TIM_ICPOLARITY_FALLING             0x0002 /* Input capture on falling edge */
#define TIM_ICPOLARITY_BOTHEDGE            0x000A /* Input capture on both edges */

// @ref TIM_Input_Capture_Selection
#define TIM_ICSELECTION_DIRECTTI           0x0001 /* TIM Input is selected to be connected to IC1/IC2/IC3/IC4 */
#define TIM_ICSELECTION_INDIRECTTI         0x0002 /* TIM Input is selected to be connected to IC2/IC1/IC4/IC3 */
#define TIM_ICSELECTION_TRC                0x0003 /* TIM Input is selected to be connected to TRC */

// @ref TIM_Input_Capture_Prescaler
#define TIM_ICPSC_DIV1                     0x0000 /* Capture performed each time an edge is detected on the capture input */
#define TIM_ICPSC_DIV2                     0x0004 /* Capture performed once every 2 events */
#define TIM_ICPSC_DIV4                     0x0008 /* Capture performed once every 4 events */
#define TIM_ICPSC_DIV8                     0x000C /* Capture performed once every 8 events */

// @ref TIM_Interrupt_Definition
#define TIM_IT_UPDATE                      0x0001 /* Update interrupt */
#define TIM_IT_CC1                         0x0002 /* Capture/Compare 1 interrupt */
#define TIM_IT_CC2                         0x0004 /* Capture/Compare 2 interrupt */
#define TIM_IT_CC3                         0x0008 /* Capture/Compare 3 interrupt */
#define TIM_IT_CC4                         0x0010 /* Capture/Compare 4 interrupt */
#define TIM_IT_TRIGGER                     0x0040 /* Trigger interrupt */

/*
 * ===============================================
 * APIs Supported by "MCAL TIMER DRIVER"
 * ===============================================
 */

// Timer Base functions
void MCAL_TIM_Base_Init(TIM_TimeBase_Config_t* TIM_Config);
void MCAL_TIM_Base_DeInit(TIM_TypeDef* TIMx);
void MCAL_TIM_Base_Start(TIM_TypeDef* TIMx);
void MCAL_TIM_Base_Stop(TIM_TypeDef* TIMx);
void MCAL_TIM_Base_SetCounter(TIM_TypeDef* TIMx, uint32_t Counter);
uint32_t MCAL_TIM_Base_GetCounter(TIM_TypeDef* TIMx);
void MCAL_TIM_Base_SetPrescaler(TIM_TypeDef* TIMx, uint16_t Prescaler);
void MCAL_TIM_Base_SetPeriod(TIM_TypeDef* TIMx, uint32_t Period);

// PWM Output Compare functions
void MCAL_TIM_PWM_Init(TIM_TypeDef* TIMx, uint16_t Channel, TIM_OC_Config_t* PWM_Config);
void MCAL_TIM_PWM_Start(TIM_TypeDef* TIMx, uint16_t Channel);
void MCAL_TIM_PWM_Stop(TIM_TypeDef* TIMx, uint16_t Channel);
void MCAL_TIM_PWM_SetDutyCycle(TIM_TypeDef* TIMx, uint16_t Channel, uint16_t DutyCycle);

// Input Capture functions
void MCAL_TIM_IC_Init(TIM_TypeDef* TIMx, uint16_t Channel, TIM_IC_Config_t* IC_Config);
void MCAL_TIM_IC_Start(TIM_TypeDef* TIMx, uint16_t Channel);
void MCAL_TIM_IC_Stop(TIM_TypeDef* TIMx, uint16_t Channel);
uint32_t MCAL_TIM_IC_GetCapture(TIM_TypeDef* TIMx, uint16_t Channel);

// Interrupt functions
void MCAL_TIM_EnableIT(TIM_TypeDef* TIMx, uint16_t Interrupt);
void MCAL_TIM_DisableIT(TIM_TypeDef* TIMx, uint16_t Interrupt);
uint8_t MCAL_TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t Flag);
void MCAL_TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t Flag);

// Callback function pointer for timer interrupts
void MCAL_TIM_SetUpdateCallback(TIM_TypeDef* TIMx, void (*Callback)(void));
void MCAL_TIM_SetCaptureCompareCallback(TIM_TypeDef* TIMx, uint16_t Channel, void (*Callback)(void));

#endif /* INC_STM32_F103C6_TIMER_DRIVER_H_ */
