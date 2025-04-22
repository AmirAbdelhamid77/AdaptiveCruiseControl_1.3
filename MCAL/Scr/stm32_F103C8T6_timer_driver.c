/*
 * stm32_F103C6_timer_driver.c
 * Auther: Amir Abdelhamid
 */

#include "stm32F103C8T6_timer_driver.h"

// Array of callback function pointers for timer update events
static void (*TIM_UpdateCallbacks[8])(void) = {NULL};

// Array of callback function pointers for timer capture/compare events
static void (*TIM_CaptureCompareCallbacks[8][4])(void) = {{NULL}};

/**================================================================
 * @Fn-TIM_EnableClock
 * @brief -Enable clock for the specified timer
 * @param [in] - TIMx: Timer instance
 * @retval -none
 * Note-.....
 */
static void TIM_EnableClock(TIM_TypeDef* TIMx) {
    if (TIMx == TIM1) {
        RCC->APB2ENR |= (1 << 11); // TIM1 clock enable
    } else if (TIMx == TIM2) {
        RCC->APB1ENR |= (1 << 0);  // TIM2 clock enable
    } else if (TIMx == TIM3) {
        RCC->APB1ENR |= (1 << 1);  // TIM3 clock enable
    } else if (TIMx == TIM4) {
        RCC->APB1ENR |= (1 << 2);  // TIM4 clock enable
    }
}

/**================================================================
 * @Fn-TIM_GetIndex
 * @brief -Get timer index for array indexing
 * @param [in] - TIMx: Timer instance
 * @retval -Timer index (0-7) or 0 if not found
 * Note-.....
 */
static uint8_t TIM_GetIndex(TIM_TypeDef* TIMx) {
    if (TIMx == TIM1) return 0;
    else if (TIMx == TIM2) return 1;
    else if (TIMx == TIM3) return 2;
    else if (TIMx == TIM4) return 3;
    else return 0; // Default to 0 if not found
}

/**================================================================
 * @Fn-TIM_GetChannelIndex
 * @brief -Get channel index for array indexing
 * @param [in] - Channel: Timer channel (@ref TIM_Channel)
 * @retval -Channel index (0-3) or 0 if not found
 * Note-.....
 */
static uint8_t TIM_GetChannelIndex(uint16_t Channel) {
    if (Channel == TIM_CHANNEL_1) return 0;
    else if (Channel == TIM_CHANNEL_2) return 1;
    else if (Channel == TIM_CHANNEL_3) return 2;
    else if (Channel == TIM_CHANNEL_4) return 3;
    else return 0; // Default to 0 if not found
}

// Timer base functions

/**================================================================
 * @Fn-MCAL_TIM_Base_Init
 * @brief -Initialize timer base functionality
 * @param [in] - TIM_Config: Pointer to Timer TimeBase configuration struct
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_Base_Init(TIM_TimeBase_Config_t* TIM_Config) {
    TIM_TypeDef* TIMx = TIM_Config->TIMx;
    
    // Enable timer clock
    TIM_EnableClock(TIMx);
    
    // Set prescaler
    TIMx->PSC = TIM_Config->Prescaler;
    
    // Set auto-reload value (period)
    TIMx->ARR = TIM_Config->Period;
    
    // Set repetition counter (for advanced timers like TIM1)
    if (TIMx == TIM1) {
        TIMx->RCR = TIM_Config->RepetitionCounter;
    }
    
    // Set counter mode and clock division
    TIMx->CR1 &= ~(0x03F0); // Clear counter mode and clock division bits
    TIMx->CR1 |= (TIM_Config->CounterMode | TIM_Config->ClockDivision);
    
    // Generate update event to load new values
    TIMx->EGR |= 0x01;
}

/**================================================================
 * @Fn-MCAL_TIM_Base_DeInit
 * @brief -Deinitialize timer and reset its registers
 * @param [in] - TIMx: Timer instance
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_Base_DeInit(TIM_TypeDef* TIMx) {
    if (TIMx == TIM1) {
        RCC->APB2RSTR |= (1 << 11);  // TIM1 reset
        RCC->APB2RSTR &= ~(1 << 11); // Clear reset bit
    } else if (TIMx == TIM2) {
        RCC->APB1RSTR |= (1 << 0);   // TIM2 reset
        RCC->APB1RSTR &= ~(1 << 0);  // Clear reset bit
    } else if (TIMx == TIM3) {
        RCC->APB1RSTR |= (1 << 1);   // TIM3 reset
        RCC->APB1RSTR &= ~(1 << 1);  // Clear reset bit
    } else if (TIMx == TIM4) {
        RCC->APB1RSTR |= (1 << 2);   // TIM4 reset
        RCC->APB1RSTR &= ~(1 << 2);  // Clear reset bit
    }
    
    // Clear callback pointers
    uint8_t timer_idx = TIM_GetIndex(TIMx);
    TIM_UpdateCallbacks[timer_idx] = NULL;
    
    for (uint8_t i = 0; i < 4; i++) {
        TIM_CaptureCompareCallbacks[timer_idx][i] = NULL;
    }
}

/**================================================================
 * @Fn-MCAL_TIM_Base_Start
 * @brief -Start timer counter
 * @param [in] - TIMx: Timer instance
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_Base_Start(TIM_TypeDef* TIMx) {
    // Enable counter
    TIMx->CR1 |= 0x01;
}

/**================================================================
 * @Fn-MCAL_TIM_Base_Stop
 * @brief -Stop timer counter
 * @param [in] - TIMx: Timer instance
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_Base_Stop(TIM_TypeDef* TIMx) {
    // Disable counter
    TIMx->CR1 &= ~0x01;
}

/**================================================================
 * @Fn-MCAL_TIM_Base_SetCounter
 * @brief -Set timer counter value
 * @param [in] - TIMx: Timer instance
 * @param [in] - Counter: Counter value to set
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_Base_SetCounter(TIM_TypeDef* TIMx, uint32_t Counter) {
    TIMx->CNT = Counter;
}

/**================================================================
 * @Fn-MCAL_TIM_Base_GetCounter
 * @brief -Get timer counter value
 * @param [in] - TIMx: Timer instance
 * @retval -Current counter value
 * Note-.....
 */
uint32_t MCAL_TIM_Base_GetCounter(TIM_TypeDef* TIMx) {
    return TIMx->CNT;
}

/**================================================================
 * @Fn-MCAL_TIM_Base_SetPrescaler
 * @brief -Set timer prescaler value
 * @param [in] - TIMx: Timer instance
 * @param [in] - Prescaler: Prescaler value to set
 * @retval -none
 * Note- Triggers an update event to load the new value immediately.
 */
void MCAL_TIM_Base_SetPrescaler(TIM_TypeDef* TIMx, uint16_t Prescaler) {
    TIMx->PSC = Prescaler;
    // Generate update event to load the new prescaler
    TIMx->EGR |= 0x01;
}

/**================================================================
 * @Fn-MCAL_TIM_Base_SetPeriod
 * @brief -Set timer auto-reload value (period)
 * @param [in] - TIMx: Timer instance
 * @param [in] - Period: Period value to set
 * @retval -none
 * Note- Triggers an update event to load the new value immediately.
 */
void MCAL_TIM_Base_SetPeriod(TIM_TypeDef* TIMx, uint32_t Period) {
    TIMx->ARR = Period;
    // Generate update event to load the new period
    TIMx->EGR |= 0x01;
}

// PWM functions

/**================================================================
 * @Fn-MCAL_TIM_PWM_Init
 * @brief -Initialize PWM mode for a specific timer channel
 * @param [in] - TIMx: Timer instance
 * @param [in] - Channel: Timer channel (@ref TIM_Channel)
 * @param [in] - PWM_Config: Pointer to Timer Output Compare configuration struct for PWM
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_PWM_Init(TIM_TypeDef* TIMx, uint16_t Channel, TIM_OC_Config_t* PWM_Config) {
    // Configure the channel based on channel number
    switch (Channel) {
        case TIM_CHANNEL_1:
            // Clear the old configuration
            TIMx->CCMR1 &= ~(0xFF << 0);
            // Set PWM mode and preload enable
            TIMx->CCMR1 |= (PWM_Config->OCMode << 4) | (1 << 3);
            // Set output polarity
            TIMx->CCER &= ~(0x3 << 0);
            TIMx->CCER |= (PWM_Config->OCPolarity << 1);
            // Set pulse width
            TIMx->CCR1 = PWM_Config->Pulse;
            
            // For advanced timers (TIM1)
            if (TIMx == TIM1) {
                // Set complementary output polarity and idle states
                TIMx->CCER &= ~(0x3 << 2);
                TIMx->CCER |= (PWM_Config->OCNPolarity << 3);
                TIMx->CR2 &= ~(0x3 << 0);
                TIMx->CR2 |= ((PWM_Config->OCIdleState >> 8) << 0) | ((PWM_Config->OCNIdleState >> 9) << 2);
            }
            break;
            
        case TIM_CHANNEL_2:
            // Clear the old configuration
            TIMx->CCMR1 &= ~(0xFF << 8);
            // Set PWM mode and preload enable
            TIMx->CCMR1 |= (PWM_Config->OCMode << 12) | (1 << 11);
            // Set output polarity
            TIMx->CCER &= ~(0x3 << 4);
            TIMx->CCER |= (PWM_Config->OCPolarity << 5);
            // Set pulse width
            TIMx->CCR2 = PWM_Config->Pulse;
            
            // For advanced timers (TIM1)
            if (TIMx == TIM1) {
                // Set complementary output polarity and idle states
                TIMx->CCER &= ~(0x3 << 6);
                TIMx->CCER |= (PWM_Config->OCNPolarity << 7);
                TIMx->CR2 &= ~(0x3 << 4);
                TIMx->CR2 |= ((PWM_Config->OCIdleState >> 8) << 4) | ((PWM_Config->OCNIdleState >> 9) << 6);
            }
            break;
            
        case TIM_CHANNEL_3:
            // Clear the old configuration
            TIMx->CCMR2 &= ~(0xFF << 0);
            // Set PWM mode and preload enable
            TIMx->CCMR2 |= (PWM_Config->OCMode << 4) | (1 << 3);
            // Set output polarity
            TIMx->CCER &= ~(0x3 << 8);
            TIMx->CCER |= (PWM_Config->OCPolarity << 9);
            // Set pulse width
            TIMx->CCR3 = PWM_Config->Pulse;
            
            // For advanced timers (TIM1)
            if (TIMx == TIM1) {
                // Set complementary output polarity and idle states
                TIMx->CCER &= ~(0x3 << 10);
                TIMx->CCER |= (PWM_Config->OCNPolarity << 11);
                TIMx->CR2 &= ~(0x3 << 8);
                TIMx->CR2 |= ((PWM_Config->OCIdleState >> 8) << 8) | ((PWM_Config->OCNIdleState >> 9) << 10);
            }
            break;
            
        case TIM_CHANNEL_4:
            // Clear the old configuration
            TIMx->CCMR2 &= ~(0xFF << 8);
            // Set PWM mode and preload enable
            TIMx->CCMR2 |= (PWM_Config->OCMode << 12) | (1 << 11);
            // Set output polarity
            TIMx->CCER &= ~(0x3 << 12);
            TIMx->CCER |= (PWM_Config->OCPolarity << 13);
            // Set pulse width
            TIMx->CCR4 = PWM_Config->Pulse;
            
            // For advanced timers (TIM1)
            if (TIMx == TIM1) {
                // No complementary output for Channel 4, only idle state
                TIMx->CR2 &= ~(0x1 << 12);
                TIMx->CR2 |= ((PWM_Config->OCIdleState >> 8) << 12);
            }
            break;
    }
    
    // For TIM1, enable Main Output
    if (TIMx == TIM1) {
        TIMx->BDTR |= (1 << 15); // MOE: Main Output Enable
    }
}

/**================================================================
 * @Fn-MCAL_TIM_PWM_Start
 * @brief -Start PWM signal generation on a specific channel
 * @param [in] - TIMx: Timer instance
 * @param [in] - Channel: Timer channel (@ref TIM_Channel)
 * @retval -none
 * Note- Starts the timer base if it's not already running.
 */
void MCAL_TIM_PWM_Start(TIM_TypeDef* TIMx, uint16_t Channel) {
    // Enable the corresponding output
    uint8_t shift = (Channel / 4) * 4; // 0, 4, 8, or 12
    TIMx->CCER |= (1 << shift);
    
    // Start the timer if not already started
    MCAL_TIM_Base_Start(TIMx);
}

/**================================================================
 * @Fn-MCAL_TIM_PWM_Stop
 * @brief -Stop PWM signal generation on a specific channel
 * @param [in] - TIMx: Timer instance
 * @param [in] - Channel: Timer channel (@ref TIM_Channel)
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_PWM_Stop(TIM_TypeDef* TIMx, uint16_t Channel) {
    // Disable the corresponding output
    uint8_t shift = (Channel / 4) * 4; // 0, 4, 8, or 12
    TIMx->CCER &= ~(1 << shift);
}

/**================================================================
 * @Fn-MCAL_TIM_PWM_SetDutyCycle
 * @brief -Set PWM duty cycle for a specific channel
 * @param [in] - TIMx: Timer instance
 * @param [in] - Channel: Timer channel (@ref TIM_Channel)
 * @param [in] - DutyCycle: Duty cycle value (0-100)
 * @retval -none
 * Note- Calculates pulse width based on current ARR value.
 */
void MCAL_TIM_PWM_SetDutyCycle(TIM_TypeDef* TIMx, uint16_t Channel, uint16_t DutyCycle) {
    uint32_t pulse = ((TIMx->ARR + 1) * DutyCycle) / 100;
    
    // Set pulse width based on channel
    switch (Channel) {
        case TIM_CHANNEL_1:
            TIMx->CCR1 = pulse;
            break;
        case TIM_CHANNEL_2:
            TIMx->CCR2 = pulse;
            break;
        case TIM_CHANNEL_3:
            TIMx->CCR3 = pulse;
            break;
        case TIM_CHANNEL_4:
            TIMx->CCR4 = pulse;
            break;
    }
}

// Input Capture functions

/**================================================================
 * @Fn-MCAL_TIM_IC_Init
 * @brief -Initialize Input Capture mode for a specific timer channel
 * @param [in] - TIMx: Timer instance
 * @param [in] - Channel: Timer channel (@ref TIM_Channel)
 * @param [in] - IC_Config: Pointer to Timer Input Capture configuration struct
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_IC_Init(TIM_TypeDef* TIMx, uint16_t Channel, TIM_IC_Config_t* IC_Config) {
    switch (Channel) {
        case TIM_CHANNEL_1:
            // Clear old configuration
            TIMx->CCMR1 &= ~(0xFF << 0);
            // Set input capture filter and prescaler
            TIMx->CCMR1 |= (IC_Config->ICFilter << 4) | (IC_Config->ICPrescaler << 2) | (IC_Config->ICSelection << 0);
            // Set input capture polarity
            TIMx->CCER &= ~(0x3 << 0);
            TIMx->CCER |= (IC_Config->ICPolarity << 1);
            break;
            
        case TIM_CHANNEL_2:
            // Clear old configuration
            TIMx->CCMR1 &= ~(0xFF << 8);
            // Set input capture filter and prescaler
            TIMx->CCMR1 |= (IC_Config->ICFilter << 12) | (IC_Config->ICPrescaler << 10) | (IC_Config->ICSelection << 8);
            // Set input capture polarity
            TIMx->CCER &= ~(0x3 << 4);
            TIMx->CCER |= (IC_Config->ICPolarity << 5);
            break;
            
        case TIM_CHANNEL_3:
            // Clear old configuration
            TIMx->CCMR2 &= ~(0xFF << 0);
            // Set input capture filter and prescaler
            TIMx->CCMR2 |= (IC_Config->ICFilter << 4) | (IC_Config->ICPrescaler << 2) | (IC_Config->ICSelection << 0);
            // Set input capture polarity
            TIMx->CCER &= ~(0x3 << 8);
            TIMx->CCER |= (IC_Config->ICPolarity << 9);
            break;
            
        case TIM_CHANNEL_4:
            // Clear old configuration
            TIMx->CCMR2 &= ~(0xFF << 8);
            // Set input capture filter and prescaler
            TIMx->CCMR2 |= (IC_Config->ICFilter << 12) | (IC_Config->ICPrescaler << 10) | (IC_Config->ICSelection << 8);
            // Set input capture polarity
            TIMx->CCER &= ~(0x3 << 12);
            TIMx->CCER |= (IC_Config->ICPolarity << 13);
            break;
    }
}

/**================================================================
 * @Fn-MCAL_TIM_IC_Start
 * @brief -Start Input Capture on a specific channel
 * @param [in] - TIMx: Timer instance
 * @param [in] - Channel: Timer channel (@ref TIM_Channel)
 * @retval -none
 * Note- Starts the timer base if it's not already running.
 */
void MCAL_TIM_IC_Start(TIM_TypeDef* TIMx, uint16_t Channel) {
    // Enable the corresponding capture input
    uint8_t shift = (Channel / 4) * 4; // 0, 4, 8, or 12
    TIMx->CCER |= (1 << shift);
    
    // Start the timer if not already started
    MCAL_TIM_Base_Start(TIMx);
}

/**================================================================
 * @Fn-MCAL_TIM_IC_Stop
 * @brief -Stop Input Capture on a specific channel
 * @param [in] - TIMx: Timer instance
 * @param [in] - Channel: Timer channel (@ref TIM_Channel)
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_IC_Stop(TIM_TypeDef* TIMx, uint16_t Channel) {
    // Disable the corresponding capture input
    uint8_t shift = (Channel / 4) * 4; // 0, 4, 8, or 12
    TIMx->CCER &= ~(1 << shift);
}

/**================================================================
 * @Fn-MCAL_TIM_IC_GetCapture
 * @brief -Get the last captured value from a specific input capture channel
 * @param [in] - TIMx: Timer instance
 * @param [in] - Channel: Timer channel (@ref TIM_Channel)
 * @retval -Captured timer value
 * Note-.....
 */
uint32_t MCAL_TIM_IC_GetCapture(TIM_TypeDef* TIMx, uint16_t Channel) {
    uint32_t value = 0;
    
    switch (Channel) {
        case TIM_CHANNEL_1:
            value = TIMx->CCR1;
            break;
        case TIM_CHANNEL_2:
            value = TIMx->CCR2;
            break;
        case TIM_CHANNEL_3:
            value = TIMx->CCR3;
            break;
        case TIM_CHANNEL_4:
            value = TIMx->CCR4;
            break;
    }
    
    return value;
}

// Interrupt functions

/**================================================================
 * @Fn-MCAL_TIM_EnableIT
 * @brief -Enable a specific timer interrupt and its corresponding NVIC line
 * @param [in] - TIMx: Timer instance
 * @param [in] - Interrupt: Interrupt type (@ref TIM_Interrupt_Definition)
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_EnableIT(TIM_TypeDef* TIMx, uint16_t Interrupt) {
    // Enable the specified timer interrupt
    TIMx->DIER |= Interrupt;
    
    // Enable the corresponding NVIC interrupt
    if (TIMx == TIM1) {
        if (Interrupt & TIM_IT_UPDATE) {
            NVIC_ISER0 |= (1 << 25); // TIM1 Update Interrupt
        }
        if (Interrupt & (TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4)) {
            NVIC_ISER0 |= (1 << 26); // TIM1 Capture Compare Interrupt
        }
    } else if (TIMx == TIM2) {
        NVIC_ISER0 |= (1 << 28); // TIM2 Global Interrupt
    } else if (TIMx == TIM3) {
        NVIC_ISER0 |= (1 << 29); // TIM3 Global Interrupt
    } else if (TIMx == TIM4) {
        NVIC_ISER0 |= (1 << 30); // TIM4 Global Interrupt
    }
}

/**================================================================
 * @Fn-MCAL_TIM_DisableIT
 * @brief -Disable a specific timer interrupt
 * @param [in] - TIMx: Timer instance
 * @param [in] - Interrupt: Interrupt type (@ref TIM_Interrupt_Definition)
 * @retval -none
 * Note- Disables the NVIC line only if all interrupts for that timer are disabled.
 */
void MCAL_TIM_DisableIT(TIM_TypeDef* TIMx, uint16_t Interrupt) {
    // Disable the specified timer interrupt
    TIMx->DIER &= ~Interrupt;
    
    // Check if all interrupts for this timer are disabled
    if ((TIMx->DIER & (TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_TRIGGER)) == 0) {
        // If all are disabled, disable the corresponding NVIC interrupt
        if (TIMx == TIM1) {
            NVIC_ICER0 |= (1 << 25); // TIM1 Update Interrupt
            NVIC_ICER0 |= (1 << 26); // TIM1 Capture Compare Interrupt
        } else if (TIMx == TIM2) {
            NVIC_ICER0 |= (1 << 28); // TIM2 Global Interrupt
        } else if (TIMx == TIM3) {
            NVIC_ICER0 |= (1 << 29); // TIM3 Global Interrupt
        } else if (TIMx == TIM4) {
            NVIC_ICER0 |= (1 << 30); // TIM4 Global Interrupt
        }
    }
}

/**================================================================
 * @Fn-MCAL_TIM_GetFlagStatus
 * @brief -Get the status of a specific timer flag
 * @param [in] - TIMx: Timer instance
 * @param [in] - Flag: Flag to check (@ref TIM_Interrupt_Definition or other SR flags)
 * @retval -Flag status (1 if set, 0 if not set)
 * Note-.....
 */
uint8_t MCAL_TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t Flag) {
    return (TIMx->SR & Flag) ? 1 : 0;
}

/**================================================================
 * @Fn-MCAL_TIM_ClearFlag
 * @brief -Clear a specific timer flag
 * @param [in] - TIMx: Timer instance
 * @param [in] - Flag: Flag to clear (@ref TIM_Interrupt_Definition or other SR flags)
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t Flag) {
    TIMx->SR &= ~Flag;
}

/**================================================================
 * @Fn-MCAL_TIM_SetUpdateCallback
 * @brief -Set the callback function for a timer's update interrupt event
 * @param [in] - TIMx: Timer instance
 * @param [in] - Callback: Pointer to the callback function (void function returning void)
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_SetUpdateCallback(TIM_TypeDef* TIMx, void (*Callback)(void)) {
    uint8_t timer_idx = TIM_GetIndex(TIMx);
    TIM_UpdateCallbacks[timer_idx] = Callback;
}

/**================================================================
 * @Fn-MCAL_TIM_SetCaptureCompareCallback
 * @brief -Set the callback function for a timer's capture/compare interrupt event on a specific channel
 * @param [in] - TIMx: Timer instance
 * @param [in] - Channel: Timer channel (@ref TIM_Channel)
 * @param [in] - Callback: Pointer to the callback function (void function returning void)
 * @retval -none
 * Note-.....
 */
void MCAL_TIM_SetCaptureCompareCallback(TIM_TypeDef* TIMx, uint16_t Channel, void (*Callback)(void)) {
    uint8_t timer_idx = TIM_GetIndex(TIMx);
    uint8_t channel_idx = TIM_GetChannelIndex(Channel);
    TIM_CaptureCompareCallbacks[timer_idx][channel_idx] = Callback;
}

// Timer Interrupt Handlers
void TIM1_UP_IRQHandler(void) {
    if (TIM1->SR & TIM_IT_UPDATE) {
        // Clear update interrupt flag
        TIM1->SR &= ~TIM_IT_UPDATE;
        
        // Call user callback if registered
        if (TIM_UpdateCallbacks[0] != NULL) {
            TIM_UpdateCallbacks[0]();
        }
    }
}

void TIM1_CC_IRQHandler(void) {
    // Check for capture/compare interrupts
    if (TIM1->SR & TIM_IT_CC1) {
        TIM1->SR &= ~TIM_IT_CC1;
        if (TIM_CaptureCompareCallbacks[0][0] != NULL) {
            TIM_CaptureCompareCallbacks[0][0]();
        }
    }
    if (TIM1->SR & TIM_IT_CC2) {
        TIM1->SR &= ~TIM_IT_CC2;
        if (TIM_CaptureCompareCallbacks[0][1] != NULL) {
            TIM_CaptureCompareCallbacks[0][1]();
        }
    }
    if (TIM1->SR & TIM_IT_CC3) {
        TIM1->SR &= ~TIM_IT_CC3;
        if (TIM_CaptureCompareCallbacks[0][2] != NULL) {
            TIM_CaptureCompareCallbacks[0][2]();
        }
    }
    if (TIM1->SR & TIM_IT_CC4) {
        TIM1->SR &= ~TIM_IT_CC4;
        if (TIM_CaptureCompareCallbacks[0][3] != NULL) {
            TIM_CaptureCompareCallbacks[0][3]();
        }
    }
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_IT_UPDATE) {
        TIM2->SR &= ~TIM_IT_UPDATE;
        if (TIM_UpdateCallbacks[1] != NULL) {
            TIM_UpdateCallbacks[1]();
        }
    }
    
    // Check for capture/compare interrupts
    if (TIM2->SR & TIM_IT_CC1) {
        TIM2->SR &= ~TIM_IT_CC1;
        if (TIM_CaptureCompareCallbacks[1][0] != NULL) {
            TIM_CaptureCompareCallbacks[1][0]();
        }
    }
    if (TIM2->SR & TIM_IT_CC2) {
        TIM2->SR &= ~TIM_IT_CC2;
        if (TIM_CaptureCompareCallbacks[1][1] != NULL) {
            TIM_CaptureCompareCallbacks[1][1]();
        }
    }
    if (TIM2->SR & TIM_IT_CC3) {
        TIM2->SR &= ~TIM_IT_CC3;
        if (TIM_CaptureCompareCallbacks[1][2] != NULL) {
            TIM_CaptureCompareCallbacks[1][2]();
        }
    }
    if (TIM2->SR & TIM_IT_CC4) {
        TIM2->SR &= ~TIM_IT_CC4;
        if (TIM_CaptureCompareCallbacks[1][3] != NULL) {
            TIM_CaptureCompareCallbacks[1][3]();
        }
    }
}

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_IT_UPDATE) {
        TIM3->SR &= ~TIM_IT_UPDATE;
        if (TIM_UpdateCallbacks[2] != NULL) {
            TIM_UpdateCallbacks[2]();
        }
    }
    
    // Check for capture/compare interrupts
    if (TIM3->SR & TIM_IT_CC1) {
        TIM3->SR &= ~TIM_IT_CC1;
        if (TIM_CaptureCompareCallbacks[2][0] != NULL) {
            TIM_CaptureCompareCallbacks[2][0]();
        }
    }
    if (TIM3->SR & TIM_IT_CC2) {
        TIM3->SR &= ~TIM_IT_CC2;
        if (TIM_CaptureCompareCallbacks[2][1] != NULL) {
            TIM_CaptureCompareCallbacks[2][1]();
        }
    }
    if (TIM3->SR & TIM_IT_CC3) {
        TIM3->SR &= ~TIM_IT_CC3;
        if (TIM_CaptureCompareCallbacks[2][2] != NULL) {
            TIM_CaptureCompareCallbacks[2][2]();
        }
    }
    if (TIM3->SR & TIM_IT_CC4) {
        TIM3->SR &= ~TIM_IT_CC4;
        if (TIM_CaptureCompareCallbacks[2][3] != NULL) {
            TIM_CaptureCompareCallbacks[2][3]();
        }
    }
}

void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_IT_UPDATE) {
        TIM4->SR &= ~TIM_IT_UPDATE;
        if (TIM_UpdateCallbacks[3] != NULL) {
            TIM_UpdateCallbacks[3]();
        }
    }
    
    // Check for capture/compare interrupts
    if (TIM4->SR & TIM_IT_CC1) {
        TIM4->SR &= ~TIM_IT_CC1;
        if (TIM_CaptureCompareCallbacks[3][0] != NULL) {
            TIM_CaptureCompareCallbacks[3][0]();
        }
    }
    if (TIM4->SR & TIM_IT_CC2) {
        TIM4->SR &= ~TIM_IT_CC2;
        if (TIM_CaptureCompareCallbacks[3][1] != NULL) {
            TIM_CaptureCompareCallbacks[3][1]();
        }
    }
    if (TIM4->SR & TIM_IT_CC3) {
        TIM4->SR &= ~TIM_IT_CC3;
        if (TIM_CaptureCompareCallbacks[3][2] != NULL) {
            TIM_CaptureCompareCallbacks[3][2]();
        }
    }
    if (TIM4->SR & TIM_IT_CC4) {
        TIM4->SR &= ~TIM_IT_CC4;
        if (TIM_CaptureCompareCallbacks[3][3] != NULL) {
            TIM_CaptureCompareCallbacks[3][3]();
        }
    }
} 
