/*
 * ultrasonic.c

 */

#include "ultrasonic.h"

// Timer for measuring echo pulse duration
static TIM_TimeBase_Config_t US_Timer_Config = {
    .TIMx = ULTRASONIC_TIMER,
    .Prescaler = ULTRASONIC_TIMER_PRESCALER,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = ULTRASONIC_TIMER_PERIOD,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .RepetitionCounter = 0
};

/**
 * @brief Initialize ultrasonic sensor
 * @param ultrasonic: Pointer to ultrasonic sensor structure
 */
void HAL_ULTRASONIC_Init(Ultrasonic_t* ultrasonic) {
    // Initialize common timer (only once)
    static uint8_t timerInitialized = 0;
    if (!timerInitialized) {
        MCAL_TIM_Base_Init(&US_Timer_Config);
        MCAL_TIM_Base_Start(ULTRASONIC_TIMER);
        timerInitialized = 1;
    }
    
    // Configure Trigger pin as output
    GPIO_PinConfig_t trigConfig = {
        .GPIO_PinNumber = ultrasonic->TrigPin,
        .GPIO_Mode = GPIO_Mode_Out_push_pull,
        .GPIO_Speed = GPIO_Speed_10MHz
    };
    MCAL_GPIO_Init(ultrasonic->TrigPort, &trigConfig);
    MCAL_GPIO_WritePin(ultrasonic->TrigPort, ultrasonic->TrigPin, GPIO_PIN_RESET);
    
    // Configure EXTI for Echo pin (both rising and falling edges)
    // The single callback provided (ultrasonic->Callback) will handle edge detection internally.
    EXTI_PINCONFIG_t extiConfig = {
        .EXTI_Map_Pin = ultrasonic->EXTI_Mapping,       // From sensor struct
        .EXTI_Trigger_State = EXTI_Rising_Falling_Trigger, // Trigger on both edges
        .IRQ_EN = EXTI_IRQ_ENABLE,                       // Enable the interrupt
        .PF_IRQ_CALLBACK = ultrasonic->Callback          // Assign the specific callback (e.g., Front_Ultrasonic_Callback)
    };
    MCAL_EXTI_Init(&extiConfig); // Initialize EXTI ONLY ONCE per pin
    
    // Initialize ultrasonic state
    ultrasonic->State = ULTRASONIC_STATE_IDLE;
    ultrasonic->Distance = 0;
    ultrasonic->IsReady = 0;
}

/**
 * @brief Start a new ultrasonic measurement
 * @param ultrasonic: Pointer to ultrasonic sensor structure
 */
void HAL_ULTRASONIC_StartMeasurement(Ultrasonic_t* ultrasonic) {
    // Reset state
    ultrasonic->State = ULTRASONIC_STATE_TRIGGERING;
    ultrasonic->IsReady = 0;
    
    // Flash LED three times rapidly to indicate trigger start
    GPIO_TypedDef* debugPort = LED_STATUS_PORT;
    uint16_t debugPin = LED_STATUS_PIN;
    
    for (int i = 0; i < 3; i++) {
        MCAL_GPIO_WritePin(debugPort, debugPin, GPIO_PIN_RESET); // LED on
        for (volatile uint32_t j = 0; j < 2000; j++); // Small delay
        MCAL_GPIO_WritePin(debugPort, debugPin, GPIO_PIN_SET);  // LED off
        for (volatile uint32_t j = 0; j < 2000; j++); // Small delay
    }
    
    // Generate 10µs trigger pulse
    MCAL_GPIO_WritePin(ultrasonic->TrigPort, ultrasonic->TrigPin, GPIO_PIN_SET);
    
    // Reset timer counter for accurate timing
    MCAL_TIM_Base_SetCounter(ULTRASONIC_TIMER, 0);
    
    // Wait for 10µs
    while (MCAL_TIM_Base_GetCounter(ULTRASONIC_TIMER) < ULTRASONIC_TRIGGER_TIME);
    
    // End trigger pulse
    MCAL_GPIO_WritePin(ultrasonic->TrigPort, ultrasonic->TrigPin, GPIO_PIN_RESET);
    
    // Update state to wait for echo
    ultrasonic->State = ULTRASONIC_STATE_WAITING_ECHO;
    
    // Start timeout timer
    MCAL_TIM_Base_SetCounter(ULTRASONIC_TIMER, 0);
}

/**
 * @brief Get the last measured distance
 * @param ultrasonic: Pointer to ultrasonic sensor structure
 * @return Distance in mm, 0 if no valid measurement
 */
uint32_t HAL_ULTRASONIC_GetDistance(Ultrasonic_t* ultrasonic) {
    // Check for timeout if still waiting for echo
    if (ultrasonic->State == ULTRASONIC_STATE_WAITING_ECHO) {
        if (MCAL_TIM_Base_GetCounter(ULTRASONIC_TIMER) > ULTRASONIC_TIMEOUT) {
            ultrasonic->State = ULTRASONIC_STATE_TIMEOUT;
            ultrasonic->IsReady = 1;
            return 0; // Timeout, no valid distance
        }
    }
    
    // Return last measured distance
    if (ultrasonic->State == ULTRASONIC_STATE_ECHO_RECEIVED) {
        // Check if distance is within valid range
        if (ultrasonic->Distance > ULTRASONIC_MAX_DISTANCE) {
            return ULTRASONIC_MAX_DISTANCE; // Cap at maximum distance
        } else if (ultrasonic->Distance < ULTRASONIC_MIN_DISTANCE) {
            return ULTRASONIC_MIN_DISTANCE; // Cap at minimum distance
        } else {
            return ultrasonic->Distance;
        }
    }
    
    return 0; // No valid measurement
}

/**
 * @brief Check if a new measurement is ready
 * @param ultrasonic: Pointer to ultrasonic sensor structure
 * @return 1 if ready, 0 if not ready
 */
uint8_t HAL_ULTRASONIC_IsReady(Ultrasonic_t* ultrasonic) {
    // Check for timeout if still waiting for echo
    if (ultrasonic->State == ULTRASONIC_STATE_WAITING_ECHO) {
        if (MCAL_TIM_Base_GetCounter(ULTRASONIC_TIMER) > ULTRASONIC_TIMEOUT) {
            ultrasonic->State = ULTRASONIC_STATE_TIMEOUT;
            ultrasonic->IsReady = 1;
        }
    }
    
    return ultrasonic->IsReady;
}

/**
 * @brief Timeout handling for ultrasonic measurement
 * @param ultrasonic: Pointer to ultrasonic sensor structure
 */
void HAL_ULTRASONIC_TimeoutCallback(Ultrasonic_t* ultrasonic) {
    if (ultrasonic->State == ULTRASONIC_STATE_WAITING_ECHO) {
        ultrasonic->State = ULTRASONIC_STATE_TIMEOUT;
        ultrasonic->IsReady = 1;
    }
}

/**
 * @brief Trigger a new ultrasonic measurement
 * @param ultrasonic: Pointer to ultrasonic sensor structure
 */
void HAL_ULTRASONIC_Trigger(Ultrasonic_t* ultrasonic) {
    // Simple wrapper around StartMeasurement
    HAL_ULTRASONIC_StartMeasurement(ultrasonic);
} 
