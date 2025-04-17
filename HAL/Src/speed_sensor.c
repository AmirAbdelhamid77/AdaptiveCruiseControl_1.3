/*
 * speed_sensor.c
 *
Created on: Some Date
 * Author: AMiR Abdelhamid
 */

#include "speed_sensor.h"

// Timer for measuring pulse periods
static TIM_TimeBase_Config_t Speed_Timer_Config = {
    .TIMx = SPEED_SENSOR_TIMER,
    .Prescaler = SPEED_SENSOR_PRESCALER,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = SPEED_SENSOR_PERIOD,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .RepetitionCounter = 0
};

// Static array to store speed sensor instances (for EXTI callback handling)
static Speed_Sensor_t* SpeedSensorInstances[SPEED_SENSOR_COUNT] = {NULL};

/**
 * @brief EXTI callback function for speed sensor pulses
 * @param None: Uses global array to retrieve instance
 */
static void Speed_Sensor_Pulse_Callback(void) {
    for (uint8_t i = 0; i < SPEED_SENSOR_COUNT; i++) {
        if (SpeedSensorInstances[i] != NULL) {
            // Update pulse information
            SpeedSensorInstances[i]->LastPulseTime = SpeedSensorInstances[i]->CurrentPulseTime;
            SpeedSensorInstances[i]->CurrentPulseTime = MCAL_TIM_Base_GetCounter(SPEED_SENSOR_TIMER);
            
            // Calculate pulse period (accounting for timer overflow)
            if (SpeedSensorInstances[i]->CurrentPulseTime >= SpeedSensorInstances[i]->LastPulseTime) {
                SpeedSensorInstances[i]->PulsePeriod = SpeedSensorInstances[i]->CurrentPulseTime - SpeedSensorInstances[i]->LastPulseTime;
            } else {
                // Timer overflow occurred
                SpeedSensorInstances[i]->PulsePeriod = (SPEED_SENSOR_PERIOD - SpeedSensorInstances[i]->LastPulseTime) + 
                                                       SpeedSensorInstances[i]->CurrentPulseTime + 1;
            }
            
            // Increment pulse count
            SpeedSensorInstances[i]->PulseCount++;
            
            // Calculate speed (cm/s)
            if (SpeedSensorInstances[i]->PulsePeriod > 0) {
                SpeedSensorInstances[i]->Speed = SPEED_CALCULATION_FACTOR / (float)SpeedSensorInstances[i]->PulsePeriod;
            }
            
            // Set ready flag
            SpeedSensorInstances[i]->IsReady = 1;
        }
    }
}

/**
 * @brief Initialize speed sensor
 * @param sensor: Pointer to speed sensor structure
 */
void HAL_SPEED_SENSOR_Init(Speed_Sensor_t* sensor) {
    // Save instance in static array for callback handling
    for (uint8_t i = 0; i < SPEED_SENSOR_COUNT; i++) {
        if (SpeedSensorInstances[i] == NULL) {
            SpeedSensorInstances[i] = sensor;
            break;
        }
    }
    
    // Initialize common timer (only once)
    static uint8_t timerInitialized = 0;
    if (!timerInitialized) {
        MCAL_TIM_Base_Init(&Speed_Timer_Config);
        MCAL_TIM_Base_Start(SPEED_SENSOR_TIMER);
        timerInitialized = 1;
    }
    
    // Configure EXTI for speed sensor pin (rising edge)
    EXTI_PINCONFIG_t extiConfig = {
        .EXTI_Map_Pin = sensor->EXTI_Mapping,
        .EXTI_Trigger_State = EXTI_Rising_Trigger,
        .IRQ_EN = EXTI_IRQ_ENABLE,
        .PF_IRQ_CALLBACK = Speed_Sensor_Pulse_Callback
    };
    MCAL_EXTI_Init(&extiConfig);
    
    // Initialize speed sensor state
    sensor->PulseCount = 0;
    sensor->LastPulseTime = 0;
    sensor->CurrentPulseTime = 0;
    sensor->PulsePeriod = 0;
    sensor->Speed = 0.0f;
    sensor->IsReady = 0;
}

/**
 * @brief Get the current speed
 * @param sensor: Pointer to speed sensor structure
 * @return Speed in cm/s
 */
float HAL_SPEED_SENSOR_GetSpeed(Speed_Sensor_t* sensor) {
    // Check if speed should be zeroed due to timeout
    if ((MCAL_TIM_Base_GetCounter(SPEED_SENSOR_TIMER) - sensor->CurrentPulseTime) > SPEED_SENSOR_TIMEOUT) {
        sensor->Speed = 0.0f;
    }
    
    return sensor->Speed;
}

/**
 * @brief Check if speed is updated
 * @param sensor: Pointer to speed sensor structure
 * @return 1 if updated, 0 if not
 */
uint8_t HAL_SPEED_SENSOR_IsUpdated(Speed_Sensor_t* sensor) {
    return sensor->IsReady;
}

/**
 * @brief Reset speed measurement
 * @param sensor: Pointer to speed sensor structure
 */
void HAL_SPEED_SENSOR_Reset(Speed_Sensor_t* sensor) {
    sensor->IsReady = 0;
}

/**
 * @brief Update speed calculation (call periodically)
 * @param sensor: Pointer to speed sensor structure
 */
void HAL_SPEED_SENSOR_Update(Speed_Sensor_t* sensor) {
    // Check if there has been no pulse for the timeout duration
    if ((MCAL_TIM_Base_GetCounter(SPEED_SENSOR_TIMER) - sensor->CurrentPulseTime) > SPEED_SENSOR_TIMEOUT) {
        // If timeout occurred, set speed to zero
        sensor->Speed = 0.0f;
        sensor->IsReady = 1; // Mark as updated (with zero speed)
    }
} 
