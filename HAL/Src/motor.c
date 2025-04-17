/*
 * motor.c
 *
 * Created on: Some Date
 * Author: AMiR Abdelhamid
 */

#include "motor.h"

// Array of all motors for global control
static Motor_t* Motors[MOTOR_COUNT] = {NULL};

// PWM timer configuration
static TIM_TimeBase_Config_t PWM_Timer_Config = {
    .TIMx = MOTOR_PWM_TIMER,
    .Prescaler = MOTOR_PWM_PRESCALER,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = MOTOR_PWM_ARR,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .RepetitionCounter = 0
};

/**
 * @brief Initialize the motor control module
 */
void HAL_MOTOR_Init(void) {
    // Initialize PWM timer
    MCAL_TIM_Base_Init(&PWM_Timer_Config);
    
    // For TIM1 (Advanced timer), we need to enable the main output
    if (MOTOR_PWM_TIMER == TIM1) {
        // Enable Main Output
        TIM1->BDTR |= (1 << 15); // MOE: Main Output Enable
    }
    
    // Start the timer
    MCAL_TIM_Base_Start(MOTOR_PWM_TIMER);
}

/**
 * @brief Configure a specific motor
 * @param motor: Pointer to motor structure
 * @param timer: Timer for PWM generation
 * @param channel: Timer channel for this motor
 */
void HAL_MOTOR_Config(Motor_t* motor, TIM_TypeDef* timer, uint16_t channel) {
    // Saving motor instance for global control
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (Motors[i] == NULL) {
            Motors[i] = motor;
            break;
        }
    }
    
    // Configure direction pins as outputs
    GPIO_PinConfig_t dirPinConfig = {
        .GPIO_PinNumber = motor->DirectionPin_Forward,
        .GPIO_Mode = GPIO_Mode_Out_push_pull,
        .GPIO_Speed = GPIO_Speed_10MHz
    };
    MCAL_GPIO_Init(motor->DirectionPort, &dirPinConfig);
    
    dirPinConfig.GPIO_PinNumber = motor->DirectionPin_Backward;
    MCAL_GPIO_Init(motor->DirectionPort, &dirPinConfig);
    
    // Set initial direction to stop
    MOTOR_SET_STOP(motor);
    
    // Configure PWM pin as alternate function output
    GPIO_PinConfig_t pwmPinConfig = {
        .GPIO_PinNumber = motor->PWMPin,
        .GPIO_Mode = GPIO_Mode_Out_AF_push_pull,
        .GPIO_Speed = GPIO_Speed_50MHz
    };
    MCAL_GPIO_Init(motor->PWMPort, &pwmPinConfig);
    
    // Configure PWM output
    TIM_OC_Config_t pwmConfig = {
        .OCMode = TIM_OCMODE_PWM1,
        .Pulse = 0, // Initial duty cycle 0%
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET
    };
    
    // Initialize PWM for this channel
    MCAL_TIM_PWM_Init(timer, channel, &pwmConfig);
    MCAL_TIM_PWM_Start(timer, channel);
    
    // Save timer and channel in motor structure
    motor->PWMTimer = timer;
    motor->PWMChannel = channel;
    
    // Initialize motor state
    motor->Direction = MOTOR_DIR_STOP;
    motor->Speed = 0;
    motor->Enabled = 0;
}

/**
 * @brief Set motor direction
 * @param motor: Pointer to motor structure
 * @param direction: Direction (FORWARD, BACKWARD, STOP)
 */
void HAL_MOTOR_SetDirection(Motor_t* motor, Motor_Direction_t direction) {
    motor->Direction = direction;
}

/**
 * @brief Set motor speed (0-100%)
 * @param motor: Pointer to motor structure
 * @param speed: Speed percentage (0-100)
 */
void HAL_MOTOR_SetSpeed(Motor_t* motor, uint8_t speed) {
    // Limit speed to valid range
    if (speed > MOTOR_MAX_SPEED) {
        speed = MOTOR_MAX_SPEED;
    }
    
    motor->Speed = speed;
}

/**
 * @brief Enable/disable motor
 * @param motor: Pointer to motor structure
 * @param enable: 1 to enable, 0 to disable
 */
void HAL_MOTOR_Enable(Motor_t* motor, uint8_t enable) {
    motor->Enabled = enable;
}

/**
 * @brief Update motor state (applies direction and speed)
 * @param motor: Pointer to motor structure
 */
void HAL_MOTOR_Update(Motor_t* motor) {
    // If motor is disabled, stop it
    if (!motor->Enabled) {
        MOTOR_SET_STOP(motor);
        MCAL_TIM_PWM_SetDutyCycle(motor->PWMTimer, motor->PWMChannel, 0);
        return;
    }
    
    // Set direction
    switch (motor->Direction) {
        case MOTOR_DIR_FORWARD:
            MOTOR_SET_FORWARD(motor);
            break;
        
        case MOTOR_DIR_BACKWARD:
            MOTOR_SET_BACKWARD(motor);
            break;
        
        case MOTOR_DIR_STOP:
        default:
            MOTOR_SET_STOP(motor);
            break;
    }
    
    // Apply speed (PWM duty cycle)
    if (motor->Direction == MOTOR_DIR_STOP) {
        MCAL_TIM_PWM_SetDutyCycle(motor->PWMTimer, motor->PWMChannel, 0);
    } else {
        MCAL_TIM_PWM_SetDutyCycle(motor->PWMTimer, motor->PWMChannel, motor->Speed);
    }
}

/**
 * @brief Stop all motors
 */
void HAL_MOTOR_StopAll(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (Motors[i] != NULL) {
            HAL_MOTOR_SetDirection(Motors[i], MOTOR_DIR_STOP);
            HAL_MOTOR_Update(Motors[i]);
        }
    }
} 
