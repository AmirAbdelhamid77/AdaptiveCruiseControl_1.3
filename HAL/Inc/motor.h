/*
 * motor.h
 *
 * Created on: Some Date
 * Author: AMiR Abdelhamid
 */

#ifndef HAL_INC_MOTOR_H_
#define HAL_INC_MOTOR_H_

//-----------------------------
// Includes
//-----------------------------
#include <stm32F103C8T6_pin_mapping.h>
#include <stm32F103C8T6_timer_driver.h>
#include "stm32f103x6.h"
#include "stm32_F103C6_gpio_driver.h"

//-----------------------------
// type definitions (structures)
//-----------------------------

typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD = 1,
    MOTOR_DIR_BACKWARD = 2
} Motor_Direction_t;

typedef struct {
    // Motor configuration
    GPIO_TypedDef* DirectionPort;
    uint16_t DirectionPin_Forward;
    uint16_t DirectionPin_Backward;
    GPIO_TypedDef* PWMPort;
    uint16_t PWMPin;
    
    // Motor control parameters
    TIM_TypeDef* PWMTimer;
    uint16_t PWMChannel;
    
    // Motor state
    Motor_Direction_t Direction;
    uint8_t Speed;     // 0-100%
    uint8_t Enabled;   // 0 or 1
} Motor_t;

//-----------------------------
// Macros and Constants
//-----------------------------

// Motor speed limits
#define MOTOR_MIN_SPEED   0
#define MOTOR_MAX_SPEED   100

// Direction pins control macros
#define MOTOR_SET_FORWARD(motor)    do { \
                                        MCAL_GPIO_WritePin((motor)->DirectionPort, (motor)->DirectionPin_Forward, GPIO_PIN_SET); \
                                        MCAL_GPIO_WritePin((motor)->DirectionPort, (motor)->DirectionPin_Backward, GPIO_PIN_RESET); \
                                    } while(0)

#define MOTOR_SET_BACKWARD(motor)   do { \
                                        MCAL_GPIO_WritePin((motor)->DirectionPort, (motor)->DirectionPin_Forward, GPIO_PIN_RESET); \
                                        MCAL_GPIO_WritePin((motor)->DirectionPort, (motor)->DirectionPin_Backward, GPIO_PIN_SET); \
                                    } while(0)

#define MOTOR_SET_STOP(motor)       do { \
                                        MCAL_GPIO_WritePin((motor)->DirectionPort, (motor)->DirectionPin_Forward, GPIO_PIN_RESET); \
                                        MCAL_GPIO_WritePin((motor)->DirectionPort, (motor)->DirectionPin_Backward, GPIO_PIN_RESET); \
                                    } while(0)

//-----------------------------
// Function Prototypes
//-----------------------------

// Initialize the motor control
void HAL_MOTOR_Init(void);

// Configure a specific motor
void HAL_MOTOR_Config(Motor_t* motor, TIM_TypeDef* timer, uint16_t channel);

// Set motor direction
void HAL_MOTOR_SetDirection(Motor_t* motor, Motor_Direction_t direction);

// Set motor speed (0-100%)
void HAL_MOTOR_SetSpeed(Motor_t* motor, uint8_t speed);

// Enable/disable motor
void HAL_MOTOR_Enable(Motor_t* motor, uint8_t enable);

// Update motor state (applies direction and speed)
void HAL_MOTOR_Update(Motor_t* motor);

// Stop all motors
void HAL_MOTOR_StopAll(void);

#endif /* HAL_INC_MOTOR_H_ */ 
