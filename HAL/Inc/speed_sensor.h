/*
 * speed_sensor.h
 *
 * Created on: Some Date
 * Author: AMiR Abdelhamid
 */

#ifndef HAL_INC_SPEED_SENSOR_H_
#define HAL_INC_SPEED_SENSOR_H_

//-----------------------------
// Includes
//-----------------------------
#include <stm32F103C8T6_pin_mapping.h>
#include <stm32F103C8T6_timer_driver.h>
#include "stm32f103x6.h"
#include "stm32_F103C6_gpio_driver.h"
#include "stm32_F103C6_EXTI_driver.h"

//-----------------------------
// User type definitions (structures)
//-----------------------------

typedef struct {
    // Speed sensor configuration
    GPIO_TypedDef* Port;
    uint16_t Pin;
    EXTI_AFIO_MAPPING_t EXTI_Mapping;
    
    // Speed calculation variables
    uint32_t PulseCount;        // Number of pulses counted
    uint32_t LastPulseTime;     // Timestamp of last pulse
    uint32_t CurrentPulseTime;  // Timestamp of current pulse
    uint32_t PulsePeriod;       // Time between pulses in timer ticks
    float Speed;                // Speed in cm/s
    uint8_t IsReady;            // Flag to indicate if a new speed measurement is ready
} Speed_Sensor_t;

//-----------------------------
// Macros and Constants
//-----------------------------

// The HC-020K speed sensor produces 20 pulses per revolution
#define SPEED_SENSOR_PULSES_PER_REV    20

// Wheel diameter in cm (adjust for your specific setup)
#define WHEEL_DIAMETER_CM              2.5

// Wheel circumference in cm
#define WHEEL_CIRCUMFERENCE_CM         (WHEEL_DIAMETER_CM * 3.14159f)

// Speed calculation constants
// We use the TIM3 timer with 100µs tick
// Speed (cm/s) = (WHEEL_CIRCUMFERENCE_CM / SPEED_SENSOR_PULSES_PER_REV) / (PulsePeriod * 0.0001)
// To simplify: Speed = (WHEEL_CIRCUMFERENCE_CM * 10000) / (SPEED_SENSOR_PULSES_PER_REV * PulsePeriod)
#define SPEED_CALCULATION_FACTOR       ((WHEEL_CIRCUMFERENCE_CM * 10000) / SPEED_SENSOR_PULSES_PER_REV)

// Timeout for speed measurement (in timer ticks)
// If no pulses are received for this time, speed is considered 0
#define SPEED_SENSOR_TIMEOUT           20000 // 2 seconds with 100µs tick

//-----------------------------
// Function Prototypes
//-----------------------------

// Initialization
void HAL_SPEED_SENSOR_Init(Speed_Sensor_t* sensor);

// Get the current speed
float HAL_SPEED_SENSOR_GetSpeed(Speed_Sensor_t* sensor);

// Check if speed is updated
uint8_t HAL_SPEED_SENSOR_IsUpdated(Speed_Sensor_t* sensor);

// Reset speed measurement
void HAL_SPEED_SENSOR_Reset(Speed_Sensor_t* sensor);

// EXTI callback for speed sensor pulses
void HAL_SPEED_SENSOR_PulseCallback(Speed_Sensor_t* sensor);

// Update speed calculation (call periodically)
void HAL_SPEED_SENSOR_Update(Speed_Sensor_t* sensor);

#endif /* HAL_INC_SPEED_SENSOR_H_ */ 
