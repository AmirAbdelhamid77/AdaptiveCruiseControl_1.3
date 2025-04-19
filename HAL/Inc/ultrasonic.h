/*
 * ultrasonic.h
 *
 * Created on: Someday
 * Author: AMiR Abdelhamid
 */

#ifndef HAL_INC_ULTRASONIC_H_
#define HAL_INC_ULTRASONIC_H_

//-----------------------------
// Includes
//-----------------------------
#include <stm32F103C8T6_pin_mapping.h>
#include <stm32F103C8T6_timer_driver.h>
#include "stm32f103x6.h"
#include "stm32_F103C8T6_gpio_driver.h"
#include "stm32_F103C8T6_EXTI_driver.h"

//-----------------------------
// type definitions (structures)
//-----------------------------

typedef enum {
    ULTRASONIC_STATE_IDLE,
    ULTRASONIC_STATE_TRIGGERING,
    ULTRASONIC_STATE_WAITING_ECHO,
    ULTRASONIC_STATE_ECHO_RECEIVED,
    ULTRASONIC_STATE_TIMEOUT
} Ultrasonic_State_t;

typedef struct {
    // Ultrasonic sensor configuration
    GPIO_TypedDef* TrigPort;
    uint16_t TrigPin;
    GPIO_TypedDef* EchoPort;
    uint16_t EchoPin;
    EXTI_AFIO_MAPPING_t EXTI_Mapping;
    
    // Callback function for when measurement is complete
    void (*Callback)(void);
    
    // State variables
    Ultrasonic_State_t State;
    uint32_t StartTime;
    uint32_t EndTime;
    uint32_t EchoTime;
    uint32_t Distance;  // Distance in mm
    uint8_t IsReady;    // Flag to indicate if a new measurement is ready
} Ultrasonic_t;

//-----------------------------
// Macros and Constants
//-----------------------------

// Speed of sound in air is 343 m/s or 0.343 mm/µs
// For calculating distance: 
// distance = (time * 0.343) / 2
// = time * 0.1715 mm
// We can use the integer approximation: distance = time * 172 / 1000 (mm)
#define ULTRASONIC_SOUND_FACTOR   172
#define ULTRASONIC_SOUND_DIVISOR  1000

#define ULTRASONIC_MAX_DISTANCE   4000  // Maximum measurable distance in mm (4m)
#define ULTRASONIC_MIN_DISTANCE   20    // Minimum measurable distance in mm (2cm)

#define ULTRASONIC_TRIGGER_TIME   10    // Trigger pulse duration in µs

#define ULTRASONIC_TIMEOUT        30000 // 30ms timeout (in µs)

//-----------------------------
// Function Prototypes
//-----------------------------

// Initialization
void HAL_ULTRASONIC_Init(Ultrasonic_t* ultrasonic);

// Trigger a new measurement (send the trigger pulse)
void HAL_ULTRASONIC_Trigger(Ultrasonic_t* ultrasonic);

// Start a new measurement
void HAL_ULTRASONIC_StartMeasurement(Ultrasonic_t* ultrasonic);

// Get the last measured distance
uint32_t HAL_ULTRASONIC_GetDistance(Ultrasonic_t* ultrasonic);

// Check if a new measurement is ready
uint8_t HAL_ULTRASONIC_IsReady(Ultrasonic_t* ultrasonic);

// Timeout handling
void HAL_ULTRASONIC_TimeoutCallback(Ultrasonic_t* ultrasonic);

#endif /* HAL_INC_ULTRASONIC_H_ */ 
