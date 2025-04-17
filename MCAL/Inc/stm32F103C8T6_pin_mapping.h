/*
 * stm32_F103C6_pin_mapping.h
 *
 *      Author: AMiR
 */

#ifndef INC_STM32_F103C6_PIN_MAPPING_H_
#define INC_STM32_F103C6_PIN_MAPPING_H_

//-----------------------------
// Includes
//-----------------------------
#include "stm32f103x6.h"
#include "stm32_F103C6_EXTI_driver.h"

//===========================================================================
// Pin Mapping Configuration
//===========================================================================

//-----------------------------
// Motor Control Pin Mapping (H-Bridge)
//-----------------------------

// Front Left Motor (Motor 1)
#define MOTOR_FL_PORT_FWD           GPIOA
#define MOTOR_FL_PIN_FWD            GPIO_PIN_0  // PA0
#define MOTOR_FL_PORT_BWD           GPIOA
#define MOTOR_FL_PIN_BWD            GPIO_PIN_1  // PA1
#define MOTOR_FL_PWM_PORT           GPIOA
#define MOTOR_FL_PWM_PIN            GPIO_PIN_8  // PA8 (TIM1_CH1)

// Front Right Motor (Motor 2)
#define MOTOR_FR_PORT_FWD           GPIOA
#define MOTOR_FR_PIN_FWD            GPIO_PIN_2  // PA2
#define MOTOR_FR_PORT_BWD           GPIOA
#define MOTOR_FR_PIN_BWD            GPIO_PIN_3  // PA3
#define MOTOR_FR_PWM_PORT           GPIOA
#define MOTOR_FR_PWM_PIN            GPIO_PIN_9  // PA9 (TIM1_CH2)

// Rear Left Motor (Motor 3)
#define MOTOR_RL_PORT_FWD           GPIOB
#define MOTOR_RL_PIN_FWD            GPIO_PIN_0  // PB0
#define MOTOR_RL_PORT_BWD           GPIOB
#define MOTOR_RL_PIN_BWD            GPIO_PIN_1  // PB1
#define MOTOR_RL_PWM_PORT           GPIOA
#define MOTOR_RL_PWM_PIN            GPIO_PIN_10 // PA10 (TIM1_CH3)

// Rear Right Motor (Motor 4)
#define MOTOR_RR_PORT_FWD           GPIOB
#define MOTOR_RR_PIN_FWD            GPIO_PIN_10 // PB10
#define MOTOR_RR_PORT_BWD           GPIOB
#define MOTOR_RR_PIN_BWD            GPIO_PIN_11 // PB11
#define MOTOR_RR_PWM_PORT           GPIOA
#define MOTOR_RR_PWM_PIN            GPIO_PIN_11 // PA11 (TIM1_CH4)

//-----------------------------
// Ultrasonic Sensors (HC-SR04) Pin Mapping
//-----------------------------

// Front Ultrasonic
#define ULTRASONIC_FRONT_TRIG_PORT  GPIOB
#define ULTRASONIC_FRONT_TRIG_PIN   GPIO_PIN_12 // PB12
#define ULTRASONIC_FRONT_ECHO_PORT  GPIOB
#define ULTRASONIC_FRONT_ECHO_PIN   GPIO_PIN_13 // PB13 (EXTI13)
#define ULTRASONIC_FRONT_EXTI       EXTI13PB13

// Rear Ultrasonic
#define ULTRASONIC_REAR_TRIG_PORT   GPIOB
#define ULTRASONIC_REAR_TRIG_PIN    GPIO_PIN_14 // PB14
#define ULTRASONIC_REAR_ECHO_PORT   GPIOB
#define ULTRASONIC_REAR_ECHO_PIN    GPIO_PIN_15 // PB15 (EXTI15)
#define ULTRASONIC_REAR_EXTI        EXTI15PB15

// Left Ultrasonic
#define ULTRASONIC_LEFT_TRIG_PORT   GPIOB
#define ULTRASONIC_LEFT_TRIG_PIN    GPIO_PIN_5  // PB5
#define ULTRASONIC_LEFT_ECHO_PORT   GPIOC
#define ULTRASONIC_LEFT_ECHO_PIN    GPIO_PIN_14 // PC14 (EXTI14)
#define ULTRASONIC_LEFT_EXTI        EXTI14PC14
// Right Ultrasonic
#define ULTRASONIC_RIGHT_TRIG_PORT  GPIOC
#define ULTRASONIC_RIGHT_TRIG_PIN   GPIO_PIN_15 // PC15
#define ULTRASONIC_RIGHT_ECHO_PORT  GPIOA
#define ULTRASONIC_RIGHT_ECHO_PIN   GPIO_PIN_12 // PA12 (EXTI12)
#define ULTRASONIC_RIGHT_EXTI       EXTI12PA12  

//-----------------------------
// Speed Sensors Pin Mapping
//-----------------------------

// Left Speed Sensor 
#define SPEED_SENSOR_LEFT_PORT      GPIOA
#define SPEED_SENSOR_LEFT_PIN       GPIO_PIN_4  
#define SPEED_SENSOR_LEFT_EXTI      EXTI4PA4    

// Right Speed Sensor
#define SPEED_SENSOR_RIGHT_PORT     GPIOB
#define SPEED_SENSOR_RIGHT_PIN      GPIO_PIN_3  // PB3 (EXTI3)
#define SPEED_SENSOR_RIGHT_EXTI     EXTI3PB3    
//-----------------------------
// LED Indicators
//-----------------------------
#define LED_STATUS_PORT             GPIOC
#define LED_STATUS_PIN              GPIO_PIN_13 // PC13 (Built-in LED)

//===========================================================================
// Timer Configuration 
//===========================================================================

// Timer Clock Sources and Frequencies
// #define SYSTEM_CLOCK_FREQ           8000000
#define APB1_TIMER_CLOCK_FREQ       8000000     // Assuming APB1 prescaler is 1
#define APB2_TIMER_CLOCK_FREQ       8000000     // Assuming APB2 prescaler is 1

//-----------------------------
// TIM1 - Advanced Timer for Motor PWM
//-----------------------------
#define MOTOR_PWM_TIMER             TIM1
#define MOTOR_PWM_TIMER_CLOCK       APB2_TIMER_CLOCK_FREQ
#define MOTOR_PWM_FREQ              20000       // Target 20 kHz PWM frequency
#define MOTOR_PWM_PRESCALER         0           // (PSC = 0)
#define MOTOR_PWM_ARR               399         // (ARR = 399) Period = ARR + 1 = 400 ticks

// TIM1 Channels for Motors
#define MOTOR_FL_PWM_CHANNEL        1           // TIM1_CH1 (PA8)
#define MOTOR_FR_PWM_CHANNEL        2           // TIM1_CH2 (PA9)
#define MOTOR_RL_PWM_CHANNEL        3           // TIM1_CH3 (PA10)
#define MOTOR_RR_PWM_CHANNEL        4           // TIM1_CH4 (PA11)

//-----------------------------
// TIM2 - For Ultrasonic Echo Measurement
//-----------------------------
#define ULTRASONIC_TIMER            TIM2
#define ULTRASONIC_TIMER_CLOCK      APB1_TIMER_CLOCK_FREQ
#define ULTRASONIC_TIMER_PRESCALER  7           // Prescaler = 7 for 1us tick
#define ULTRASONIC_TIMER_PERIOD     0xFFFF      // Maximum period (ARR)
#define ULTRASONIC_TIMEOUT_US       30000       // Timeout in microseconds
#define ULTRASONIC_TIMEOUT_TICKS    ULTRASONIC_TIMEOUT_US // Timeout in 1us timer ticks
#define ULTRASONIC_TRIGGER_TICKS    10          // 10µs trigger pulse (in CPU clocks or a separate delay)

//-----------------------------
// TIM3 - For Speed Sensor Measurements
//-----------------------------
#define SPEED_SENSOR_TIMER          TIM3
#define SPEED_SENSOR_TIMER_CLOCK    APB1_TIMER_CLOCK_FREQ
#define SPEED_SENSOR_PRESCALER      799         // Prescaler = 799 for 100us tick
#define SPEED_SENSOR_PERIOD         999       // ARR: 1 millisecond measurement window (1000 * 100us = 100ms)

//-----------------------------
// TIM4 - For General System Timing
//-----------------------------
#define SYSTEM_TIMER                TIM4
#define SYSTEM_TIMER_CLOCK          APB1_TIMER_CLOCK_FREQ
#define SYSTEM_TIMER_PRESCALER      799         // Prescaler = 799 for 100us tick
#define SYSTEM_TIMER_PERIOD         499         // ARR = 499 => Period = 500 ticks
#define CONTROL_UPDATE_RATE_MS      50          // 50ms control loop rate (matches timer)

//-----------------------------
// Timer Configuration Types
//-----------------------------
typedef struct {
    TIM_TypeDef* Timer;
    uint16_t Prescaler;
    uint16_t Period;
    uint8_t ClockDivision;
    uint8_t CounterMode;
    uint8_t RepetitionCounter;
} Timer_Config_t;

typedef struct {
    TIM_TypeDef* Timer;
    uint8_t Channel;
    uint16_t Pulse;
    uint8_t OCMode;
    uint8_t OCPolarity;
    uint8_t OCNPolarity;
    uint8_t OCIdleState;
    uint8_t OCNIdleState;
} PWM_Channel_Config_t;

//-----------------------------
// Timer IRQ Priorities 
//-----------------------------
#define MOTOR_PWM_IRQ_PRIORITY      3           // Lowest priority (0-15, 0 is highest)
#define ULTRASONIC_TIMER_IRQ_PRIORITY 1         // High priority
#define SPEED_SENSOR_IRQ_PRIORITY   2           // Medium priority
#define SYSTEM_TIMER_IRQ_PRIORITY   0           // Highest priority

//===========================================================================
// Configuration Structures
//===========================================================================

// Motor Configuration Structure
typedef struct {
    GPIO_TypedDef* DirectionPort;
    uint16_t DirectionPin_Forward;
    uint16_t DirectionPin_Backward;
    GPIO_TypedDef* PWMPort;
    uint16_t PWMPin;
    uint8_t Direction;  // Using MOTOR_DIRECTION_* defines
    uint8_t Speed;      // 0-100 percentage
    uint8_t Enabled;    // 0 or 1
} Motor_Config_t;

// Add these missing direction defines
#define MOTOR_DIRECTION_STOP      0
#define MOTOR_DIRECTION_FORWARD   1
#define MOTOR_DIRECTION_BACKWARD  2

// Motor Configuration Structure
#define MOTOR_CONFIG_ARRAY { \
    { \
        .DirectionPort = MOTOR_FL_PORT_FWD, \
        .DirectionPin_Forward = MOTOR_FL_PIN_FWD, \
        .DirectionPin_Backward = MOTOR_FL_PIN_BWD, \
        .PWMPort = MOTOR_FL_PWM_PORT, \
        .PWMPin = MOTOR_FL_PWM_PIN, \
        .Direction = MOTOR_DIRECTION_STOP, \
        .Speed = 0, \
        .Enabled = 0 \
    }, \
    { \
        .DirectionPort = MOTOR_FR_PORT_FWD, \
        .DirectionPin_Forward = MOTOR_FR_PIN_FWD, \
        .DirectionPin_Backward = MOTOR_FR_PIN_BWD, \
        .PWMPort = MOTOR_FR_PWM_PORT, \
        .PWMPin = MOTOR_FR_PWM_PIN, \
        .Direction = MOTOR_DIRECTION_STOP, \
        .Speed = 0, \
        .Enabled = 0 \
    }, \
    { \
        .DirectionPort = MOTOR_RL_PORT_FWD, \
        .DirectionPin_Forward = MOTOR_RL_PIN_FWD, \
        .DirectionPin_Backward = MOTOR_RL_PIN_BWD, \
        .PWMPort = MOTOR_RL_PWM_PORT, \
        .PWMPin = MOTOR_RL_PWM_PIN, \
        .Direction = MOTOR_DIRECTION_STOP, \
        .Speed = 0, \
        .Enabled = 0 \
    }, \
    { \
        .DirectionPort = MOTOR_RR_PORT_FWD, \
        .DirectionPin_Forward = MOTOR_RR_PIN_FWD, \
        .DirectionPin_Backward = MOTOR_RR_PIN_BWD, \
        .PWMPort = MOTOR_RR_PWM_PORT, \
        .PWMPin = MOTOR_RR_PWM_PIN, \
        .Direction = MOTOR_DIRECTION_STOP, \
        .Speed = 0, \
        .Enabled = 0 \
    } \
}

// Ultrasonic Configuration Structure
typedef struct {
    GPIO_TypedDef* TrigPort;
    uint16_t TrigPin;
    GPIO_TypedDef* EchoPort;
    uint16_t EchoPin;
    EXTI_AFIO_MAPPING_t EXTI_Mapping;  // Added for EXTI integration
} ULTRASONIC_Config_t;

#define ULTRASONIC_CONFIG_ARRAY { \
    { \
        .TrigPort = ULTRASONIC_FRONT_TRIG_PORT, \
        .TrigPin = ULTRASONIC_FRONT_TRIG_PIN, \
        .EchoPort = ULTRASONIC_FRONT_ECHO_PORT, \
        .EchoPin = ULTRASONIC_FRONT_ECHO_PIN, \
        .EXTI_Mapping = ULTRASONIC_FRONT_EXTI \
    }, \
    { \
        .TrigPort = ULTRASONIC_REAR_TRIG_PORT, \
        .TrigPin = ULTRASONIC_REAR_TRIG_PIN, \
        .EchoPort = ULTRASONIC_REAR_ECHO_PORT, \
        .EchoPin = ULTRASONIC_REAR_ECHO_PIN, \
        .EXTI_Mapping = ULTRASONIC_REAR_EXTI \
    }, \
    { \
        .TrigPort = ULTRASONIC_LEFT_TRIG_PORT, \
        .TrigPin = ULTRASONIC_LEFT_TRIG_PIN, \
        .EchoPort = ULTRASONIC_LEFT_ECHO_PORT, \
        .EchoPin = ULTRASONIC_LEFT_ECHO_PIN, \
        .EXTI_Mapping = ULTRASONIC_LEFT_EXTI \
    }, \
    { \
        .TrigPort = ULTRASONIC_RIGHT_TRIG_PORT, \
        .TrigPin = ULTRASONIC_RIGHT_TRIG_PIN, \
        .EchoPort = ULTRASONIC_RIGHT_ECHO_PORT, \
        .EchoPin = ULTRASONIC_RIGHT_ECHO_PIN, \
        .EXTI_Mapping = ULTRASONIC_RIGHT_EXTI \
    } \
}

// Speed Sensor Configuration Structure
typedef struct {
    GPIO_TypedDef* Port;
    uint16_t Pin;
    EXTI_AFIO_MAPPING_t EXTI_Mapping;  // Added for EXTI integration
} SPEED_SENSOR_Config_t;

#define SPEED_SENSOR_CONFIG_ARRAY { \
    { \
        .Port = SPEED_SENSOR_LEFT_PORT, \
        .Pin = SPEED_SENSOR_LEFT_PIN, \
        .EXTI_Mapping = SPEED_SENSOR_LEFT_EXTI \
    }, \
    { \
        .Port = SPEED_SENSOR_RIGHT_PORT, \
        .Pin = SPEED_SENSOR_RIGHT_PIN, \
        .EXTI_Mapping = SPEED_SENSOR_RIGHT_EXTI \
    } \
}

// Sensor ID Enumerations
typedef enum {
    ULTRASONIC_FRONT = 0,
    ULTRASONIC_REAR,
    ULTRASONIC_LEFT,
    ULTRASONIC_RIGHT,
    ULTRASONIC_COUNT
} Ultrasonic_ID_t;

typedef enum {
    SPEED_SENSOR_LEFT = 0,
    SPEED_SENSOR_RIGHT,
    SPEED_SENSOR_COUNT
} Speed_Sensor_ID_t;

typedef enum {
    MOTOR_FRONT_LEFT = 0,
    MOTOR_FRONT_RIGHT,
    MOTOR_REAR_LEFT,
    MOTOR_REAR_RIGHT,
    MOTOR_COUNT
} Motor_ID_t;

#endif /* INC_STM32_F103C6_PIN_MAPPING_H_ */ 
