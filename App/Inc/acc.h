/*
 * acc.h - Adaptive Cruise Control
 *
 
 */

#ifndef APP_INC_ACC_H_
#define APP_INC_ACC_H_

//-----------------------------
// Includes
//-----------------------------
#include <stm32F103C8T6_timer_driver.h>
#include "stm32f103x6.h"
#include "ultrasonic.h"
#include "speed_sensor.h"
#include "motor.h"

//-----------------------------
// User type definitions (structures)
//-----------------------------

typedef enum {
    ACC_STATE_IDLE,
    ACC_STATE_ACTIVE,
    ACC_STATE_EMERGENCY_STOP,
    ACC_STATE_ERROR
} ACC_State_t;

typedef struct {
    // Sensors
    Ultrasonic_t* FrontUltrasonic;
    Ultrasonic_t* RearUltrasonic;
    Ultrasonic_t* LeftUltrasonic;
    Ultrasonic_t* RightUltrasonic;
    Speed_Sensor_t* LeftSpeedSensor;
    Speed_Sensor_t* RightSpeedSensor;
    
    // Motors
    Motor_t* FrontLeftMotor;
    Motor_t* FrontRightMotor;
    Motor_t* RearLeftMotor;
    Motor_t* RearRightMotor;
    
    // ACC parameters
    float TargetSpeed;               // Target speed in cm/s
    uint32_t SafeDistance;           // Safe distance to maintain in mm
    
    // Control parameters
    float SpeedError;                // Difference between target and actual speed
    float SpeedErrorIntegral;        // Integral of speed error (for PID control)
    float SpeedErrorDerivative;      // Derivative of speed error (for PID control)
    float LastSpeedError;            // Last speed error (for derivative calculation)
    
    // PID constants
    float Kp;                        // Proportional gain
    float Ki;                        // Integral gain
    float Kd;                        // Derivative gain
    
    // ACC state
    ACC_State_t State;
    uint8_t IsInitialized;
    uint8_t ReverseEnabled;          // Flag to enable/disable reverse motion
} ACC_t;

//-----------------------------
// Macros and Constants
//-----------------------------

// ACC safe distances (in mm)
#define ACC_DEFAULT_SAFE_DISTANCE           1000    // Default 1m
#define ACC_MIN_SAFE_DISTANCE               500     // Minimum 0.5m
#define ACC_EMERGENCY_BRAKE_DISTANCE        250     // Emergency brake at 25cm

// Default target speed (in cm/s)
#define ACC_DEFAULT_TARGET_SPEED            50.0f   // 50 cm/s = 0.5 m/s

// PID control default values
#define ACC_DEFAULT_KP                      2.0f
#define ACC_DEFAULT_KI                      0.1f
#define ACC_DEFAULT_KD                      0.5f

// Control update rate 
#define ACC_CONTROL_UPDATE_RATE_MS          50      // 50ms = 20Hz update rate

//-----------------------------
// Function Prototypes
//-----------------------------

// Initialization
void ACC_Init(ACC_t* acc);

// Set target speed
void ACC_SetTargetSpeed(ACC_t* acc, float targetSpeed);

// Set safe distance
void ACC_SetSafeDistance(ACC_t* acc, uint32_t safeDistance);

// Set PID constants
void ACC_SetPIDConstants(ACC_t* acc, float kp, float ki, float kd);

// Main control functions
void ACC_Start(ACC_t* acc);
void ACC_Stop(ACC_t* acc);
void ACC_EmergencyStop(ACC_t* acc);
void ACC_Update(ACC_t* acc);

// Motor control functions
void ACC_SetSpeedPercentage(ACC_t* acc, uint8_t percentage);
void ACC_AdjustSteering(ACC_t* acc, int8_t adjustment);
void ACC_DisableReverse(ACC_t* acc);
void ACC_EnableReverse(ACC_t* acc);

// Helper functions
float ACC_GetCurrentSpeed(ACC_t* acc);
uint32_t ACC_GetFrontDistance(ACC_t* acc);
uint8_t ACC_IsObstacleDetected(ACC_t* acc);

#endif /* APP_INC_ACC_H_ */ 
