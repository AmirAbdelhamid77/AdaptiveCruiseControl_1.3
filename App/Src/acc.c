/*
 * acc.c - Adaptive Cruise Control
 * Auther: AMiR Abdelhamid
 
 */

#include "acc.h"

// System timer for scheduling control updates
static TIM_TimeBase_Config_t System_Timer_Config = {
    .TIMx = SYSTEM_TIMER,
    .Prescaler = SYSTEM_TIMER_PRESCALER,
    .CounterMode = TIM_COUNTERMODE_UP,
    .Period = SYSTEM_TIMER_PERIOD,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .RepetitionCounter = 0
};

// Static pointer to ACC instance for timer callback
static ACC_t* ACC_Instance = NULL;

/**
 * @brief Timer update callback for periodic control updates
 */
static void ACC_TimerCallback(void) {
    if (ACC_Instance != NULL && ACC_Instance->IsInitialized) {
        ACC_Update(ACC_Instance);
    }
}

/**
 * @brief Initialize the ACC system
 * @param acc: Pointer to ACC structure
 */
void ACC_Init(ACC_t* acc) {
    // Save global instance for callback
    ACC_Instance = acc;
    
    // Initialize system timer for periodic updates
    MCAL_TIM_Base_Init(&System_Timer_Config);
    MCAL_TIM_EnableIT(SYSTEM_TIMER, TIM_IT_UPDATE);
    MCAL_TIM_SetUpdateCallback(SYSTEM_TIMER, ACC_TimerCallback);
    
    // Set default parameters
    acc->TargetSpeed = ACC_DEFAULT_TARGET_SPEED;
    acc->SafeDistance = ACC_DEFAULT_SAFE_DISTANCE;
    acc->Kp = ACC_DEFAULT_KP;
    acc->Ki = ACC_DEFAULT_KI;
    acc->Kd = ACC_DEFAULT_KD;
    
    // Initialize control parameters
    acc->SpeedError = 0.0f;
    acc->SpeedErrorIntegral = 0.0f;
    acc->SpeedErrorDerivative = 0.0f;
    acc->LastSpeedError = 0.0f;
    
    // Set initial state
    acc->State = ACC_STATE_IDLE;
    acc->IsInitialized = 1;
}

/**
 * @brief Set the target speed for the ACC system
 * @param acc: Pointer to ACC structure
 * @param targetSpeed: Target speed in cm/s
 */
void ACC_SetTargetSpeed(ACC_t* acc, float targetSpeed) {
    // Update target speed
    acc->TargetSpeed = targetSpeed;
    
    // Reset control parameters
    acc->SpeedErrorIntegral = 0.0f;
}

/**
 * @brief Set the safe distance for the ACC system
 * @param acc: Pointer to ACC structure
 * @param safeDistance: Safe distance in mm
 */
void ACC_SetSafeDistance(ACC_t* acc, uint32_t safeDistance) {
    // Ensure minimum safe distance
    if (safeDistance < ACC_MIN_SAFE_DISTANCE) {
        safeDistance = ACC_MIN_SAFE_DISTANCE;
    }
    
    acc->SafeDistance = safeDistance;
}

/**
 * @brief Set the PID control constants
 * @param acc: Pointer to ACC structure
 * @param kp: Proportional gain
 * @param ki: Integral gain
 * @param kd: Derivative gain
 */
void ACC_SetPIDConstants(ACC_t* acc, float kp, float ki, float kd) {
    acc->Kp = kp;
    acc->Ki = ki;
    acc->Kd = kd;
    
    // Reset control parameters
    acc->SpeedErrorIntegral = 0.0f;
}

/**
 * @brief Start the ACC system
 * @param acc: Pointer to ACC structure
 */
void ACC_Start(ACC_t* acc) {
    // Reset control parameters
    acc->SpeedErrorIntegral = 0.0f;
    acc->SpeedErrorDerivative = 0.0f;
    acc->LastSpeedError = 0.0f;
    
    // Start the timer for control updates
    MCAL_TIM_Base_Start(SYSTEM_TIMER);
    
    // Set state to active
    acc->State = ACC_STATE_ACTIVE;
}

/**
 * @brief Stop the ACC system
 * @param acc: Pointer to ACC structure
 */
void ACC_Stop(ACC_t* acc) {
    // Stop motors
    HAL_MOTOR_SetDirection(acc->FrontLeftMotor, MOTOR_DIR_STOP);
    HAL_MOTOR_SetDirection(acc->FrontRightMotor, MOTOR_DIR_STOP);
    HAL_MOTOR_SetDirection(acc->RearLeftMotor, MOTOR_DIR_STOP);
    HAL_MOTOR_SetDirection(acc->RearRightMotor, MOTOR_DIR_STOP);
    
    HAL_MOTOR_Update(acc->FrontLeftMotor);
    HAL_MOTOR_Update(acc->FrontRightMotor);
    HAL_MOTOR_Update(acc->RearLeftMotor);
    HAL_MOTOR_Update(acc->RearRightMotor);
    
    // Stop the timer
    MCAL_TIM_Base_Stop(SYSTEM_TIMER);
    
    // Set state to idle
    acc->State = ACC_STATE_IDLE;
}

/**
 * @brief Emergency stop the ACC system
 * @param acc: Pointer to ACC structure
 */
void ACC_EmergencyStop(ACC_t* acc) {
    // Stop all motors immediately
    HAL_MOTOR_StopAll();
    
    // Set state to emergency stop
    acc->State = ACC_STATE_EMERGENCY_STOP;
}

/**
 * @brief Get the current vehicle speed
 * @param acc: Pointer to ACC structure
 * @return Current speed in cm/s (average of left and right wheels)
 */
float ACC_GetCurrentSpeed(ACC_t* acc) {
    // Calculate average speed from both sensors
    float leftSpeed = HAL_SPEED_SENSOR_GetSpeed(acc->LeftSpeedSensor);
    float rightSpeed = HAL_SPEED_SENSOR_GetSpeed(acc->RightSpeedSensor);
    
    return (leftSpeed + rightSpeed) / 2.0f;
}

/**
 * @brief Get the distance to the front obstacle
 * @param acc: Pointer to ACC structure
 * @return Distance in mm
 */
uint32_t ACC_GetFrontDistance(ACC_t* acc) {
    return HAL_ULTRASONIC_GetDistance(acc->FrontUltrasonic);
}

/**
 * @brief Check if an obstacle is detected
 * @param acc: Pointer to ACC structure
 * @return 1 if obstacle detected, 0 otherwise
 */
uint8_t ACC_IsObstacleDetected(ACC_t* acc) {
    uint32_t frontDistance = HAL_ULTRASONIC_GetDistance(acc->FrontUltrasonic);
    
    return (frontDistance < acc->SafeDistance) ? 1 : 0;
}

/**
 * @brief Main control update function (called periodically)
 * @param acc: Pointer to ACC structure
 */
void ACC_Update(ACC_t* acc) {
    // Only update if in active state
    if (acc->State != ACC_STATE_ACTIVE) {
        return;
    }
    
    // Trigger ultrasonic measurements
    HAL_ULTRASONIC_StartMeasurement(acc->FrontUltrasonic);
    
    // Update speed sensors
    HAL_SPEED_SENSOR_Update(acc->LeftSpeedSensor);
    HAL_SPEED_SENSOR_Update(acc->RightSpeedSensor);
    
    // Get current measurements
    float currentSpeed = ACC_GetCurrentSpeed(acc);
    uint32_t frontDistance = ACC_GetFrontDistance(acc);
    
    // Check for emergency stop condition
    if (frontDistance < ACC_EMERGENCY_BRAKE_DISTANCE) {
        ACC_EmergencyStop(acc);
        return;
    }
    
    // Calculate adjusted target speed based on obstacle distance
    float adjustedTargetSpeed = acc->TargetSpeed;
    
    // If obstacle detected, reduce speed proportionally
    if (frontDistance < acc->SafeDistance) {
        // Calculate adjustment factor (0-1 based on how close we are to the minimum distance)
        float distanceFactor = (float)(frontDistance - ACC_EMERGENCY_BRAKE_DISTANCE) / 
                              (float)(acc->SafeDistance - ACC_EMERGENCY_BRAKE_DISTANCE);
        
        // Limit factor to 0-1 range
        if (distanceFactor < 0.0f) distanceFactor = 0.0f;
        if (distanceFactor > 1.0f) distanceFactor = 1.0f;
        
        // Adjust target speed (reduce to 0 as we approach emergency distance)
        adjustedTargetSpeed *= distanceFactor;
    }
    
    // Calculate PID control values
    acc->SpeedError = adjustedTargetSpeed - currentSpeed;
    
    // Accumulate integral with anti-windup
    acc->SpeedErrorIntegral += acc->SpeedError * (ACC_CONTROL_UPDATE_RATE_MS / 1000.0f);
    
    // Limit integral term to prevent windup
    float maxIntegral = 50.0f; // Adjust as needed
    if (acc->SpeedErrorIntegral > maxIntegral) acc->SpeedErrorIntegral = maxIntegral;
    if (acc->SpeedErrorIntegral < -maxIntegral) acc->SpeedErrorIntegral = -maxIntegral;
    
    // Calculate derivative term
    acc->SpeedErrorDerivative = (acc->SpeedError - acc->LastSpeedError) / (ACC_CONTROL_UPDATE_RATE_MS / 1000.0f);
    acc->LastSpeedError = acc->SpeedError;
    
    // Calculate PID output
    float pidOutput = (acc->Kp * acc->SpeedError) + 
                      (acc->Ki * acc->SpeedErrorIntegral) + 
                      (acc->Kd * acc->SpeedErrorDerivative);
    
    // Convert PID output to motor control
    // Positive output means accelerate, negative means decelerate
    if (pidOutput > 0) {
        // Accelerate
        HAL_MOTOR_SetDirection(acc->FrontLeftMotor, MOTOR_DIR_FORWARD);
        HAL_MOTOR_SetDirection(acc->FrontRightMotor, MOTOR_DIR_FORWARD);
        HAL_MOTOR_SetDirection(acc->RearLeftMotor, MOTOR_DIR_FORWARD);
        HAL_MOTOR_SetDirection(acc->RearRightMotor, MOTOR_DIR_FORWARD);
        
        // Calculate speed (0-100%)
        uint8_t motorSpeed = (uint8_t)(pidOutput > 100.0f ? 100.0f : pidOutput);
        
        HAL_MOTOR_SetSpeed(acc->FrontLeftMotor, motorSpeed);
        HAL_MOTOR_SetSpeed(acc->FrontRightMotor, motorSpeed);
        HAL_MOTOR_SetSpeed(acc->RearLeftMotor, motorSpeed);
        HAL_MOTOR_SetSpeed(acc->RearRightMotor, motorSpeed);
    } else {
        // Decelerate or keep current speed
        if (pidOutput < -10.0f) {
            // Braking required (reverse motors briefly)
            HAL_MOTOR_SetDirection(acc->FrontLeftMotor, MOTOR_DIR_BACKWARD);
            HAL_MOTOR_SetDirection(acc->FrontRightMotor, MOTOR_DIR_BACKWARD);
            HAL_MOTOR_SetDirection(acc->RearLeftMotor, MOTOR_DIR_BACKWARD);
            HAL_MOTOR_SetDirection(acc->RearRightMotor, MOTOR_DIR_BACKWARD);
            
            // Calculate braking force (0-100%)
            uint8_t brakeForce = (uint8_t)(-pidOutput > 100.0f ? 100.0f : -pidOutput);
            
            HAL_MOTOR_SetSpeed(acc->FrontLeftMotor, brakeForce);
            HAL_MOTOR_SetSpeed(acc->FrontRightMotor, brakeForce);
            HAL_MOTOR_SetSpeed(acc->RearLeftMotor, brakeForce);
            HAL_MOTOR_SetSpeed(acc->RearRightMotor, brakeForce);
        } else {
            // Just coast (stop actively driving)
            HAL_MOTOR_SetDirection(acc->FrontLeftMotor, MOTOR_DIR_STOP);
            HAL_MOTOR_SetDirection(acc->FrontRightMotor, MOTOR_DIR_STOP);
            HAL_MOTOR_SetDirection(acc->RearLeftMotor, MOTOR_DIR_STOP);
            HAL_MOTOR_SetDirection(acc->RearRightMotor, MOTOR_DIR_STOP);
        }
    }
    
    // Apply motor updates
    HAL_MOTOR_Update(acc->FrontLeftMotor);
    HAL_MOTOR_Update(acc->FrontRightMotor);
    HAL_MOTOR_Update(acc->RearLeftMotor);
    HAL_MOTOR_Update(acc->RearRightMotor);
}

/**
 * @brief Set motor speeds based on percentage of maximum
 * @param acc: Pointer to ACC structure
 * @param percentage: Speed percentage (0-100)
 */
void ACC_SetSpeedPercentage(ACC_t* acc_instance, uint8_t percentage) {
    // Ensure percentage is within valid range
    if (percentage > 100) {
        percentage = 100;
    }
    
    // Set speed for all motors
    HAL_MOTOR_SetSpeed(acc_instance->FrontLeftMotor, percentage);
    HAL_MOTOR_SetSpeed(acc_instance->FrontRightMotor, percentage);
    HAL_MOTOR_SetSpeed(acc_instance->RearLeftMotor, percentage);
    HAL_MOTOR_SetSpeed(acc_instance->RearRightMotor, percentage);
    
    // Update all motors
    HAL_MOTOR_Update(acc_instance->FrontLeftMotor);
    HAL_MOTOR_Update(acc_instance->FrontRightMotor);
    HAL_MOTOR_Update(acc_instance->RearLeftMotor);
    HAL_MOTOR_Update(acc_instance->RearRightMotor);
}

/**
 * @brief Adjust steering by changing differential between left and right motors
 * @param acc: Pointer to ACC structure
 * @param adjustment: Steering adjustment (-100 to 100)
 *                    0 = straight, negative = left, positive = right
 */
void ACC_AdjustSteering(ACC_t* acc_instance, int8_t adjustment) {
    // Limit adjustment to valid range
    if (adjustment < -100) adjustment = -100;
    if (adjustment > 100) adjustment = 100;
    
    // Get current base speed from motors
    uint8_t baseSpeed = acc_instance->FrontLeftMotor->Speed;
    
    // Calculate left and right adjustments based on turning direction
    uint8_t leftSpeed, rightSpeed;
    
    if (adjustment < 0) {
        // Turn left: slow down left wheels
        leftSpeed = baseSpeed * (100 + adjustment) / 100;
        rightSpeed = baseSpeed;
    } else if (adjustment > 0) {
        // Turn right: slow down right wheels
        leftSpeed = baseSpeed;
        rightSpeed = baseSpeed * (100 - adjustment) / 100;
    } else {
        // No adjustment, keep speeds equal
        leftSpeed = baseSpeed;
        rightSpeed = baseSpeed;
    }
    
    // Apply the adjusted speeds
    HAL_MOTOR_SetSpeed(acc_instance->FrontLeftMotor, leftSpeed);
    HAL_MOTOR_SetSpeed(acc_instance->RearLeftMotor, leftSpeed);
    HAL_MOTOR_SetSpeed(acc_instance->FrontRightMotor, rightSpeed);
    HAL_MOTOR_SetSpeed(acc_instance->RearRightMotor, rightSpeed);
    
    // Update all motors
    HAL_MOTOR_Update(acc_instance->FrontLeftMotor);
    HAL_MOTOR_Update(acc_instance->FrontRightMotor);
    HAL_MOTOR_Update(acc_instance->RearLeftMotor);
    HAL_MOTOR_Update(acc_instance->RearRightMotor);
}

/**
 * @brief Disable reverse motion
 * @param acc: Pointer to ACC structure
 */
void ACC_DisableReverse(ACC_t* acc_instance) {
    // Set direction to forward or stop only
    if (acc_instance->FrontLeftMotor->Direction == MOTOR_DIR_BACKWARD) {
        HAL_MOTOR_SetDirection(acc_instance->FrontLeftMotor, MOTOR_DIR_STOP);
        HAL_MOTOR_SetDirection(acc_instance->FrontRightMotor, MOTOR_DIR_STOP);
        HAL_MOTOR_SetDirection(acc_instance->RearLeftMotor, MOTOR_DIR_STOP);
        HAL_MOTOR_SetDirection(acc_instance->RearRightMotor, MOTOR_DIR_STOP);
    }
    
    // Set flag to prevent reverse
    acc_instance->ReverseEnabled = 0;
    
    // Update all motors
    HAL_MOTOR_Update(acc_instance->FrontLeftMotor);
    HAL_MOTOR_Update(acc_instance->FrontRightMotor);
    HAL_MOTOR_Update(acc_instance->RearLeftMotor);
    HAL_MOTOR_Update(acc_instance->RearRightMotor);
}

/**
 * @brief Enable reverse motion
 * @param acc: Pointer to ACC structure
 */
void ACC_EnableReverse(ACC_t* acc_instance) {
    // Set flag to allow reverse
    acc_instance->ReverseEnabled = 1;
} 
