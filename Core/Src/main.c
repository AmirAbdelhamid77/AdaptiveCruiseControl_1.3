#include <stm32F103C8T6_pin_mapping.h>
#include <stm32F103C8T6_timer_driver.h>
#include "stm32f103x6.h"
#include "stm32_F103C6_gpio_driver.h"
#include "stm32_F103C6_EXTI_driver.h"
#include "RCC.h"
#include "ultrasonic.h"
#include "speed_sensor.h"
#include "motor.h"
#include "acc.h"

// Constants for calculations
#define SPEED_OF_SOUND_MM_PER_US    0.343   // Speed of sound in mm/us
#define US_TIMER_TICK_US            1       // Timer resolution (from TIM2 config)
#define SPEED_TIMER_INTERVAL_S      1.0     // Interval for speed calculation (from TIM3 config)
#define WHEEL_CIRCUMFERENCE_MM      660     // Example: ~21cm diameter wheel
#define ENCODER_PULSES_PER_REV      20      // Example: 20 pulses per wheel revolution
#define MM_PER_PULSE                (WHEEL_CIRCUMFERENCE_MM / ENCODER_PULSES_PER_REV)

// Sensor instances
Ultrasonic_t frontUltrasonic;
Ultrasonic_t rearUltrasonic;
Ultrasonic_t leftUltrasonic;
Ultrasonic_t rightUltrasonic;

Speed_Sensor_t leftSpeedSensor;
Speed_Sensor_t rightSpeedSensor;

// Add fields to store results from speed sensor timer
volatile uint32_t leftPulseCount = 0;
volatile uint32_t rightPulseCount = 0;

// Motor instances
Motor_t frontLeftMotor;
Motor_t frontRightMotor;
Motor_t rearLeftMotor;
Motor_t rearRightMotor;

// ACC instance
ACC_t acc;

// System timer flag
volatile uint8_t timer_flag = 0;

// Obstacle detection states
typedef enum {
    ZONE_SAFE,           // No obstacles in range
    ZONE_CAUTION,        // Obstacle detected but not immediate danger
    ZONE_WARNING,        // Obstacle getting closer, reduce speed
    ZONE_DANGER,         // Obstacle very close, stop or take protective action
    ZONE_CRITICAL        // Imminent collision, emergency stop
} ObstacleZone_t;

// Obstacle detection thresholds (in mm)
#define OBSTACLE_DIST_CRITICAL     150     // <15cm - emergency stop
#define OBSTACLE_DIST_DANGER       300     // <30cm - significant speed reduction
#define OBSTACLE_DIST_WARNING      600     // <60cm - moderate speed reduction
#define OBSTACLE_DIST_CAUTION      1000    // <1m - slight speed reduction
#define OBSTACLE_DIST_SAFE         1500    // <1.5m - maintain target speed, but be aware

// Obstacle detection structure
typedef struct {
    ObstacleZone_t front;
    ObstacleZone_t rear;
    ObstacleZone_t left;
    ObstacleZone_t right;
} ObstacleDetection_t;

ObstacleDetection_t obstacleState;

// Function Prototypes
void SystemClock_Config(void);
void Initialize_Sensors(void);
void Initialize_Motors(void);
void Initialize_ACC(void);
void Timer_Init(void);
void System_Timer_Callback(void);
void Speed_Timer_Callback(void);
void Trigger_Ultrasonic_Measurements(void);
void Process_Obstacle_Detection(void);
ObstacleZone_t Evaluate_Distance(uint32_t distance);

// Ultrasonic Callbacks
void Front_Ultrasonic_Callback(void);
void Rear_Ultrasonic_Callback(void);
void Left_Ultrasonic_Callback(void);
void Right_Ultrasonic_Callback(void);
static void Generic_Ultrasonic_Callback(Ultrasonic_t* sensor);

// Speed Sensor Callbacks
void Left_Speed_Callback(void);
void Right_Speed_Callback(void);

// Add function declarations at the top, after the existing function prototypes
// Add these lines after all the other existing function prototypes, around line 93
void ACC_SetSpeedPercentage(ACC_t* acc_instance, uint8_t percentage);
void ACC_AdjustSteering(ACC_t* acc_instance, int8_t adjustment);
void ACC_DisableReverse(ACC_t* acc_instance);
void ACC_EnableReverse(ACC_t* acc_instance);

int main(void) {
    // Configure system clock to 8MHz
    SystemClock_Config();
    
    // Initialize peripherals
    Initialize_Sensors();
    Initialize_Motors();
    Initialize_ACC();
    
    // Initialize system timer for regular control updates
    Timer_Init(); // Initializes TIM4 (System Timer)
    
    // Status LED
    GPIO_PinConfig_t ledConfig = {
        .GPIO_PinNumber = LED_STATUS_PIN,
        .GPIO_Mode = GPIO_Mode_Out_push_pull,
        .GPIO_Speed = GPIO_Speed_10MHz
    };
    MCAL_GPIO_Init(LED_STATUS_PORT, &ledConfig);
    MCAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET); // LED off (active low)
    
    // Start ACC system
    ACC_Start(&acc);
    
    // Main loop
    while (1) {
        if (timer_flag) {
            // Reset flag
            timer_flag = 0;
            
            // Toggle status LED to show system is running
            MCAL_GPIO_TogglePin(LED_STATUS_PORT, LED_STATUS_PIN);
            
            // Trigger ultrasonic measurements sequentially (results via EXTI)
            Trigger_Ultrasonic_Measurements();
            
            // Process obstacle detection using the latest sensor data
            // Data is updated asynchronously by EXTI callbacks
            Process_Obstacle_Detection();
            
            // Update ACC system with obstacle information and speed
            ACC_Update(&acc);
        }
        
        // CPU can perform other tasks here
        // Or enter a low-power mode until the next interrupt
    }
}

/**
 * @brief Trigger ultrasonic measurements sequentially
 * Results will be received via the EXTI callbacks
 */
void Trigger_Ultrasonic_Measurements(void) {
    // Trigger front ultrasonic
    HAL_ULTRASONIC_StartMeasurement(&frontUltrasonic);
    
    // Small delay to avoid signal interference between sensors
    for (volatile uint32_t i = 0; i < 10000; i++);
    
    // Trigger rear ultrasonic
    HAL_ULTRASONIC_StartMeasurement(&rearUltrasonic);
    
    // Small delay to avoid signal interference between sensors
    for (volatile uint32_t i = 0; i < 10000; i++);
    
    // Trigger left ultrasonic
    HAL_ULTRASONIC_StartMeasurement(&leftUltrasonic);
    
    // Small delay to avoid signal interference between sensors
    for (volatile uint32_t i = 0; i < 10000; i++);
    
    // Trigger right ultrasonic
    HAL_ULTRASONIC_StartMeasurement(&rightUltrasonic);
}

/**
 * @brief Process obstacle detection using ultrasonic sensor data
 */
void Process_Obstacle_Detection(void) {
    // Evaluate each direction and update obstacle zones
    obstacleState.front = Evaluate_Distance(frontUltrasonic.Distance);
    obstacleState.rear = Evaluate_Distance(rearUltrasonic.Distance);
    obstacleState.left = Evaluate_Distance(leftUltrasonic.Distance);
    obstacleState.right = Evaluate_Distance(rightUltrasonic.Distance);
    
    // Adjust ACC behavior based on obstacle detection
    switch (obstacleState.front) {
        case ZONE_CRITICAL:
            // Emergency stop
            ACC_EmergencyStop(&acc);
            // Flash LED rapidly to indicate emergency
            MCAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET);
            break;
            
        case ZONE_DANGER:
            // Significant speed reduction or stop
            ACC_SetSpeedPercentage(&acc, 10);  // 10% of target speed
            break;
            
        case ZONE_WARNING:
            // Moderate speed reduction
            ACC_SetSpeedPercentage(&acc, 50);  // 50% of target speed
            break;
            
        case ZONE_CAUTION:
            // Slight speed reduction
            ACC_SetSpeedPercentage(&acc, 70);  // 70% of target speed
            break;
            
        case ZONE_SAFE:
            // Maintain target speed
            ACC_SetSpeedPercentage(&acc, 100); // 100% of target speed
            break;
    }
    
    // Handle lateral obstacles (left and right)
    if (obstacleState.left == ZONE_DANGER || obstacleState.left == ZONE_CRITICAL) {
        // Obstacle on left side - adjust trajectory right
        ACC_AdjustSteering(&acc, 20);  // Turn 20% right
    }
    else if (obstacleState.right == ZONE_DANGER || obstacleState.right == ZONE_CRITICAL) {
        // Obstacle on right side - adjust trajectory left
        ACC_AdjustSteering(&acc, -20); // Turn 20% left
    }
    else {
        // No lateral obstacles - maintain straight path
        ACC_AdjustSteering(&acc, 0);   // Keep straight
    }
    
    // Handle rear obstacles
    if (obstacleState.rear == ZONE_DANGER || obstacleState.rear == ZONE_CRITICAL) {
        // Prevent reversing if there's an obstacle behind
        ACC_DisableReverse(&acc);
    }
    else {
        // Allow reversing if needed
        ACC_EnableReverse(&acc);
    }
}

/**
 * @brief Evaluate distance measurement and determine obstacle zone
 * @param distance: Distance measurement in mm
 * @return: Obstacle zone classification
 */
ObstacleZone_t Evaluate_Distance(uint32_t distance) {
    if (distance == 0) // Consider 0 distance (timeout/error) as critical
        return ZONE_CRITICAL;
    if (distance < OBSTACLE_DIST_CRITICAL)
        return ZONE_CRITICAL;
    else if (distance < OBSTACLE_DIST_DANGER)
        return ZONE_DANGER;
    else if (distance < OBSTACLE_DIST_WARNING)
        return ZONE_WARNING;
    else if (distance < OBSTACLE_DIST_CAUTION)
        return ZONE_CAUTION;
    else
        return ZONE_SAFE;
}

/**
 * @brief Generic ultrasonic sensor EXTI callback logic
 * @param sensor: Pointer to the specific Ultrasonic_t sensor instance
 */
static void Generic_Ultrasonic_Callback(Ultrasonic_t* sensor) {
    uint32_t current_time = MCAL_TIM_Base_GetCounter(ULTRASONIC_TIMER);
    
    if (MCAL_GPIO_ReadPin(sensor->EchoPort, sensor->EchoPin) == GPIO_PIN_SET) {
        // Rising edge detected - record start time
        sensor->StartTime = current_time;
        sensor->State = ULTRASONIC_STATE_WAITING_ECHO;
    } else {
        // Falling edge detected
        if (sensor->State == ULTRASONIC_STATE_WAITING_ECHO) {
            sensor->EchoTime = current_time;
            uint32_t echo_duration_ticks;
            
            // Calculate echo duration, handling timer overflow
            if (sensor->EchoTime >= sensor->StartTime) {
                echo_duration_ticks = sensor->EchoTime - sensor->StartTime;
            } else {
                // Timer overflowed between rising and falling edge
                echo_duration_ticks = (ULTRASONIC_TIMER_PERIOD - sensor->StartTime) + sensor->EchoTime + 1;
            }
            
            // Check for plausible duration (avoid glitches, max range)
            if (echo_duration_ticks > 50 && echo_duration_ticks < ULTRASONIC_TIMEOUT_TICKS) { 
                // Calculate distance: distance (mm) = time (us) * speed_of_sound (mm/us) / 2
                sensor->Distance = (uint32_t)((echo_duration_ticks * US_TIMER_TICK_US * SPEED_OF_SOUND_MM_PER_US) / 2.0);
            } else {
                // Timeout or glitch - set distance to 0 or max
                sensor->Distance = 0; // Indicate error/timeout
            }
            sensor->IsReady = 1;
            sensor->State = ULTRASONIC_STATE_IDLE;
        }
    }
    // Note: EXTI pending bit is cleared in the EXTI driver's ISR

    // In main loop or system timer handler:
    if (sensor->State == ULTRASONIC_STATE_WAITING_ECHO && 
        (current_time - sensor->StartTime) > ULTRASONIC_TIMEOUT_TICKS) {
        sensor->State = ULTRASONIC_STATE_IDLE;
        sensor->Distance = 0; // Indicate timeout
        sensor->IsReady = 1;
    }
}

/**
 * @brief Front ultrasonic sensor EXTI callback
 */
void Front_Ultrasonic_Callback(void) {
    Generic_Ultrasonic_Callback(&frontUltrasonic);
}

/**
 * @brief Rear ultrasonic sensor EXTI callback
 */
void Rear_Ultrasonic_Callback(void) {
    Generic_Ultrasonic_Callback(&rearUltrasonic);
}

/**
 * @brief Left ultrasonic sensor EXTI callback
 */
void Left_Ultrasonic_Callback(void) {
    Generic_Ultrasonic_Callback(&leftUltrasonic);
}

/**
 * @brief Right ultrasonic sensor EXTI callback
 */
void Right_Ultrasonic_Callback(void) {
    Generic_Ultrasonic_Callback(&rightUltrasonic);
}

/**
 * @brief Left speed sensor EXTI callback
 */
void Left_Speed_Callback(void) {
    leftPulseCount++;
}

/**
 * @brief Right speed sensor EXTI callback
 */
void Right_Speed_Callback(void) {
    rightPulseCount++;
}

/**
 * @brief Initialize system timer (TIM4)
 */
void Timer_Init(void) {
    // Configure TIM4 for a 50ms (control update rate) period
    TIM_TimeBase_Config_t sysTimerConfig = {
        .TIMx = SYSTEM_TIMER,                    // TIM4
        .Prescaler = SYSTEM_TIMER_PRESCALER,     // 799 for 100us tick @ 8MHz
        .CounterMode = TIM_COUNTERMODE_UP,
        .Period = SYSTEM_TIMER_PERIOD,           // 499 for 50ms period
        .ClockDivision = TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0
    };
    MCAL_TIM_Base_Init(&sysTimerConfig);
    MCAL_TIM_SetUpdateCallback(SYSTEM_TIMER, System_Timer_Callback);
    MCAL_TIM_EnableIT(SYSTEM_TIMER, TIM_IT_UPDATE);
    MCAL_TIM_Base_Start(SYSTEM_TIMER);

    // Use these instead of NVIC_SetPriority
    NVIC_IRQ28_TIM2_Enable;  // For ultrasonic timer
    NVIC_IRQ29_TIM3_Enable;  // For speed sensor timer
    NVIC_IRQ30_TIM4_Enable;  // For system timer
}

/**
 * @brief System timer (TIM4) update callback function
 */
void System_Timer_Callback(void) {
    // Set flag for main loop
    timer_flag = 1;
    // Note: Timer flag is cleared in TIM4_IRQHandler in driver
}

/**
 * @brief Speed timer (TIM3) update callback function
 */
void Speed_Timer_Callback(void) {
    // Capture current pulse counts
    uint32_t currentLeftPulses = leftPulseCount;
    uint32_t currentRightPulses = rightPulseCount;
    
    // Reset pulse counters for the next interval
    leftPulseCount = 0;
    rightPulseCount = 0;
    
    // Store pulse counts for later use
    leftSpeedSensor.PulseCount = currentLeftPulses;
    rightSpeedSensor.PulseCount = currentRightPulses;
    
    // Calculate speed (mm/s)
    // Speed = (Pulses / Interval) * (mm / Pulse)
    leftSpeedSensor.Speed = (float)(currentLeftPulses / SPEED_TIMER_INTERVAL_S) * MM_PER_PULSE;
    rightSpeedSensor.Speed = (float)(currentRightPulses / SPEED_TIMER_INTERVAL_S) * MM_PER_PULSE;
    
    leftSpeedSensor.IsReady = 1;
    rightSpeedSensor.IsReady = 1;
    
    // Note: Timer flag is cleared in TIM3_IRQHandler in driver
}

/**
 * @brief System Clock Configuration - Set to 8MHz using HSI
 */
void SystemClock_Config(void) {
    // Ensure the HSI is on and ready
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    
    // Select HSI as system clock
    RCC->CFGR &= ~RCC_CFGR_SW; // Clear SW bits
    RCC->CFGR |= RCC_CFGR_SW_HSI; // Select HSI
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Wait for HSI to be system clock
    
    // Reset HSEON, CSSON and PLLON bits
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
    
    // Reset PLL configuration
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    
    // Disable all interrupts
    RCC->CIR = 0x00000000;
    
    // Configure Flash: Enable Prefetch Buffer and set Latency to 0 (for 8 MHz)
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    
    // Configure clock dividers: AHB=1, APB1=1, APB2=1
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    
    // Enable peripheral clocks
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // Ensure TIM1 clock is enabled for PWM
}

/**
 * @brief Initialize all sensors
 */
void Initialize_Sensors(void) {
    
    // --- Initialize Ultrasonic Sensors & Timer (TIM2) ---
    
    // Configure TIM2 for 1us tick (for ultrasonic echo measurement)
    TIM_TimeBase_Config_t usTimerConfig = {
        .TIMx = ULTRASONIC_TIMER,           // TIM2
        .Prescaler = ULTRASONIC_TIMER_PRESCALER, // 7 for 1us tick @ 8MHz
        .CounterMode = TIM_COUNTERMODE_UP,
        .Period = ULTRASONIC_TIMER_PERIOD,      // 0xFFFF (max count)
        .ClockDivision = TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0
    };
    MCAL_TIM_Base_Init(&usTimerConfig);
    MCAL_TIM_Base_Start(ULTRASONIC_TIMER);
    
    // Front ultrasonic
    frontUltrasonic.TrigPort = ULTRASONIC_FRONT_TRIG_PORT;
    frontUltrasonic.TrigPin = ULTRASONIC_FRONT_TRIG_PIN;
    frontUltrasonic.EchoPort = ULTRASONIC_FRONT_ECHO_PORT;
    frontUltrasonic.EchoPin = ULTRASONIC_FRONT_ECHO_PIN;
    frontUltrasonic.EXTI_Mapping = ULTRASONIC_FRONT_EXTI;
    frontUltrasonic.Callback = Front_Ultrasonic_Callback; // Assign EXTI callback
    HAL_ULTRASONIC_Init(&frontUltrasonic); // Configures GPIO, EXTI (rising/falling), registers callback
    
    // Rear ultrasonic
    rearUltrasonic.TrigPort = ULTRASONIC_REAR_TRIG_PORT;
    rearUltrasonic.TrigPin = ULTRASONIC_REAR_TRIG_PIN;
    rearUltrasonic.EchoPort = ULTRASONIC_REAR_ECHO_PORT;
    rearUltrasonic.EchoPin = ULTRASONIC_REAR_ECHO_PIN;
    rearUltrasonic.EXTI_Mapping = ULTRASONIC_REAR_EXTI;
    rearUltrasonic.Callback = Rear_Ultrasonic_Callback;
    HAL_ULTRASONIC_Init(&rearUltrasonic);
    
    // Left ultrasonic
    leftUltrasonic.TrigPort = ULTRASONIC_LEFT_TRIG_PORT;
    leftUltrasonic.TrigPin = ULTRASONIC_LEFT_TRIG_PIN;
    leftUltrasonic.EchoPort = ULTRASONIC_LEFT_ECHO_PORT;
    leftUltrasonic.EchoPin = ULTRASONIC_LEFT_ECHO_PIN;
    leftUltrasonic.EXTI_Mapping = ULTRASONIC_LEFT_EXTI;
    leftUltrasonic.Callback = Left_Ultrasonic_Callback;
    HAL_ULTRASONIC_Init(&leftUltrasonic);
    
    // Right ultrasonic
    rightUltrasonic.TrigPort = ULTRASONIC_RIGHT_TRIG_PORT;
    rightUltrasonic.TrigPin = ULTRASONIC_RIGHT_TRIG_PIN;
    rightUltrasonic.EchoPort = ULTRASONIC_RIGHT_ECHO_PORT;
    rightUltrasonic.EchoPin = ULTRASONIC_RIGHT_ECHO_PIN;
    rightUltrasonic.EXTI_Mapping = ULTRASONIC_RIGHT_EXTI;
    rightUltrasonic.Callback = Right_Ultrasonic_Callback;
    HAL_ULTRASONIC_Init(&rightUltrasonic);
    
    // Initialize obstacle detection state
    obstacleState.front = ZONE_SAFE;
    obstacleState.rear = ZONE_SAFE;
    obstacleState.left = ZONE_SAFE;
    obstacleState.right = ZONE_SAFE;
    
    // --- Initialize Speed Sensors & Timer (TIM3) ---
    
    // Configure TIM3 for 1s update interrupt (for speed calculation)
    TIM_TimeBase_Config_t speedTimerConfig = {
        .TIMx = SPEED_SENSOR_TIMER,             // TIM3
        .Prescaler = SPEED_SENSOR_PRESCALER,    // 799 for 100us tick @ 8MHz
        .CounterMode = TIM_COUNTERMODE_UP,
        .Period = SPEED_SENSOR_PERIOD,          // 10000 for 1s period (10000 * 100us)
        .ClockDivision = TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0
    };
    MCAL_TIM_Base_Init(&speedTimerConfig);
    MCAL_TIM_SetUpdateCallback(SPEED_SENSOR_TIMER, Speed_Timer_Callback); // Register TIM3 update callback
    MCAL_TIM_EnableIT(SPEED_SENSOR_TIMER, TIM_IT_UPDATE); // Enable TIM3 update interrupt
    MCAL_TIM_Base_Start(SPEED_SENSOR_TIMER);
    
    // For left speed sensor
    leftSpeedSensor.Port = SPEED_SENSOR_LEFT_PORT;
    leftSpeedSensor.Pin = SPEED_SENSOR_LEFT_PIN;
    leftSpeedSensor.EXTI_Mapping = SPEED_SENSOR_LEFT_EXTI;
    leftSpeedSensor.PulseCount = 0;
    leftSpeedSensor.Speed = 0.0;
    HAL_SPEED_SENSOR_Init(&leftSpeedSensor);
    
    // For right speed sensor
    rightSpeedSensor.Port = SPEED_SENSOR_RIGHT_PORT;
    rightSpeedSensor.Pin = SPEED_SENSOR_RIGHT_PIN;
    rightSpeedSensor.EXTI_Mapping = SPEED_SENSOR_RIGHT_EXTI;
    rightSpeedSensor.PulseCount = 0;
    rightSpeedSensor.Speed = 0.0;
    HAL_SPEED_SENSOR_Init(&rightSpeedSensor);
}

/**
 * @brief Initialize all motors (Placeholder - Assumes HAL does the work)
 */
void Initialize_Motors(void) {
    // Initialize the motor control system
    HAL_MOTOR_Init();
    
    // Configure front left motor
    frontLeftMotor.DirectionPort = MOTOR_FL_PORT_FWD;
    frontLeftMotor.DirectionPin_Forward = MOTOR_FL_PIN_FWD;
    frontLeftMotor.DirectionPin_Backward = MOTOR_FL_PIN_BWD;
    frontLeftMotor.PWMPort = MOTOR_FL_PWM_PORT;
    frontLeftMotor.PWMPin = MOTOR_FL_PWM_PIN;
    // Remove ID field reference, as it's not part of the Motor_t struct
    HAL_MOTOR_Config(&frontLeftMotor, MOTOR_PWM_TIMER, MOTOR_FL_PWM_CHANNEL);
    HAL_MOTOR_Enable(&frontLeftMotor, 1);
    
    // Configure front right motor
    frontRightMotor.DirectionPort = MOTOR_FR_PORT_FWD;
    frontRightMotor.DirectionPin_Forward = MOTOR_FR_PIN_FWD;
    frontRightMotor.DirectionPin_Backward = MOTOR_FR_PIN_BWD;
    frontRightMotor.PWMPort = MOTOR_FR_PWM_PORT;
    frontRightMotor.PWMPin = MOTOR_FR_PWM_PIN;
    // Remove ID field reference
    HAL_MOTOR_Config(&frontRightMotor, MOTOR_PWM_TIMER, MOTOR_FR_PWM_CHANNEL);
    HAL_MOTOR_Enable(&frontRightMotor, 1);
    
    // Configure rear left motor
    rearLeftMotor.DirectionPort = MOTOR_RL_PORT_FWD;
    rearLeftMotor.DirectionPin_Forward = MOTOR_RL_PIN_FWD;
    rearLeftMotor.DirectionPin_Backward = MOTOR_RL_PIN_BWD;
    rearLeftMotor.PWMPort = MOTOR_RL_PWM_PORT;
    rearLeftMotor.PWMPin = MOTOR_RL_PWM_PIN;
    // Remove ID field reference
    HAL_MOTOR_Config(&rearLeftMotor, MOTOR_PWM_TIMER, MOTOR_RL_PWM_CHANNEL);
    HAL_MOTOR_Enable(&rearLeftMotor, 1);
    
    // Configure rear right motor
    rearRightMotor.DirectionPort = MOTOR_RR_PORT_FWD;
    rearRightMotor.DirectionPin_Forward = MOTOR_RR_PIN_FWD;
    rearRightMotor.DirectionPin_Backward = MOTOR_RR_PIN_BWD;
    rearRightMotor.PWMPort = MOTOR_RR_PWM_PORT;
    rearRightMotor.PWMPin = MOTOR_RR_PWM_PIN;
    // Remove ID field reference
    HAL_MOTOR_Config(&rearRightMotor, MOTOR_PWM_TIMER, MOTOR_RR_PWM_CHANNEL);
    HAL_MOTOR_Enable(&rearRightMotor, 1);
}

/**
 * @brief Initialize ACC system (Placeholder - Assumes HAL/Logic exists)
 */
void Initialize_ACC(void) {
    // Link sensors and motors to ACC instance
    acc.FrontUltrasonic = &frontUltrasonic;
    acc.RearUltrasonic = &rearUltrasonic;
    acc.LeftUltrasonic = &leftUltrasonic;
    acc.RightUltrasonic = &rightUltrasonic;
    
    acc.LeftSpeedSensor = &leftSpeedSensor;
    acc.RightSpeedSensor = &rightSpeedSensor;
    
    acc.FrontLeftMotor = &frontLeftMotor;
    acc.FrontRightMotor = &frontRightMotor;
    acc.RearLeftMotor = &rearLeftMotor;
    acc.RearRightMotor = &rearRightMotor;
    
    // Initialize ACC system logic (if needed by ACC module)
    // ACC_Init(&acc);
    
    // Configure ACC parameters (Example values - adjust as needed)
    // ACC_SetTargetSpeed(&acc, 500); // mm/s
    // ACC_SetSafeDistance(&acc, 500); // mm
    // ACC_SetPIDConstants(&acc, 1.0, 0.1, 0.01);
}

