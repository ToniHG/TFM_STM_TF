/**
 * @file    radar.c
 * @brief   Redundant radar module for Slave node (STM32F407G-DISC1)
 *
 * Depends on CubeMX hardware initialization in main.c:
 * - PA2  → HC-SR04 TRIG  (GPIO_Output)
 * - PA1  → HC-SR04 ECHO  (TIM2_CH2, Input Capture, 32 bits, AF1)
 * - PA6  → SG90 SERVO    (TIM3_CH1, PWM Generation, 16 bits, AF2)
 *
 * @author  Antonio Hermoso García
 * @date    2026
 */

#include "radar.h"
#include "can_protocol.h"
#include "fault_tolerance.h"
#include "crc16.h"
#include "FreeRTOS.h"
#include "stm32f407xx.h"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
extern QueueHandle_t can_tx_queue;
extern volatile uint8_t g_slave_hw_status;
/* TIM3 (PWM Servo) y TIM2 (Input Capture Eco) */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
#define htim_servo htim3
#define htim_echo  htim2
/* --- SERVO --- */
#define SERVO_TIM               TIM3
#define SERVO_TIM_CHANNEL       TIM_CHANNEL_1
#define SERVO_PULSE_MIN_US      (620U)     /* Width pulse 0°   */
#define SERVO_PULSE_MAX_US      (2420U)    /* Width pulse 180° */
#define RADAR_STEP_DEG          (10U)      /* Angular resolution */
#define RADAR_ANGLE_MIN         (0U)       /* Minimum angle for scanning */
#define RADAR_ANGLE_MAX         (180U)     /* Maximum angle for scanning */
#define SERVO_SETTLE_MS         (80U)      /* Stabilization time */
#define RADAR_TASK_PERIOD_MS    (50U)     /* Radar task period */
/* --- ULTRASONIC    --- */
#define HCSR04_TIM              TIM2
#define HCSR04_TIM_CHANNEL      TIM_CHANNEL_2
#define HCSR04_GPIO_PORT        GPIOA
#define HCSR04_TRIG_PIN         GPIO_PIN_2
#define HCSR04_ECHO_TIMEOUT_MS  (30U)      /* Timeout for echo signal (5 m) */
#define HCSR04_MAX_DIST_CM      (400U)
#define HCSR04_INVALID_DIST     (0xFFFFU)
/* Binary semaphore for synchronizing the capture ISR with the task */
static SemaphoreHandle_t xEchoSemaphore = NULL;

static volatile uint32_t ulCaptureRise = 0U;
static volatile uint32_t ulCaptureFall = 0U;
static volatile uint8_t  ucEchoPhase   = 0U; /* 0 = Waiting for rise, 1 = Waiting for fall */


static void pr_ServoInit(void) {

    /* CubeMX has already configured clocks, pins and TIM3 in MX_TIM3_Init().
       We only need to start PWM generation. */
    HAL_TIM_PWM_Start(&htim_servo, SERVO_TIM_CHANNEL);
}

static void pr_ServoSetAngle(uint8_t angle) {

    if (angle > RADAR_ANGLE_MAX) angle = RADAR_ANGLE_MAX;

    /* Lineal conversion angle to pulse width */
    uint32_t pulse = SERVO_PULSE_MIN_US +
                     ((uint32_t)angle * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US))
                     / RADAR_ANGLE_MAX;

    __HAL_TIM_SET_COMPARE(&htim_servo, SERVO_TIM_CHANNEL, pulse);
}


static void pr_HCSR04Init(void) {

    /* Ensure the trigger pin is inactive by default */
    HAL_GPIO_WritePin(HCSR04_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    /* CubeMX has already configured TIM2 and interrupts in MX_TIM2_Init().
       Start Input Capture with interrupt generation. */
    HAL_TIM_IC_Start_IT(&htim_echo, HCSR04_TIM_CHANNEL);
}

static uint16_t pr_HCSR04Measure(void) {

    ucEchoPhase = 0U;
    
    /* Clear the semaphore to avoid false positives from previous readings */
    xSemaphoreTake(xEchoSemaphore, 0); 

    /* Generate the trigger pulse (minimum 10 microseconds) */
    HAL_GPIO_WritePin(HCSR04_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    
    /* Delay calibrated for 168 MHz (≈ 10 us). Do not use vTaskDelay due to brevity. */
    volatile uint32_t d = 1680U; 
    while (d--);
    
    HAL_GPIO_WritePin(HCSR04_GPIO_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    /* Deterministic task block until ISR response or timeout */
    if (xSemaphoreTake(xEchoSemaphore, pdMS_TO_TICKS(HCSR04_ECHO_TIMEOUT_MS)) == pdFALSE)
    {
        return HCSR04_INVALID_DIST; 
    }

    /* Calculate pulse duration */
    uint32_t ulDuration;
    if (ulCaptureFall >= ulCaptureRise) {
        ulDuration = ulCaptureFall - ulCaptureRise;
    } else {
        /* Handle wrap-around in case of overflow (unlikely in 32-bit) */
        ulDuration = (0xFFFFFFFFU - ulCaptureRise) + ulCaptureFall + 1U;
    }

    /* Convert time to distance (cm) based on 343 m/s */
    uint16_t dist = (uint16_t)(ulDuration / 58U);

    if (dist > HCSR04_MAX_DIST_CM) {
        return HCSR04_INVALID_DIST;
    }
    return dist;
}

/**
 * @brief Native interrupt routine for TIM2 (replaces CubeMX handler)
 */
void TIM2_IRQHandler(void) {

    HAL_TIM_IRQHandler(&htim_echo);
}

/**
 * @brief HAL callback handled by FreeRTOS.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

    if (htim->Instance != HCSR04_TIM) return;
    
    /* Verify the interrupt comes from our channel (TIM2_CH2) */
    if (htim->Channel  != HAL_TIM_ACTIVE_CHANNEL_2) return;

    if (ucEchoPhase == 0U)
    {
        /* Capture rising edge (start of echo) */
        ulCaptureRise = HAL_TIM_ReadCapturedValue(htim, HCSR04_TIM_CHANNEL);
        ucEchoPhase = 1U;
        
        /* Reconfigure hardware live to capture the falling edge */
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, HCSR04_TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
        /* Capture falling edge (end of echo) */
        ulCaptureFall = HAL_TIM_ReadCapturedValue(htim, HCSR04_TIM_CHANNEL);
        ucEchoPhase = 0U;
        
        /* Restore hardware for the next measurement */
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, HCSR04_TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);

        /* Immediate unblock of the synchronous FreeRTOS task without latency */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xEchoSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void pr_SendRadarFrame(uint8_t angle, uint16_t dist_cm) {

    /* Initialize the CAN frame */
    can_frame_payload_t my_tx_frame;
    memset(&my_tx_frame, 0, sizeof(can_frame_payload_t));
    /* Packaging data for CAN message */
    my_tx_frame.msg_type = MSG_TYPE_RADAR_DATA;
    my_tx_frame.payload_data[0] = angle;                      /* Angle */
    my_tx_frame.payload_data[1] = (uint8_t)(dist_cm >> 8U);   /* High byte */
    my_tx_frame.payload_data[2] = (uint8_t)(dist_cm & 0xFFU); /* Low byte */
    my_tx_frame.payload_data[3] = 0;                          /* Spare */
    /* Send data to CAN queue */
    xQueueSend(can_tx_queue, &my_tx_frame, 0);
}

/**
 * @brief  FreeRTOS radar task. Manages sequential scanning and deterministic
 * telemetry transmission over the CAN bus.
 */

void vRadarTask(void *argument) {

    /* Initialize Servo and HC-SR04 */
    pr_ServoInit();
    pr_HCSR04Init();

    uint8_t  angle     = RADAR_ANGLE_MIN;
    int8_t   direction = (int8_t)RADAR_STEP_DEG;
    uint8_t consecutive_timeouts = 0;

    while(1)
    {
        /* Move and wait for mechanics to settle */
        pr_ServoSetAngle(angle);
        vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_MS));
        
        /* Physical trigger */
        uint16_t dist = pr_HCSR04Measure();

        if (dist == HCSR04_INVALID_DIST) {
            consecutive_timeouts++;
            if (consecutive_timeouts >= 10) {
                g_slave_hw_status = 0x01; /* ERROR_SENSOR */
            }
        } else {
            consecutive_timeouts = 0;
            g_slave_hw_status = 0x00; /* OK */
        }
        
        /* Send data to CAN queue */
        pr_SendRadarFrame(angle, dist);
        /* Radar barrels from 0 to 180 degrees */
        int16_t next = (int16_t)angle + direction;
        if (next >= (int16_t)RADAR_ANGLE_MAX)
        {
            next      = (int16_t)RADAR_ANGLE_MAX;
            direction = -(int8_t)RADAR_STEP_DEG;
        }
        else if (next <= (int16_t)RADAR_ANGLE_MIN)
        {
            next      = (int16_t)RADAR_ANGLE_MIN;
            direction = (int8_t)RADAR_STEP_DEG;
        }
        angle = (uint8_t)next;

        /* Delay for the next iteration */
        vTaskDelay(pdMS_TO_TICKS(RADAR_TASK_PERIOD_MS));
    }
}

/**
 * @brief  Initialize radar logic (semaphores and tasks).
 * NOTE: Peripherals (TIM2, TIM3 and GPIO) must have been
 * initialized previously by MX_..._Init() in main.c.
 * @return pdTRUE if successful, pdFALSE if resource allocation failed (FreeRTOS).
 */

BaseType_t Radar_Init(void) {

    /* Create the semaphore for echo handling */
    xEchoSemaphore = xSemaphoreCreateBinary();
    if (xEchoSemaphore == NULL) {
        return pdFALSE; 
    }
    
    /* Create the radar task  */
    BaseType_t ret = xTaskCreate(vRadarTask, "RadarTask", 256, NULL, 1, NULL);

    return ret;
}