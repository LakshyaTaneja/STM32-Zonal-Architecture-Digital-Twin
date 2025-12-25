/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : NODE B - Zonal Controller & Digital Twin
  * Runs ECU Logic + Vehicle Physics + Battery Model
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include <stdio.h>  // For printf/sprintf
#include <string.h>
#include <stdlib.h>  // For rand()
#include <math.h>   // For sqrtf, tanhf, fabsf

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- GLOBAL SIMULATION STATE ---
typedef struct {
    float Speed_ms;       // Vehicle Velocity [m/s]
    float SoC;            // Battery State of Charge [0.0 - 1.0]
    float Distance_m;     // Odometer
} VehicleState_t;

VehicleState_t Vehicle = {0.0f, 0.80f, 0.0f}; // Start at 0 m/s, 80% SoC

// --- INPUTS (From Node A via CAN) ---
volatile uint8_t rx_accel_pct = 0;
volatile uint8_t rx_brake_pct = 0;

// --- CONTROL SIGNALS ---
float Prop_Cmd_Nm = 0.0f;
float Regen_Cmd_Nm = 0.0f;
float Friction_Cmd_Nm = 0.0f;

// --- CAN BUFFERS ---
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);

/* --- MATH HELPER --- */
// Clamps value between Min and Max
float CLAMP(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

float Add_Gaussian_Noise(float mean, float std_dev) {
    static int has_spare = 0;
    static float spare;

    if (has_spare) {
        has_spare = 0;
        return mean + std_dev * spare;
    }

    has_spare = 1;
    float u, v, s;
    do {
        u = (rand() / ((float)RAND_MAX)) * 2.0f - 1.0f;
        v = (rand() / ((float)RAND_MAX)) * 2.0f - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);

    s = sqrtf(-2.0f * logf(s) / s);
    spare = v * s;
    return mean + std_dev * (u * s);
}

/* * Simple Low Pass Filter (Exponential Moving Average)
 * - current: The new noisy reading
 * - prev: The filtered value from the last step
 * - alpha: Tuning factor (0.05 = Strong smoothing)
 */
float LowPassFilter(float current, float prev, float alpha) {
    return (alpha * current) + ((1.0f - alpha) * prev);
}

// Bathtub Curve Logic for Internal Resistance
float Get_Internal_Resistance(float soc) {
    float R_base = 0.022f; // Nominal resistance (22 mOhm)

    // 1. Low SoC Penalty (Resistance spikes when empty)
    // If SoC < 10%, add penalty.
    float R_low = (soc < 0.1f) ? (0.05f * (0.1f - soc)) : 0.0f;

    // 2. High SoC Penalty (Resistance rises when full)
    // If SoC > 90%, add small penalty.
    float R_high = (soc > 0.9f) ? (0.02f * (soc - 0.9f)) : 0.0f;

    return R_base + R_low + R_high;
}

/* ============================================================================
 * MODULE 1: ECU LOGIC
 * ============================================================================ */
void Run_ECU_Logic(float Accel_Norm, float Brake_Norm, float Speed_ms, float Current_SoC)
{
    // --- PARAMETERS (Keep your tuned values) ---
    const float MAX_PROP_TRQ  = 3000.0f;
    const float MAX_BRAKE_TRQ = -4710.0f;
    const float ONE_PEDAL_TRQ = -710.0f;
    const float TOP_SPEED     = 45.0f;
    const float DEADBAND      = 0.05f;

    float req_prop = 0.0f;
    float req_brake = 0.0f;
    float req_oneped = 0.0f;
    uint8_t is_coasting = 0;

    // --- DRIVER INTENT ---
    if (Brake_Norm > 0.95f) Brake_Norm = 1.0f;
    if (Accel_Norm > 0.95f) Accel_Norm = 1.0f;

    if (Brake_Norm > DEADBAND) {
        req_prop = 0.0f;
        req_brake = Brake_Norm * MAX_BRAKE_TRQ;
        is_coasting = 0;
    }
    else {
        req_brake = 0.0f;
        if (Accel_Norm > DEADBAND && Speed_ms < TOP_SPEED) {
            req_prop = Accel_Norm * MAX_PROP_TRQ;
            is_coasting = 0;
            if (Speed_ms > (TOP_SPEED - 5.0f)) {
                float factor = (TOP_SPEED - Speed_ms) / 5.0f;
                req_prop *= factor;
            }
        } else {
            req_prop = 0.0f;
            is_coasting = 1;
        }
    }

    req_oneped = (is_coasting) ? ONE_PEDAL_TRQ : 0.0f;
    float total_req = req_prop + req_oneped + req_brake;

    // --- REGEN ENVELOPE ---
    float max_regen_lim = 0.0f;
    if (Speed_ms <= 5.0f) max_regen_lim = -200.0f * Speed_ms;
    else if (Speed_ms <= 10.0f) max_regen_lim = -1000.0f + (-300.0f * (Speed_ms - 5.0f));
    else if (Speed_ms <= 25.0f) max_regen_lim = -2500.0f;
    else if (Speed_ms <= 45.0f) max_regen_lim = -2500.0f + (65.0f * (Speed_ms - 25.0f));
    else max_regen_lim = -1200.0f;

    if (max_regen_lim > 0.0f) max_regen_lim = 0.0f;

    // --- TORQUE SPLIT (SEPARATED) ---
    float final_prop = 0.0f;
    float final_regen = 0.0f;
    float final_frict = 0.0f;

    if (total_req >= 0.0f) {
        // PROPULSION CASE
        final_prop = (total_req > MAX_PROP_TRQ) ? MAX_PROP_TRQ : total_req;
        final_regen = 0.0f;
        final_frict = 0.0f;
    } else {
        // BRAKING CASE
        final_prop = 0.0f;
        if (total_req > max_regen_lim) final_regen = total_req;
        else final_regen = max_regen_lim;

        final_frict = total_req - final_regen;
    }

    // Assign to SEPARATE Globals for Plotting
    Prop_Cmd_Nm     = final_prop;
    Regen_Cmd_Nm    = final_regen;
    Friction_Cmd_Nm = final_frict;
}

/* ============================================================================
 * MODULE 2: VEHICLE PLANT MODEL (Physics Engine)
 * ============================================================================ */
float Run_Vehicle_Plant(float Total_Trq, float Speed_ms)
{
    // Parameters
    const float MASS   = 1600.0f;
    const float RADIUS = 0.3f;
    const float Cd     = 0.22f;
    const float AREA   = 2.2f;
    const float Crr    = 0.008f;
    const float RHO    = 1.225f;
    const float G      = 9.81f;

    // 1. Tractive Force
    float F_tractive = Total_Trq / RADIUS;

    // 2. Aero Drag
    // F = 0.5 * rho * Cd * A * v^2
    float F_aero = 0.5f * RHO * Cd * AREA * (Speed_ms * fabsf(Speed_ms));

    // 3. Rolling Resistance
    // Uses tanh for smooth zero-crossing
    float F_roll = Crr * MASS * G * tanhf(Speed_ms * 10.0f);

    // 4. Zero-Speed Brake Clamp
    // Prevents brakes from reversing the car at standstill
    if (fabsf(Speed_ms) < 0.1f && F_tractive < 0.0f) {
        return 0.0f; // Force clamped, no acceleration
    }

    // 5. Net Force & Acceleration
    float F_net = F_tractive - F_aero - F_roll;
    float Accel_mps2 = F_net / MASS;

    return Accel_mps2;
}

/* ============================================================================
 * MODULE 3: EXTENDED KALMAN FILTER (EKF)
 * Fuses Current Integration with Voltage Feedback to estimate True SoC
 * ============================================================================ */

// EKF State Variables
float EKF_SoC = 0.80f;     // The Estimate (Starts at 80%)
float EKF_P   = 0.01f;     // Uncertainty (Variance)

// --- TUNING PARAMETERS ---
const float Q = 0.00001f;
const float R = 3000.0f;

void Run_EKF(float current_A, float voltage_V, float dt)
{
    // --- 1. PREDICT STEP ---
    float dSoC = -(current_A * dt) / (50.0f * 3600.0f);
    EKF_SoC = EKF_SoC + dSoC;
    EKF_P = EKF_P + Q;

    // --- 2. UPDATE STEP (ALWAYS ON) ---
    // Gate widened to 1000A so it works during Emergency Braking
    if (fabsf(current_A) < 1000.0f)
    {
        float OCV_Pred = 350.0f + (EKF_SoC * 70.0f);

        float R_Dynamic = Get_Internal_Resistance(EKF_SoC);
        float V_Pred = OCV_Pred - (current_A * R_Dynamic);

        float z = voltage_V;
        float y = z - V_Pred;

        float H = 70.0f;
        float S = (H * EKF_P * H) + R;
        float K = (EKF_P * H) / S;

        EKF_SoC = EKF_SoC + (K * y);
        EKF_P = (1.0f - (K * H)) * EKF_P;
    }

    // Safety Clamps
    if (EKF_SoC > 1.0f) EKF_SoC = 1.0f;
    if (EKF_SoC < 0.0f) EKF_SoC = 0.0f;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();

  // --- HARDWARE CHECK: STARTUP FLASH ---
  // Turn LED ON for 1 second to prove it works
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  // CAN Setup
  CAN_FilterTypeDef canfilter;
  canfilter.FilterActivation = CAN_FILTER_ENABLE;
  canfilter.FilterBank = 0;
  canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilter.FilterIdHigh = 0x102 << 5;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0xFFE0;
  canfilter.FilterMaskIdLow = 0x0000;
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &canfilter);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // --- SIMULATION PARAMETERS ---
  const float dt = 0.015f;
  char uart_buf[150]; // Buffer size is chosen for 7 columns

  // Filter Memory
  static float filtered_voltage_V = 0.0f;
  if (filtered_voltage_V == 0.0f) filtered_voltage_V = 350.0f + (Vehicle.SoC * 70.0f);

  while (1)
  {
	  static float current_time = 0.0f;
	  current_time += dt;
      // --- STEP 1: INPUTS ---
      float accel_norm = (float)rx_accel_pct / 100.0f;
      float brake_norm = (float)rx_brake_pct / 100.0f;

      // --- STEP 2: LOGIC ---
      Run_ECU_Logic(accel_norm, brake_norm, Vehicle.Speed_ms, Vehicle.SoC);

      // --- STEP 3: PHYSICS ---
      float total_trq = Prop_Cmd_Nm + Regen_Cmd_Nm + Friction_Cmd_Nm;
      float accel_mps2 = Run_Vehicle_Plant(total_trq, Vehicle.Speed_ms);

      float elec_trq = Prop_Cmd_Nm + Regen_Cmd_Nm;
      float true_power_kw = (elec_trq * Vehicle.Speed_ms) / 1000.0f;
      // Apply 95% Efficiency (Heat Loss)
      if (true_power_kw > 0.0f) true_power_kw /= 0.95f; // Propulsion: Drain more
      else                      true_power_kw *= 0.95f; // Regen: Capture less

      float true_current_A = (true_power_kw * 1000.0f) / 300.0f;
      float true_ocv = 350.0f + (Vehicle.SoC * 70.0f);
      // Calculate dynamic resistance based on Bathtub Curve
      float R_inst = Get_Internal_Resistance(Vehicle.SoC);
      float true_voltage_V = true_ocv - (true_current_A * R_inst);
      // --- STEP 4: SENSOR NOISE & CONDITIONING ---
      float sensor_current_A = Add_Gaussian_Noise(true_current_A, 2.0f) + 1.5f; // Bias
      float raw_voltage_V = Add_Gaussian_Noise(true_voltage_V, 1.5f);           // Noise

      // Signal Conditioning (Alpha 0.005 for strong smoothing)
      filtered_voltage_V = LowPassFilter(raw_voltage_V, filtered_voltage_V, 0.1f);

      // --- STEP 5: ESTIMATORS ---
      // Drift Monitor
      static float Bad_SoC = 0.80f;
      float dSoC_bad = -(sensor_current_A * dt) / (50.0f * 3600.0f);
      Bad_SoC += dSoC_bad;

      // EKF (Using Clean Voltage)
      Run_EKF(sensor_current_A, filtered_voltage_V, dt);

      // --- STEP 6: INTEGRATION ---
      float dSoC_true = -(true_current_A * dt) / (50.0f * 3600.0f);
      Vehicle.SoC += dSoC_true;
      Vehicle.Speed_ms += accel_mps2 * dt;
      if(Vehicle.Speed_ms < 0.0f) Vehicle.Speed_ms = 0.0f;
      Vehicle.SoC = CLAMP(Vehicle.SoC, 0.0f, 1.0f);

      // --- STEP 7: MASTER TELEMETRY (7 CHANNELS) ---
      // 1. Speed
      // 2. True SoC
      // 3. EKF SoC
      // 4. Bad SoC (Drift)
      // 5. Prop Torque
      // 6. Regen Torque
      // 7. Friction Torque

      int len = sprintf(uart_buf, "%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f\r\n",
                        current_time,
                        Vehicle.Speed_ms,
                        Vehicle.SoC * 100.0f,
                        EKF_SoC * 100.0f,
                        Bad_SoC * 100.0f,
                        Prop_Cmd_Nm,
                        Regen_Cmd_Nm,
                        Friction_Cmd_Nm);

      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 10);

      // --- STEP 8: LED LOGIC (BRAKE LIGHT) ---
      // If Regen is active (more negative than -100), turn ON.
      if (Regen_Cmd_Nm < -100.0f) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      } else {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      }

      HAL_Delay(10);
  }
}
// --- INTERRUPTS & INIT (Keep standard) ---
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        if (RxHeader.StdId == 0x102) {
            rx_accel_pct = RxData[0];
            rx_brake_pct = RxData[1];
        }
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    HAL_PWREx_EnableOverDrive();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void MX_CAN1_Init(void) {
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 5;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    HAL_CAN_Init(&hcan1);
}

static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // CAN PINS (PA11=RX, PA12=TX)
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // UART PINS (PA2=TX, PA3=RX)
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
