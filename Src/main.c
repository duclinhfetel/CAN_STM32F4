/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define COUNTER_PERIOD_TIM3   50000
#define TIME_SIMPLE           0.01
#define MAX_SPEED_PULSE       300
#define FORWARD_DUTY_CYCLE    1480
#define MODE_COLLECT          0
#define MODE_AUTOPILOT        1
#define CAN_ID_STM_TO_TX2     0x1A0
#define CAN_ID_TX2_TO_STM     0x1A5
// servo 1460 950 1950

typedef struct {
  uint16_t speed;
  uint16_t steering;
  uint32_t flag;
}datacan;

volatile datacan *dataFrameTX;
volatile datacan *dataFrameRX;

static void CAN1_CONFIG(void);
static void CAN2_CONFIG(void);
static void pid_calculation(uint16_t setPointPulse);

volatile float g_Kp = 0.015;
volatile float g_Kd = 0.01;
volatile float g_Ki = 0.0;

volatile uint16_t g_uStartTime_RC = 0 , g_uEndTime_RC = 0;
volatile uint16_t g_uStartTime_DC = 0 , g_uEndTime_DC = 0;

volatile uint16_t g_Encoder = 0;

volatile uint16_t g_uPulseWidth_RC = 0;
volatile uint16_t g_uPulseWidth_DC = 0;

volatile uint8_t g_pulse_set_pid = 0;
volatile uint16_t g_steering = 0;
volatile double g_pulseOutput_DC = 0;

volatile uint8_t countToggleCollect = 0;
volatile uint32_t flag = 0;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void pid_calculation(uint16_t setPointPulse)
{
   static volatile double iPart = 0, pPart = 0, dPart = 0;
   static volatile double output = 0, preOutput = 0;
   static volatile uint32_t uPrePulse = 0;
   static volatile int32_t nPulse = 0;
   static volatile int32_t nError = 0, nPreError = 0, nPrePreError = 0;
   
   nError = (setPointPulse - g_Encoder);

   pPart  = g_Kp*(nError - nPreError);
   iPart  = g_Ki*(0.5*(nError + nPreError)*TIME_SIMPLE);
   dPart  = g_Kd*(nError - 2*nPreError + nPrePreError)*(1/TIME_SIMPLE);

   output += preOutput + pPart + dPart + iPart;

   nPrePreError = nPreError;
   nPreError = nError;
   preOutput += pPart + dPart + iPart;

   if (output > MAX_SPEED_PULSE)
   {
      output = MAX_SPEED_PULSE;
   }
   else if (output < 0)
   {
      output = 0;
   }
   g_pulseOutput_DC = output + FORWARD_DUTY_CYCLE;
   
   //__HAL_TIM_SetCompare(&htim2 , TIM_CHANNEL_3, (int)g_pulseOutput_DC);
   
}
static void CAN1_CONFIG(void)
{
   CAN_FilterConfTypeDef  sFilterConfig;
   static CanTxMsgTypeDef TxMessage;
   static CanRxMsgTypeDef RxMessage;

   hcan1.pTxMsg = &TxMessage;
   hcan1.pRxMsg = &RxMessage;

   sFilterConfig.FilterNumber = 0;
   sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterIdHigh = 0x0000;
   sFilterConfig.FilterIdLow = 0x0000;
   sFilterConfig.FilterMaskIdHigh = 0x0000;
   sFilterConfig.FilterMaskIdLow = 0x0000;
   sFilterConfig.FilterFIFOAssignment = 0;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.BankNumber = 14;

   if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
   {
      Error_Handler();
   }

   hcan1.pTxMsg->StdId = CAN_ID_STM_TO_TX2;
   hcan1.pTxMsg->ExtId = 0x01;
   hcan1.pTxMsg->RTR = CAN_RTR_DATA;
   hcan1.pTxMsg->IDE = CAN_ID_STD;
   hcan1.pTxMsg->DLC = 8;
}
static void CAN2_CONFIG(void)
{
   CAN_FilterConfTypeDef  sFilterConfig;
   static CanTxMsgTypeDef TxMessage;
   static CanRxMsgTypeDef RxMessage;

   hcan2.pTxMsg = &TxMessage;
   hcan2.pRxMsg = &RxMessage;

   sFilterConfig.FilterNumber = 14;
   sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterIdHigh = 0x0000;
   sFilterConfig.FilterIdLow = 0x0000;
   sFilterConfig.FilterMaskIdHigh = 0x0000;
   sFilterConfig.FilterMaskIdLow = 0x0000;
   sFilterConfig.FilterFIFOAssignment = 0;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.BankNumber = 14;

   if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
   {
      Error_Handler();
   }

   hcan2.pTxMsg->StdId = 0x324;
   hcan2.pTxMsg->ExtId = 0x02;
   hcan2.pTxMsg->RTR = CAN_RTR_DATA;
   hcan2.pTxMsg->IDE = CAN_ID_STD;
   hcan2.pTxMsg->DLC = 8;
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle)
{
   /*if(CanHandle->Instance == hcan1.Instance)
   {
      if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK)
      {
         HAL_CAN_ErrorCallback(&hcan1);
      }
      if(hcan1.pRxMsg->StdId == CAN_ID_TX2_TO_STM){
         HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
         dataFrameRX = (datacan*)hcan1.pRxMsg->Data;
      }
      
   }*/
   /*
   if(CanHandle->Instance == hcan2.Instance)
   {
      if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK)
      {
         Error_Handler();
      }
      if(hcan2.pRxMsg->StdId == CAN_ID_STM_TO_TX2){
         HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
      }
   }
   */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INPUT_PWM_RC_SERVO_Pin)
  {
    if(HAL_GPIO_ReadPin(INPUT_PWM_RC_SERVO_GPIO_Port, INPUT_PWM_RC_SERVO_Pin) == 1)
    {
        g_uStartTime_RC = htim3.Instance->CNT;
    }
    else if (HAL_GPIO_ReadPin(INPUT_PWM_RC_SERVO_GPIO_Port, INPUT_PWM_RC_SERVO_Pin) == 0)
    {
      g_uEndTime_RC = htim3.Instance->CNT;
      if (g_uStartTime_RC <= g_uEndTime_RC)
      {
         g_uPulseWidth_RC = g_uEndTime_RC - g_uStartTime_RC;
      }
      else
      {
         g_uPulseWidth_RC = COUNTER_PERIOD_TIM3 + g_uEndTime_RC - g_uStartTime_RC ;
      }
    }
    //__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, g_uPulseWidth_RC);
  }
  else
  {
    if(HAL_GPIO_ReadPin(INPUT_PWM_DC_MOTOR_GPIO_Port, INPUT_PWM_DC_MOTOR_Pin) == 1)
    {
        g_uStartTime_DC = htim3.Instance->CNT;
    }
    else if (HAL_GPIO_ReadPin(INPUT_PWM_DC_MOTOR_GPIO_Port, INPUT_PWM_DC_MOTOR_Pin) == 0)
    {
      g_uEndTime_DC = htim3.Instance->CNT;
      if (g_uStartTime_DC <= g_uEndTime_DC)
      {
         g_uPulseWidth_DC = g_uEndTime_DC - g_uStartTime_DC;
      }
      else
      {
         g_uPulseWidth_DC = COUNTER_PERIOD_TIM3 + g_uEndTime_DC - g_uStartTime_DC ;
      }
    }
    //__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, g_uPulseWidth_DC);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if(htim->Instance == htim4.Instance)
  {
    g_Encoder = __HAL_TIM_GetCounter(&htim1);
    __HAL_TIM_SetCounter(&htim1, 0x00);
    if(HAL_GPIO_ReadPin(INPUT_MODE_GPIO_Port, INPUT_MODE_Pin) == MODE_AUTOPILOT && flag == 1)
    {
      pid_calculation(g_pulse_set_pid);
    }
    else
    {
      g_pulseOutput_DC = 1480;
    }
  }
}


/* USER CODE END 0 */
HAL_StatusTypeDef stt;
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  CAN1_CONFIG();
  CAN2_CONFIG();
  /*
  if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK)
  {
    HAL_CAN_ErrorCallback(&hcan1);
  }
  
  if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK)
  {
    Error_Handler();
  }*/

  HAL_Delay(500);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  HAL_Delay(500);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 1500);
  HAL_Delay(500);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 1500);
  HAL_Delay(500);

  g_pulse_set_pid = 0;
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
    if(HAL_GPIO_ReadPin(INPUT_MODE_GPIO_Port, INPUT_MODE_Pin) == MODE_COLLECT)
    {
      countToggleCollect++;
      
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  
      dataFrameTX = (datacan*)hcan1.pTxMsg->Data;
      dataFrameTX->speed = g_Encoder;
      dataFrameTX->steering = g_uPulseWidth_RC;
      /*
      hcan1.pTxMsg->Data[0] = g_Encoder;
      hcan1.pTxMsg->Data[1] = g_uPulseWidth_RC;
      hcan1.pTxMsg->Data[2] = g_uPulseWidth_RC >> 8;
      status = HAL_CAN_Transmit(&hcan1, 10);*/
      
      if(HAL_CAN_Transmit(&hcan1, 10) != HAL_OK)
      {
        HAL_CAN_ErrorCallback(&hcan1);
      }
      
      __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, g_uPulseWidth_RC);
      __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, g_uPulseWidth_DC);
      HAL_Delay(20);
      if( countToggleCollect >= 5)
      {
        countToggleCollect = 0;
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
      }
    }
    else
    {

      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
      stt = HAL_CAN_Receive(&hcan1, CAN_FIFO0, 10);
      /*if (HAL_CAN_Receive(&hcan1, CAN_FIFO0, 10)!= HAL_OK)
      {
        HAL_CAN_ErrorCallback(&hcan1);
      }*/
      if(hcan1.pRxMsg->StdId == CAN_ID_TX2_TO_STM)
      {
         HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
         dataFrameRX = (datacan*)hcan1.pRxMsg->Data;
         g_pulse_set_pid = dataFrameRX->speed;
         g_steering = dataFrameRX->steering;
         flag = dataFrameRX->flag;
      }
      __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, g_steering);
      __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, g_pulseOutput_DC);
      //HAL_Delay(10);
    }
    
  }
  /* USER CODE END 3 */
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_14TQ;
  hcan1.Init.BS2 = CAN_BS2_6TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CAN2 init function */
static void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_14TQ;
  hcan2.Init.BS2 = CAN_BS2_6TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = DISABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 15;
  if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : INPUT_PWM_RC_SERVO_Pin INPUT_PWM_DC_MOTOR_Pin */
  GPIO_InitStruct.Pin = INPUT_PWM_RC_SERVO_Pin|INPUT_PWM_DC_MOTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_MODE_Pin */
  GPIO_InitStruct.Pin = INPUT_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT_MODE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
