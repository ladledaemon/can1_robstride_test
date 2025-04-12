/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"
#include "CAN_Robstride_Def.h"
#include "CAN_Robstride.h"
#include "CAN_Robstride_System.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan3;

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint8_t master_can_id;
Robstride_DeviceInfo *robstride_dev_info_global;
Robstride_FeedbackData *robstride_fb;
uint8_t num_of_robstride;
float throw_angle = M_PI/6.0;
float release_velocity = 3.0*M_PI;
float target_velocity = 0.0f;
float target_before = 0.0f;;
float target_angular_velocity = 0.0f;
float initial_angle=0.0*M_PI/6.0;
bool release_complete = true;
bool load_complete = false;
bool throw_complete = false;
bool throw_test=false;
int velocity_count = 0;
float velocity_fb[3];
int step=1500;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ETH_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(uint8_t ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart3, &ch, 1, 500);
    return ch;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
//	printf("CAN_TxMailbox0CompleteCallback\n\r");
	Robstride_WhenTxMailboxCompleteCallbackCalled(hcan);
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan){
//	printf("CAN_TxMailbox0AbortCallback\n\r");
	Robstride_WhenTxMailboxAbortCallbackCalled(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
//	printf("CAN_TxMailbox1CompleteCallback\n\r");
	Robstride_WhenTxMailboxCompleteCallbackCalled(hcan);
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan){
//	printf("CAN_TxMailbox1AbortCallback\n\r");
	Robstride_WhenTxMailboxAbortCallbackCalled(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
//	printf("CAN_TxMailbox2CompleteCallback\n\r");
	Robstride_WhenTxMailboxCompleteCallbackCalled(hcan);
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan){
//	printf("CAN_TxMailbox2AbortCallback\n\r");
    Robstride_WhenTxMailboxAbortCallbackCalled(hcan);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
//	printf("CAN_RxFifo1MsgPendingCallback\n\r");
    Robstride_WhenCANRxFifo1MsgPending(hcan);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ETH_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_CAN1_Init();
  MX_CAN3_Init();
  /* USER CODE BEGIN 2 */
  num_of_robstride=1;
  robstride_dev_info_global=malloc(sizeof(Robstride_DeviceInfo)*num_of_robstride);
  robstride_fb=malloc(sizeof(Robstride_FeedbackData)* num_of_robstride);
  Init_Robstride_CAN_System(&hcan3);
  Robstride_Init(robstride_dev_info_global, num_of_robstride);

  if(num_of_robstride>=1){
	  for (int i = 0; i < num_of_robstride; i++) {
		  robstride_dev_info_global[i].device_id = i+1;
		  robstride_dev_info_global[i].master_id = master_can_id;
		  robstride_dev_info_global[i].device = Robstride_02;
		  robstride_dev_info_global[i].phcan = &hcan3;
		  robstride_dev_info_global[i].ctrl_param.use_internal_offset = ROBSTRIDE_USE_OFFSET_POS_INITIAL;
		  robstride_dev_info_global[i].ctrl_param.offset_pos = 2.0*M_PI;
//		  robstride_dev_info_global[i].ctrl_param.ctrl_type = ROBSTRIDE_CTRL_OPERATION;
		  robstride_dev_info_global[i].ctrl_param.ctrl_type = ROBSTRIDE_CTRL_VEL;
		  robstride_dev_info_global[i].ctrl_param.velocity_limit = ROBSTRIDE_VELOCITY_LIMIT_DISABLE;
		  robstride_dev_info_global[i].ctrl_param.current_limit = ROBSTRIDE_CURRENT_LIMIT_DISABLE;
		  robstride_dev_info_global[i].ctrl_param.torque_limit = ROBSTRIDE_TORQUE_LIMIT_DISABLE;
		  robstride_dev_info_global[i].ctrl_param.rotation = ROBSTRIDE_ROT_CW;
		  robstride_dev_info_global[i].ctrl_param.velocity_limit_size = 5.0*M_PI;
		  robstride_dev_info_global[i].ctrl_param.current_limit_size = 0.5f;
		  robstride_dev_info_global[i].ctrl_param.torque_limit_size = 10.0f;
		  robstride_dev_info_global[i].ctrl_param.quant_per_rot = 1.0f;
		  robstride_dev_info_global[i].ctrl_param.pid.kp_pos = 3.0f;
		  robstride_dev_info_global[i].ctrl_param.pid.kp_vel = 6.0f;
		  robstride_dev_info_global[i].ctrl_param.pid.ki_vel = 1.0f;
		  robstride_dev_info_global[i].ctrl_param.pid.filter_vel = 0.06f;
		  robstride_dev_info_global[i].ctrl_param.pid.kp_cur = 0.05f;
		  robstride_dev_info_global[i].ctrl_param.pid.ki_cur = 0.05f;
		  robstride_dev_info_global[i].ctrl_param.pid.filter_cur = 0.06f;
		  Robstride_Calibration(&robstride_dev_info_global[i], 1.0f, ROBSTRIDE_SWITCH_NO, GPIOB, GPIO_PIN_0, &hcan3);
		  Robstride_PresetParameters(&robstride_dev_info_global[i]);
//		  Robstride_SetAutoFeedback(&robstride_dev_info_global[i]);
	 }
	 Robstride_WaitForConnect(robstride_dev_info_global, num_of_robstride);
	 for(int i=0; i<num_of_robstride; i++){
		 Robstride_ControlEnable(&robstride_dev_info_global[i]);
	  	 Robstride_SetVelocityLimit(&robstride_dev_info_global[i]);
	  	 Robstride_SetCurrentLimit(&robstride_dev_info_global[i]);
	  	 Robstride_SetTorqueLimit(&robstride_dev_info_global[i]);
//	  	 Robstride_RequestID(&robstride_dev_info_global[i]);
	 }
  }
  int accel_count = 0;
  load_complete=true;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for(int i=0; i<num_of_robstride; i++){
		  robstride_fb[i] = Get_Robstride_FeedbackData(&robstride_dev_info_global[i]);
//		  printf("vel:%f\n\r", robstride_fb[i].velocity);
//		  printf("cur:%f\n\r", robstride_fb[i].current);
//		  printf("pos:%f\n\r", robstride_fb[i].position);
//		  printf("torque:%f\n\r", robstride_fb[i].torque);
//		  accel_count++;
//		  Robstride_SetTarget(&robstride_dev_info_global[0], 5.0*M_PI*accel_count/(float)step);
//		  Robstride_PID_Pos(&robstride_dev_info_global[i], 10.0*M_PI);
//		  Robstride_RequestID(&robstride_dev_info_global[i]);
//		  printf("MCU_ID:%d\n\r", robstride_fb[i].mcu_id);
//		  Robstride_SetTarget_Operation(&robstride_dev_info_global[i], 0.3f, M_PI*3.0, -M_PI/8.0, 3.0f, 3.0f);
		  Robstride_SetTarget(&robstride_dev_info_global[i], 4.0*M_PI);
//		  HAL_Delay(1000);
	  }
//	  robstride_fb[0] = Get_Robstride_FeedbackData(&robstride_dev_info_global[0]);
//	  float now_arm=robstride_fb[0].position+initial_angle;
//	  	  float now_angle=fmodf(now_arm, 2*M_PI);
//	  	  printf("now_angle:%f\n\r", now_angle);
//	  	  	  float now_angular_velocity=robstride_fb[0].velocity;
//	  	  	  float throw_arm_angle=3.0*M_PI/2.0+throw_angle-initial_angle;
//	  	  	printf("throw_arm_angle:%f\n\r", throw_arm_angle);
//	  	  	  velocity_fb[2]=velocity_fb[1];
//	  	  	  velocity_fb[1]=velocity_fb[0];
//	  	  	  velocity_fb[0]=now_angular_velocity;
//	  	  	  load_complete = true;
//	  	  	  if(!throw_complete){
//	  	  		target_angular_velocity = release_velocity;
//	  	  			  	  				for(int i=0; i<2; i++){
//	  	  			  	  			if(accel_count<step){
//	  	  			  	  				  	  			  	  				accel_count++;
//	  	  			  	  				  	  			  	  				}
//	  	  			  	  				}
//	  	  			  	  				if (velocity_fb[0] - release_velocity >= -0.2*M_PI
//	  	  			  	  					  					&& velocity_fb[0] - release_velocity <= 0.2*M_PI
//	  	  			  	  					  					&& velocity_fb[1] - release_velocity >= -0.2*M_PI
//	  	  			  	  					  					&& velocity_fb[1] - release_velocity <= 0.2*M_PI
//	  	  			  	  					  					&& velocity_fb[2] - release_velocity >= -0.2*M_PI
//	  	  			  	  					  					&& velocity_fb[2] - release_velocity <= 0.2*M_PI
//	  	  			  	  					  					&& now_angle - throw_arm_angle >= -0.025*M_PI
//	  	  			  	  					  					&& now_angle - throw_arm_angle <= 0.025*M_PI) {
//	  	  			  	  									HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // LD3 (RED) -> ON
//	  	  			  	  					  				HAL_GPIO_WritePin(AIRPIN_GPIO_Port, AIRPIN_Pin, GPIO_PIN_SET);
//	  	  			  	  					  				throw_complete = true;
//	  	  			  	  					  				HAL_Delay(1000);
//	  	  			  	  					  			}
////	  	  		  if(!load_complete){
////	  	  				if (now_angle / (3.0 * M_PI / 4.0) >= 0.9 && now_angle / (3.0 * M_PI / 4.0) <= 1.1) {
////	  	  					HAL_GPIO_WritePin(AIRPIN_GPIO_Port, AIRPIN_Pin, GPIO_PIN_SET);
////	  //	  					HAL_GPIO_WritePin(Air_Load_GPIO_Port, Air_Load_Pin, GPIO_PIN_SET);
////	  //	  					pid_flag = false;
////	  	  					target_angular_velocity = M_PI/24.0;
////	  	  				} else if (now_angle / (5.0 * M_PI / 4.0) >= 0.9 && now_angle / (5.0 * M_PI / 4.0) <= 1.1) {
////	  	  					HAL_GPIO_WritePin(AIRPIN_GPIO_Port, AIRPIN_Pin, GPIO_PIN_RESET);
////	  	  					load_complete = true;
////	  	  					target_angular_velocity = release_velocity;
////	  //	  					osDelay(500);
////	  //	  					HAL_GPIO_WritePin(Air_Load_GPIO_Port, Air_Load_Pin, GPIO_PIN_RESET);
////	  	  				}
////	  	  			} else{
////	  	  				target_angular_velocity = release_velocity;
////	  	  				if(accel_count<step){
////	  	  				accel_count++;
////	  	  				}
////	  	  				if (velocity_fb[0] / release_velocity >= 0.9
////	  	  					  					&& velocity_fb[0] / release_velocity <= 1.1
////	  	  					  					&& velocity_fb[1] / release_velocity >= 0.9
////	  	  					  					&& velocity_fb[1] / release_velocity <= 1.1
////	  	  					  					&& velocity_fb[2] / release_velocity >= 0.9
////	  	  					  					&& velocity_fb[2] / release_velocity <= 1.1
////	  	  					  					&& now_angle / throw_arm_angle >= 0.9
////	  	  					  					&& now_angle / throw_arm_angle <= 1.1) {
////	  	  					  				HAL_GPIO_WritePin(AIRPIN_GPIO_Port, AIRPIN_Pin, GPIO_PIN_SET);
////	  	  					  				throw_complete = true;
////	  	  					  				HAL_Delay(1000);
////	  	  					  			}
////	  	  			}
//	  	  	  }else{
//	  	  		if(accel_count>0){
//	  	  			  	  				  	  			accel_count --;
//	  	  			  	  				  	  		  }
//	  	  	  }
//	  	  	  Robstride_SetTarget(&robstride_dev_info_global[0], target_angular_velocity*accel_count/(float)step);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN3_Init(void)
{

  /* USER CODE BEGIN CAN3_Init 0 */

  /* USER CODE END CAN3_Init 0 */

  /* USER CODE BEGIN CAN3_Init 1 */

  /* USER CODE END CAN3_Init 1 */
  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 6;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = DISABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = DISABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN3_Init 2 */

  /* USER CODE END CAN3_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AIRPIN_GPIO_Port, AIRPIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AIRPIN_Pin */
  GPIO_InitStruct.Pin = AIRPIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AIRPIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
