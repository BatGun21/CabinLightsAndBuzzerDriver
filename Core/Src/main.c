/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Clock_Frequency 8000 //KHz
#define GPIO_PORT_LEDS GPIOC
#define GPIO_PORT_CABINLEDS GPIOB
#define GPIO_PORT_BUZZER GPIOB
#define GPIO_PORT_SWITCH GPIOA
#define GPIO_PORT_DOORSWITCHES GPIOA
#define GPIO_PORT_DICKEYSWITCH GPIOB
#define GPIO_PORT_MOTORDRIVE GPIOB
#define GPIO_PORT_CAB_ON_DOOR GPIOB
#define GPIO_RED_LED_PIN GPIO_PIN_6
#define GPIO_BLUE_LED_PIN GPIO_PIN_7
#define GPIO_ORANGE_LED_PIN GPIO_PIN_8
#define GPIO_GREEN_LED_PIN GPIO_PIN_9
#define GPIO_PIN_BUZZER_CTRL GPIO_PIN_2
#define GPIO_PIN_CAB_LIGHT_FRNT_CTRL GPIO_PIN_3
#define GPIO_PIN_CAB_LIGHT_REAR_CTRL GPIO_PIN_4
#define GPIO_PIN_CAB_LIGHT_DICKEY_CTRL GPIO_PIN_5
#define GPIO_PIN_CAB_DOOR_SW_DICKEY GPIO_PIN_6
#define GPIO_PIN_CAB_DOOR_SW_REAR_L GPIO_PIN_15
#define GPIO_PIN_CAB_DOOR_SW_REAR_R GPIO_PIN_8
#define GPIO_PIN_CAB_DOOR_SW_FRNT_L GPIO_PIN_5
#define GPIO_PIN_CAB_DOOR_SW_FRNT_R GPIO_PIN_4
#define GPIO_PIN_MOTORDRIVE_SIGNAL GPIO_PIN_7
#define GPIO_PIN_CAB_ON_DOOR_SIGNAL GPIO_PIN_8
#define KillSwitch_PIN GPIO_PIN_0
#define ON 1
#define OFF 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int killSwitchFlagRE = 0;
int counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void EXTI_Init(void);
void SetWaitOneSec(void);
void DelayMSW(unsigned int time);
void LED_init(void);
void SysTick_Init(uint32_t ticks);
uint16_t debounceSwitch(uint16_t pin);
int time_expired (int delayTime, int currentTime);
void FrontLightRelayCTRL(int state);
void BackLightRelayCTRL(int state);
void DickeyLightRelayCTRL(int state);
void BuzzerCTRL(int state);
void ConfigureOutputPins(void);
void ConfigureInputPins(void);
void killSwitch_Handler(void);
int Check_Front_Door_Switches(void);
int Check_Rear_Door_Switches(void);
int Check_Dickey_Door_Switch(void);
int Check_Motor_Drive_Signal(void);
int Check_Cab_On_Door_Signal(void);
void BuzzerDriver(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Wait {
	int currentTime;
	int delayTime ;
	int activeFlag;

};

struct Wait OneSec = {0, 1000, 0};
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
  SysTick_Init(Clock_Frequency);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  LED_init();
  ConfigureOutputPins();
  ConfigureInputPins();
  EXTI_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  SetWaitOneSec();
//	  if (time_expired(OneSec.delayTime, OneSec.currentTime)){
//		  GPIO_PORT_CABINLEDS->ODR ^= GPIO_PIN_CAB_LIGHT_FRNT_CTRL;
//		  GPIO_PORT_CABINLEDS->ODR ^= GPIO_PIN_CAB_LIGHT_REAR_CTRL;
//		  GPIO_PORT_CABINLEDS->ODR ^= GPIO_PIN_CAB_LIGHT_DICKEY_CTRL;
//		  GPIO_PORT_BUZZER->ODR ^= GPIO_PIN_BUZZER_CTRL;
//		  OneSec.activeFlag = 0;
//	  }

	  if (killSwitchFlagRE){
		  killSwitch_Handler();
	  }else{
		  if(Check_Motor_Drive_Signal()){
			  BuzzerDriver();
		  }

		  if(Check_Cab_On_Door_Signal()){

			  if (Check_Front_Door_Switches()){
				  FrontLightRelayCTRL(ON);
			  }else{
				  FrontLightRelayCTRL(OFF);
			  }

			  if (Check_Rear_Door_Switches()){
				  BackLightRelayCTRL(ON);
			  }else{
				  BackLightRelayCTRL(OFF);
			  }

			  if(Check_Dickey_Door_Switch()){
				  DickeyLightRelayCTRL(ON);
			  }else{
				  DickeyLightRelayCTRL(OFF);
			  }
		  }
	  }
    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void killSwitch_Handler(void){
	  uint16_t pin = 0;
	  pin  = debounceSwitch(GPIO_PORT_SWITCH->IDR & KillSwitch_PIN);
	  if (pin!=0){
		  //Turn of all Relays
		  FrontLightRelayCTRL(OFF);
		  BackLightRelayCTRL(OFF);
		  DickeyLightRelayCTRL(OFF);
		  BuzzerCTRL(OFF);
		  while(1){
			//Halt Operation
			  SetWaitOneSec();
			  if (time_expired(OneSec.delayTime, OneSec.currentTime)){
				  GPIO_PORT_LEDS->ODR ^= GPIO_RED_LED_PIN; //Error Blink
				  OneSec.activeFlag = 0;
			  }
		  }
	  }
}

void SetWaitOneSec(void){
	  if (!OneSec.activeFlag){
		  OneSec.currentTime = counter;
		  OneSec.activeFlag = 1;
	  }
}

void EXTI0_1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) { // Check if EXTI Line 0 triggered the interrupt
    	killSwitchFlagRE = 1;
    }
    EXTI->PR = EXTI_PR_PR0; // Clear the interrupt pending bit by writing '1' to it
}

uint16_t debounceSwitch(uint16_t pin){
	uint16_t currPin = 0;
	uint16_t temp = 0;
	temp = pin;
	DelayMSW(1);
	if (pin==temp){
		DelayMSW(1);
		if (pin==temp){
			DelayMSW(1);
			if (pin==temp){
				currPin = temp;
			}
		}
	}else{
		currPin = pin;
	}
	return currPin;
}

void EXTI_Init(void) {
	  // Enabling Clock for Port A
	  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	  // Enabling Input for PA0
	  GPIOA->MODER &= 0xfffffffc;
	  // Setting the speed of the pin PA0 to High Speed
	  GPIOA->OSPEEDR |= 0x00000003;
	  // Enabling Pull Down for PA0
	  GPIOA->PUPDR |= 0x00000002;

	  // Configure EXTI Line 0 for PA0 with a rising edge trigger
	  EXTI->IMR |= EXTI_IMR_MR0; // Enable interrupt on line 0
	  EXTI->RTSR |= EXTI_RTSR_TR0; // Trigger on rising edge

	  // Enable EXTI0_1_IRQn (EXTI Line 0 and 1) in the NVIC
	  NVIC_EnableIRQ(EXTI0_1_IRQn);
	  NVIC_SetPriority(EXTI0_1_IRQn, 0);
}

void LED_init(void){
	  // Enabling Clock for Port C
	  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	  GPIOC->MODER |= GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0;
}

int time_expired (int delayTime, int currentTime){
	int timeExpiredFlag = 0;
	if (counter> currentTime+delayTime){
		timeExpiredFlag = 1;
	}else{
		timeExpiredFlag = 0;
	}
	return timeExpiredFlag;
}

void SysTick_Init(uint32_t ticks){

	SysTick->CTRL = 0; // Disable SysTick

	SysTick->LOAD = ticks-1; // Set Reload Register

	// Setting Interrupt Priority to the highest
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1);

	SysTick->VAL = 0; // Reset the SysTick counter value

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // Selecting internal clock source
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Enabling SysTick exception Request when 0


	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable SysTick
}

void DelayMSW(unsigned int time){
	for(int i=0; i<=time; i++){
		while ((SysTick->CTRL & 0x00010000) == 0){
				//Wait for 1 millisec.
		}
	}
}

void SysTick_Handler(void) {

	if (counter == 0xffffffff) {
        counter = 0; // Reset the counter if the maximum value is reached
    } else {
        counter++; // Increment the counter
    }
}

void FrontLightRelayCTRL(int state){
	if (state){
		GPIO_PORT_CABINLEDS->ODR |= GPIO_PIN_CAB_LIGHT_FRNT_CTRL; // Front CabinLight is ON
	}else{
		GPIO_PORT_CABINLEDS->ODR &= ~(GPIO_PIN_CAB_LIGHT_FRNT_CTRL); // Front CabinLight is OFF
	}
}

void BackLightRelayCTRL(int state){
	if (state){
		GPIO_PORT_CABINLEDS->ODR |= GPIO_PIN_CAB_LIGHT_REAR_CTRL;  // Back CabinLight is ON
	}else{
		GPIO_PORT_CABINLEDS->ODR &= ~(GPIO_PIN_CAB_LIGHT_REAR_CTRL); // Back CabinLight is OFF
	}
}

void DickeyLightRelayCTRL(int state){
	if (state){
		GPIO_PORT_CABINLEDS->ODR |= GPIO_PIN_CAB_LIGHT_DICKEY_CTRL;  // Dickey CabinLight is ON
	}else{
		GPIO_PORT_CABINLEDS->ODR &= ~(GPIO_PIN_CAB_LIGHT_DICKEY_CTRL); // Dickey CabinLight is OFF
	}
}

void BuzzerCTRL(int state){
	if (state){
		GPIO_PORT_BUZZER->ODR |= GPIO_PIN_BUZZER_CTRL;  // Buzzer is ON
	}else{
		GPIO_PORT_BUZZER->ODR &= ~(GPIO_PIN_BUZZER_CTRL); // Buzzer is OFF
	}
}

void BuzzerDriver(void){
	if (Check_Front_Door_Switches()||Check_Rear_Door_Switches()||Check_Dickey_Door_Switch()){
		BuzzerCTRL(ON);
	}else{
		BuzzerCTRL(OFF);
	}
}

void ConfigureOutputPins(void){

	// Enable clock for GPIO Port B and C
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    // Configure pins PB3, PB4, PB5 output
    GPIO_PORT_CABINLEDS->MODER |= GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0;

    // Configure pins PC3, PC4, PC15 as high-speed output
    GPIO_PORT_CABINLEDS->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5;

    // Set pins PB3, PB4, PB5 as push-pull
    GPIO_PORT_CABINLEDS->OTYPER &= ~(GPIO_OTYPER_OT_3 | GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT_5);

    // Configure pin PB2 output
    GPIO_PORT_BUZZER->MODER |= GPIO_MODER_MODER2_0;

    // Configure pin PB2 as high-speed output
    GPIO_PORT_BUZZER->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;

    // Set pin PB2 as push-pull
    GPIO_PORT_BUZZER->OTYPER &= ~GPIO_OTYPER_OT_2;
}

void ConfigureInputPins(void){
    // Enable clock for GPIO Port A and Port B
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    // Configure PA4, PA5, PA8, PA15 as digital input with internal pull-up for Front Right/Left and Back  Right/Left Switch
    GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER8 | GPIO_MODER_MODER15); // Clear bits
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0 | GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR15_0; // Set pull-up

    // Configure PB6 as digital input with internal pull-up for Dickey Switch
    GPIOB->MODER &= ~GPIO_MODER_MODER6; // Clear bits
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0; // Set pull-up

    // Configure PB7 and PB8 as digital input with internal pull-down for Motor Drive Input and Cab_ON_Signal
    GPIOB->MODER &= ~(GPIO_MODER_MODER7 | GPIO_MODER_MODER8); // Clear bits
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_1 | GPIO_PUPDR_PUPDR8_1; // Set pull-down
}

int Check_Front_Door_Switches(void){
    int state = 0;
    int frontLeftSwitch = GPIO_PORT_DOORSWITCHES->IDR & GPIO_PIN_CAB_DOOR_SW_FRNT_L;
    int frontRightSwitch = GPIO_PORT_DOORSWITCHES->IDR & GPIO_PIN_CAB_DOOR_SW_FRNT_R;

    if (frontLeftSwitch == 0 || frontRightSwitch == 0) {
        int frontLeftDebounced = debounceSwitch(frontLeftSwitch);
        int frontRightDebounced = debounceSwitch(frontRightSwitch);

        if (frontLeftDebounced == 0 || frontRightDebounced == 0) {
            state = 1;
        }
    }

    return state;
}

int Check_Rear_Door_Switches(void){
    int state = 0;
    int rearLeftSwitch = GPIO_PORT_DOORSWITCHES->IDR & GPIO_PIN_CAB_DOOR_SW_REAR_L;
    int rearRightSwitch = GPIO_PORT_DOORSWITCHES->IDR & GPIO_PIN_CAB_DOOR_SW_REAR_R;

    if (rearLeftSwitch == 0 || rearRightSwitch == 0) {
        int rearLeftDebounced = debounceSwitch(rearLeftSwitch);
        int rearRightDebounced = debounceSwitch(rearRightSwitch);

        if (rearLeftDebounced == 0 || rearRightDebounced == 0) {
            state = 1;
        }
    }

    return state;
}

int Check_Dickey_Door_Switch(void){
    int state = 0;
    int dickeySwitch = GPIO_PORT_DICKEYSWITCH->IDR & GPIO_PIN_CAB_DOOR_SW_DICKEY;

    if (dickeySwitch == 0) {
        int dickeyDebounced = debounceSwitch(dickeySwitch);
        if (dickeyDebounced == 0) {
            state = 1;
        }
    }

    return state;
}

int Check_Motor_Drive_Signal(void){
	if (((GPIO_PORT_MOTORDRIVE->IDR & GPIO_PIN_MOTORDRIVE_SIGNAL) != 0)){
		return 1;
	}else{
		return 0;
	}
}

int Check_Cab_On_Door_Signal(void){
	if (((GPIO_PORT_CAB_ON_DOOR->IDR & GPIO_PIN_CAB_ON_DOOR_SIGNAL) != 0)){
		return 1;
	}else{
		return 0;
	}
}

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
