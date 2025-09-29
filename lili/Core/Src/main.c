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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */
// ADC-DMA的循环缓冲区，用于存放ADC的转换结果
uint16_t adc_dma_buffer[64];

// UART-DMA的发送缓冲区
uint8_t uart_tx_buffer[50];


RTC_TimeTypeDef gTime;
RTC_DateTypeDef gDate;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// --- SPI Flash Helper Functions ---

// 发送和接收一个字节
uint8_t SPI_Flash_SendByte(uint8_t byte)
{
    uint8_t rx_byte;
    HAL_SPI_TransmitReceive(&hspi1, &byte, &rx_byte, 1, 100);
    return rx_byte;
}

// 读取Flash的状态寄存器
uint8_t SPI_Flash_ReadStatusRegister(void)
{
    uint8_t status;
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); // 片选拉低
    SPI_Flash_SendByte(0x05); // 发送“读取状态寄存器1”指令
    status = SPI_Flash_SendByte(0xFF); // 发送一个哑字节来接收数据
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);   // 片选拉高
    return status;
}

// 等待Flash不处于“忙”状态
void SPI_Flash_Wait_Busy(void)
{
    // 检查状态寄存器的第0位(BUSY位)，如果是1，就一直等待
    while((SPI_Flash_ReadStatusRegister() & 0x01) == 0x01);
}

// 发送“写使能”指令
void SPI_Flash_Write_Enable(void)
{
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    SPI_Flash_SendByte(0x06); // 发送“写使能”指令
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

// 擦除一个扇区 (4KB)
void SPI_Flash_Erase_Sector(uint32_t sector_addr)
{
    SPI_Flash_Write_Enable(); // 擦除前必须先写使能
    SPI_Flash_Wait_Busy();
    
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    SPI_Flash_SendByte(0x20); // 发送“扇区擦除”指令
    // 发送24位的地址
    SPI_Flash_SendByte((sector_addr & 0xFF0000) >> 16);
    SPI_Flash_SendByte((sector_addr & 0x00FF00) >> 8);
    SPI_Flash_SendByte(sector_addr & 0x0000FF);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
    
    SPI_Flash_Wait_Busy(); // 等待擦除完成
}

// 写入数据
void SPI_Flash_Write_Data(uint32_t addr, uint8_t *data, uint16_t size)
{
    SPI_Flash_Write_Enable(); // 写入前必须先写使能
    
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    SPI_Flash_SendByte(0x02); // 发送“页编程”指令
    // 发送24位地址
    SPI_Flash_SendByte((addr & 0xFF0000) >> 16);
    SPI_Flash_SendByte((addr & 0x00FF00) >> 8);
    SPI_Flash_SendByte(addr & 0x0000FF);
    
    // 发送数据
    HAL_SPI_Transmit(&hspi1, data, size, 1000);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
    
    SPI_Flash_Wait_Busy(); // 等待写入完成
}

// 读取数据
void SPI_Flash_Read_Data(uint32_t addr, uint8_t *data, uint16_t size)
{
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    SPI_Flash_SendByte(0x03); // 发送“读取数据”指令
    // 发送24位地址
    SPI_Flash_SendByte((addr & 0xFF0000) >> 16);
    SPI_Flash_SendByte((addr & 0x00FF00) >> 8);
    SPI_Flash_SendByte(addr & 0x0000FF);
    
    // 接收数据
    HAL_SPI_Receive(&hspi1, data, size, 1000);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	uint8_t startup_msg[] = "SYSTEM READY\r\n";
	HAL_UART_Transmit(&huart1, startup_msg, sizeof(startup_msg)-1, 100);
	
//	// 启动定时器3的中断模式
//	HAL_TIM_Base_Start_IT(&htim3);
	// 启动定时器3的PWM通道4
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
	// 启动ADC，并命令DMA开始将结果搬运到adc_dma_buffer中
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, 64);
	uint8_t read_data;
  uint8_t tx_buffer[50];
	
	 // 从EEPROM的地址0x00处，读取1个字节的数据
  HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, &read_data, 1, 1000);

  // 通过串口打印读到的数据
  sprintf((char*)tx_buffer, "Read from EEPROM at startup: %d\r\n", read_data);
  HAL_UART_Transmit(&huart1, tx_buffer, strlen((char*)tx_buffer), 100);
	
	uint8_t startup_ms[] = "--SPI Flash ID Reader--\r\n";
  HAL_UART_Transmit(&huart1, startup_ms, sizeof(startup_ms)-1, 100);
	
	  // 定义发送和接收的缓冲区
  uint8_t tx_data[4];
  uint8_t rx_data[4];
  uint8_t tx_buffe[100]; // 用于串口打印

  // 1. 将CS线拉低，选中Flash芯片
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  // 2. 准备要发送的指令
  tx_data[0] = 0x9F; // JEDEC ID 读取指令

  // 3. 发送指令并接收返回的数据
  //    这里我们用 TransmitReceive 函数，虽然只发1字节，但为了接收，需要一个同步的过程
  //    我们总共需要接收3个字节的ID，加上发送的1字节指令，总共通信4个字节
  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 4, 100);

  // 4. 将CS线拉高，取消选中
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  // 5. 通过串口打印接收到的ID
  //    rx_data[0] 是通信过程时钟同步的无效数据，真正的ID从rx_data[1]开始
  sprintf((char*)tx_buffe, "Manufacture ID: 0x%02X\r\n", rx_data[1]);
  HAL_UART_Transmit(&huart1, tx_buffe, strlen((char*)tx_buffe), 100);

  sprintf((char*)tx_buffe, "Device ID: 0x%02X 0x%02X\r\n", rx_data[2], rx_data[3]);
  HAL_UART_Transmit(&huart1, tx_buffe, strlen((char*)tx_buffe), 100);
	
	
	uint8_t startup_m[] = "--SPI Flash Reader/Writer--\r\n";
  HAL_UART_Transmit(&huart1, startup_m, strlen((char*)startup_m), 100);

  // 定义一个足够大的缓冲区来存放读取的字符串
  uint8_t read_buffer[100] = {0}; 
  uint8_t tx_buff[150];

  // 从Flash的地址0开始，读取20个字节
  SPI_Flash_Read_Data(0x000000, read_buffer, 20);

  // 打印读取到的内容
  sprintf((char*)tx_buff, "Data read from Flash at startup: [%s]\r\n", read_buffer);
  HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), 100);
	
	
	RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  // --- 下面是我们要设置的初始时间和日期 ---
  sTime.Hours = 18;  // 时 (24小时制)
  sTime.Minutes = 8; // 分
  sTime.Seconds = 0; // 秒

  sDate.WeekDay = RTC_WEEKDAY_SATURDAY; // 星期六
  sDate.Month = RTC_MONTH_SEPTEMBER;    // 9月
  sDate.Date = 27;                      // 27日
  sDate.Year = 25;                      // 年 (20xx年)
  // ------------------------------------

  // 将上面设置好的时间和日期写入RTC
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	
	 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 // --- 任务1: ADC -> PWM 控制 ---
	uint16_t current_adc_val = adc_dma_buffer[0];
	uint16_t pwm_pulse = current_adc_val * 999 / 4095;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_pulse);

	// --- 任务2: 读取RTC时间 ---
	// 注意：对于F1系列，必须先读时间，再读日期
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
	
	// --- 任务3: 准备并发送所有状态信息 ---
	int len = sprintf((char*)uart_tx_buffer, "ADC=%4d | PWM=%3d | Time: %02d:%02d:%02d\r\n", 
											current_adc_val, 
											pwm_pulse,
											gTime.Hours, gTime.Minutes, gTime.Seconds);

	// 统一使用DMA方式发送，避免冲突
	HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, len);

	// --- 任务4: CPU休息 ---
	// 主循环每秒执行一次，更新所有信息
	HAL_Delay(2000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  if(GPIO_Pin == KEY0_Pin)
//  {
////    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//		
//		// ����һ�������������Ҫ���͵��ַ���
//    uint8_t tx_buffer[50]; 
//		
//		// ���LED��ת���״̬
//		if(HAL_GPIO_ReadPin(LED0_GPIO_Port, LED0_Pin) == GPIO_PIN_RESET) // LED�� (�͵�ƽ��)
//		{
//			// ʹ��sprintf����Ϣ��ʽ����������
//			sprintf((char*)tx_buffer, "Button Pressed! LED is now ON\r\n");
//		}
//		else // LED��
//		{
//			sprintf((char*)tx_buffer, "Button Pressed! LED is now OFF\r\n");
//		}
//		// ���͸�ʽ�������Ϣ
//    HAL_UART_Transmit(&huart1, tx_buffer, strlen((char*)tx_buffer), 100);
//  }
//}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  // 判断是否是KEY1 (PC13) 按键触发的中断
//  if(GPIO_Pin == KEY1_Pin)
//  {
//    uint8_t write_data = 250; // 准备要写入的数据
//    uint8_t tx_buffer[50];

//    // 向EEPROM的地址0x00处，写入1个字节的数据
//    HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, &write_data, 1, 1000);

//    // 串口提示写入完成
//    sprintf((char*)tx_buffer, "Wrote %d to EEPROM!\r\n", write_data);
//    HAL_UART_Transmit(&huart1, tx_buffer, strlen((char*)tx_buffer), 100);
//	
////		uint8_t tx_buffer[100];
//    // UTF-8编码的 "蒋泽华牛逼"
//    uint8_t write_string[] = {0xE8, 0x92, 0x8B, 0xE6, 0xB3, 0xBD, 0xE5, 0x8D, 0x8E, 0xE7, 0x89, 0x9B, 0xE9, 0x80, 0xBC, 0x00};

//    // 1. 擦除地址0所在的整个扇区 (4KB)
//    sprintf((char*)tx_buffer, "Erasing sector 0...\r\n");
//    HAL_UART_Transmit(&huart1, tx_buffer, strlen((char*)tx_buffer), 100);
//    SPI_Flash_Erase_Sector(0x000000);

//    // 2. 将字符串写入地址0
//    sprintf((char*)tx_buffer, "Writing string to address 0...\r\n");
//    HAL_UART_Transmit(&huart1, tx_buffer, strlen((char*)tx_buffer), 100);
//    SPI_Flash_Write_Data(0x000000, write_string, sizeof(write_string));

//    // 3. 提示完成
//    sprintf((char*)tx_buffer, "Write complete!\r\n\r\n");
//    HAL_UART_Transmit(&huart1, tx_buffer, strlen((char*)tx_buffer), 100);	
//		
//		
//    // HAL_Delay(100); // 增加一个延时，防止抖动和过快写入
//  }
//}
//// 定时器周期溢出回调函数
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  // 判断是否是定时器3触发的中断
//  if (htim->Instance == TIM3)
//  {
//    // 翻转LED0的状态
//    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//  }
//}
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
