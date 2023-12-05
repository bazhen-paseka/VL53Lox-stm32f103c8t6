/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include <string.h>
	#include "stdio.h"
	#include "i2c_techmaker_sm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#define		UART_DEBUG &huart1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

	void UartDebug(char* _text) ;
	void Scan_I2C_to_UART(void) ;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	char		debugString[0XFF]	= { 0 } ;
	sprintf(debugString,"\r\n\r\n\tVL-53-LOX stm32f103c8t6 \r\n" ) ;
	UartDebug(debugString);
	#define 	DATE_as_int_str 	(__DATE__)
	#define 	TIME_as_int_str 	(__TIME__)
	sprintf(debugString,"\tBuild: %s. Time: %s.\r\n" ,	DATE_as_int_str , TIME_as_int_str ) ;
	UartDebug(debugString);

	I2Cdev_init(&hi2c1);
	I2C_ScanBusFlow(&hi2c1, &huart1);

	#define ADR	0x29
	uint8_t vl53[10]  ={0};

	I2Cdev_readBytes( ADR, 0 , 4,vl53, 100);
	sprintf(debugString,"0x00: %d %d %d %d\r\n" ,	vl53[0], vl53[1],vl53[2],vl53[3] ) ;
	UartDebug(debugString);

	I2Cdev_readBytes( ADR, 0xA , 4,vl53, 100);
	sprintf(debugString,"0x0A: %d %d %d %d\r\n" ,	vl53[0], vl53[1],vl53[2],vl53[3]  ) ;
	UartDebug(debugString);

	I2Cdev_readBytes( ADR, 0x8A , 4,vl53, 100);
	sprintf(debugString,"I2C adr: %x %d %d %d\r\n" ,	vl53[0], vl53[1],vl53[2],vl53[3]  ) ;
	UartDebug(debugString);

	I2Cdev_readBytes( ADR, 0xC0 , 4,vl53, 100);
	sprintf(debugString,"MODEL_ID: %x %d %d %d\r\n" ,	vl53[0], vl53[1],vl53[2],vl53[3]  ) ;
	UartDebug(debugString);

	I2Cdev_readBytes( ADR, 0xC2 , 4,vl53, 100);
	sprintf(debugString,"REVISION_ID: %x %d %d %d\r\n" ,	vl53[0], vl53[1],vl53[2],vl53[3]  ) ;
	UartDebug(debugString);

//	I2Cdev_readBytes( ADR, 0x8A , 4,vl53, 100);
//	sprintf(debugString,"I2C adr: %x %d %d %d\r\n" ,	vl53[0], vl53[1],vl53[2],vl53[3]  ) ;
//	UartDebug(debugString);

	//Scan_I2C_to_UART();
	//HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint8_t value[0xFF]  ={0};
		I2Cdev_readBytes( ADR, 0 , 0xff, value, 100);
	  for (int i=50; i<100; i++) {
			sprintf(debugString,"%02x " ,value[i] ) ;
			UartDebug(debugString);
	  }
	  sprintf(debugString,"\r" ) ;
	  			UartDebug(debugString);
	  HAL_Delay(100);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
}

/* USER CODE BEGIN 4 */
void UartDebug(char* _text) {
	HAL_UART_Transmit(UART_DEBUG, (uint8_t*)_text, strlen(_text), 100);
} //***********************************************************************

void Scan_I2C_to_UART(void ) {
	char DataChar[100];
	int device_serial_numb = 0;

	sprintf(DataChar,"Start scan I2C:\r\n" ) ;
	HAL_UART_Transmit(UART_DEBUG, (uint8_t *)DataChar, strlen(DataChar), 1000 ) ;

	for ( int device_i2c_address_int = 0x07; device_i2c_address_int < 0x78; device_i2c_address_int++) {
		if (HAL_I2C_IsDeviceReady( &hi2c1 , device_i2c_address_int << 1, 10, 100) == HAL_OK) {

			switch (device_i2c_address_int) {
				case 0x23: sprintf(DataChar,"%d) BH1750", device_serial_numb ); break;
				case 0x27: sprintf(DataChar,"%d) FC113 ", device_serial_numb ); break;
				case 0x29: sprintf(DataChar,"%d) VL53LOX", device_serial_numb ); break;
				case 0x38: sprintf(DataChar,"%d) PCF8574", device_serial_numb ); break;
				//case 0x57: sprintf(DataChar,"%d) AT24C32", device_serial_numb ); break;
				case 0x57: sprintf(DataChar,"%d) MAX30100", device_serial_numb ); break;
				case 0x68: sprintf(DataChar,"%d) DS3231", device_serial_numb ); break;
				//case 0x68: sprintf(DataChar_I2C,"%d) MPU9250", device_serial_numb ); break;
				case 0x76: sprintf(DataChar,"%d) BMP280", device_serial_numb ); break;
				case 0x77: sprintf(DataChar,"%d) BMP180", device_serial_numb ); break;
				default:   sprintf(DataChar,"%d) Unknown", device_serial_numb ); break;
			}// end switch
			device_serial_numb++;

			char DataChar2[150];
			sprintf(DataChar2,"%s\tAdr: 0x%x 0x%x\r\n", DataChar, device_i2c_address_int, (device_i2c_address_int<<1) ) ;
			HAL_UART_Transmit(UART_DEBUG, (uint8_t *)DataChar2, strlen(DataChar2), 1000 ) ;

		} //end if HAL I2C1
	} // end for device_i2c_address_int i2c1
	sprintf(DataChar,"End scan I2C.\r\n" ) ;
	HAL_UART_Transmit(UART_DEBUG, (uint8_t *)DataChar, strlen(DataChar), 1000 ) ;
}
//======================================================================

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
