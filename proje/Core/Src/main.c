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
 *	PA9_TX
 *	PA10_RX

 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t saysay 		 = 0;

uint8_t VeriTextTX [ 256 ];
uint8_t VeriTextTX_MC 	 = 0;
uint8_t VeriTextTX_index = 0;

uint8_t VeriTextRX_index = 0;
uint8_t VeriTextRX_Buffer[ 1 ];
uint8_t VeriTextRX [ 256 ];
uint8_t VeriTextRX_MC 	 = 0;

uint8_t VeriTextRX_Modbus [ 256 ];
uint8_t VeriTextRX_Modbus_index = 1;
uint8_t Veri_Bitti_M 			= 0;
uint8_t Veri_Bitti_FS 			= 0;
uint8_t Veri_Bitti_ilk			= 0;

uint16_t CrC_kontrol = 0;

uint32_t sayac 		 = 0;
uint32_t sayac_byte  = 0;

uint16_t VeriBilgi_B [ 256 ];

uint8_t  Veri  = 0;
uint16_t DATA  = 0;
uint16_t DATA1 = 1;

uint16_t DATATX  = 0;
uint16_t DATA1TX = 1;
uint16_t tim_period_hlf 				= 0;
uint16_t tim_period_cplt_Counter 		= 0;
uint16_t tim_period_Counter				= 0;
uint8_t i;
uint8_t sayac1     						= 0;
uint8_t Test      					    = 0;
uint16_t pinsayac 						= 0;
uint16_t adc1      						= 0;
uint16_t adc2     						= 0;
uint16_t dac1     						= 0;
uint16_t dac2      						= 0;
uint16_t TEST      						= 0;
//Multiple Array//
uint8_t	 Data_Array  [ 256 ];
uint8_t	 Data_Array1 [ 256 ];
//READ//
uint16_t  Veri_Sakla [ 16 ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0 };
uint16_t adc_values [ 2 ];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	//HAL_GPIO_WritePin( GPIOB, (uint16_t)0x8000U , 0);
	HAL_UART_Receive_IT(&huart1, VeriTextRX_Buffer, 1);
	//HAL_TIM_Base_Start_IT ( &htim2 );
	//HAL_GPIO_WritePin( USER_PWM_GPIO_Port, USER_PWM_Pin , 0);
	VeriBilgi_B[0] = 41;
	VeriBilgi_B[1] = 0;
	VeriBilgi_B[2] = 1;
	VeriBilgi_B[3] = 0;
	VeriBilgi_B[4] = 85;
	VeriBilgi_B[5] = 69;
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE START WHILE */


			 	adc1 = 10 ;

				Veri_Sakla [ 0 ] = adc1;

				Veri_Sakla [ 1 ] = adc1;

				pinsayac = (uint16_t)0x0001U ;

				for (i = 2 ; i <= 7 ; i++)
				{
					Veri_Sakla [ i ] = HAL_GPIO_ReadPin ( GPIOC, pinsayac );
					pinsayac = pinsayac * 2;
				}
				dac1	 = 15;

				dac2	 = 15;

				pinsayac = (uint16_t)0x0040U;
				for (i = 10 ; i <= 15 ; i++)
				{
					HAL_GPIO_WritePin ( GPIOC, pinsayac, Veri_Sakla [ i ] );
					pinsayac = pinsayac*2;
				}
		 		if ( !VeriTextTX_MC ) {
					HAL_UART_Transmit_IT ( &huart1, VeriTextTX, VeriTextTX_index + 1 );
					//HAL_Delay(1000);
				}

	}
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t ModRTU_CRC(uint16_t buf[], int len) {
	uint16_t crc = 0xFFFF;

	for (int pos = 0; pos < len; pos++) {
		crc ^= (uint16_t) buf[pos];      // XOR byte into least sig. byte of crc

		for (int i = 8; i != 0; i--) {    // Loop over each bit
			if ((crc & 0x0001) != 0) {      // If the LSB is set
				crc >>= 1;                    // Shift right and XOR 0xA001
				crc ^= 0xA001;
			} else
				// Else LSB is not set
				crc >>= 1;                    // Just shift right
		}
	}
	// Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
	return crc;
}
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart ) {
	if ( huart->Instance == USART1 ) {
		VeriTextTX_MC = 1;
	}
}
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart ) {
	if ( huart->Instance == USART1 ) {
		if ( VeriTextRX_MC == 1 ) {

			HAL_TIM_Base_Stop_IT ( &htim1 );
			VeriTextRX_Modbus [ VeriTextRX_Modbus_index ] = VeriTextRX_Buffer [ 0 ];
			VeriTextRX_Modbus_index++;
		}

		Veri = VeriTextRX_Buffer [ 0 ];

		if ( Veri == 41 ) {
			VeriTextRX_MC = 1;
			VeriTextRX_Modbus [ 0 ] = VeriTextRX_Buffer [ 0 ];
			Veri_Bitti_ilk = VeriTextRX_Modbus_index;
		}
		if ( Veri_Bitti_FS != 0 ) {

			HAL_TIM_Base_Start_IT ( &htim1 );
		}
		sayac = 0;
		//HAL_TIM_Base_Start_IT(&htim1);
		Veri_Bitti_FS = 1;
		VeriTextRX [ VeriTextRX_index ] = VeriTextRX_Buffer [ 0 ];
		VeriTextRX_index++;
		HAL_UART_Receive_IT ( &huart1, VeriTextRX_Buffer, 1 );

	}
}

void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim ) {

	if ( htim -> Instance == TIM1 ) {
		sayac++;
		if ( sayac == 4 ) {
			sayac_byte++;
			sayac = 0;
			if ( sayac_byte == 7 ) {

				sayac_byte = 0;

				HAL_TIM_Base_Stop_IT(&htim1);

				Veri_Bitti( VeriTextRX_Modbus_index, 1 );
			}
		}
	}

 }


void Veri_Bitti(uint16_t son_index, uint16_t ilk_index) {

	//HAL_GPIO_WritePin(GPIOO6_GPIO_Port,GPIOO6_Pin,1);
	for (uint8_t i = 0; i < VeriTextTX_index+1; i++) {
		VeriTextTX[i] = 0;
	}
	for (uint8_t i = 0; i < son_index - 2; i++) {
		VeriBilgi_B[i] = VeriTextRX_Modbus[i];
	}
	son_index = son_index - 2 ;
	CrC_kontrol = ModRTU_CRC( VeriBilgi_B, son_index );
	VeriBilgi_B[son_index] = CrC_kontrol & 0xff;
	VeriBilgi_B[son_index + 1 ] = (CrC_kontrol >> 8) & 0xff;
	if (VeriTextRX_Modbus[son_index] == VeriBilgi_B[son_index]	&& VeriTextRX_Modbus[son_index + 1] == VeriBilgi_B[son_index+1]) {

		switch (VeriTextRX_Modbus[ilk_index]) {

		case (3):                    //Read Multiple
				//////////////////////////////////////////
				//Data register
				DATA  = ( VeriTextRX_Modbus [ ilk_index + 1 ] << 8 )| VeriTextRX_Modbus [ ilk_index + 2 ];
				//kaç tane alınacağı bilgisi
				DATA1 = ( VeriTextRX_Modbus [ ilk_index + 3 ] << 8  )| VeriTextRX_Modbus [ ilk_index + 4 ];
				//////////////////////////////////////////

					VeriTextTX [ 0 ] = 41;
					VeriTextTX [ 1 ] = 03;
					VeriTextTX [ 2 ] = DATA1 * 2;
					Test = DATA;

					for ( i = 0 ; i < DATA1 ; i++ )
					{
						Test++;
						Data_Array [ i ] = Veri_Sakla [ Test ];
					}
					for( i = 0 ; i <= DATA1 ; i++ )
					{
						Data_Array1 [ sayac1 ] = Data_Array [ i ] >> 8;
						sayac1++;
						Data_Array1 [ sayac1 ] = Data_Array [ i ] ;
						sayac1++;

					}
					for ( i = 3 ; i < 3 + sayac1  ; i++ )
					{
						VeriTextTX [ i ] = Data_Array1 [ i - 3 ];
					}
					// CRC hesaplama
					CrC_kontrol      = 0;
					CrC_kontrol      = ModRTU_CRC( VeriTextTX, sayac1 + 3 );
					VeriTextTX [ sayac1 + 1 ] =   CrC_kontrol & 0xff;
					VeriTextTX [ sayac1 + 2 ] = ( CrC_kontrol >> 8 ) & 0xff;
					VeriTextTX_index = sayac1 + 3 ;
			break;


		case (4):                    //Read Single

		//////////////////////////////////////////
		//Data register
		DATA  = ( VeriTextRX_Modbus [ ilk_index + 1 ] << 8 )| VeriTextRX_Modbus [ ilk_index + 2 ];
		//kaç tane alınacağı bilgisi
		DATA1 = ( VeriTextRX_Modbus [ ilk_index + 3 ] << 8  )| VeriTextRX_Modbus [ ilk_index + 4 ];
		//////////////////////////////////////////

			VeriTextTX [ 0 ] = 41;
			VeriTextTX [ 1 ] = 04;

			VeriTextTX [ 2 ] = 2;

			DATA1TX 		 = Veri_Sakla[ DATA ];
			VeriTextTX [ 3 ] = DATA1TX >> 8;
			VeriTextTX [ 4 ] = DATA1TX ;

			// CRC hesaplama
			CrC_kontrol      = 0;
			CrC_kontrol      =   ModRTU_CRC(VeriTextTX, 5);
			VeriTextTX [ 5 ] =  CrC_kontrol & 0xff;
			VeriTextTX [ 6 ] = (CrC_kontrol >> 8) & 0xff;
			VeriTextTX_index = 6;


			break;
		case (6):
					//Write Single
				    //////////////////////////////////////////
				    //Data register
					DATA  = ( VeriTextRX_Modbus [ ilk_index + 1 ] << 8 )| VeriTextRX_Modbus [ ilk_index + 2 ];
				    //Moment
				    DATA1 = ( VeriTextRX_Modbus [ ilk_index + 3 ] << 8  )| VeriTextRX_Modbus [ ilk_index + 4 ];
				    //////////////////////////////////////////

				    Veri_Sakla [ DATA ] = DATA1;

				    VeriTextTX [ 0 ] = 41;
					VeriTextTX [ 1 ] = 06;

					VeriTextTX [ 2 ] = DATA >> 8;
					VeriTextTX [ 3 ] = DATA;

					DATA1TX 		 = Veri_Sakla [ DATA ];
					VeriTextTX [ 4 ] = DATA1TX >> 8;
					VeriTextTX [ 5 ] = DATA1TX ;

					// CRC hesaplama
					CrC_kontrol      = 0;
					CrC_kontrol      =  ModRTU_CRC(VeriTextTX, 6);
					VeriTextTX [ 6 ] = CrC_kontrol & 0xff;
					VeriTextTX [ 7 ] = (CrC_kontrol >> 8) & 0xff;
					VeriTextTX_index = 7;
			break;
		case (16):                  //Write Multiple
						//////////////////////////////////////////
						//İlk Register
						DATA  = ( VeriTextRX_Modbus [ ilk_index + 1 ] << 8 )| VeriTextRX_Modbus [ ilk_index + 2 ];
						//kaç tane alınacağı bilgisi
						DATA1 = (VeriTextRX_Modbus [ ilk_index + 3 ] << 8 )  | VeriTextRX_Modbus [ ilk_index + 4 ];
						//////////////////////////////////////////
							for (i = 0 ; i < DATA1*2 ; i+=2)
							{

								Veri_Sakla [ DATA + sayac1 ] = ( VeriTextRX_Modbus [ ilk_index + 6 + i ] << 8 ) | VeriTextRX_Modbus [ ilk_index + 7 + i ] ;
								sayac1++;
							}
							VeriTextTX [ 0 ] = 41;
							VeriTextTX [ 1 ] = 16;
							VeriTextTX [ 2 ] = DATA >> 8;
							VeriTextTX [ 3 ] = DATA;
							VeriTextTX [ 4 ] = DATA1 >> 8;
							VeriTextTX [ 5 ] = DATA1;
							// CRC hesaplama
							CrC_kontrol      = 0;
							CrC_kontrol      = ModRTU_CRC( VeriTextTX, 6 );
							VeriTextTX [ 6 ] =   CrC_kontrol & 0xff;
							VeriTextTX [ 7 ] = ( CrC_kontrol >> 8 ) & 0xff;
							VeriTextTX_index = 8 ;
			break;
		default:                    //default

			break;

		}
		for (uint8_t i = 0; i <= 254; i++) {
					Data_Array[i] = 0;
				}
		for (uint8_t i = 0; i < 16; i++) {
					Data_Array1 [i] = 0;
				}
		for (uint8_t i = 0; i <= 254; i++) {
			VeriBilgi_B[i] = 0;
		}
		for (uint8_t i = 0; i <= 254; i++) {
			VeriTextRX_Modbus[i] = 0;
		}
		sayac1					= 0;
		VeriTextRX_Modbus_index = 0;
		VeriTextTX_MC = 0;
 		VeriTextRX_MC = 0;
		Veri_Bitti_FS = 0;
		VeriTextRX_Modbus_index = 1;
		//HAL_GPIO_WritePin(GPIOO6_GPIO_Port,GPIOO6_Pin,0);
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
	while (1) {
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

