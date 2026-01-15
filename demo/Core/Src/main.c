/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* USER CODE BEGIN PV */

/* Command Buffer - SIMPLIFIED */
#define CMD_BUFFER_SIZE  32
volatile char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t cmd_index = 0;
volatile uint8_t cmd_ready = 0;

/* LED State */
volatile uint8_t led_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Process_Command(void);
void Send_String(const char* str);
void Send_Status(void);
void UART_Restart_Receive(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* UART Receive Interrupt - FIXED & DEBUGGED */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    char rx_char = cmd_buffer[cmd_index];

    // **ECHO CHARACTER BACK** - So you can see what you type!
    HAL_UART_Transmit(&huart3, (uint8_t*)&rx_char, 1, 10);

    // Check for Enter key (Carriage Return)
    if (rx_char == '\r')
    {
      cmd_buffer[cmd_index] = '\0';  // Null-terminate string
      cmd_ready = 1;                 // Mark command ready
      Send_String("\r\n");           // New line
    }
    // Backspace handling
    else if (rx_char == '\b' || rx_char == 127)
    {
      if (cmd_index > 0)
      {
        cmd_index--;
        Send_String("\b \b");  // Erase character on screen
      }
    }
    // Normal character
    else if (cmd_index < CMD_BUFFER_SIZE - 1)
    {
      cmd_index++;
    }

    // **RESTART RECEIVE** for next character
    UART_Restart_Receive();
  }
}

/* Button Interrupt */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == USER_Btn_Pin)
  {
    led_state = !led_state;
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, led_state);
    Send_String("\r\n[BTN] LED toggled to ");
    Send_String(led_state ? "ON\r\n> " : "OFF\r\n> ");
  }
}

/* Send string via UART */
void Send_String(const char* str)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);
}

/* Send LED status */
void Send_Status(void)
{
  char status_msg[64];
  sprintf(status_msg, "\r\n[STATUS] LED is %s\r\n> ", led_state ? "ON" : "OFF");
  Send_String(status_msg);
}

/* Restart UART receive */
void UART_Restart_Receive(void)
{
  HAL_UART_Receive_IT(&huart3, (uint8_t*)&cmd_buffer[cmd_index], 1);
}

/* Process command */
void Process_Command(void)
{
  char cmd[32];
  strcpy(cmd, (char*)cmd_buffer);  // Copy command

  // Convert to uppercase
  for(int i = 0; cmd[i]; i++) {
    cmd[i] = toupper(cmd[i]);
  }

  if (strcmp(cmd, "ON") == 0)
  {
    led_state = 1;
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, led_state);
    Send_String("LED turned ON\r\n> ");
  }
  else if (strcmp(cmd, "OFF") == 0)
  {
    led_state = 0;
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, led_state);
    Send_String("LED turned OFF\r\n> ");
  }
  else if (strcmp(cmd, "STATUS") == 0)
  {
    Send_Status();
  }
  else if (strcmp(cmd, "HELP") == 0)
  {
    Send_String("\r\n--- Commands ---\r\n");
    Send_String("ON     : Turn LED ON\r\n");
    Send_String("OFF    : Turn LED OFF\r\n");
    Send_String("STATUS : Show LED status\r\n");
    Send_String("HELP   : Show this help\r\n");
    Send_String("Press USER button to toggle LED\r\n> ");
  }
  else if (strlen(cmd) > 0)
  {
    Send_String("Unknown: ");
    Send_String(cmd);
    Send_String("\r\nType HELP for commands\r\n> ");
  }

  // Reset buffer
  cmd_index = 0;
  cmd_ready = 0;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  // ** CRITICAL: Start UART receive interrupt **
  UART_Restart_Receive();

  // Send welcome message
  Send_String("\r\n\r\n========================================\r\n");
  Send_String("  Nucleo-F746ZG Control System\r\n");
  Send_String("========================================\r\n");
  Send_String("Type HELP and press ENTER\r\n> ");

  // Flash LED to show system is alive
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    // Process UART commands
    if (cmd_ready)
    {
      Process_Command();
    }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

/* USER CODE END 4 */

/* MPU Configuration */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
    // Blink LED fast to indicate error
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
