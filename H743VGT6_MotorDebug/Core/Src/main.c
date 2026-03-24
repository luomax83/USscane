/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : UART2 <-> UART4 transparent bridge for motor debugging
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t data[256];
  volatile uint16_t length;
  volatile uint32_t last_rx_ms;
  volatile uint32_t frames_forwarded;
  volatile uint32_t bytes_forwarded;
  volatile uint32_t overflow_count;
} BridgeBuffer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BRIDGE_IDLE_GAP_MS 2U
#define BRIDGE_FORWARD_TIMEOUT_MS 2000U
#define HOST_RS485_TX_PORT GPIOD
#define HOST_RS485_TX_PIN GPIO_PIN_7
#define MOTOR_RS485_TX_PORT GPIOB
#define MOTOR_RS485_TX_PIN GPIO_PIN_7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t g_uart2_rx_byte = 0U;
static uint8_t g_uart4_rx_byte = 0U;
static BridgeBuffer g_host_to_motor;
static BridgeBuffer g_motor_to_host;
static uint8_t g_bridge_tx_buffer[256];
volatile uint32_t g_debug_fault_marker = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void App_StartBridgeReception(void);
static void App_ProcessBridge(void);
static void App_RestartRx(UART_HandleTypeDef *huart);
static void App_BufferPushByte(BridgeBuffer *buffer, uint8_t byte);
static uint8_t App_BufferReadyToFlush(const BridgeBuffer *buffer, uint32_t now_ms);
static uint16_t App_TakeBufferSnapshot(BridgeBuffer *buffer, uint8_t *target, uint16_t target_size);
static void App_SendToMotor(const uint8_t *data, uint16_t length);
static void App_SendToHost(const uint8_t *data, uint16_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_StartBridgeReception(void)
{
  HAL_GPIO_WritePin(HOST_RS485_TX_PORT, HOST_RS485_TX_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_RS485_TX_PORT, MOTOR_RS485_TX_PIN, GPIO_PIN_RESET);

  __HAL_UART_CLEAR_PEFLAG(&huart2);
  __HAL_UART_CLEAR_FEFLAG(&huart2);
  __HAL_UART_CLEAR_NEFLAG(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  SET_BIT(huart2.Instance->RQR, USART_RQR_RXFRQ);

  __HAL_UART_CLEAR_PEFLAG(&huart4);
  __HAL_UART_CLEAR_FEFLAG(&huart4);
  __HAL_UART_CLEAR_NEFLAG(&huart4);
  __HAL_UART_CLEAR_OREFLAG(&huart4);
  SET_BIT(huart4.Instance->RQR, USART_RQR_RXFRQ);

  (void)HAL_UART_Receive_IT(&huart2, &g_uart2_rx_byte, 1U);
  (void)HAL_UART_Receive_IT(&huart4, &g_uart4_rx_byte, 1U);
}

static void App_ProcessBridge(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (App_BufferReadyToFlush(&g_host_to_motor, now_ms) != 0U)
  {
    uint16_t length = App_TakeBufferSnapshot(&g_host_to_motor, g_bridge_tx_buffer, sizeof(g_bridge_tx_buffer));
    if (length > 0U)
    {
      App_SendToMotor(g_bridge_tx_buffer, length);
    }
  }

  if (App_BufferReadyToFlush(&g_motor_to_host, now_ms) != 0U)
  {
    uint16_t length = App_TakeBufferSnapshot(&g_motor_to_host, g_bridge_tx_buffer, sizeof(g_bridge_tx_buffer));
    if (length > 0U)
    {
      App_SendToHost(g_bridge_tx_buffer, length);
    }
  }
}

static void App_RestartRx(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_PEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    SET_BIT(huart2.Instance->RQR, USART_RQR_RXFRQ);
    (void)HAL_UART_Receive_IT(&huart2, &g_uart2_rx_byte, 1U);
  }
  else if (huart->Instance == UART4)
  {
    __HAL_UART_CLEAR_PEFLAG(&huart4);
    __HAL_UART_CLEAR_FEFLAG(&huart4);
    __HAL_UART_CLEAR_NEFLAG(&huart4);
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    SET_BIT(huart4.Instance->RQR, USART_RQR_RXFRQ);
    (void)HAL_UART_Receive_IT(&huart4, &g_uart4_rx_byte, 1U);
  }
}

static void App_BufferPushByte(BridgeBuffer *buffer, uint8_t byte)
{
  if (buffer->length < sizeof(buffer->data))
  {
    buffer->data[buffer->length++] = byte;
  }
  else
  {
    buffer->overflow_count++;
  }
  buffer->last_rx_ms = HAL_GetTick();
}

static uint8_t App_BufferReadyToFlush(const BridgeBuffer *buffer, uint32_t now_ms)
{
  return (buffer->length > 0U) && ((now_ms - buffer->last_rx_ms) >= BRIDGE_IDLE_GAP_MS);
}

static uint16_t App_TakeBufferSnapshot(BridgeBuffer *buffer, uint8_t *target, uint16_t target_size)
{
  uint16_t length = 0U;

  __disable_irq();
  if (buffer->length > 0U)
  {
    length = buffer->length;
    if (length > target_size)
    {
      length = target_size;
    }
    memcpy(target, buffer->data, length);
    buffer->length = 0U;
  }
  __enable_irq();

  return length;
}

static void App_SendToMotor(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  HAL_GPIO_WritePin(MOTOR_RS485_TX_PORT, MOTOR_RS485_TX_PIN, GPIO_PIN_SET);
  HAL_Delay(1U);
  (void)HAL_UART_Transmit(&huart4, (uint8_t *)data, length, BRIDGE_FORWARD_TIMEOUT_MS);
  while (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET)
  {
  }
  HAL_GPIO_WritePin(MOTOR_RS485_TX_PORT, MOTOR_RS485_TX_PIN, GPIO_PIN_RESET);

  g_host_to_motor.frames_forwarded++;
  g_host_to_motor.bytes_forwarded += length;
}

static void App_SendToHost(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  HAL_GPIO_WritePin(HOST_RS485_TX_PORT, HOST_RS485_TX_PIN, GPIO_PIN_SET);
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)data, length, BRIDGE_FORWARD_TIMEOUT_MS);
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET)
  {
  }
  HAL_GPIO_WritePin(HOST_RS485_TX_PORT, HOST_RS485_TX_PIN, GPIO_PIN_RESET);

  g_motor_to_host.frames_forwarded++;
  g_motor_to_host.bytes_forwarded += length;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    App_BufferPushByte(&g_host_to_motor, g_uart2_rx_byte);
    App_RestartRx(huart);
  }
  else if (huart->Instance == UART4)
  {
    App_BufferPushByte(&g_motor_to_host, g_uart4_rx_byte);
    App_RestartRx(huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  App_RestartRx(huart);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  MPU_Config();
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();

  memset(&g_host_to_motor, 0, sizeof(g_host_to_motor));
  memset(&g_motor_to_host, 0, sizeof(g_motor_to_host));
  App_StartBridgeReception();

  while (1)
  {
    App_ProcessBridge();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                              | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x024000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
