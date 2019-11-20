
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "mcp_can_controller.h"

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
MCP_MasterConfig g_mcp_master_config = {
    .nominal_bit_rate_seg1 = 31,
    .nominal_bit_rate_seg2 = 9,

    .data_bit_rate_seg1 = 31,
    .data_bit_rate_seg2 = 9,

    .transmit_event_config = {
        .message_depth = 2,
        .use_timestamp = 1
    },

    .transmit_queue_config = {
        .payload_size = MCP_PAYLOAD_64_BYTES,
        .message_depth = 2
    },

    .receive_fifo_config = {
        {
            .payload_size = MCP_PAYLOAD_64_BYTES,
            .message_depth = 6,
            .use_timestamp = 1
        }
    },

    .filter_config = {
        {
            .use_filter = 1,
            .fifo_destination = 1,
            .frame_type = MCP_FILTER_ACCEPT_ANY,
            .filter_mask = 0,
            .filter_object = 0
        }
    }
};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
int main(){
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */



  mcp_init(&g_mcp_master_config);

  MX_USB_DEVICE_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* uint8_t data[8] = {'C', 'o', 'c', 'k', 't', 'i', 't', 's'}; */

    /* MCP_Message object = { */
    /*     .use_fd_format = 1, */
    /*     .use_bit_rate_switch = 0, */
    /*     .use_extended_id = 1, */
    /*     .error_active = 1, */

    /*     .frame_id = 0x7babe, */
    /*     .sequence_number = 0x55aa55, */

    /*     .data_length = MCP_DATA_LENGTH_08_BYTES, */
    /*     .p_data = data */
    /* }; */

    mcp_gpio_latch();
    mcp_gpio_unlatch();

  while (1)
  {
    MCP_Message receive_object;

    uint8_t fifo_empty = 0;

    fifo_empty = mcp_receive(&receive_object, 1);
    if(!fifo_empty){
        free(receive_object.p_data);
    }

    uint8_t stx = 0x02;
    uint8_t etx = 0x03;

    uint8_t send_command[24] = {
        stx,
        0x20,
        1,
        0,
        0,
        1,

        0x00,
        0x00,
        0x00,
        0xab,

        0x00,
        0x00,
        0x00,
        0x06,

        0x04,
        'T',
        'i',
        't',
        's',

        0x00,
        0x00,
        0x00,
        0x00,

        etx
    };

    uint32_t crc = crc_calculate(send_command + 1, 18);
    send_command[19] = (uint8_t)(crc >> 24);
    send_command[20] = (uint8_t)(crc >> 16);
    send_command[21] = (uint8_t)(crc >> 8);
    send_command[22] = (uint8_t)(crc);

    for(int i = 0; i < 24; i++){
        packet_build(send_command[i]);
    }

    /* mcp_send(&object); */

    /* fifo_empty = mcp_receive(&receive_object, 1); */
    /* if(!fifo_empty){ */
    /*     free(receive_object.p_data); */
    /* } */



    HAL_Delay(5);
    /* mcp_check_transmit_event(&event); */
    /* fifo = any_fifos(); */
    /* HAL_Delay(5); */
    /* get_2(); */
    /* HAL_Delay(500); */


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE END Error_Handler_Debug */
}
