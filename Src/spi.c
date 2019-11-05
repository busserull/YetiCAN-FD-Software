#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef g_spi1_handle;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  g_spi1_handle.Instance = SPI1;
  g_spi1_handle.Init.Mode = SPI_MODE_MASTER;
  g_spi1_handle.Init.Direction = SPI_DIRECTION_2LINES;
  g_spi1_handle.Init.DataSize = SPI_DATASIZE_8BIT;
  g_spi1_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
  g_spi1_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
  g_spi1_handle.Init.NSS = SPI_NSS_SOFT;
  g_spi1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  g_spi1_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
  g_spi1_handle.Init.TIMode = SPI_TIMODE_DISABLE;
  g_spi1_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  g_spi1_handle.Init.CRCPolynomial = 7;
  g_spi1_handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  g_spi1_handle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&g_spi1_handle) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
