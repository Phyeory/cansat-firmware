#pragma once
#include "stm32g4xx_hal.h"
#include "config.h"

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim8;

void board_init(void);
