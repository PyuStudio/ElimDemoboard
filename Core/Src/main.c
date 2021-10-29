/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "command.h"
#include "iic.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USB_DATA_BUFFER_SIZE    123


typedef enum {
    ELIM_ERR_NONE = IIC_ERR_NONE, 
    ELIM_ERR_NO_ACK = IIC_ERR_NO_ACK, 
    ELIM_ERR_OVER_LOAD = IIC_ERR_OVER_LOAD, 
    ELIM_ERR_TIMEOUT = (uint16_t) (-3), 
    ELIM_ERR_BUSY = (uint16_t) (-4), 
} ELIMError;




typedef struct {
    uint16_t        code;
    uint8_t         data[7];
} IicResult;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RESP_ERR_NO_ACK         response_error("no ACK error")
#define RESP_ERR_OVER_LOAD      response_error("power over load error")
#define RESP_ERR_TIMEOUT        resp_err_timeout()
#define RESP_ERR_BUSY           response_error("busy")
#define RETRY                   80


#define TURN_ON_D2_RED          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define TURN_ON_D3_GREEN        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)
#define TURN_ON_D4_BLUE         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define TURN_ON_D5_ORANGE       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define TURN_ON_PC13            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)


#define TURN_OFF_D2_RED          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define TURN_OFF_D3_GREEN        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)
#define TURN_OFF_D4_BLUE         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define TURN_OFF_D5_ORANGE       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define TURN_OFF_PC13            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t elim_addr = 0x5E;
IicHandle iic;
uint8_t sensor_status;
static __IO uint8_t evt_usb = 0;
static __IO uint32_t elim_deactive_clk = 0;
uint8_t led_off_count =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
    /* Place your implementation of fputc here */
    /*  */
    HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 0xFFFF);
    
    return ch;
}


void active_elim()
{
    elim_deactive_clk = HAL_GetTick() + 5000;
    HAL_GPIO_WritePin(_CS_GPIO_Port, _CS_Pin, GPIO_PIN_RESET);

    HAL_Delay(5);
}


void deactive_elim()
{
    uint32_t t = HAL_GetTick();

    if (elim_deactive_clk && t > elim_deactive_clk) {
        elim_deactive_clk = 0;
        HAL_GPIO_WritePin(_CS_GPIO_Port, _CS_Pin, GPIO_PIN_SET);
    }
}


__STATIC_INLINE void transmit_resp(uint8_t * data, uint16_t bytes)
{    
    uint32_t clk = HAL_GetTick() + 500;
    while(HAL_GetTick() < clk){
        if(CDC_Transmit_FS(data, bytes) == USBD_OK){
            break;
        }
    }
}


void response_data(uint8_t * data, uint16_t size)
{
    uint8_t buf[32];
    uint8_t bytes = 0;
;

    sprintf((char *) buf, "$%d\r\n", size);
    bytes = strlen((char *)buf);

    memcpy(buf + bytes, data, size);
    bytes += size;
    
    strcpy((char *)buf + bytes, "\r\n");
    bytes += 2;
    transmit_resp(buf, bytes);
}


void response_ok()
{
    transmit_resp((uint8_t *) "+ok\r\n", 5);
}


void response_error(char * what)
{
    char resp[64];

    sprintf(resp, "-%s\r\n", what);
    transmit_resp((uint8_t *) resp, strlen(resp));
}


__STATIC_INLINE void resp_err_timeout()
{
    char err_text[32];

    sprintf(err_text, "timeout, status:%02X", sensor_status);
    response_error(err_text);
}




uint32_t text_2_number(uint8_t * text)
{
    uint32_t val = 0;

    for (; *text; text++) {
        if (*text >= '1' && *text <= '9') {
            val = val * 10 + *text - '1' + 1;
        }
        else if (*text == '0') {
            if (val == 0) {
                return 0;
            }
            else {
                val = val * 10;
            }
        }
        else {
            return 0;
        }
    }

    return val;
}



__STATIC_INLINE uint16_t elim_read_status(IicHandle * hi2c)
{

    return iic_read(elim_addr | 0x01, &sensor_status, 1, hi2c);
}


static uint8_t elim_wait_ready(IicHandle * hi2c)
{
    uint8_t ret = 0;
    int i = 0;

    while (1) {
        ret = elim_read_status(hi2c);

        if (ret == IIC_ERR_NONE && (sensor_status & 0x01) == 0) {
            // busy bit is cleared, chip is ready
            printf("check status:%d\r\n", sensor_status);
            break;
        }

        i++;

        if (i > RETRY) {
            ret = 1;
            break;
        }
        else 
            // delay_us(250);
            HAL_Delay(50);
    }

    return ret;
}


ELIMError elim_reset()
{
    uint8_t cmd[3] = {
        0x00, 0x00, 0x00
    };
    uint16_t iic_err = IIC_ERR_NONE;

#if 0
    HAL_GPIO_WritePin(_CS_GPIO_Port, _CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(ELIM_VDD_GPIO_Port, ELIM_VDD_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    if (elim_mode == ELIM_MODE_CM) {
        cmd[0] = 0xA9;
    }

    // execute START_CM or read reg 00
    // find out the address of the chip
    for (uint8_t addr = 0; addr < 128; addr++) {
        iic_err = iic_write(addr << 1, cmd, 3, (void *) &hi2c1);

        if (iic_err == IIC_ERR_NO_ACK) {
            continue;
        }

        if (iic_err == IIC_ERR_NONE) {
            printf("chip addr is:%x\r\n", addr);
            elim_addr = addr << 1;

            if (elim_wait_ready(&hi2c1)) {
                return ELIM_ERR_TIMEOUT;
            }

            return ELIM_ERR_NONE;
        }

    }

    sensor_status = 0x9D;                           // 0b10011101 0: Not powered on; 0: not busy; 11: unknonw mode; 1: memory error; 0: sm config 1; 1: ALU saturation
#endif

    return ELIM_ERR_NONE;
}


int validate_raw_response(IicResult * result)
{
    char buf[32];
    char * p = buf;
    int val = 0;
    int sum = 0;

    for (uint8_t i = 1; i < 5; i++) {
        sum += result->data[i];
        val = (val << 8) | result->data[i];
        sprintf(p, "%02X ", result->data[i]);
        p   += 3;

    }

    sum = sum & 0xFF;


    printf("status:%d, data:%s, val:%d, checksum:%d %d\r\n", result->data[0], buf, val, result->data[5], sum);

    return (result->data[0] == 0) && (sum == result->data[5]);
}


void execute_iic(uint8_t * params, uint8_t param_bytes, uint8_t bytes_want, IicResult * result)
{
    uint16_t iic_err = IIC_ERR_NONE;


    active_elim();
    TURN_ON_D5_ORANGE;

    result->code = 0;

    if (params[0] == 0xFF) {
        // chip reset request
        iic_err = elim_reset();
        result->data[0] = sensor_status;
    }
    else {
        // execute iic read/write
        iic_err = iic_write(elim_addr, params, param_bytes, (void *) &iic);

        if (bytes_want) {
            if (iic_err == IIC_ERR_NONE) {
                if (elim_wait_ready(&iic)) {
                    iic_err = ELIM_ERR_TIMEOUT;
                }
                else {
                    iic_err = iic_read(elim_addr | 1, result->data, bytes_want, (void *) &iic);
                }
            }
        }
    }
    TURN_OFF_D5_ORANGE;

    result->code = iic_err;
}


// read_MTP
void read_user_data()
{
    if (getCommandFieldsCount() == 2 && FieldBytes(1) == 1) {
        IicResult result;
        uint8_t * p = getCommandField(1);
        uint8_t i=0;

        do {
            execute_iic(p, 0x01, 6, &result);
        }
        while(!validate_raw_response(&result) && i++ < 50);

        switch (result.code)
        {
            case ELIM_ERR_NONE:
                response_data(result.data, 6);
                break;

            case ELIM_ERR_NO_ACK:
                RESP_ERR_NO_ACK;
                break;

            case IIC_ERR_OVER_LOAD:
                RESP_ERR_OVER_LOAD;
                break;

            case ELIM_ERR_TIMEOUT:
                RESP_ERR_TIMEOUT;
                break;

            case ELIM_ERR_BUSY:
                RESP_ERR_BUSY;
                break;

            default:
                break;
        }
    }
    else {
        response_error("read user data");
    }
}


// write_MTP
void write_user_data()
{
    if (getCommandFieldsCount() == 2 && FieldBytes(1) == 5) {
        IicResult result;
        uint8_t * p = getCommandField(1);

        printf("write user data:");

        for (uint8_t k = 0; k < 5; k++) {
            printf(" %02x", p[k]);
        }

        printf("\r\n");

        execute_iic(p, 5, 1, &result);

        switch (result.code)
        {
            case ELIM_ERR_NONE:
                response_data(result.data, 1);
                break;

            case ELIM_ERR_NO_ACK:
                RESP_ERR_NO_ACK;
                break;

            case IIC_ERR_OVER_LOAD:
                RESP_ERR_OVER_LOAD;
                break;

            case ELIM_ERR_TIMEOUT:
                RESP_ERR_TIMEOUT;
                break;

            case ELIM_ERR_BUSY:
                RESP_ERR_BUSY;
                break;

            default:
                break;
        }
    }
    else {
        response_error("write user data");
    }
}



void firmware()
{
    if (getCommandFieldsCount() == 1) {
        const char * p = "DemoBoard, ver 0.4, build 0811-1423";

        response_data((uint8_t *) p, strlen(p));
    }
    else {
        response_error("firmware");
    }
}


void NotifyUsbCommandEvent(void)
{
    evt_usb = 1;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint32_t blink = HAL_GetTick() + 250;

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
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  TURN_OFF_D4_BLUE;
  TURN_OFF_D2_RED;
  TURN_OFF_D5_ORANGE;
  TURN_OFF_PC13;
  HAL_TIM_Base_Start_IT(&htim2);
  
    iic.scl_pin = SCL_Pin;
    iic.scl_port = SCL_GPIO_Port;
    iic.sda_pin = GPIO_PIN_1;
    iic.sda_port = SDA_GPIO_Port;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        if (evt_usb) {
            printf("usb evt\r\n");
            evt_usb = 0;

            if (haveUsbData()) {
                updateCommandBuffer();

                while (parseCommand() == WITH_COMMAND) {
                    printf("recv cmd: %s\r\n", getCommandField(0));
                    TURN_ON_PC13;


                    if (isCommand((uint8_t *) "firmware")) {
                        firmware();
                    }

                    if (isCommand((uint8_t *) "read user data")) {
                        read_user_data();
                    }
                    else if (isCommand((uint8_t *) "write user data")) {
                        write_user_data();
                    }


                    consumeCommand();
                }
                led_off_count = 2; 
            }
        }

        deactive_elim();

        if (HAL_GetTick() > blink) {
            blink = HAL_GetTick() + 250;
        }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PC13_GPIO_Port, PC13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KEY_GPIO_Port, KEY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, _CS_Pin|SDA_Pin|SCL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13_Pin */
  GPIO_InitStruct.Pin = PC13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PC13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : _CS_Pin */
  GPIO_InitStruct.Pin = _CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDA_Pin SCL_Pin */
  GPIO_InitStruct.Pin = SDA_Pin|SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14
                           PB15 PB3 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{


}




/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
    if (htim->Instance == TIM2) {
        if( led_off_count ){
            led_off_count--;
            if( led_off_count == 0 ){
            TURN_OFF_PC13;
            }
        }
        // HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
        // RESP_ERR_OVER_LOAD;
        // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    }

  /* USER CODE END Callback 1 */
}

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
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
