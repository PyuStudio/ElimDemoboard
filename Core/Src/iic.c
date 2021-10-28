/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iic.h"


#define SCL_LOW(h)   HAL_GPIO_WritePin(h->scl_port, h->scl_pin, GPIO_PIN_RESET)
#define SCL_HIGH(h)  HAL_GPIO_WritePin(h->scl_port, h->scl_pin, GPIO_PIN_SET)
#define SDA_LOW(h)   HAL_GPIO_WritePin(h->sda_port, h->sda_pin, GPIO_PIN_RESET)
#define SDA_HIGH(h)  HAL_GPIO_WritePin(h->sda_port, h->sda_pin, GPIO_PIN_SET)


#define ACK   1
#define NACK  0

#define BIT(h) HAL_GPIO_ReadPin(h->sda_port, h->sda_pin)
#define CLK(h) HAL_GPIO_ReadPin(h->scl_port, h->scl_pin)


void iic_delay()
{
    // delay 5us
  for(uint16_t i=0; i<237; i++){
    __NOP();
  }
}

void debug_delay(uint8_t n)
{
  for(uint8_t i=0; i<n; i++)
  {
    iic_delay();
  }
}


uint16_t iic_start(IicHandle* hIic)
{
  SCL_HIGH(hIic);
  SDA_HIGH(hIic);
  iic_delay();
  if( BIT(hIic) && CLK(hIic) ){
    SDA_LOW(hIic);
    iic_delay();
    SCL_LOW(hIic);
    iic_delay();
    return IIC_ERR_NONE;
  }

  return IIC_ERR_OVER_LOAD;
}

uint8_t iic_stop(IicHandle* hIic)
{
  SDA_LOW(hIic);
  iic_delay();
  SDA_HIGH(hIic);
  iic_delay();
  SDA_LOW(hIic);
  iic_delay();
  SCL_HIGH(hIic);
  iic_delay();
  SDA_HIGH(hIic);
  iic_delay();
    
  return 0;
}

uint16_t iic_byte_write(IicHandle* hIic, uint8_t byte)
{
  uint8_t ack;
  
  for(uint8_t i=0; i<8; i++, byte=byte<<1){
    if( byte & 0x80 ){
      SDA_HIGH(hIic);
    }
    else{
      SDA_LOW(hIic);
    }
    iic_delay();
    SCL_HIGH(hIic);
    iic_delay();
    SCL_LOW(hIic);
  }

  
  // read ack
  SDA_HIGH(hIic);
  iic_delay();
  iic_delay();
  SCL_HIGH(hIic);
  iic_delay();
  ack = BIT(hIic);
  SCL_LOW(hIic);
  iic_delay();

  if( ack ){
    return IIC_ERR_NO_ACK;
  }

  return IIC_ERR_NONE;
}

uint8_t iic_byte_read(IicHandle* hIic, uint8_t *byte, uint8_t ack)
{
  SDA_HIGH(hIic);
  *byte = 0;
  for(uint8_t i=0; i<8; i++){
    iic_delay();
    SCL_HIGH(hIic);
    iic_delay();
    *byte = (*byte << 1) | ((uint8_t)BIT(hIic));
    SCL_LOW(hIic);
  }
  // send ack
  if( ack )
    SDA_LOW(hIic);
  else
    SDA_HIGH(hIic);
  iic_delay();
  iic_delay();
  SCL_HIGH(hIic);
  iic_delay();
  SCL_LOW(hIic);
  iic_delay();
  
  return 0;
}


uint16_t iic_write(uint8_t addr, uint8_t* data, uint16_t len, void* user_data)
{
  IicHandle* hIic = (IicHandle *) user_data;
  uint16_t err = iic_start(hIic);
  if( err != IIC_ERR_NONE ){
    return err;
  }
  err = iic_byte_write(hIic, addr);
  if( err != IIC_ERR_NONE ){
    return err;
  }
  
  debug_delay(4);
  for(uint16_t i=0; i<len; i++){
    iic_byte_write(hIic, data[i]);
    debug_delay(4);
  }
  iic_stop(hIic);
  debug_delay(8);
  // iic_byte_read(&d);
  return IIC_ERR_NONE;
}


uint16_t iic_read(uint8_t addr, uint8_t* data, uint16_t len, void* user_data)
{  
  IicHandle* hIic = (IicHandle *) user_data;

  uint16_t last_index = len - 1;
  uint16_t err = iic_start(hIic);
  if( err != IIC_ERR_NONE ){
    return err;
  }
  
  err = iic_byte_write(hIic, addr);
  if( err != IIC_ERR_NONE ){
    return err;
  }
  debug_delay(4);
  for(uint16_t i=0; i<len; i++){
    if( i== last_index ){
      iic_byte_read(hIic, data+i, NACK);
    }
    else{
      iic_byte_read(hIic, data+i, ACK);
    }
    debug_delay(4);
  }
  debug_delay(2);
  iic_stop(hIic);
  debug_delay(8);
  
  return 0;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
