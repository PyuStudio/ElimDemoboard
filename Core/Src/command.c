#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "command.h"
#include "usbd_cdc_if.h"
#include "main.h"


// #include <stdio.h>
// #include "cmsis_os.h"


// extern osThreadId taskMainHandle;

#define RESULT_OK  1
#define RESULT_FAILED 0

#define RX_BUF_SIZE 512
#define CMD_BUF_SIZE RX_BUF_SIZE*2


#define COMMAND_FIELD_MAX_COUNT 20
typedef struct{
    uint32_t offset;
    uint32_t bytes;
}Field;

uint8_t UsbRxBuffer[RX_BUF_SIZE];
uint32_t UsbRxBufLen = 0;

uint8_t UsbCmdBuffer[CMD_BUF_SIZE];
uint32_t UsbCmdBufLen = 0;



uint8_t gCommandFieldCount=0;
Field gCommandFields[COMMAND_FIELD_MAX_COUNT]={0};

const uint32_t INVALID_POS = (uint32_t)-1;


uint32_t look_up_star(uint32_t from){
  for(;from<UsbCmdBufLen; from++){
    if( UsbCmdBuffer[from] == '*' ){
      return from;
    }
  }
  return INVALID_POS;
}



void on_decode_command_error(){
    gCommandFieldCount = 0;
}

uint8_t line_ending(uint32_t pos){
  if( UsbCmdBuffer[pos] == '\r' ){
    pos++;
    if( pos < UsbCmdBufLen && UsbCmdBuffer[pos] == '\n' ){
      return RESULT_OK;
    }
  }
  return RESULT_FAILED;
}


uint8_t decode_command_number(uint32_t begin, uint32_t* line_break){
    uint8_t val = 0;
    for(uint32_t i=begin; i<UsbCmdBufLen; i++){
        char ch=UsbCmdBuffer[i];
        if( ch >='1' && ch <='9' ){
            val = val*10 + ch - '1' + 1;
        }
        else if( ch == '0' ){
            if( val == 0 ){
              return 0;
            }
            else{
              val = val*10;
            }
        }
        else if( ch == '\r' ){
          if( line_ending(i) ){
            *line_break = i;
            return val;
          }
          else{
            return 0;
          }
        }
        else{
            return 0;
        }
    }
   return 0;
}

uint8_t decode_command_field(uint32_t from, Field* field){
    if( UsbCmdBuffer[from] != '$' ){
        return RESULT_FAILED;
    }
    uint32_t number_ending;
    uint8_t bytes = decode_command_number(from + 1, &number_ending);
    if( bytes ){
      uint32_t begin = number_ending + 2;
      uint32_t field_ending = begin + bytes;
      if( field_ending < UsbCmdBufLen && line_ending(field_ending) ){
        // correct field, save it
        field->offset = begin;
        field->bytes = bytes;
        return RESULT_OK;
      }
    }
    return RESULT_FAILED;
}

uint8_t decode_command(uint32_t star_pos){
  uint32_t pos;
  
  gCommandFieldCount = decode_command_number(star_pos + 1, &pos);
  if( gCommandFieldCount ){
    uint8_t count=0;
    for(; count<COMMAND_FIELD_MAX_COUNT && pos < UsbCmdBufLen; count++){
      if( decode_command_field(pos+2, gCommandFields+count) ){
          pos = gCommandFields[count].offset + gCommandFields[count].bytes;
      }
      else{
          break;
      }
    }
    if( count != gCommandFieldCount ){
        on_decode_command_error();
        return RESULT_FAILED;
    }
    return RESULT_OK;
  }
  
  return RESULT_FAILED;
}

uint8_t parseCommand(){
  for(uint32_t star_pos=look_up_star(0); star_pos!=INVALID_POS; star_pos=look_up_star(star_pos + 1)){
    if( decode_command(star_pos) ){
      return WITH_COMMAND;
    }
  }
  return NO_COMMAND;  
}


void saveUsbRx(uint8_t* buf, uint32_t len){
  if( UsbRxBufLen == 0 ){
    memcpy(UsbRxBuffer, buf, len);
    UsbRxBufLen = len;
    NotifyUsbCommandEvent();
  }
}

uint8_t haveUsbData(){
  return UsbRxBufLen;
}

void updateCommandBuffer(){
  if( UsbRxBufLen ){
    if( (UsbCmdBufLen + UsbRxBufLen)<CDC_DATA_HS_MAX_PACKET_SIZE ){
      memcpy(UsbCmdBuffer+UsbCmdBufLen,  UsbRxBuffer,  UsbRxBufLen);
      UsbCmdBufLen += UsbRxBufLen;
      UsbRxBufLen = 0;
    }
  }
}


uint8_t getCommandFieldsCount() {
    return gCommandFieldCount;
}

uint8_t* getCommandField(uint8_t index){
    if( index < gCommandFieldCount ){
        UsbCmdBuffer[gCommandFields[index].offset + gCommandFields[index].bytes] = 0;
        return UsbCmdBuffer + gCommandFields[index].offset;
    }
    return NULL;
}

uint8_t FieldBytes(uint8_t index){
    if( index < gCommandFieldCount ){
        return gCommandFields[index].bytes;
    }
    return 0;
}

uint8_t isCommand(uint8_t* cmd){
  uint8_t idx = 0;
  for(; idx<gCommandFields[0].bytes; idx++){
    if( UsbCmdBuffer[gCommandFields[0].offset + idx] != cmd[idx]){
      return 0;    
    }
  }
  if( cmd[idx] )
    return 0;
  return 1;
}

void consumeCommand(){
  if( gCommandFieldCount ){
    uint32_t skips = gCommandFields[gCommandFieldCount - 1].offset + gCommandFields[gCommandFieldCount - 1].bytes + 2;
    if( UsbCmdBufLen > skips ){
      memcpy(UsbCmdBuffer, UsbCmdBuffer+skips, UsbCmdBufLen-skips);
    }
    UsbCmdBufLen -= skips;
    gCommandFieldCount = 0;
  }
}

