#ifndef CMD_H__
#define CMD_H__
#include <stdint.h>

#define NO_COMMAND  0
#define WITH_COMMAND  1


void saveUsbRx(uint8_t* buf, uint32_t len);
uint8_t haveUsbData(void);
void updateCommandBuffer(void);

uint8_t parseCommand(void);
uint8_t isCommand(uint8_t* cmd);
void consumeCommand(void);
uint8_t* getCommandField(uint8_t index);
uint8_t getCommandFieldsCount(void);
uint8_t FieldBytes(uint8_t index);





#endif
