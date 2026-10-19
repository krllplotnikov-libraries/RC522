#ifndef RC522_H_
#define RC522_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Registers
#define PCD_COMMAND_REG      0x01
#define PCD_COM_IRQ_REG      0x04
#define PCD_ERROR_REG        0x06
#define PCD_STATUS2_REG      0x08
#define PCD_FIFO_DATA_REG    0x09
#define PCD_FIFO_LEVEL_REG   0x0A
#define PCD_BIT_FRAMING_REG  0x0D
#define PCD_TX_CONTROL_REG   0x14
#define PCD_TX_AUTO_REG      0x15
#define PCD_RF_CFG_REG       0x26
#define PCD_T_MODE_REG       0x2A
#define PCD_T_PRESCALER_REG  0x2B
#define PCD_T_RELOAD_REG_L   0x2C
#define PCD_T_RELOAD_REG_H   0x2D
#define PCD_DEMOD_REG        0x19
#define PCD_VERSION_REG      0x37

// Commands
#define PCD_IDLE            0x00
#define PCD_TRANSCEIVE      0x0C
#define PCD_SOFT_RESET      0x0F

// PICC commands
#define PICC_REQA          0x26
#define PICC_SEL_CL1       0x93

typedef enum RC522_Status RC522_Status_et;
typedef enum RC522_CardState RC522_CardState_et;
typedef enum RC522_CardEvent RC522_CardEvent_et;

typedef struct RC522_Descriptor RC522_Descriptor_st;
typedef struct RC522            RC522_st;

typedef void (*RC522_SPI_Transmit)             (RC522_st* rc522, uint8_t* txBuf, size_t size);
typedef void (*RC522_SPI_Receive)              (RC522_st* rc522, uint8_t* rxBuf, size_t size);
typedef void (*RC522_SPI_ChipSelect)           (RC522_st* rc522, uint8_t level);
typedef void (*RC522_Reset)                    (RC522_st* rc522, uint8_t level);
typedef void (*RC522_CardEvent_Callback)       (RC522_st* rc522, RC522_CardEvent_et event);
typedef uint32_t (*RC522_GetTick)              ();

enum RC522_Status{
	RC522_STATUS_OK,
	RC522_STATUS_ERROR,
	RC522_STATUS_TIMEOUT,
};

enum RC522_CardState{
	RC522_CARD_OUT,
	RC522_CARD_IN,
};

enum RC522_CardEvent{
	RC522_CARD_DETECTED,
	RC522_CARD_REMOVED,
};

struct RC522_Descriptor{
	RC522_SPI_Transmit         spiTransmit;
	RC522_SPI_Receive          spiReceive;
	RC522_SPI_ChipSelect       spiChipSelect;
	RC522_Reset                reset;
	RC522_GetTick              getTick;
	RC522_CardEvent_Callback   eventCallback;
};

struct RC522{
	RC522_Descriptor_st descriptor;
	RC522_CardState_et cardState;
	uint8_t atqa[2];
};

void RC522_Init(RC522_st* rc522, RC522_Descriptor_st* descriptor);
void RC522_Run(RC522_st* rc522);
RC522_CardState_et RC522_GetCardState(RC522_st* rc522);
RC522_Status_et RC522_ReadUid(RC522_st* rc522, uint8_t *uid);
RC522_Status_et RC522_WaitcardRemove(RC522_st* rc522);
RC522_Status_et RC522_WaitcardDetect(RC522_st* rc522);

#ifdef __cplusplus
}
#endif

#endif
