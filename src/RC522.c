#include "RC522.h"

//-----Private Function Prototype-----
static void RC522_WriteReg(RC522_st* rc522, uint8_t reg, uint8_t value);
static uint8_t RC522_ReadReg(RC522_st* rc522, uint8_t reg);
static void RC522_SetBitMask(RC522_st* rc522, uint8_t reg, uint8_t mask);
static void RC522_ClearBitMask(RC522_st* rc522, uint8_t reg, uint8_t mask);
static void RC522_AntennaOn(RC522_st* rc522);
static void RC522_AntennaOff(RC522_st* rc522);
static RC522_Status_et RC522_RequestA(RC522_st* rc522);
RC522_Status_et RC522_Anticoll(RC522_st* rc522, uint8_t *uid);
static void RC522_Delay(RC522_st* rc522, uint32_t ms);



//-----Public Function Definition-----
void RC522_Init(RC522_st* rc522, RC522_Descriptor_st* descriptor){
	memset(rc522, 0, sizeof(RC522_st));
	memcpy(&rc522->descriptor, descriptor, sizeof(RC522_Descriptor_st));

	// Hardware reset
	rc522->descriptor.reset(rc522, 0);
    RC522_Delay(rc522, 50);
    rc522->descriptor.reset(rc522, 1);
    RC522_Delay(rc522, 50);

    // Soft reset
    RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_SOFT_RESET);
    RC522_Delay(rc522, 50);

    // Clear interrupts
    RC522_WriteReg(rc522, PCD_COM_IRQ_REG, 0x7F);

    // Flush FIFO
    RC522_WriteReg(rc522, PCD_FIFO_LEVEL_REG, 0x80);

    // Timer: ~25ms timeout
	RC522_WriteReg(rc522, PCD_T_MODE_REG, 0x80);       // Timer starts immediately
    RC522_WriteReg(rc522, PCD_T_PRESCALER_REG, 0xA9);  // 80kHz clock
    RC522_WriteReg(rc522, PCD_T_RELOAD_REG_H, 0x03);   // 1000 ticks = ~12.5ms
    RC522_WriteReg(rc522, PCD_T_RELOAD_REG_L, 0xE8);

    // RF settings
    RC522_WriteReg(rc522, PCD_TX_AUTO_REG, 0x40);     // 100% ASK modulation
    RC522_WriteReg(rc522, PCD_RF_CFG_REG, 0x7F);      // Max gain (48dB)
    RC522_WriteReg(rc522, PCD_DEMOD_REG, 0x4D);      // Sensitivity for clones

    // Enable antenna
    RC522_AntennaOn(rc522);
    RC522_Delay(rc522, 10);  // Let RF stabilize

    //uint8_t version = RC522_ReadReg(rc522, PCD_VERSION_REG);
    //uint8_t txCtrl = RC522_ReadReg(rc522, PCD_TX_CONTROL_REG);
}

void RC522_Run(RC522_st* rc522){
	RC522_Status_et status = RC522_RequestA(rc522);
    if (status == RC522_STATUS_OK) {
    	if(rc522->cardState == RC522_CARD_OUT){
    		rc522->descriptor.eventCallback(rc522, RC522_CARD_DETECTED);
    		rc522->cardState = RC522_CARD_IN;
    	}

    }
    else if (status != RC522_STATUS_OK) {
    	if(rc522->cardState == RC522_CARD_IN){
    		rc522->descriptor.eventCallback(rc522, RC522_CARD_REMOVED);
    		rc522->cardState = RC522_CARD_OUT;
    	}
    }
}

RC522_CardState_et RC522_GetCardState(RC522_st* rc522){
	return rc522->cardState;
}

RC522_Status_et RC522_ReadUid(RC522_st* rc522, uint8_t *uid){
    // Card detected, read UID
    uint8_t rawUid[5];
    if (RC522_Anticoll(rc522, rawUid) != RC522_STATUS_OK) {
        return RC522_STATUS_ERROR;
    }
    // Copy UID (drop BCC)
    for (int i = 0; i < 4; i++) {
        uid[i] = rawUid[i];
    }
    return RC522_STATUS_OK;
}

RC522_Status_et RC522_WaitcardRemove(RC522_st* rc522){
    while (1) {
        if (RC522_RequestA(rc522) != RC522_STATUS_OK) {
            return RC522_STATUS_OK; // Card removed, return success
        }
        RC522_Delay(rc522, 100); // Poll every 100ms to check if card is still present
    }
}

RC522_Status_et RC522_WaitcardDetect(RC522_st* rc522){
	rc522->atqa[0] = rc522->atqa[1] = 0;
	while (1){
	    if (RC522_RequestA(rc522) == RC522_STATUS_OK) {
	        return RC522_STATUS_OK;
	    }
	    RC522_Delay(rc522, 100);	// Poll every 100ms to check if card is  present
	}
}



//-----Private Function Definition-----
static void RC522_WriteReg(RC522_st* rc522, uint8_t reg, uint8_t value){
	uint8_t addr = (reg << 1) & 0x7E;
	rc522->descriptor.spiChipSelect(rc522, 0);
	rc522->descriptor.spiTransmit(rc522, &addr, 1);
	rc522->descriptor.spiTransmit(rc522, &value, 1);
	rc522->descriptor.spiChipSelect(rc522, 1);
	RC522_Delay(rc522, 1);
}

static uint8_t RC522_ReadReg(RC522_st* rc522, uint8_t reg){
	uint8_t addr = ((reg << 1) & 0x7E) | 0x80;
	uint8_t value = 0;
	rc522->descriptor.spiChipSelect(rc522, 0);
	rc522->descriptor.spiTransmit(rc522, &addr, 1);
	rc522->descriptor.spiReceive(rc522, &value, 1);
	rc522->descriptor.spiChipSelect(rc522, 1);
	RC522_Delay(rc522, 1);
	return value;
}

static void RC522_SetBitMask(RC522_st* rc522, uint8_t reg, uint8_t mask){
    uint8_t tmp = RC522_ReadReg(rc522, reg);
    RC522_WriteReg(rc522, reg, tmp | mask);
}

static void RC522_ClearBitMask(RC522_st* rc522, uint8_t reg, uint8_t mask){
    uint8_t tmp = RC522_ReadReg(rc522, reg);
    RC522_WriteReg(rc522, reg, tmp & (~mask));
}

static void RC522_AntennaOn(RC522_st* rc522){
	RC522_SetBitMask(rc522, PCD_TX_CONTROL_REG, 0x03);
}

static void RC522_AntennaOff(RC522_st* rc522){
	RC522_ClearBitMask(rc522, PCD_TX_CONTROL_REG, 0x03);
}

static RC522_Status_et RC522_RequestA(RC522_st* rc522){
    RC522_AntennaOff(rc522);  // Reset RF
    RC522_Delay(rc522, 5);  // Allow chip to stabilize
    RC522_AntennaOn(rc522);
    RC522_Delay(rc522, 5);  // Ensure RF is ready
    RC522_WriteReg(rc522, PCD_COM_IRQ_REG, 0x7F);      // Clear IRQs
    RC522_WriteReg(rc522, PCD_FIFO_LEVEL_REG, 0x80);   // Flush FIFO
    RC522_WriteReg(rc522, PCD_BIT_FRAMING_REG, 0x07);  // 7 bits for REQA
    RC522_WriteReg(rc522, PCD_FIFO_DATA_REG, PICC_REQA);
    RC522_Delay(rc522, 2);  // Increased for counterfeit chip stability
    RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_TRANSCEIVE);
    RC522_SetBitMask(rc522, PCD_BIT_FRAMING_REG, 0x80);

    // Poll for completion (25ms timeout)
    uint32_t timeout = rc522->descriptor.getTick() + 25;
    while (rc522->descriptor.getTick() < timeout) {
        uint8_t status2 = RC522_ReadReg(rc522, PCD_STATUS2_REG);
        if (status2 & 0x01) {  // Command complete
            uint8_t err = RC522_ReadReg(rc522, PCD_ERROR_REG);
            if (err & 0x1D) {  // Protocol/parity/buffer errors
            	RC522_AntennaOff(rc522);
                RC522_Delay(rc522, 5);
                RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE); // Stop command
                return RC522_STATUS_ERROR;
            }
            uint8_t fifoLvl = RC522_ReadReg(rc522, PCD_FIFO_LEVEL_REG);
            if (fifoLvl >= 2) {  // ATQA is 2 bytes
                rc522->atqa[0] = RC522_ReadReg(rc522, PCD_FIFO_DATA_REG);
                rc522->atqa[1] = RC522_ReadReg(rc522, PCD_FIFO_DATA_REG);
                RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE); // Stop command
                RC522_Delay(rc522, 2);  // Post-command delay
                return RC522_STATUS_OK;
            }
            RC522_AntennaOff(rc522);
            RC522_Delay(rc522, 5);
            RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
            return RC522_STATUS_ERROR;
        }
        RC522_Delay(rc522, 1);  // Mimic debug log timing
    }
    RC522_AntennaOff(rc522);
    RC522_Delay(rc522, 5);
    RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
    return RC522_STATUS_TIMEOUT;
}

RC522_Status_et RC522_Anticoll(RC522_st* rc522, uint8_t *uid){  // Returns 4-byte UID + BCC
    RC522_WriteReg(rc522, PCD_COM_IRQ_REG, 0x7F);      // Clear IRQs
    RC522_WriteReg(rc522, PCD_FIFO_LEVEL_REG, 0x80);   // Flush FIFO
    RC522_WriteReg(rc522, PCD_BIT_FRAMING_REG, 0x00);  // Full frame
    RC522_WriteReg(rc522, PCD_FIFO_DATA_REG, PICC_SEL_CL1);  // 0x93
    RC522_WriteReg(rc522, PCD_FIFO_DATA_REG, 0x20);    // Fixed CRC
    RC522_Delay(rc522, 2);  // Delay for stability
    RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_TRANSCEIVE);
    RC522_SetBitMask(rc522, PCD_BIT_FRAMING_REG, 0x80);

    uint32_t timeout = rc522->descriptor.getTick() + 25;
    while (rc522->descriptor.getTick() < timeout) {
        uint8_t status2 = RC522_ReadReg(rc522, PCD_STATUS2_REG);
        if (status2 & 0x01) {  // Command complete
            uint8_t err = RC522_ReadReg(rc522, PCD_ERROR_REG);
            if (err & 0x1D) {
                RC522_AntennaOff(rc522);
                RC522_Delay(rc522, 5);
                RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
                return RC522_STATUS_ERROR;
            }
            uint8_t fifoLvl = RC522_ReadReg(rc522, PCD_FIFO_LEVEL_REG);
            if (fifoLvl == 5) {  // 4-byte UID + BCC
                for (int i = 0; i < 5; i++) {
                    uid[i] = RC522_ReadReg(rc522, PCD_FIFO_DATA_REG);
                }
                // Validate BCC
                uint8_t calcBcc = uid[0] ^ uid[1] ^ uid[2] ^ uid[3];
                if (uid[4] != calcBcc) {
                    RC522_AntennaOff(rc522);
                    RC522_Delay(rc522, 5);
                    RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
                    return RC522_STATUS_ERROR;
                }
                RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
                RC522_Delay(rc522, 2);  // Post-command delay
                return RC522_STATUS_OK;
            }
            RC522_AntennaOff(rc522);
            RC522_Delay(rc522, 5);
            RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
            return RC522_STATUS_ERROR;
        }
        RC522_Delay(rc522, 1);  // Mimic debug log timing
    }
    RC522_AntennaOff(rc522);
    RC522_Delay(rc522, 5);
    RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
    return RC522_STATUS_TIMEOUT;
}

static void RC522_Delay(RC522_st* rc522, uint32_t ms){
	uint32_t startTime = rc522->descriptor.getTick();
	while(startTime + ms > rc522->descriptor.getTick()){}
}
