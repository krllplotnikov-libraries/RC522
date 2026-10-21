#include "RC522.h"

//-----Private Function Prototype-----
static void RC522_WriteReg(RC522_st* rc522, uint8_t reg, uint8_t value);
static uint8_t RC522_ReadReg(RC522_st* rc522, uint8_t reg);
static void RC522_SetBitMask(RC522_st* rc522, uint8_t reg, uint8_t mask);
static void RC522_ClearBitMask(RC522_st* rc522, uint8_t reg, uint8_t mask);
static void RC522_AntennaOn(RC522_st* rc522);
static void RC522_AntennaOff(RC522_st* rc522);
static RC522_Status_et RC522_RequestA(RC522_st* rc522);
static RC522_Status_et RC522_Transceive(RC522_st* rc522, const uint8_t* tx, uint8_t txLen,
                                        uint8_t* rx, uint8_t* rxLen);
static void RC522_CalcCrcA(const uint8_t* data, uint8_t len, uint8_t* crc);
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

RC522_Status_et RC522_ReadUid(RC522_st* rc522, uint8_t *uid, uint8_t *uidSize){
    static const uint8_t selCmd[3] = { PICC_SEL_CL1, PICC_SEL_CL2, PICC_SEL_CL3 };
    uint8_t size = 0;

    for (uint8_t level = 0; level < 3; level++) {
        uint8_t frame[9];
        uint8_t rxLen;
        RC522_Status_et st;

        frame[0] = selCmd[level];
        frame[1] = PICC_NVB_ANTICOLL;
        rxLen = 5;
        st = RC522_Transceive(rc522, frame, 2, &frame[2], &rxLen);
        if (st != RC522_STATUS_OK) return st;
        if (rxLen != 5) return RC522_STATUS_ERROR;
        if ((frame[2] ^ frame[3] ^ frame[4] ^ frame[5]) != frame[6]) return RC522_STATUS_ERROR;  // BCC

        frame[1] = PICC_NVB_SELECT;
        RC522_CalcCrcA(frame, 7, &frame[7]);
        uint8_t sakResp[3];
        rxLen = 3;
        st = RC522_Transceive(rc522, frame, 9, sakResp, &rxLen);
        if (st != RC522_STATUS_OK) return st;
        if (rxLen != 3) return RC522_STATUS_ERROR;
        uint8_t crc[2];
        RC522_CalcCrcA(sakResp, 1, crc);
        if (crc[0] != sakResp[1] || crc[1] != sakResp[2]) return RC522_STATUS_ERROR;

        if (sakResp[0] & PICC_SAK_UID_NOT_COMPLETE) {
            if (frame[2] != PICC_CASCADE_TAG) return RC522_STATUS_ERROR;
            memcpy(&rc522->uid[size], &frame[3], 3);
            size += 3;
        } else {
            memcpy(&rc522->uid[size], &frame[2], 4);
            size += 4;
            rc522->uidSize = size;
            rc522->sak = sakResp[0];
            if (uid)     memcpy(uid, rc522->uid, size);
            if (uidSize) *uidSize = size;
            return RC522_STATUS_OK;
        }
    }
    return RC522_STATUS_ERROR;
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

static RC522_Status_et RC522_Transceive(RC522_st* rc522, const uint8_t* tx, uint8_t txLen,
                                        uint8_t* rx, uint8_t* rxLen){
    RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
    RC522_WriteReg(rc522, PCD_COM_IRQ_REG, 0x7F);       // Clear IRQs
    RC522_WriteReg(rc522, PCD_FIFO_LEVEL_REG, 0x80);    // Flush FIFO
    for (uint8_t i = 0; i < txLen; i++) {
        RC522_WriteReg(rc522, PCD_FIFO_DATA_REG, tx[i]);
    }
    RC522_WriteReg(rc522, PCD_BIT_FRAMING_REG, 0x00);   // Full bytes
    RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_TRANSCEIVE);
    RC522_SetBitMask(rc522, PCD_BIT_FRAMING_REG, 0x80); // StartSend

    RC522_Status_et result = RC522_STATUS_TIMEOUT;
    uint32_t start = rc522->descriptor.getTick();
    while ((uint32_t)(rc522->descriptor.getTick() - start) < 25) {
        uint8_t irq = RC522_ReadReg(rc522, PCD_COM_IRQ_REG);
        if (irq & 0x30) {
            result = RC522_STATUS_OK;
            break;
        }
        if (irq & 0x01) {
            break;
        }
    }
    RC522_ClearBitMask(rc522, PCD_BIT_FRAMING_REG, 0x80);

    if (result != RC522_STATUS_OK) {
        RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
        return result;
    }

    if (RC522_ReadReg(rc522, PCD_ERROR_REG) & 0x1B) {
        RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
        return RC522_STATUS_ERROR;
    }

    uint8_t n = RC522_ReadReg(rc522, PCD_FIFO_LEVEL_REG) & 0x7F;
    if (n > *rxLen) {
        RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
        return RC522_STATUS_ERROR;
    }
    for (uint8_t i = 0; i < n; i++) {
        rx[i] = RC522_ReadReg(rc522, PCD_FIFO_DATA_REG);
    }
    *rxLen = n;
    RC522_WriteReg(rc522, PCD_COMMAND_REG, PCD_IDLE);
    return RC522_STATUS_OK;
}

static void RC522_CalcCrcA(const uint8_t* data, uint8_t len, uint8_t* crc){
    uint16_t c = 0x6363;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = data[i] ^ (uint8_t)(c & 0xFF);
        b ^= (uint8_t)(b << 4);
        c = (c >> 8) ^ ((uint16_t)b << 8) ^ ((uint16_t)b << 3) ^ (b >> 4);
    }
    crc[0] = (uint8_t)(c & 0xFF);
    crc[1] = (uint8_t)(c >> 8);
}

static void RC522_Delay(RC522_st* rc522, uint32_t ms){
	uint32_t startTime = rc522->descriptor.getTick();
	while(startTime + ms > rc522->descriptor.getTick()){}
}
