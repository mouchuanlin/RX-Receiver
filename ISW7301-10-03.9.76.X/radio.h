//
// radio.h
//

#ifndef RADIO_H
#define	RADIO_H

/*****************************************************
 * INCLUDES
 ****************************************************/
#include <stdint.h>
#include <stdbool.h>

/*****************************************************
 * FUNCTION PROTOTYPES
 ****************************************************/
typedef uint8_t rfStatus_t;

bool receivedData(uint8_t rxBuffer[], uint8_t* rxBytes, uint8_t* marcStatus);
uint16_t calcCRC(uint8_t crcData, uint16_t crcReg);
extern void createAckPacket(uint8_t txBuffer[]);
rfStatus_t cc1120SpiReadRxFifo(uint8_t *pData, uint8_t len);
void calibrateRCOsc(void);
void manualCalibration(void);
void start_rssi_timer();
void refresh_rssi_timer();
void stop_rssi_timer();
bool rssi_over_threshold();
void power_down_radio();

/*****************************************************
 * VARIABLES
 ****************************************************/
#define CRC_ENABLE          TRUE
#define CRC_INIT            0xFFFF
#define CRC16_POLY          0x8005

#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2

uint16_t RSSI_THRESH = 0x0190;
uint32_t RSSI_NTHRESH = 0x06D0;//700

#endif	/* RADIO_H */