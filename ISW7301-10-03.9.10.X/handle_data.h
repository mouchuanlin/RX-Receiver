/* 
 * File:   handle_data.h
 * Author: Scott
 *
 * Created on February 8, 2018, 12:27 PM
 */

#ifndef HANDLE_DATA_H
#define	HANDLE_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#define SERIAL_DATA     PORTCbits.RC1
#define PKT_LEN         0x06

enum RadioState{
    DetectRSSI,
    QualifyPreamble,
    WaitingForSync,
    ReadingPkt,
};

enum RadioState radioState;

bool crcOK(uint8_t rxBuffer[], uint8_t payloadLength);
bool isUniqueTransmission(uint8_t *receiveBuf);
void sendAck();
void cycle_radio();
void check_packet_timer();
void check_for_packet();
void check_reception(uint8_t currentBit);

bool receivedSync = false, receivedPreamble = false, receivedBit = false;
bool firstReadBit = false;
uint32_t serialByte = 0;
uint8_t syncWords[32] = {0};
uint8_t currentWordCount = 0;
uint8_t currentBitByte = 0, pktBitCount = 0;
uint32_t pktBuf[2] = {0};
uint32_t SYNC_WORDS = 0xC1DEC1DE;
uint32_t PREAMBLE_WORDS = 0xAAAAAAAA;
uint32_t ALT_PREAMBLEWORDS = 0x55555555;


uint8_t pktBuffer[48] = {0};//6] = {0};


extern void flashing_green();

#endif	/* HANDLE_DATA_H */

