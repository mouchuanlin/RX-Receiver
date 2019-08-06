//
// handle_data.c
//

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "cc1120_reg_config.h"
#include "handle_data.h"
#include "uart.h"
#include "spi.h"

bool crcOK(uint8_t *rxBuffer, uint8_t payloadLength) {
    /* Start with entire packet including CRC, then compute CRC >>> if the
     * resulting computation is 0, then the packet has not been corrupted */
    /* rxBuffer[payloadLength] is MSB of CRC; payloadLength + 1 is LSB */
    uint16_t init_val = (uint16_t) (rxBuffer[payloadLength] << 8) | (uint16_t) (rxBuffer[payloadLength + 1] << 0);
    uint16_t crc_val = 0x0000;
    for (uint8_t i = 0; i < payloadLength; i++) {
        crc_val = calcCRC(rxBuffer[i], crc_val);
    }
    if (crc_val == 0x0000) // if CRC matches what it should have been
        return true; // return true
    else
        return false;
}

bool isUniqueTransmission(uint8_t *receiveBuf) {
    uint8_t j = 0, i = 0;
    uint32_t temp = *receiveBuf++;
    temp <<= 8;
    temp |= *receiveBuf++;
    temp <<= 8;
    temp |= *receiveBuf++;
    temp <<= 8;
    temp |= *receiveBuf++;
    bool msgMatches = false;

    // check if it's a test
    test1 = temp & 0x04;
    testMatch = (bool) (test1 == 0x04);

    while (!msgMatches && i < endMsgPtr) // parse all messages already received in buffer, exit if no match
    {
        msgMatches |= (bool) (temp == msgReceived[i++]);
    }
    if (!msgMatches) {
        msgReceived[endMsgPtr] = temp;

        msgTmrCnt[endMsgPtr] = 0x00; // Setting at 0x00 renews timer count
    }
    return (bool) (!msgMatches); // if it matches, "not unique"; if it doesn't match, "is unique"
}

void cycle_radio()
{
    // comment for RSSI read on every wake-up
    if (rssiTimerStarted){// && !receivedBit) {
        if (rssiCnt++ >= RSSI_6S) {
            refresh_rssi_timer();
            sixSecondsUp = true;
        }
    }
    // Green LED on every Rx strobe pulse
    // Red LED on at the end of every 6s time the rssi threshold has been met
    if (sixSecondsUp && radioState == DetectRSSI) {
        sixSecondsUp = false;
        if (rssi_over_threshold()) {
//            start_rssi_timer();
            enable_serial_int();
            radioState = QualifyPreamble;
            stop_rssi_timer();
        }
        else
        {
            power_down_radio();
            start_rssi_timer();
        }
    } 
    else if ((sixSecondsUp || pktTimedOut) && radioState != DetectRSSI && !receivedSync) {
        pktTimedOut = false;
        sixSecondsUp = false;
        currentWordCount = 0;
        pktBitCount = 0;
        firstReadBit = true;
        serialByte = 0;
        stop_preamble_timer();
        if (!rssi_over_threshold() || !receivedPreamble)
        {
//            if (!receivedPreamble)
//                heighten_rssi_thresh();
//            else                      // may have received preamble but not received sync
//                lower_rssi_thresh();
            power_down_radio();
        start_rssi_timer();
        }
        else
        {
            receivedPreamble = false;
            radioState = QualifyPreamble;
//            trxCmdStrobe(CC1120_SRX);
            start_preamble_timer();
        }
        receivedSync = false;
        receivedPreamble = false;
    }
}

void check_packet_timer() {
    // Increment Timer
    for (uint8_t i = 0; i < endMsgPtr; i++) {
        // Use 30s time-out for demo
        if (msgTmrState[i] == ON || msgTmrState[i] == TEST)
            msgTmrCnt[i] += _30S_TICK;
        if (msgTmrCnt[i] >= _4MIN)
            msgTmrState[i] = TIMER_DONE;
    }
}

void check_for_packet()
{
    uint8_t rxBuffer[10] = {0};
    uint8_t rxBytes, marcStatus, chipState;
    
    if (radioState != DetectRSSI && receivedBit)
    {
        if (receivedBit)
        {
            if (!firstReadBit || radioState != ReadingPkt)
                serialByte <<= 1;
            serialByte |= currentBitByte;
            
            pktBitCount++;
            receivedBit = false;
            check_reception(currentBitByte);
        }
    }
    
    if (receivedData(rxBuffer, &rxBytes, &marcStatus))
    {
        if (crcOK(rxBuffer, 6))
        {
            // TODO: TESTIN ONLY. Comment this this out for continuous receiving RF slave data.
            //if (isUniqueTransmission(rxBuffer))
            {
                successLED();
                
                sendAck();
                tell_mother(rxBuffer, 6);
            }
        }
        pktTimedOut = false;
        sixSecondsUp = false;
        currentWordCount = 0;
        pktBitCount = 0;
        firstReadBit = true;
        serialByte = 0;
        stop_preamble_timer();
        receivedSync = false;
        receivedPreamble = false;
        trxCmdStrobe(CC1120_SIDLE);
        while ((trxCmdStrobe(CC1120_SNOP) & 0x70) != 0x00)
            NOP();
        trxCmdStrobe(CC1120_SPWD);
        start_rssi_timer();
        
//        INTCONbits.IOCIE = 1;
    }
}

void check_reception(uint8_t currentBit)
{
    switch (radioState)
    {
        case QualifyPreamble:
            if (pktBitCount >= 48)//give 2 chances
            {
                pktBitCount = 0;
                if (serialByte == PREAMBLE_WORDS || serialByte == ALT_PREAMBLEWORDS)
                {
                    serialByte = 0;
                    radioState = WaitingForSync;
                    receivedPreamble = true;
                    start_preamble_timer();
                }
                else
                    pktTimedOut = true;
            }
            break;
        case WaitingForSync:
            if (serialByte == SYNC_WORDS)
            {
                currentWordCount = 0;
                pktBitCount = 0;
                radioState = ReadingPkt;
                firstReadBit = true;
                serialByte = 0;
            }
            break;
        case ReadingPkt:
            if (firstReadBit)
                firstReadBit = false;
            if (pktBitCount >= 8)
            {
                pktBitCount = 0;
                pktBuffer[currentWordCount++] = (uint8_t)(serialByte & 0x00FF);
                
                if (currentWordCount >= PKT_LEN)
                {
                    currentWordCount = 0;
                    receivedSync = true;
                    radioState = DetectRSSI;
                    disable_serial_int();
                    power_down_radio();
                }
            }
            break;
    }
}