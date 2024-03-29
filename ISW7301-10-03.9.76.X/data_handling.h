/* 
 * File:   data_handling.h
 * Author: Scott
 *
 * Created on February 8, 2018, 12:27 PM
 */

#ifndef DATA-HANDLING_H
#define	DATA_HANDLING_H

#include <stdint.h>
#include <stdbool.h>

bool crcOK(uint8_t rxBuffer[], uint8_t payloadLength);
bool isUniqueTransmission(uint8_t *receiveBuf);
void sendAck();
bool get_test_mode(uint8_t *receiveBuf);

bool receivedSync = false;


#endif	/* HANDLE_DATA_H */

