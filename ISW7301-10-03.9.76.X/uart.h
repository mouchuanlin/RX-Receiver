/* 
 * File:   uart.h
 * Author: Scott
 *
 * Created on February 12, 2018, 1:58 PM
 */

#ifndef UART_H
#define	UART_H


#define RXD_IN      TRISC5
#define TXD_IN      TRISC4
#define GSM_INT     RA2

// The Carriage Return (CR) character (0x0D, \r) moves the cursor to the beginning of the line without advancing to the next line. 
// The Line Feed (LF) character (0x0A, \n) moves the cursor down to the next line without returning to the beginning of the line.
#define CR 		0x0D    // \r
#define LF 		0x0A    // \n

void init_uart();
void start_uart();
void construct_uart_packet(uint8_t rxBuffer[], uint8_t numBytes, uint8_t txBuffer[]);
void terminate_uart();
void tell_mother(uint8_t rxBuffer[], uint8_t numBytes);
void wakeup_GSM_and_send_header(uint8_t headerByte);
void enable_RX_uart_interrupt();
void write_uart(unsigned char data);


unsigned char RS232Buf[6];
volatile bool haveHeader = false;
volatile bool uartRxIsFull = false;
volatile bool receivedCR = false;
volatile bool receivedLF = false;
uint8_t repeatUART = 5;         /* Repeat UART transmission this many times */
uint8_t pos = 0;
uint8_t test1;
bool testMatch;

#endif	/* UART_H */

