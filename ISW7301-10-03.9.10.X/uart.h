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
void write_uart(uint8_t data);
void flashing_red();
void flashing_green();
bool check_ACK_data(uint8_t rxBuffer[]);

#define MAX_SIZE 7
volatile bool receivedACK = false;
uint8_t rx_cnt = 0;

uint8_t  RS232Buf[MAX_SIZE];
//volatile bool haveHeader = false;
//volatile bool uartRxIsFull = false;
//volatile bool receivedCR = false;
//volatile bool receivedLF = false;
//volatile bool receivedUARTChar = false;
uint8_t repeatUART = 5;         /* Repeat UART transmission this many times */
uint8_t pos = 0;
uint8_t test1;
bool testMatch;

#endif	/* UART_H */

