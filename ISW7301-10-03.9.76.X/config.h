/* 
 * File:   config.h
 * Author: Scott
 *
 * Created on November 14, 2017, 2:49 PM
 */

#ifndef CONFIG_H
#define	CONFIG_H

// CONFIG1
#pragma config FOSC = XT       // Oscillator Selection (ECH, External Clock, High Power Mode (4-32 MHz): device clock supplied to CLKIN pin)
#pragma config WDTE = SWDTEN        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = ON          // Flash Program Memory Code Protection (Program memory code protection is enabled)
#pragma config CPD = ON         // Data Memory Code Protection (Data memory code protection is enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ  4000000
#define BAUD        9600
#define SDI_PIN     TRISB4
#define SDO_PIN     TRISC7
//#define JUMPER    RC4

// From PIC16 datasheet
//    REGISTER 10-1: WDTCON: WATCHDOG TIMER CONTROL REGISTER
//    bit 5-1 WDTPS<4:0>: Watchdog Timer Period Select bits(1)
//    Bit Value = Prescale Rate
//    00000 = 1:32 (Interval 1 ms nominal)
//    00001 = 1:64 (Interval 2 ms nominal)
//    00010 = 1:128 (Interval 4 ms nominal)
//    00011 = 1:256 (Interval 8 ms nominal)
//    00100 = 1:512 (Interval 16 ms nominal)
//    00101 = 1:1024 (Interval 32 ms nominal)
//    00110 = 1:2048 (Interval 64 ms nominal)
//    00111 = 1:4096 (Interval 128 ms nominal)
//    01000 = 1:8192 (Interval 256 ms nominal)
//    01001 = 1:16384 (Interval 512 ms nominal)
//    01010 = 1:32768 (Interval 1s nominal)
//    01011 = 1:65536 (Interval 2s nominal) (Reset value)
//    01100 = 1:131072 (217) (Interval 4s nominal)
//    01101 = 1:262144 (218) (Interval 8s nominal)
//    01110 = 1:524288 (219) (Interval 16s nominal)
//    01111 = 1:1048576 (220) (Interval 32s nominal)
//    10000 = 1:2097152 (221) (Interval 64s nominal)
//    10001 = 1:4194304 (222) (Interval 128s nominal)
//    10010 = 1:8388608 (223) (Interval 256s nominal)

#define WATCHDOG_SLEEP_128ms    0b00111
#define WATCHDOG_SLEEP_256ms    0b01000
#define WATCHDOG_SLEEP_512ms    0b01001
#define WATCHDOG_SLEEP_1S       0b01010
#define WATCHDOG_SLEEP_2S       0b01011
#define WATCHDOG_SLEEP_4S       0b01100
#define WATCHDOG_SLEEP_8S       0b01101
#define WATCHDOG_SLEEP_16S      0b01110
#define WATCHDOG_SLEEP_32S      0b01111
#define WATCHDOG_SLEEP_64S      0b10000
#define WATCHDOG_SLEEP_128S     0b10001
#define WATCHDOG_SLEEP_256S     0b10010


#define G_LED           LATCbits.LATC2
#define R_LED           LATBbits.LATB5
#define SW_1            RA0                 // Active-low
#define GPIO0           RC3
#define GPIO2           RB7
#define CSense_ASSERTED     RC3                 // GPIO0

#define G_LED_OFF	G_LED=0
#define G_LED_ON	G_LED=1
#define R_LED_OFF	R_LED=0
#define R_LED_ON	R_LED=1

#define SMOKE_TYPE      0b1000
#define FLOOD_TYPE      0b0110
#define CO_TYPE         0b0010
#define GLASS_TYPE      0b1100
#define MOTION_TYPE     0b1001
#define DOOR_TYPE       0b0011
#define PANIC_TYPE      0b0001
#define HVAC_TYPE       0b1011
#define APPLIANCE_TYPE  0b0101
//#define TBD10_TYPE      0b0100
//#define TBD11_TYPE      0b0111
//#define TBD12_TYPE      0b1010
//#define TBD13_TYPE      0b1101
//#define TBD14_TYPE      0b1110
//#define TBD15_TYPE      0b0000
//#define TBD16_TYPE      0b1111
    
#define MAX_CMDS_STORED 20
#define _7MIN           801
#define _4MIN           938
#define _30S_TICK       7           // ticks required for test signal timeout of ~30s.

#define _T1_7S          14
#define _T1_6S          12
#define RSSI_6S         45          //46

#include <stdint.h>
#include <stdbool.h>

enum TimerState {
    OFF,
    ON,
    TIMER_DONE,
    TEST            // shortened timeout to 30s
};


void check7minTimer();
void start_rssi_timer();
void refresh_rssi_timer();
void stop_rssi_timer();
bool rssi_over_threshold();

extern unsigned short packetCounter;
bool _7minTimerOn = false;
bool alreadyAsserted = false;
bool rssiTimerDone = false;
bool rssiTimerStarted = false;
bool alreadyReset = true;
uint8_t delayCount = 0;
uint8_t t1cnt = 0;
bool sixSecondsUp = false;

uint16_t msgTmrCnt[MAX_CMDS_STORED] = 0x00;
enum TimerState msgTmrState[MAX_CMDS_STORED] = 0x00;
uint32_t msgReceived[MAX_CMDS_STORED];           // Stores all received messages
uint8_t endMsgPtr = 0;              // tracks the end of the received message buffer

uint8_t tt = 0;
uint8_t rssiCnt = 0;

const uint8_t UNIT_NUM @ 0x0080 = 0x01;

#endif	/* CONFIG_H */