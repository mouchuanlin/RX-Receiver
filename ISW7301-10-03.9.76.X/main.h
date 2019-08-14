//
// main.h
//

#ifndef MAIN_H
#define	MAIN_H

/*****************************************************
 * INCLUDES
 ****************************************************/
#include <stdint.h>
#include <stdbool.h>

/*****************************************************
 * FUNCTION PROTOTYPES
 ****************************************************/
void startup_blinking();
static void initializePIC(void);
static void registerConfig(void);
void sendAck(void);
static void successLED(void);
void check7minTimer();
void check_rssi_6s_timer();
void check_unique_msg_timer();
void process_RF_data();
void check_rssi();
void enter_sleep();

/*****************************************************
 * VARIABLES
 ****************************************************/
#define REVISION "A03"
#define PREAMBLE_TIMESPAN   6000        // WOR periodicity is currently 6s

volatile uint8_t TempScale;
volatile bit io_change;
uint16_t packetCounter = 0;
bool waitingForPkt = false;
bool toggleLED = false;

#endif	/* MAIN_H */