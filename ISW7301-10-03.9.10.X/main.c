/*
 * This program is intended to verify operation of the Instant Care 906MHz smoke hub 
 * receiver (7301-01).
 *  
 * File:   main.c
 * Author: JV
 *
 * Code Version: 2.8
 * 
 * Created on November 14, 2017, 2:49 PM
 * 2.5->2.6: 18 Aug. 2018. JV. Receiver ignores already-received msgs for 7min after
 *                             it receives the first message successfully.
 *           04 Sep. 2018. JV. Added back in "repeat UART" loop, repeats until it 
 *                             gets proper chars or after 5 retries and it gives up. 
 *                             If it gives up, it doesn't start timer so it will be repeated
 *                             on next msg.
 * 2.6->2.7: 09 Sep. 2018. JV. Added placeholder UART values for other devices.
 * 2.7->2.8: 11 Sep. 2018. JV. Distinguishes between sensors for "unit 1" vs "unit 2" for 
 *                             customer demo purposes only, to prevent overlap on signals across
 *                             multiple units. Intended for hub code without LEARN MODE.
 * 2.8->2.9: 11 Sep. 2018; JV. Changed UART protocol to feed-through format with header and
 *                             terminator bytes. Removed pseudo-zone-checking introduced in 
 *                             v2.8.
 * 
 * 3.5->3.6: 26 Mar. 2019; JV. Working on currently-known bug that causes radio to 
 *                             indefinitely enter IDLE state upon interrupting off radio
 *                             and exiting WOR mode. Polishing code.
 * 3.6/3.7->3.9: 29 May 2019; JV. TEST/DEMO ONLY. Increased AGC thr to lower avg current draw
 *                             in noisy workplace.
 * 3.9->3.9.1: 05 June 2019; JV. Using 6s timer to manually perform WOR. 6s timer via WDT.
 * 3.9.1->3.9.2: 07 June 2019; JV. Easily allows the switch to RSSI reads or normal (manual)
 *                             WOR detection. No USART usage. LEDs blink to indicate packet
 *                             reception success. TEST ONLY.
 * 3.9.2->3.9.3: 12 June 2019; JV. Tuned RSSI threshold (hard-coded, not dynamic). TEST ONLY.
 *                             Removed 4min wait for repeat signals - DEMO PURPOSES.
 * 3.9.6->3.9.7: 16 Jul. 2019; JV. Changed R_LED pulse time to 200ms upon reception of hub ACK
 *                             packet to prevent resetting.
 * 3.9.7->3.9.8: 20 Jul. 2019; JV. Testing serial synchronous readout in place of FIFO.
 * 3.9.8->3.9.9: 26 Jul. 2019; JV. Added preamble and sync word detection to serial readout.
 *                             Decodes packet OK.
 * 3.9.9->3.9.10: 29 Jul. 2019; JV. Started from v3.9.7 and added synchronous serial decoding.
 *                             Changed radio settings to interrupt on 32bit sync word.
 */

#include <xc.h>
//#include <htc.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <pic16lf1829.h>
#include "config.h"
#include "cc1120_reg_config.h"
#include "handle_data.h"
#include "uart.h"
#include "spi.h"

/******************************
 * STATIC FUNCTIONS
 */
static void initializePIC(void);
void writePreamble(uint16_t preambleTime);
void startup_flash();

#define REVISION "A03"
#define PREAMBLE_TIMESPAN   6000        // WOR periodicity is currently 6s

/******************************************************************************
 * VARIABLES
 */

void __interrupt isr()
{   
    uint8_t temp;
    
    // ACK from hub - 7 bytes data in HEX - '$' + 3byte ID + 1byte status + <CR> + <LF>
    if (!receivedACK && PIR1bits.RCIF)
    {
        temp = RCREG;
        RS232Buf[rx_cnt++] = temp;
        
        PIR1bits.RCIF == 0;
        
        if(rx_cnt > MAX_SIZE)
            rx_cnt = 0;
        
        if (temp == LF)
        {
            rx_cnt = 0;
            receivedACK = true;
        }
    }
    
    /* GPIO0 interrupt */
    if(IOCBFbits.IOCBF7)        // BF7 HERE**
    {
        INTCONbits.IOCIF = 0;
        IOCBFbits.IOCBF7 = 0;   // BF7 HERE**
        receivedBit = true;
        currentBitByte = (SERIAL_DATA & 0x01);
    }
    
    if (PIR1bits.TMR1IF)
    {
        PIR1bits.TMR1IF = 0;
        t1cnt++;
        reload_preamble_timer();
        if (t1cnt >= 12)//24)//_T1_6S)
        {
            t1cnt = 0;
            pktTimedOut = true;
        }
            
    }
}

void main(void)
{   
    initializePIC();
    
    /* START-UP FLASH */
    startup_flash();

    initializeSPI();
    
    /* Write Radio Regs */
    registerConfig();
    
    /* Calibrate */
    manualCalibration();
    calibrateRCOsc();
    
    /* Flush RX FIFO */
//    trxCmdStrobe(CC1120_SFRX);
//    __delay_ms(1);
    trxCmdStrobe(CC1120_SIDLE);
    while ((trxCmdStrobe(CC1120_SNOP) & 0x70) != 0x00)
        NOP();
    // comment for rssi
    //trxCmdStrobe(CC1120_SPWD);
        trxCmdStrobe(CC1120_SRX);
        CLRWDT();
//        __delay_ms(50);
        CLRWDT();
    trxCmdStrobe(CC1120_SIDLE);
    while ((trxCmdStrobe(CC1120_SNOP) & 0x70) != 0x00)
        NOP();
    trxCmdStrobe(CC1120_SPWD);
   
    start_rssi_timer();
    
    /* Inf Loop */
    while(1)
    {
        CLRWDT();
        cycle_radio();
        check_for_packet();
        
        if (_7minTimerOn)
            check7minTimer();
        
        if (!receivedSync && radioState == DetectRSSI)
        {
            // TODO - 4min timer will have to be lowered to compensate 
            //          for the instance when this timer is not advanced
            //          during other packet reception
            check_packet_timer();   

            flashing_red();
            WDTCONbits.SWDTEN = 1; 
            CLRWDT();
            SLEEP();
            NOP();
            NOP();
            NOP();
            
            WDTCONbits.SWDTEN = 0;
            
            flashing_green();
            //WPUB4 = 0;
        }
    }
}

void startup_flash()
{
    WDTCONbits.SWDTEN = 0;
    for (uint8_t i = 0; i < 5; i++)
    {
        //WDTCONbits.SWDTEN = 0;
        R_LED_ON;
        G_LED_ON;
        __delay_ms(50);
        R_LED_OFF;
        G_LED_OFF;   
        __delay_ms(50);
        
    }
    WDTCONbits.SWDTEN = 1;
}

static void initializePIC(void)
{
    CLRWDT();
    
    // SW control WDT
    WDTCONbits.SWDTEN = 0;
    
   // Config WDT interval
    WDTCONbits.WDTPS = WATCHDOG_SLEEP_128ms;
    WDTCONbits.SWDTEN = 1;
    INTCON = 0;                         /* Disable all interrupts */
    INTCONbits.GIE = 1;     
    INTCONbits.IOCIF = 0;
    
    //IOCAP=0b00000010;       /* detect positive edge, RA1:Tx, RA3: switch,RA4: cover sw */
    //IOCAN=0b00100000;     /* detect negative edge, RA5:Prog SW */

    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    
    OSCTUNE = 0;
//    OSCCON = 0b01111000;    // INTOSC 16MHz
    
    
    OPTION_REG = 0b00000111;  // WPU are enabled by individ. WPU latch vals.
    APFCON0 = 0b10000100;   // RC4->USART TX , RC5->USART RX
   
    TRISA = 0b00111011;//00111001;
    WPUA = 0b00000001;
    
    TRISB = 0b10010000;     // RB7 on GPIO2, RB4 on SDI
    WPUB = 0b00000000;      // no WPU on SDI
    
    TRISC = 0b00011011;//00111010;     // RC3 on GPIO0, RC1 on GPIO3
    WPUC = 0b00001001;      // no WPU on ~SS, n_reset
    
//    LATA = 0x00;
//    LATB = 0x00;
//    LATC = 0b01111111;
//    INTCONbits.PEIE = 1;
    
//    T1CONbits.nT1SYNC = 1;
//    T1CONbits.TMR1CS = 0b00;//10
//    T1CONbits.T1CKPS = 0b11;
    
//    init_uart();
//    start_uart();
}

void registerConfig(void) 
{
    uint8_t writeByte;
    uint16_t i;

    // Reset radio
    trxCmdStrobe(CC1120_SRES);
    trxCmdStrobe(CC1120_SIDLE);
    while ((trxCmdStrobe(CC1120_SNOP) & 0x70) != 0x00)
        NOP();
    
    // Write registers to radio
    for( i = 0; i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) 
    {
        writeByte = preferredSettings[i].data;
        cc1120SpiWriteReg(preferredSettings[i].addr, &writeByte, 1);
    }
}

extern void sendAck(void)
{
    CLRWDT();
    // Initialize packet buffer of size PKTLEN + 1
    uint8_t txBuffer[1] = {0};
    
    // Calibrate radio according to errata
    manualCalibration();
    CLRWDT();
    // Create a packet for ACK transmission (consists of short preamble, 16bit sync, single byte ACK)
    createAckPacket(txBuffer);
    cc1120SpiWriteTxFifo(txBuffer, sizeof(txBuffer));
    // Strobe TX to send packet
    trxCmdStrobe(CC1120_SFTX);
    __delay_ms(10);
    trxCmdStrobe(CC1120_STX);
    __delay_ms(20);
    /* Write Radio Regs */
    registerConfig();
    
    /* Calibrate */
    manualCalibration();
    calibrateRCOsc();
    //trxCmdStrobe(CC1120_SWORRST);
    __delay_ms(10);
    /* Start WOR */
    //trxCmdStrobe(CC1120_SWOR);
    CLRWDT();
}

void successLED(void)
{
    G_LED_ON;
    CLRWDT();
    __delay_ms(100);
    CLRWDT();
    __delay_ms(100);
    CLRWDT();
    G_LED_OFF;
}

void check7minTimer() // delete one at a time
{
    uint8_t deleteIndex = 0xFF, i = 0;
    bool allTimersOff = true;
    while ((deleteIndex == 0xFF) && (i < endMsgPtr))
    {
        if (msgTmrState[i] == TIMER_DONE)
        {
            msgTmrState[i] = OFF;
            msgTmrCnt[i] = 0x00;
            msgReceived[i] = 0x00;
            deleteIndex = i;
        }
        allTimersOff &= (bool)(msgTmrState[i] == OFF);
        i++;
    }
    if (deleteIndex == 0xFF && allTimersOff)
    {
        _7minTimerOn = false;
        T1CONbits.TMR1ON = 0;
        T1CONbits.T1OSCEN = 0;
        PIE1bits.TMR1IE = 0;
        endMsgPtr = 0;
    }
    if (deleteIndex != 0xFF)
    {
        for (i = 0; i < endMsgPtr; i++)   // position in all of these buffers corresp. to one another
        {
            msgReceived[i] = msgReceived[(uint8_t)(i + 1)];
            msgTmrState[i] = msgTmrState[(uint8_t)(i + 1)];
            msgTmrCnt[i]   = msgTmrCnt[(uint8_t)(i + 1)];
        }
        msgReceived[(uint8_t)(endMsgPtr - 1)] = OFF;     // Make sure final index is defined explicitly
        endMsgPtr--;
    }
}
