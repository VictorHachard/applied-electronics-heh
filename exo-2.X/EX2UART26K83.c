/*
 * File:   UART26k83.c
 * Author: gaetan.paulet
 * 
 * Created on 26 septembre 2023, 11:17
 * 
 * Communication UART entre le 18F16k83 et le PC.
 * TX = RC6
 * RX = RC7
 */
// PIC18F26K83 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = OFF     // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be set and cleared repeatedly)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = OFF     // Multi-vector enable bit (Interrupt contoller does not use vector table to prioritze interrupts)
#pragma config IVT1WAY = OFF    // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set repeatedly)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTC = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (HV on MCLR/VPP must be used for programming)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// CONFIG5H

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 64000000          //Oscillateur interne
#define RS PORTAbits.RA0
#define EN1 PORTAbits.RA1
#define D4 PORTAbits.RA2
#define D5 PORTAbits.RA3
#define D6 PORTAbits.RA4
#define D7 PORTAbits.RA5

#include <xc.h>
#include "lcd.h"


void main(void) {
    
    TRISA = 0b11000000;
    TRISB = 0;
    TRISC = 0b10000011;
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    PORTB = 0;
    PORTC = 0;
    
    U1RXPPS = 0b00010111;   //RC7 d�finie comme RX1 (valeur par d�faut maintenue)
    RC6PPS  = 0b00010011;   //RC6 d�finie comme TX1 
    
    U1CON0 = 0b10100000;    //BRGS=1 ; No auto-baud ; TXEN = 1 ; RXEN = 0 ; Asynchrone 8bits
    U1CON1 = 0b10000000;    //Serial ON ; on garde les autres bits � 0 (valeurs par d�faut) car fonctions non utilis�es
    U1CON2 = 0;             //Valeurs par d�faut
    U1BRG  = 138;          //Valeur sur 16bits (U1BRGH et U1BRGL) : Avec BRGS = 1 et Fosc = 64Mhz
                            //Donne 115200
    
    Lcd_Init();             //Initialisation du LCD
    Lcd_Clear();            //Effacement du LCD
    
    char l_start = 0;

    while(1)
    {
        char message[] = {65,112,112,117,105,32,98,111,117,116,111,110,32,82,67,48,13,10}; // "Appui bouton RC0" + CR + LF

        if (PORTCbits.RC0 == 0) {
            for (int i = 0; i < sizeof(message)/sizeof(message[0]); i++) {
                U1TXB = message[i];
                __delay_ms(5);
            }
        }

        U1CON0bits.RXEN = 1;
        if (U1FIFObits.RXBE == 0) {
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            char c = U1RXB;
            char txt[2] = { c, '\0' };
            Lcd_Write_String(txt);
            if (U1RXB == 108) {
                l_start = 1;
                PORTCbits.RC2 = 1;
            }
            else if (l_start == 1 && U1RXB == 52) {
                l_start = 0;
                PORTCbits.RC2 = 0;
                __delay_ms(5);
                PORTBbits.RB4 = 1;
                __delay_ms(5);
                PORTBbits.RB5 = 0;
            }
            else if (l_start == 1 && U1RXB == 53) {
                l_start = 0;
                PORTCbits.RC2 = 0;
                __delay_ms(5);
                PORTBbits.RB5 = 1;
                __delay_ms(5);
                PORTBbits.RB4 = 0;
            }
            else {
                l_start = 0;
            }
            U1FIFObits.RXBE = 1;
        }
    }
    
    return;
}
