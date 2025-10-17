/*
 * File:   BLU26k83.c
 * Author: gaetan.paulet
 * 
 * Created on 23 octobre 2023, 08:34
 * 
 * Communication UART entre le 18F26k83 et le module Bluetooth HC-05.
 * TX2 = RB6
 * RX2 = RB7
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
    char ligne = 1, etatRA6 = 1, etatRA7 = 1;
    
    TRISA = 0b11000000;
    TRISB = 0b10000000;  //RB7 = RX2(in), RB6 = TX2(out)
    TRISC = 0b10000011;
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    PORTB = 0;
    PORTC = 0;
    
    U2RXPPS = 0b00001111;   //RB7 définie comme RX2 
    RB6PPS  = 0b00010110;   //RB6 définie comme TX2 
    
    U2CON0 = 0b10100000;    //BRGS=1 ; No auto-baud ; TXEN = 1 ; RXEN = 0 ; Asynchrone 8bits
    U2CON1 = 0b10000000;    //Serial ON ; on garde les autres bits à 0 (valeurs par défaut)        car fonctions non utilisées
    U2CON2 = 0;             //Valeurs par défaut
    U2BRG  = 1666;          //Valeur sur 16bits (U2BRGH et U2BRGL) : Avec BRGS = 1 et Fosc = 64Mhz
                            //Donne 9600 Baud (9598 en réalité = 0,02% erreur)
    Lcd_Init();
    Lcd_Clear();
    Lcd_Set_Cursor(ligne, 1);
            
    U2CON0bits.RXEN = 1;    //Autorisation de réception
        
    
    while(1)
    {
        
        ///////////////////////////////////////////////////////////////
        //BLOC1 - BLOC1 - BLOC1 - BLOC1 - BLOC1 - BLOC1 - BLOC1 - BLOC1 
        //Attente de réception du caractère A. Quand c'est le cas, le µC renvoie OK.
        if (U2FIFObits.RXBE == 0) //Flag à 1 lorsque le buffer de réception est vide
        {
            if (U2RXB == 65)      //Buffer de réception contient A ?
            {
                PORTCbits.RC2 = !PORTCbits.RC2; 
                U2TXB = 79;                     //Permet de renvoyer le cartère O
                while(PIR7bits.U2TXIF == 0);    //On attend que le premier octet soit envoyé
                U2TXB = 75;                     //Permet de renvoyer le cartère O                
            }                
        }
        //BLOC1 - BLOC1 - BLOC1 - BLOC1 - BLOC1 - BLOC1 - BLOC1 - BLOC1
        ///////////////////////////////////////////////////////////////
        
        
        /*
        ///////////////////////////////////////////////////////////////
        //BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2 
        //Les caractères reçus sont affichés sur le LCD, un appui sur RA7
        //change de ligne, un appui sur RA6 efface l'écran
        if (U2FIFObits.RXBE == 0) //Flag à 1 lorsque le buffer de réception est vide
        {
           Lcd_Write_Char(U2RXB);                
           PORTCbits.RC2 = !PORTCbits.RC2;
        }
        if (PORTAbits.RA7 == 0 && etatRA7 == 1)
        {
            ligne = ligne ^ 0b00000011;
            Lcd_Set_Cursor(ligne, 1);
            etatRA7 = 0;
            __delay_ms(5);
        }
        if (PORTAbits.RA7 == 1 && etatRA7 == 0)
        {
            etatRA7 = 1;
            __delay_ms(5);
        }
        if (PORTAbits.RA6 == 0 && etatRA6 == 1)
        {
            ligne = 1;
            Lcd_Clear();
            Lcd_Set_Cursor(ligne, 1);
            etatRA6 = 0;
            __delay_ms(5);
        }
        if (PORTAbits.RA6 == 1 && etatRA6 == 0)
        {
            etatRA6 = 1;
            __delay_ms(5);
        }
        //BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2
        ///////////////////////////////////////////////////////////////
       */
    
    }
    
    return;
}
