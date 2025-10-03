/*
 * File:   HW26k83.c
 * Author: gaetan.paulet
 *
 * Created on 25 septembre 2023, 15:15
 * 
 * Programme "Hello World" :
 * -Bits de configuration
 * -Initialisations des SFR de base
 * -Utilisation des sorties (clignotement LED) : LED blinking
 * -Utilisation des entr�es (lecture de l'�tat d'un bouton poussoir) : BLOC 2
 * -D�tection de l'appui sur un bouton  : BLOC 3
 * -Utilisation de l'�cran LCD : BLOC 4
 * 
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
    char etat = 1;
    
    TRISA = 0b11000000;     //RA7 et RA6 : boutons ; RA5 � RA0 : connexions LCD
    TRISB = 0;              //RB5 et RB4 : LEDs ; les autres bits sont non utilis�s pour l'instant
    TRISC = 0b00000011;     //RC2 : LED ; RC1  et RC0 : boutons ; les autres bits sont non utilis�s pour l'instant
    
    ANSELA = 0;             //D�sactivation
    ANSELB = 0;             //des
    ANSELC = 0;             //entr�es analogiques
    
    PORTB = 0;
    PORTC = 0;
    
    Lcd_Init();             //Initialisation du LCD
    Lcd_Clear();            //Effacement du LCD
    
    while(1)
    {
        //////////////////////////////////////////////////////////////////////////
        //LED blinking : La LED connect�e � RC2 change d'�tat � chaque
        //it�ration de la boucle
        PORTCbits.RC2 = !PORTCbits.RC2;    //Structure qui permet l'acc�s
                                           //� un bit d'un registre
        __delay_ms(1000);                  //D�lai de 1000ms = 1 seconde
        //LED blinking - LED blinking - LED blinking - LED blinking - LED blinking
        //////////////////////////////////////////////////////////////////////////
        
        //////////////////////////////////////////////////////////////////////////
        //BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2
        if (PORTAbits.RA7 == 0) //Bouton RA7 appuy�
        {
            PORTBbits.RB5 = 1;  //On allume la LED sur RB5
        }
        else                    //Bouton RA7 rel�ch�
        {
            PORTBbits.RB5 = 0;  //On �teint la LED sur RB5
        }
        //BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2 - BLOC2
        //////////////////////////////////////////////////////////////////////////
        
        //////////////////////////////////////////////////////////////////////////
        //BLOC3 - BLOC3 - BLOC3 - BLOC3 - BLOC3 - BLOC3 - BLOC3
        if (PORTCbits.RC1 == 0 && etat == 1) //Si on vient d'appuyer sur RC1
        {
            PORTB = PORTB ^ 0b00010000;      //On change uniquement RB4 gr�ce � un XOR
            etat = 0;                        //On �vite de rentrer une deuxi�me fois dans
            __delay_ms(5);                   //le if sans avoir rel�ch� le bouton            
        }
        if (PORTCbits.RC1 == 1 && etat == 0) //Si on rel�che le bouton
        {
            etat = 1;                       //La variable etat est remise � 1 pour
            __delay_ms(5);                  //pouvoir prendre en compte le prochain appui
        }
        //BLOC3 - BLOC3 - BLOC3 - BLOC3 - BLOC3 - BLOC3 - BLOC3
        //////////////////////////////////////////////////////////////////////////
        
        //////////////////////////////////////////////////////////////////////////
        //BLOC4 - BLOC4 - BLOC4 - BLOC4 - BLOC4 - BLOC4 - BLOC4
        Lcd_Clear();
        Lcd_Set_Cursor(1,1);
        Lcd_Write_String("Bienvenue en"); 
        Lcd_Set_Cursor(2,4);
        Lcd_Write_String("electronique");        
        //BLOC4 - BLOC4 - BLOC4 - BLOC4 - BLOC4 - BLOC4 - BLOC4
        //////////////////////////////////////////////////////////////////////////
        
    }    
    return;
}
