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
#include <stdio.h>

void format_time(char *buffer, unsigned char hours, unsigned char minutes, unsigned char seconds) {
    buffer[0] = (hours / 10) + '0';
    buffer[1] = (hours % 10) + '0';
    buffer[2] = ':';
    buffer[3] = (minutes / 10) + '0';
    buffer[4] = (minutes % 10) + '0';
    buffer[5] = ':';
    buffer[6] = (seconds / 10) + '0';
    buffer[7] = (seconds % 10) + '0';
    buffer[8] = '\0';
}

char rtc_read(int reg_adress) {  // like 0x01
    I2C2CNT = 1;                        //Count = 1 : adresse registre
    I2C2ADB1 = 0b11010000;              //Adresse en �criture de la DS1307
    I2C2TXB = reg_adress;               //Adresse du registre de la DS1307 dans lequel on veut �crire
    I2C2CON0bits.S = 1;                 //Demande d'envoi du start
    while (I2C2PIRbits.SCIF == 0);      //Attente de l'envoi du start
    I2C2PIRbits.SCIF = 0;               //Remise � 0 du flag
    while (I2C2STAT1bits.TXBE == 0);    //Attente de l'envoi de l'adresse de la RTC
    while (I2C2PIRbits.PCIF == 0);      //Attente de l'envoi de la donn�e et du stop (Le stop est envoy� automatiquement)
    I2C2PIRbits.PCIF = 0;               //Remise � 0 du flag
    
    I2C2ADB1 = 0b11010001;              //Adresse en lecture de la DS1307
    I2C2CNT = 1;                        //Count = 1 : 1 registre � lire
    I2C2CON0bits.S = 1;                 //Demande d'envoi du start
    while (I2C2PIRbits.SCIF == 0);      //Attente de l'envoi du start
    I2C2PIRbits.SCIF = 0;               //Remise � 0 du flag
    I2C2CON1bits.ACKCNT = 1;            //Permet d'envoyer un ACK apr�s r�ception de la donn�e
    while (I2C2STAT1bits.RXBF == 0);    //Attente de r�ception de la donn�e
    char result = I2C2RXB;               //On r�cup�re le contenu du buffer de r�ception dans lectureRTC
    //Le stop est envoy� automatiquement apr�s l'ACK
    while (I2C2PIRbits.PCIF == 0);      //Attente du stop
    I2C2PIRbits.PCIF = 0;               //Remise � 0 du flag
    return result;
}

void bt_send_str(const char *s) {
    while (*s) {
        while (PIR7bits.U2TXIF == 0);
        U2TXB = *s++;
    }
    while (PIR7bits.U2TXIF == 0);
    U2TXB = '\r';
    while (PIR7bits.U2TXIF == 0);
    U2TXB = '\n';
}

void rtc_write(uint8_t reg_address, uint8_t value) {
    I2C2CON0bits.RSEN = 0;      // No repeated start
    I2C2CNT = 2;                // 2 bytes: register address + data
    I2C2ADB1 = 0b11010000;      // DS1307 write address
    I2C2TXB = reg_address;      // Target register
    I2C2CON0bits.S = 1;         // Start condition
    while (I2C2PIRbits.SCIF == 0);
    I2C2PIRbits.SCIF = 0;
    while (I2C2STAT1bits.TXBE == 0);
    I2C2TXB = value;            // Data to write
    while (I2C2PIRbits.PCIF == 0);
    I2C2PIRbits.PCIF = 0;
}

void rtc_write_dec(uint8_t reg_address, uint8_t value_dec) {
    rtc_write(reg_address, (uint8_t)(((value_dec / 10) << 4) | (value_dec % 10)));
}

uint8_t rtc_read_dec(uint8_t reg_address) {
    uint8_t bcd = rtc_read(reg_address);
    return (uint8_t)(((bcd >> 4) * 10) + (bcd & 0x0F));
}

void main(void) {   
    TRISA = 0b11000000;
    TRISB = 0b10000000;  //RB7 = RX2(in), RB6 = TX2(out)
    TRISC = 0b10000011;
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ODCONB = 0b00000110;    //RB1 et RB2 en drain ouvert pour l'I2C
    PORTB = 0;
    PORTC = 0;
    
    U2RXPPS = 0b00001111;   //RB7 d�finie comme RX2 
    RB6PPS  = 0b00010110;   //RB6 d�finie comme TX2 

    //Config PPS (Entr�es et sorties � fonctions p�riph�riques sp�cifiques)
    U1RXPPS = 0b00010111;    //RC7 d�finie comme RX1 (valeur par d�faut maintenue)
    RC6PPS  = 0b00010011;    //RC6 d�finie comme TX1 
    I2C2SCLPPS = 0b00001001; //RB1 = SCL2 (valeur par d�faut)
    I2C2SDAPPS = 0b00001010; //RB2 = SDA2 (valeur par d�faut)
    RB1PPS     = 0b00100011;
    RB2PPS     = 0b00100100;
    
    U2CON0 = 0b10100000;    //BRGS=1 ; No auto-baud ; TXEN = 1 ; RXEN = 0 ; Asynchrone 8bits
    U2CON1 = 0b10000000;    //Serial ON ; on garde les autres bits � 0 (valeurs par d�faut)        car fonctions non utilis�es
    U2CON2 = 0;             //Valeurs par d�faut
    U2BRG  = 1666;          //Valeur sur 16bits (U2BRGH et U2BRGL) : Avec BRGS = 1 et Fosc = 64Mhz
                            //Donne 9600 Baud (9598 en r�alit� = 0,02% erreur)

    //Config I2C2
    I2C2CON0 = 0b10000100;       //I2C ON ; I2C Master mode, 7 bit-address
    I2C2CLK = 0b00000011;        //CLK = MFINTOSC (500kHz)

    Lcd_Init();
    Lcd_Clear();

    U2CON0bits.RXEN = 1;    //Autorisation de r�ception

    Lcd_Clear();
    Lcd_Set_Cursor(1, 1);
    Lcd_Write_String("BT+RTC Ready");
    __delay_ms(500);

    rtc_write_dec(0x00, 0); // seconds
    rtc_write_dec(0x01, 30); // minutes
    rtc_write_dec(0x02, 14); // hours
    
    while(1)
    {
        
        if (U2FIFObits.RXBE == 0) { // octet reçu
            if (U2RXB == 72 || U2RXB == 104) { // 'H' ou 'h'
                PORTCbits.RC2 = !PORTCbits.RC2; 

                uint8_t s = rtc_read_dec(0x00);
                uint8_t m = rtc_read_dec(0x01);
                uint8_t h = rtc_read_dec(0x02);

                char time_str[9];
                format_time(time_str, h, m, s);

                Lcd_Clear();
                Lcd_Set_Cursor(1, 1);
                Lcd_Write_String(time_str);

                bt_send_str(time_str);
            }
        }
    }
    
    return;
}
