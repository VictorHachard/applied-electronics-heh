/*
 * File:   SPI26K83.c
 * Author: gaetan.paulet
 *
 * Created on 9 octobre 2023, 14:13
 * 
 * Communication SPI entre le PIC18F26k83 et l'EEPROM M93C66.
 * SDO = RC5
 * SDI = RC4
 * SCK = RC3
 * SS = RB3
 * Le composant est c�bl� pour travailler avec 512 registres de 8 bits
 * Les adresses internes � l'EEPROM vont donc de 0x000 � 0x1FF
 * 
 * L'initialisation configure les registres li�s au SPI et active les �critures
 * 
 * Dans la boucle infinie,
 *  1) L'appui sur RC0 �crit la valeur 164 � l'adresse 0x000
 *  2) L'appui sur RC1 lit la valeur � l'adresse 0x000 et l'affiche sur le LCD
 * 
 * Le code n'est pas optimis�, il faut transformer ce code pour travailler avec des fonctions
 * d'�criture et de lecture sur le bus SPI qui prennent en param�tres l'adresse du registre 
 * auquel on veut acc�der et la valeur � �crire (pour l'�criture)
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

//Association des sorties du �C aux entr�es du LCD utilis�es dans lcd.h
#define RS PORTAbits.RA0
#define EN1 PORTAbits.RA1
#define D4 PORTAbits.RA2
#define D5 PORTAbits.RA3
#define D6 PORTAbits.RA4
#define D7 PORTAbits.RA5

#include <xc.h>
#include "lcd.h"        //Ne pas oublier d'ajouter ce fichier dans le dossier du projet !
#include <stdio.h>

static void u8_hex(char*out, uint8_t v) {
    const char *h="0123456789ABCDEF";
    out[0]='0'; out[1]='x'; out[2]=h[v>>4]; out[3]=h[v&0xF]; out[4]=0;
}

void uart_send_string(const char *s) {
    while (*s) {
        while (!PIR3bits.U1TXIF);
        U1TXB = *s++;
    }
    while (!PIR3bits.U1TXIF);
    U1TXB = '\r';
    while (!PIR3bits.U1TXIF);
    U1TXB = '\n';
}

static void eeprom_EWEN(void) {
    SPI1CON0bits.EN = 1;
    PORTBbits.RB3 = 1;
    SPI1TWIDTH = 4;
    SPI1TCNTL  = 1;
    SPI1TCNTH  = 0;
    SPI1CON2bits.TXR = 1;
    SPI1CON0bits.EN = 1;

    SPI1TXB = 0b10011000;
    while(SPI1STATUSbits.TXBE == 0);
    SPI1TXB = 0b00001111;
    while(SPI1STATUSbits.TXBE == 0);

    SPI1CON2bits.TXR = 0;
    PORTBbits.RB3 = 0;
    SPI1CON0bits.EN = 0;
}

uint8_t eeprom_read_8(uint16_t addr) {
    uint8_t b1 = (0b110 << 5) | ((addr >> 4) & 0x1F); // 110 A8..A4
    uint8_t b2 = ((addr & 0x0F) << 4) | 0x0F; // A3..A0 1111
    uint8_t val2, val3;

    PORTBbits.RB3 = 1;
    SPI1TWIDTH = 0;
    SPI1TCNTL  = 3;
    SPI1TCNTH  = 0;
    SPI1CON2bits.TXR = 1;
    SPI1CON2bits.RXR = 1;
    SPI1CON0bits.EN  = 1;

    SPI1TXB = b1;
    while(SPI1STATUSbits.TXBE == 0);
    (void)SPI1RXB; // dummy

    SPI1TXB = b2;
    while(SPI1STATUSbits.TXBE == 0);
    val2 = SPI1RXB; // 4 dummy + D7..D4

    SPI1TXB = 0xFF; // dummy
    while(SPI1STATUSbits.TXBE == 0);
    val3 = SPI1RXB; // D3..D0 + 4 dummy

    SPI1CON2bits.RXR = 0;
    SPI1CON2bits.TXR = 0;
    PORTBbits.RB3 = 0;
    SPI1CON0bits.EN = 0;

    return (uint8_t)(((val2 & 0x0F) << 4) | ((val3 >> 4) & 0x0F));
}

void eeprom_write_8(uint16_t addr, uint8_t data) {
    uint8_t b1 = (0b101 << 5) | ((addr >> 4) & 0x1F); // 101 A8..A4
    uint8_t b2 = ((addr & 0x0F) << 4) | ((data >> 4) & 0x0F); // A3..A0 D7..D4
    uint8_t b3 = ((data & 0x0F) << 4) | 0x0F; // D3..D0 1111

    eeprom_EWEN();

    PORTBbits.RB3 = 1;
    SPI1TWIDTH = 4;
    SPI1TCNTL  = 2;
    SPI1TCNTH  = 0;
    SPI1CON2bits.TXR = 1;
    SPI1CON0bits.EN = 1;

    SPI1TXB = b1;
    while(SPI1STATUSbits.TXBE == 0);
    SPI1TXB = b2;
    while(SPI1STATUSbits.TXBE == 0);
    SPI1TXB = b3;
    while(SPI1STATUSbits.TXBE == 0);

    SPI1CON2bits.TXR = 0;
    PORTBbits.RB3 = 0;
    SPI1CON0bits.EN = 0;

    __delay_ms(5);
}

void main(void) {
    char etatC0 = 1;
    
    
    TRISA = 0b11000000;     //RA7 et RA6 : boutons, reste du PORTA connect� LCD
    TRISB = 0;              //RB5 et RB4 = LEDs ; RB3 = SS
    TRISC = 0b10010011;     //RC7 = RX1 ; RC6 = TX1 ; RC5 = SDO ; 
                            //RC4 = SDI ; RC3 = SCK ; RC2 = LED ;
                            //RC1 et RC0 = boutons
    ANSELA = 0;             //Pas d'entr�es analogiques
    ANSELB = 0;             //Pas d'entr�es analogiques   
    ANSELC = 0;             //Pas d'entr�es analogiques
    ODCONB = 0b00000110;    //RB1 et RB2 en drain ouvert pour l'I2C
    PORTB = 0;
    PORTC = 0;
    PORTBbits.RB3 = 0;      //RB3 = SS
    
    //Config PPS (Entr�es et sorties � fonctions p�riph�riques sp�cifiques)
    //UART
    U1RXPPS = 0b00010111;    //RC7 d�finie comme RX1 (valeur par d�faut maintenue)
    RC6PPS  = 0b00010011;    //RC6 d�finie comme TX1 
    //I2C
    I2C2SCLPPS = 0b00001001; //RB1 = SCL2 (valeur par d�faut)
    I2C2SDAPPS = 0b00001010; //RB2 = SDA2 (valeur par d�faut)
    RB1PPS     = 0b00100011;
    RB2PPS     = 0b00100100;
    //SPI
    SPI1SDIPPS = 0b00010100; //RC4 = SDI (input)
    RC5PPS     = 0b00011111; //RC5 = SDO (output)
    RC3PPS     = 0b00011110; //RC3 = SCK (output)
        
    //Config UART1 (vers PC))
    U1CON0 = 0b10100000;    //BRGS=1 ; No auto-baud ; TXEN = 1 ; RXEN = 0 ; Asynchrone 8bits
    U1CON1 = 0b10000000;    //Serial ON ; on garde les autres bits � 0 (valeurs par d�faut) car fonctions non utilis�es
    U1CON2 = 0;             //Valeurs par d�faut
    U1BRG  = 1666;          //Valeur sur 16bits (U1BRGH et U1BRGL) : Avec BRGS = 1 et Fosc = 64Mhz
                            //Donne 9600 Baud (9592 ne r�alit� = 0,08% erreur)
    
    //Config I2C2
    I2C2CON0 = 0b10000100;       //I2C ON ; I2C Master mode, 7 bit-address
    I2C2CLK = 0b00000000;        //CLK = MFINTOSC (500kHz)
    //Les autres valeurs sont laiss�es par d�faut
    
    //Config SPI
    SPI1CON0 = 0b00000010; //Mode master
    SPI1CON1 = 0b11000000; //�tat IDLE des bus = 0
            
    SPI1TWIDTH = 0;
    SPI1TCNTL  = 0;
    SPI1TCNTH  = 0;
    
    SPI1CLK  = 0b00000010; //Source de la CLK = MFINTOSC
    SPI1BAUD = 100;        //Fclk = ?? � calculer
    
            
    Lcd_Init();
    Lcd_Clear();
    PORTBbits.RB5 = 1;
    __delay_ms(100);

    while(1)
    {
        __delay_ms(100);
        PORTBbits.RB4 = !PORTBbits.RB4;
        
        if (PORTCbits.RC0 == 0 && etatC0 == 1)
        {
            uint16_t addr = 0x000;
            uint8_t  data = 0xA4;
            char buf[16], hx[8];

            eeprom_write_8(addr, data);
            uint8_t r = eeprom_read_8(addr);

            u8_hex(hx, r);
            sprintf(buf, "[0x%03X]=%s", addr, hx); // 14 char
            
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String(buf);

            uart_send_string(buf);
            
            etatC0 = 0;
        }       
        if (PORTCbits.RC0 == 1 && etatC0 == 0)
        {
            etatC0 = 1;
            __delay_ms(10);
        }                  
    }
    return;
}
