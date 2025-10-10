/*
 * File:   I2C26k83.c
 * Author: gaetan.paulet
 * 
 * Created on 26 septembre 2023, 13:17
 * 
 * Communication I2C entre le 18F16k83 et DS1307(0b11010000) et BMP280(0xEC).
 * SDA2 = RB2
 * SCL2 = RB1
 * 
 * Apr�s les configurations et initialisations de registres et du LCD,
 * le code initialise le registre des secondes de la RTC DS1307 � 0
 * 
 * Dans la boucle infinie, le premier bloc fait une lecture du registre des secondes de la RTC,
 * si la valeur lue est diff�rente du dernier r�sultat enregistr� (secondes), la LED RC2 change 
 * d'�tat.
 * 
 * Le deuxi�me bloc fait une lecture du registre de l'ID du BMP280 et affiche cet ID sur le LCD.
 * 
 * Le code n'est pas optimis�, il faut transformer ce code pour travailler avec des fonctions
 * d'�criture et de lecture sur le bus I2C qui prennent en param�tres l'adresse du composant,
 * l'adresse du registre auquel on veut acc�der et la valeur � �crire (pour l'�criture)
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

void bcd_to_dec(char* bcd) {
    *bcd = ((*bcd >> 4) * 10) + (*bcd & 0x0F);
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

void main(void) {
    char secondes = 0;
    char minute = 0;
    char last_secondes = -1;
    
    TRISA = 0b11000000;     //RA7 et RA6 : boutons, reste du PORTA connect� LCD
    TRISB = 0;              //RB5 et RB4 = LEDs ; RB2 = SDA2 ; RB1 = SCL2
    TRISC = 0b10000011;     //RC7 = RX1 ; RC6 = TX1 ; RC2 = LED ; RC1 et RC0 = boutons
    ANSELA = 0;             //Pas d'entr�es analogiques
    ANSELB = 0;             //Pas d'entr�es analogiques   
    ANSELC = 0;             //Pas d'entr�es analogiques
    ODCONB = 0b00000110;    //RB1 et RB2 en drain ouvert pour l'I2C
    PORTB = 0;
    PORTC = 0;
    
    //Config PPS (Entr�es et sorties � fonctions p�riph�riques sp�cifiques)
    U1RXPPS = 0b00010111;    //RC7 d�finie comme RX1 (valeur par d�faut maintenue)
    RC6PPS  = 0b00010011;    //RC6 d�finie comme TX1 
    I2C2SCLPPS = 0b00001001; //RB1 = SCL2 (valeur par d�faut)
    I2C2SDAPPS = 0b00001010; //RB2 = SDA2 (valeur par d�faut)
    RB1PPS     = 0b00100011;
    RB2PPS     = 0b00100100;
    
    //Config UART1 (vers PC))
    U1CON0 = 0b10100000;    //BRGS=1 ; No auto-baud ; TXEN = 1 ; RXEN = 0 ; Asynchrone 8bits
    U1CON1 = 0b10000000;    //Serial ON ; on garde les autres bits � 0 (valeurs par d�faut) car fonctions non utilis�es
    U1CON2 = 0;             //Valeurs par d�faut
    U1BRG  = 1666;          //Valeur sur 16bits (U1BRGH et U1BRGL) : Avec BRGS = 1 et Fosc = 64Mhz
                            //Donne 9600 Baud (9592 ne r�alit� = 0,08% erreur)
    
    //Config I2C2
    I2C2CON0 = 0b10000100;       //I2C ON ; I2C Master mode, 7 bit-address
    I2C2CLK = 0b00000011;        //CLK = MFINTOSC (500kHz)
    //Les autres valeurs sont laiss�es par d�faut
            
    Lcd_Init();
    Lcd_Clear();
    PORTBbits.RB5 = 1;
    __delay_ms(200);
    
    //Exemple d'�criture de la valeur 0x45 dans le registre 0 (secondes) de la DS1307
    I2C2CON0bits.RSEN = 0;              //On ne travaille pas avec un Repeated start
    I2C2CNT = 2;                        //Count = 2 : adresse registre + donn�e
    I2C2ADB1 = 0b11010000;              //Adresse en �criture de la DS1307
    I2C2TXB = 0;                        //Adresse du registre de la DS1307 dans lequel on veut �crire
    I2C2CON0bits.S = 1;                 //Demande d'envoi du start
    while (I2C2PIRbits.SCIF == 0);      //Attente de l'envoi du start
    I2C2PIRbits.SCIF = 0;               //Remise � 0 du flag
    while (I2C2STAT1bits.TXBE == 0);    //Attente de l'envoi de l'adresse de la RTC
    I2C2TXB = 0x45;                     //Valeur � �crire dans le registre de la DS1307
    while (I2C2PIRbits.PCIF == 0);      //Attente de l'envoi de la donn�e et du stop (Le stop est envoy� automatiquement)
    I2C2PIRbits.PCIF = 0;               //Remise � 0 du flag
    
    PORTBbits.RB5 = 0;                  //Si la LED RB5 s'�teint, c'est que l'�criture du 0 dans les secondes de la RTC est OK
     
    while(1)
    {
        __delay_ms(5);
        // get seconde and minute 
        secondes = rtc_read(0x00);
        minute = rtc_read(0x01);
        bcd_to_dec(&secondes);
        bcd_to_dec(&minute);

        if (secondes != last_secondes) {
            last_secondes = secondes;
            PORTCbits.RC2 = !PORTCbits.RC2;
            char buffer[6];
            sprintf(buffer, "%02d:%02d", minute, secondes);
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String(buffer);
            uart_send_string(buffer);
        }

    }
    
    return;
}
