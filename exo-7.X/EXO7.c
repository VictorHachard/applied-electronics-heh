/*
 * Exo interruptions + Bluetooth (HC-05)
 * - RC2 : clignote toutes les 2 s (boucle principale)
 * - IT IOC sur RC0 : envoie un message via BT (UART2)
 * - IT UART2 RX : 'A' -> start blink RB5 (1 s), 'B' -> stop blink RB5
 */

#pragma config FEXTOSC=OFF
#pragma config RSTOSC=HFINTOSC_64MHZ
#pragma config CLKOUTEN=OFF, PR1WAY=OFF, CSWEN=ON, FCMEN=ON
#pragma config MCLRE=EXTMCLR, PWRTS=PWRT_OFF, MVECEN=OFF, IVT1WAY=OFF, LPBOREN=OFF, BOREN=SBORDIS
#pragma config BORV=VBOR_2P45, ZCD=OFF, PPS1WAY=OFF, STVREN=ON, DEBUG=OFF, XINST=OFF
#pragma config WDTCPS=WDTCPS_31, WDTE=OFF
#pragma config WDTCWS=WDTCWS_7, WDTCCS=SC
#pragma config BBSIZE=BBSIZE_512, BBEN=OFF, SAFEN=OFF, WRTAPP=OFF
#pragma config WRTB=OFF, WRTC=OFF, WRTD=OFF, WRTSAF=OFF, LVP=OFF
#pragma config CP=OFF

#define _XTAL_FREQ 64000000UL

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

volatile bool blink_rb5 = false;
volatile bool ioc_rc0_event = false;

static void bt_send_str(const char *s) {
    while (*s) {
        while (PIR7bits.U2TXIF == 0);
        U2TXB = *s++;
    }
    while (PIR7bits.U2TXIF == 0);
    U2TXB = '\r';
    while (PIR7bits.U2TXIF == 0);
    U2TXB = '\n';
}

static void init_hw(void)
{
    /* GPIO */
    TRISA = 0b11000000;         // RA7/RA6 boutons
    TRISB = 0b10000000;         // RB7 RX2 (in), RB6 TX2 (out), RB5 LED, RB4 libre
    TRISC = 0b00000011;         // RC1/RC0 boutons ; RC2 LED
    ANSELA = 0; ANSELB = 0; ANSELC = 0;
    PORTB = 0; PORTC = 0;

    /* PPS UART2 (RB7=RX2, RB6=TX2) – idem BLU26K83.c */
    U2RXPPS = 0b00001111;       // RB7 -> RX2
    RB6PPS  = 0b00010110;       // TX2 -> RB6

    /* UART2 @ 9600 bauds (BRGS=1, Fosc=64 MHz -> U2BRG=1666) */
    U2CON0 = 0b10100000;        // BRGS=1, TXEN=1, RXEN=0 (on activera RX après)
    U2CON1 = 0b10000000;        // ON
    U2CON2 = 0;
    U2BRG  = 1666;              // ~9600 bauds

    /* Interruptions GPIO on-change (IOC) pour RC0 (flanc descendant = appui) */
    INTCON0 = 0b10000000;       // GIE=1, pas de priorités
    PIE0    = 0b10000000;       // IOC enabled
    IOCCN   = 0b00000001;       // flanc descendant sur RC0
    IOCCF   = 0;                // clear flags

    /* IT UART2 RX */
    PIR7bits.U2RXIF = 0;        // clear
    PIE7bits.U2RXIE = 1;        // enable RX interrupt

    /* Activer la réception UART2 */
    U2CON0bits.RXEN = 1;
}

void __interrupt() isr(void)
{
    /* IOC sur RC0 : envoi d’un petit message BT directement depuis l’IT */
    if (PIR0bits.IOCIF && IOCCFbits.IOCCF0) {
        IOCCFbits.IOCCF0 = 0;       // clear flag ligne
        bt_send_str("RC0");         // message BT à l’appui
        PIR0bits.IOCIF = 0;         // clear global IOCIF si besoin
    }

    /* UART2 RX : lecture et action */
    if (PIE7bits.U2RXIE && PIR7bits.U2RXIF) {
        uint8_t c = U2RXB;              // lit l’octet reçu
        if (c == 'A') {
            blink_rb5 = true;
            bt_send_str("OK_A");
        } else if (c == 'B') {
            blink_rb5 = false;
            PORTBbits.RB5 = 0;          // éteint proprement
            bt_send_str("OK_B");
        }
        PIR7bits.U2RXIF = 0;            // clear flag RX
    }
}

int main(void)
{
    init_hw();

    uint8_t tick50_rc2 = 0; // 20 ticks -> 2 s
    uint8_t tick50_rb5 = 0; // 10 ticks -> 1 s

    while (1) {
        __delay_ms(50);

        /* RC2 : clignote toutes les 2 s */
        if (++tick50_rc2 >= 40) {
            PORTCbits.RC2 ^= 1;
            tick50_rc2 = 0;
        }

        /* RB5 : clignote à 1 s si BT */
        if (blink_rb5) {
            if (++tick50_rb5 >= 20) {
                PORTBbits.RB5 ^= 1;
                tick50_rb5 = 0;
            }
        } else {
            tick50_rb5 = 0;
        }
    }
}
