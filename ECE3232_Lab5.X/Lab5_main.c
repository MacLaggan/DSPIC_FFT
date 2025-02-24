/*
 * File:   Lab5_main.c
 * Author: MDMac
 *
 * Created on February 11, 2025, 2:53 PM
 */


// CONFIG1
#pragma config FEXTOSC = ECH    // External Oscillator mode selection bits (EC above 8MHz; PFM set to high power)
#pragma config RSTOSC = HFINT1 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#define _XTAL_FREQ 32000000

void configPIN(){
    //LED digital output
    //TRISAbits.TRISA0 = 0;
    //ANSELAbits.ANSA0 = 0;
    
    //Analog output
    ANSELBbits.ANSB7 = 1;
    TRISBbits.TRISB7 = 0;
    
}

void configTIMER(){
    //TIMER0 is an 8-bit timer
    T0CON0bits.T016BIT = 0;
    //Setting input to FOSC/4
    T0CON1bits.T0CS = 0b010;
    //Setting prescalar to 1:1024
    T0CON1bits.T0CKPS = 0b110;
}

void configINT(){
    //Global interrupt enabled
    INTCONbits.GIE = 1;
    //Peripheral interrupt enabled
    INTCONbits.PEIE = 1;
    //TIMER0 interrupt enable
    PIE0bits.TMR0IE = 1;
}

void configDAC(){
    //Configuring DAC output
    DAC1CON0bits.DAC1OE1 = 1;
    DAC1CON0bits.DAC1OE2 = 1;
    
    //Configuring DAC referance voltages
    DAC1CON0bits.PSS = 00;  //VDD
    DAC1CON0bits.NSS = 0;   //VSS
    
    //Enabling DAC1
    DAC1CON0bits.DAC1EN = 1;
}

//Global variable
int LED = 31;
int IN = 0;
void main(void) {
    configPIN();
    configTIMER();
    configINT();
    configDAC();
    T0CON0bits.T0EN = 1;
    
    while(1){
        NOP();
    }
    
    return;
}

void __interrupt() TIMER0(){
    if(PIR0bits.TMR0IF == 1){
        if(IN == 0){
            if(LED != 8){
                LED--;
            }
            else{
                IN = 1;
            }
            DAC1CON1bits.DAC1R = LED;
        }
        if(IN == 1){
            if(LED != 31){
                LED++;
            }
            else{
                IN = 0;
            }
            DAC1CON1bits.DAC1R = LED;
        }
        
        PIR0bits.TMR0IF = 0;
    }
}