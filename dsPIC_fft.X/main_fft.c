/*
 * File:   mainXC16.c
 * Author: MDMac
 *
 * Created on January 30, 2025, 4:50 AM
 */
//========================================
// DSPIC33EP32MC202 Configuration Bit Settings
// 'C' source line config statements

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Source Selection (Fast RC Oscillator with divide-by-N with PLL module (FRCPLL) )
#pragma config PWMLOCK = OFF            // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)
  
// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)
//========================================

//Included Libraries
#include <xc.h>
#include <dsp.h>
#include <math.h>
//Definitions
#define _XTAL_FREQ 4000000
#define Fsamp 8000 //Sample frequency
#define FFT_SIZE 256 //Sample size

//Global Variables
fractcomplex sample[256] __attribute__((space(ymemory), aligned(FFT_SIZE*2*2))); //Global variable for the sample, stored in ymemory 
fractcomplex *psamp = &sample[0]; //Pointer to array containing the sample
int counter = 0;

//========================================
//Function declarations
//-delay
//-setup ADC
//-setup timer 1
//-setup clock frequency
//-setup pins (I/O)
//-write to LED register
//-setup interrupts

//delay = desired time / Tclk = Delay integer
void _delay(int time, int mult){
    for(int i = 0; i<mult; i++){
        TMR1 = 0;
        T1CONbits.TON = 1;
        while(TMR1 < time){
            Nop();        
        }
        T1CONbits.TON = 0;
    }
}

//Setup Analog to Digital Converter
void setupADC(){
    //Configuring Analog Input
    ANSELAbits.ANSA0 = 1; //Input
    TRISAbits.TRISA0 = 1; //Analog Pin
    
    //Select ADC conversion clock to be 1MHz
    AD1CON3bits.ADCS = 102; //(37)*TCLK = TAD (50 * 1/50MHz = 1/1.0MHz))
    
    AD1CON2bits.VCFG = 0b000; //Set to VDD and VSS
    
    //Setting ADC to Automatic Sampling
    AD1CON1bits.ASAM = 1; //Automatically sample
    AD1CON1bits.SSRCG = 0; //Allowing hardware to clear SAMP bit
    AD1CON1bits.SSRC = 0b111; //Auto-convert sample. 
    
    //Channel Select
    AD1CHS123bits.CH123NB = 00;
    AD1CHS123bits.CH123SB = 1;
    
    AD1CON1bits.FORM = 0b10; //Output form set to fractional

    AD1CON1bits.AD12B = 1; //Using 12-bit ADC
    
    AD1CON3bits.SAMC = 31; //auto sample time
    
    AD1CON4bits.ADDMAEN = 0; //No direct memory access (not required)
    
    AD1CON2bits.SMPI = 0b0000; //Interrupt after every sample and conversion

    //Clear ADC interrupt flag
    //Enable ADC interrupt
    //Enable peripheral interrupt (PEIE bit)
    //Enable global interrupt (GIE bit)
}

//Setup timer 1
void setupTimer1(){
    //Configure Timer
    T1CONbits.TCS = 0;
    T1CONbits.TGATE = 0;
    T1CONbits.TCKPS = 1;
    TMR1 = 0;
    //Disable Watchdog Timer
    FWDTEN_OFF;
}

//Setup DSPIC internal clock frequency
void setupCLK(){
    //Configure Clock
    PLLFBDbits.PLLDIV = 405;
    CLKDIVbits.PLLPRE = 8;
    CLKDIVbits.PLLPOST = 2;
}

//Setting up IO pins
void setupIO(){
    //Configuring Register Control Pins
    TRISAbits.TRISA1 = 0; //Digital output
    ANSELAbits.ANSA1 = 0; //Pin A1 is assigned to the SER pin on the register
    
    TRISBbits.TRISB3 = 0; //Digital output
    ANSELBbits.ANSB3 = 0; //Pin B3 is assigned to the SER pin on the register
    
    TRISBbits.TRISB0 = 0; //Digital output
    ANSELBbits.ANSB0 = 0; //Pin B0 is assigned to the OE' pin on the register
    
    TRISBbits.TRISB1 = 0; //Digital output
    ANSELBbits.ANSB1 = 0; //Pin B1 is assigned to the RCLK pin on the register
    
    TRISBbits.TRISB2 = 0; //Digital output
    ANSELBbits.ANSB2 = 0; //Pin B2 is assigned to the SRCLK pin on the register
    
    TRISBbits.TRISB15 = 1; //Pin B15 is configured as digital input for button
    TRISBbits.TRISB4 = 0;
    
    TRISBbits.TRISB14 = 0;
    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB12 = 0;
    
    TRISBbits.TRISB12 = 0; //Digital output for piezo speaker
    LATAbits.LATA4 = 0;
    
    LATBbits.LATB1 = 1;  
    
    //<<<<<   TI Handshake pin setup   >>>>>
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB3 = 0;
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    ODCBbits.ODCB2 = 1;  // Enable open-drain on RB0
    ODCBbits.ODCB3 = 1;  // Enable open-drain on RB1
    CNPUBbits.CNPUB2 = 1;  // Enable weak pull-up on RB0 (pin 11)
    CNPUBbits.CNPUB3 = 1;  // Enable weak pull-up on RB1 (pin 12)
    
    
}

//Write integer to shift register. Must have delay between function calls.
void regWrite(int input){
    LATBbits.LATB0 = 1;
    int outList[8];
    //Write to Register
    for(int j=0; j<8; j++){
        outList[j] = input >> j;
    }
    //print list to register
    for(int j=0; j<8; j++){
        LATBbits.LATB3 = outList[j];

        //Pulse SRCLK
        LATBbits.LATB2 = 1; 
        _delay(50,1);
        LATBbits.LATB2 = 0;

        //RESET SER
        LATBbits.LATB3 = 0;

        //Pulse RCLK
        LATBbits.LATB1 = 1;
        _delay(50,1); 
        LATBbits.LATB1 = 0;
    }
    LATBbits.LATB0 = 0;   
}

//Setting up interrupts for 
void setupINT(){
    IEC0bits.AD1IE = 1;     // Enable ADC interrupt
    IPC3bits.AD1IP = 0b111; //Highest priority
    INTCON1bits.NSTDIS = 1; // Enable interrupt nesting
    INTCON2bits.GIE = 1;    // Global Interrupt Enable
}

void setupUART(){
    // Set the baud rate (9600 baud for 40 MHz system clock)
    // Baud rate generator for 9600 baud (Fosc = 40MHz)

    // Set UART1 configuration
    U2MODEbits.UARTEN = 0; // Disable UART
    U2MODEbits.PDSEL = 0;  // 8-bit data, no parity
    U2MODEbits.STSEL = 0;  // 1 stop bit
    U2MODEbits.BRGH = 1;   // High baud rate (BRGH = 1)
    U2STAbits.UTXINV = 0;
    U2BRG = 968;
    U2MODEbits.UARTEN = 1;  // Enable UART1 module
    U2STAbits.UTXEN = 1;   // Enable UART transmitter
    //__builtin_write_OSCCONL(OSCCON & ~(1<<6));
    RPOR1bits.RP36R = 0b11; //Assigned to RP35
    //__builtin_write_OSCCONL(OSCCON | (1<<6));
}

void sendData(uint8_t data){
    while (U2STAbits.UTXBF);  // Wait until UART transmit buffer is not full
    _delay(30,30);
    U2TXREG = data;           // Write data to transmit register
}

void sendSound(int loc, int time){
    int freq = 24000 / (loc/3); 
    for(int i=0;i<loc*time;i++){
        LATBbits.LATB12 = 1;
        _delay((int)freq,1);
        LATBbits.LATB12 = 0;
        _delay((int)freq,1);
    }
}

//<<<<<   TI ASYNCHRONOUS SERIAL   >>>>>
uint16_t TI_Recieve(){
    //<<<<<   Local Variable Declaration   >>>>>
    uint8_t stopBit = 1; // Flag which flips if a stop bit is detected (both lines low)
    uint16_t message = 0; // Message recieved from other microcontroller
    uint8_t framingError = 0; // Flag to detect framing error or overrun
    //<<<<<   Timeout Timer Beginning   >>>>>
    
    //<<<<<   Protocal Begins   >>>>>
    while((PORTBbits.RB2 & PORTBbits.RB3) == 1){    // While both lines are high, wait for communication (waiting till RB0 is low)
    }
    // RB0 is low
    LATBbits.LATB3 = 0;                             // Responding to start with 'ready'  
    while((PORTBbits.RB2) == 0){                    // Waiting till RB0 is brought high
    }
    LATBbits.LATB3 = 1;                             // Releasing ready bit
    
    //<<<<<   Data Transfer Beginning   >>>>>
    while(stopBit & (framingError < 17)){               // Exit loop on stop bit or error
        while((PORTBbits.RB2 & PORTBbits.RB3) == 1){    // Waitin till a bit is sent
        }
        if((PORTBbits.RB2 & PORTBbits.RB3) == 0){       // A stop bit was sent, end transmission
            stopBit = 0;
            while((PORTBbits.RB2 | PORTBbits.RB3) == 0){// Waiting till both lines are high
            }
            LATBbits.LATB2 = 0;                         // Both are pulled low for "stop bit recieved"
            LATBbits.LATB3 = 0;
        }
        else if(PORTBbits.RB2 == 0){                    // A '0' was sent, a '0' is inserted to the right of lsb (msb first communication)
            message = (message << 1);

            LATBbits.LATB3 = 0;                         // Pulling RB1 down to signify 'recieved'
            while(PORTBbits.RB2 == 0){                  // Wait till RB0 is released
            }
            LATBbits.LATB3 = 1;                         // Releasing RB1
            // Bit Recieve Complete!
        }
        else{                                           // Otherwise a '1' was sent, message is left shifted and appended a '1'.
            message = (message << 1) + 1;
            LATBbits.LATB2 = 0;                         // Pulling RB0 down to signify 'recieved'
            while(PORTBbits.RB3 == 0){                  // Wait till RB1 is released
            }
            LATBbits.LATB2 = 1;                         // Releasing RB0
            // Bit Recieve Complete!
        }
        framingError ++;
    } 
    return(message);
}

void TI_Send(uint16_t message, uint8_t length){
    //<<<<<   Local Variable Declaration   >>>>>
    uint8_t bitCounter = 0;                     // Counter which keeps track of bits sent
    uint8_t bit = 0;                            // individual bit sent
    uint16_t mask = (1 << (length - 1));
    mask = 0b1000000000000000;
    //<<<<<   Protocal Begins   >>>>>
    //while((PORTBbits.RB2 & PORTBbits.RB3) != 1){ // Ensure both lines are high before continuing
    //}
    LATBbits.LATB2 = 0;                         // RB0 is pulled low to signify start bit
    while(PORTBbits.RB3 == 1){                  // Wait until ready is sent on RB1
    }                                           
    LATBbits.LATB2 = 1;                         // Releasing start bit
    while(PORTBbits.RB3 == 0){                  // Waiting till ready bit is released
    }
    
    //<<<<<   Data Transfer Begin   >>>>>
    for(bitCounter; bitCounter < length; bitCounter++){
        if((message & mask) > 0){
            LATBbits.LATB3 = 0;                 // Sending a '0'
            while(PORTBbits.RB2 == 1){          // Waiting for response
            }
            LATBbits.LATB3 = 1;                 // Releasing RB1
            while(PORTBbits.RB2 == 0){          // Waiting until other line is released
            }
        }
        else{
            LATBbits.LATB2 = 0;                 // Sending a '1'
            while(PORTBbits.RB3 == 1){          // Waiting for response
            }
            LATBbits.LATB2 = 1;                 // Releasing RB0
            while(PORTBbits.RB3 == 0){          // Waiting till other line is released
            }
        }
        message = message << 1;
    }
    LATBbits.LATB2 = 0;
    LATBbits.LATB3 = 0;
    _delay(300,3);
    LATBbits.LATB2 = 1;
    LATBbits.LATB3 = 1;
    while((PORTBbits.RB2 & PORTBbits.RB3) == 1){// Waiting till both lines are pulled low 
    }
    while((PORTBbits.RB2 & PORTBbits.RB3) == 0){// Waiting till both lines are pulled high
    }
    // Transmission over
}
//========================================


int main(void) {
    //Run setup functions
    setupCLK();
    setupTimer1();
    setupIO();
    setupADC();
    setupINT();
    setupUART();
    LATBbits.LATB2 = 1;
    LATBbits.LATB3 = 1;
    //Variable declaration
    fractcomplex twidFactors[FFT_SIZE/2] __attribute__((space(xmemory))); //Twiddle factor array stored in x memory
    fractional comSqMag[FFT_SIZE]; //Array for the magnitude of the FFT output
    uint16_t maxFreq = 0;
    
    //Start sound
    sendSound(28, 5);
    _delay(1000,100);
    sendSound(28, 5);
    _delay(1000,100);
    sendSound(35, 5);
    
    //Initialize twiddle factors 
    TwidFactorInit(8, &twidFactors[0], 0);
    while(1){
        //1- If pin B15 = 1, begin FFT and sampling process
        if(PORTBbits.RB15 == 0){ 
            //Begin Fast Fourier Transform Algorythm
            LATBbits.LATB1 = 0;
            LATAbits.LATA1 = 1; //turning on indicator LED
            counter = 0; //initializing counter before fft
            psamp = &sample[0]; //initializing pointer before fft
            
            //2- Begin sampling
            AD1CON1bits.ADON = 1; //Turn on ADC module
            

            while(counter != (FFT_SIZE - 1)){
                //wait here while sampling
            }
            
            _delay(10000,1000);
            counter = 0;
            //3- Verify sample does not hit 0b0000000000100000 or below(Clipping)
            for(int i=0;i<256;i++){
                if(sample[i].real < 0x20){
                   counter++; 
                }    
            }
            if(counter > 25){
                //Sample failed clipping test
                LATBbits.LATB13 = 1;
                _delay(100,10000);  //Playing clipping error sound
                sendSound(20, 5);
                _delay(1000,100);
                sendSound(20, 5);
                _delay(1000,100);
                LATBbits.LATB13 = 0;
                LATAbits.LATA1 = 0; //Turning off indicator, sampling is finished.
                LATBbits.LATB1 = 1;
                sendData(99); //Transmitting upper half of the dominant frequency
                _delay(30,300);
                sendData(99); //Transmitting lower half of the dominant frequency
            }
            else{
                //Sample passed clipping test

                //*****FFT Begins*****
                //5- Run FFT 
                FFTComplexIP(8, &sample[0], &twidFactors[0], 0xFF00); 

                //6- Perform bit reversal on the data
                BitReverseComplex(8, &sample[0]);

                //7- Determine the magnitude of each bin
                SquareMagnitudeCplx(256, &sample[0], &comSqMag[0]);

                //8- Determine the dominant frequency
                uint8_t loc;
                maxFreq = 0;
                for(int i=3; i<=127; i++){
                    if(comSqMag[i] > maxFreq){
                        maxFreq = comSqMag[i];
                        loc = i;
                    }
                }
                //sendData(loc); //Transmitting bin number
                //sendData((loc*31.25)/256); //Transmitting upper half of the dominant frequency
                //_delay(300,300);
                //sendData((loc*31.25)); //Transmitting lower half of the dominant frequency
                TI_Send(loc*31.25, 16);
                LATAbits.LATA1 = 0; //Turning off indicator, sampling is finished.
                sendSound(loc, 20); //Playing dominant frequency on piezo
                LATBbits.LATB1 = 1; //Turning on ready indicator LED
            }
        }
    }
    return(0);
}


void __attribute__((interrupt, auto_psv)) _AD1Interrupt(void)
{
    // Clear the interrupt flag
    LATAbits.LATA1 = 0;
    Nop();
    LATAbits.LATA1 = 1;
    if(counter == 255){
        //Turning off ADC
        AD1CON1bits.ADON = 0;     
        //Capturing last result
        psamp->real = ADC1BUF0;
        //Disabling ADC temporarily       
        AD1CON1bits.ADON = 0; //Turn off ADC module    
    }
    else{
        //Capturing result
        psamp->real = ADC1BUF0;
        //Moving pointer to next element of array (must increase by two to avoid fractcomplex structure)
        psamp++;
        //incrementing sample counter
        counter++;  
    }
    IFS0bits.AD1IF = 0;
}