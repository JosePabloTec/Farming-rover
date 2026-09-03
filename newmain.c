/* 
 * File:   newmain.c
 * Author: jpman
 *
 * Created on August 18, 2026, 10:31 AM
 */

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>


#define testLED RC7
#define _XTAL_FREQ 16000000

// LCD  port pins

#define RS RB2
#define RW RB1
#define EN RB0

#define switch1 RB3
#define switch2 RB4
#define switch3 RB5
#define switch4 RB6

// Commands for LCD display (taken from Act0 by Dr. Alejandro Aragón)
#define ClrScreen 0x01 // LCD clear display screen
#define ReturnHome 0x02 // LCD return home
#define DecCursor 0x04 // LCD decrement cursor (shift cursor to left)
#define IncCursor 0x06 // LCD increment cursor (shift cursor to right)
#define ShiftRight 0x05 // Shift display right
#define ShiftLeft 0x07 // Shift display left
#define DispOFFCurOFF 0x08 // Display OFF, cursor OFF
#define DispOFFCurON 0x0A // Display OFF, cursor ON
#define DispONCurOFF 0x0C // Display ON, cursor OFF
#define DispONCurBk 0x0E // Display ON cursor Blinking
#define DispOFFCurBk 0x0F // Display OFF cursor Blinking
#define ShiftCurLeft 0x10 // Shift cursor position to left
#define ShiftCurRight 0x14 // Shift cursor position to right
#define ShiftDispLeft 0x18 // Shift entire display to the left
#define ShiftDispRight 0x1C // Shift entire display to the right
#define FirstLine 0x80 // Cursor beginning of first line
#define SecondLine 0xC0 // Cursor beginning of second line
#define TwoLines57Mat 0x38 // Two lines, 5x7 matrix

// Constants (delays)
#define Delay_LCD 20 // Delay for LCD ENABLE pin
#define Delay_Shift 100 // Delay for char shifts in LCD
#define Delay_Auto 500 // Delay for automatic display
#define Delay_LED 200 // Delay for blinking LED


// PORT definitions
#define LCD_Port PORTD // LCD display connected to PORTD
#define LCD_Tris TRISD // LCD display I/O pin selection

void Init_Ports(void)
{
    TRISCbits.TRISC7 = 0; // The test LED is declared as an output 
    PORTCbits.RC7 = 0; // The test LED is initialized to LOW
    
    TRISBbits.TRISB0 = 0; // EN declared as output for LCD display
    TRISBbits.TRISB1 = 0; // RW declared as output for LCD display
    TRISBbits.TRISB2 = 0; // RS declared as output for LCD display
    
    LCD_Tris = 0; // All LCD pins declared as output    
    LCD_Port = 0; // Initialize display PORT buffer to 0
    PORTB = 0;    // Initialize PORTB to 0
    
    TRISAbits.TRISA0 = 1; // First analogue input
    TRISAbits.TRISA1 = 1; // Second analogue input
    
    TRISBbits.TRISB3 = 1; // Switch 1 input (automatic or manual)
    TRISBbits.TRISB4 = 1; // Switch 2 input (Temperature)
    TRISBbits.TRISB5 = 1; // Switch 3 input (Light intensity)
    
   
}


// The next function writes a command to the LCD display, written by te teacher
void Lcd_CmdWrite(unsigned char c){
    LCD_Port = c; // Place char ASCII in LCD data bus
    RS = 0; // To send commands to LCD: RS = 0
    RW = 0; // This one always to GND
    EN = 1; // Send data to LCD now
    __delay_ms(Delay_LCD); // Wait for line to stabilise
    EN = 0; // Ready, all sent
}

// The next function writes data to the LCD display, written by te teacher
void Lcd_DataWrite(unsigned char d){
    LCD_Port = d; // Place char ASCII in LCD data bus
    RS = 1; // To send data to LCD: RS = 1
    RW = 0; // This one always to GND
    EN = 1; // Send data to LCD now
    __delay_ms(Delay_LCD); // Wait for line to syabilise
    EN = 0; // Ready, all sent
}

// The next function uses the previous function to write a full text string to the LCD, written by te teacher
void Message_LCD(unsigned char *s){
    while(*s){
    Lcd_DataWrite(*s++);
    }
}

// The next function gest the LCD set up, written by the teacher
void Init_LCD(){
    Lcd_CmdWrite(TwoLines57Mat);
    Lcd_CmdWrite(DispONCurOFF);
    Lcd_CmdWrite(ClrScreen);
    Lcd_CmdWrite(FirstLine);
}


void Init_ADC_Module(){
    ADFM = 1;
    CHS3 = 0;
    CHS2 = 0;
    CHS1 = 0;
    CHS0 = 0;  // start with channel A0
    VCFG1 = 0;
    VCFG0 = 0;
    PCFG3 = 1;
    PCFG2 = 1;
    PCFG1 = 1;
    PCFG0 = 0;
    ADCS2 = 0;
    ADCS1 = 1;
    ADCS0 = 0;
    ADON = 1;
}

// The enxt function read as an analogue input
void Read_Analogue_Input(bool is_A1) {
    char data[8]; // // data can store 6 characters, we don't need more cause we are only displaying one-digit numbers with three decimal places
    float ADC_read;
    Lcd_CmdWrite(FirstLine);
    
    if (is_A1){
      Message_LCD("Voltage 2: ");   
    }
    
    else{
      Message_LCD("Voltage 1: ");  
    }
    
    GO_DONE = 1;
    while(GO_DONE) {
    }
    ADC_read = ((ADRESH*256.0+ADRESL) * (5/1023.0));
    sprintf(data,"%.3f",ADC_read);
    Lcd_CmdWrite(SecondLine);
    Message_LCD(data);
    Message_LCD(" V");
}

void send_error(){
    Lcd_CmdWrite(FirstLine);
    Message_LCD("Error, select");
    Lcd_CmdWrite(SecondLine);
    Message_LCD("a valid config");
}

void send_manual(){
    Lcd_CmdWrite(FirstLine);
    Message_LCD("Manual mode");
    Lcd_CmdWrite(SecondLine);
    Message_LCD("Select a config");
}


// the next function reads the state of the dip switch and generates a 4 bit binary number
unsigned char read_dip_switch(void) {
    unsigned char state;
    state = (switch1 << 3) | (switch2 << 2) | (switch3 << 1) | (switch4 << 0);
    return state;
}


// change to channel AN0
void Set_AN0_Channel(void){
    CHS3 = 0;
    CHS2 = 0;
    CHS1 = 0;
    CHS0 = 0;
}

// change to the other channel
void Set_AN1_Channel(void){
    CHS3 = 0;
    CHS2 = 0;
    CHS1 = 0;
    CHS0 = 1;
}


void Init_sequence(void)
{
  testLED = 1;
  __delay_ms(200);
  testLED = 0;
  __delay_ms(200);
  testLED = 1;
  __delay_ms(200); // wait 200 milliseconds
  testLED = 0;
  __delay_ms(200);
  testLED = 1;
  __delay_ms(200);
  testLED = 0;
  __delay_ms(200);
  testLED = 1;
  Lcd_CmdWrite(ClrScreen);
  Lcd_CmdWrite(FirstLine);
  Message_LCD("All goodie");
  Lcd_CmdWrite(SecondLine);
  Message_LCD("in the hoodie");
  __delay_ms(1000);
  Lcd_CmdWrite(ClrScreen);
}

void main(void) {
    Init_Ports();
    Init_LCD();
    Init_sequence();
    Init_ADC_Module();
    Set_AN0_Channel();
    unsigned char state;
    
    while(1) {
    state = read_dip_switch();
    
        if (state == 0b1000) {       // 1000 automatic mode
            Set_AN0_Channel();
            Read_Analogue_Input(0);
            __delay_ms(500);
            Set_AN1_Channel();
            Read_Analogue_Input(1);
            __delay_ms(500);
        }

        else if (state == 0b1100) {  // 1100 manual mode selected
            send_manual();
            __delay_ms(750);
            Lcd_CmdWrite(ClrScreen);
        }

        else if (state == 0b1110) {  // 1110 sensor 1 (manual))
            Set_AN0_Channel();
            Read_Analogue_Input(0);
            __delay_ms(500);
        }    
    
        else if (state == 0b1111) {  // 1111 sensor 2 (manual))
            Set_AN1_Channel();
            Read_Analogue_Input(1);
            __delay_ms(500);
        }   
        
        else{    
            send_error();
            __delay_ms(750);
            Lcd_CmdWrite(ClrScreen);
        }
    }
}
