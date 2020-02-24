/*=============================================================================
 * Program:     Project 5 (Mid-Project)
 * File Name:   41_Proj5.c
 * Authors:     Michael Rawding and Andrew Riegner
 * Team Name:   Robo Cops
 * Team Number: 41       
 * Date:        12/1/2017
 * Description:     
 * This program causes the robot to drive in different directions by sending different
 * PWM signals to the Servo motors depending on which switches are pressed
==============================================================================*/
#ifndef _SUPPRESS_PLIB_WARNING
#define _SUPPRESS_PLIB_WARNING
#endif

//include the relevant library files used in the project
#include <math.h>
#include <plib.h>
#include <stdlib.h>
#include <stdio.h>
#include <xc.h>
#include "adc.h"
#include "btn.h"
#include "config.h"
#include "lcd.h"
#include "led.h"
#include "mic.h"
#include "pmods.h"
#include "srv.h"
#include "ssd.h"
#include "swt.h"
#include "utils.h"

//set the system clock to 80MHz frequency and peripheral bus clock frequency to 40MHz
#pragma config FNOSC = PRIPLL //initializes phase-locked loop
#pragma config POSCMOD = XT //sets primary oscillator mode
#pragma config FPLLIDIV = DIV_2 //sets PLL input divider to divide by 2
#pragma config FPLLMUL = MUL_20 //sets PLL multiplier to multiply by 20
#pragma config FPLLODIV = DIV_1 //sets PLL output divider to divide by 1
#pragma config FPBDIV = DIV_1 //sets peripheral bus clock divider to divide by 2

#define SYS_FREQ (80000000L) //define frequency of system clock as a variable (80MHz)
#define CORE_TICK_RATE (SYS_FREQ/2/1000) //define tick rate of core timer as a variable (cylces per ms)

//define global variable for each LED
#define led0 LATAbits.LATA0
#define led1 LATAbits.LATA1
#define led2 LATAbits.LATA2
#define led3 LATAbits.LATA3
#define led4 LATAbits.LATA4
#define led5 LATAbits.LATA5
#define led6 LATAbits.LATA6
#define led7 LATAbits.LATA7

enum MODE {FORWARD, BACKWARD, STOP}; //different possible modes for each of the motors
enum BOARD {TEST, READY, GO, FINISH}; //different possible modes that the board can be in
int board_mode = TEST; //initial mode should be READY
int left_mode = STOP; //left motor should initially be in stopped mode
int right_mode = STOP; //right motor should initially be in stopped mode
float time = 0.0; //for timer on SSD; initialize to 0
float time2 = 0.0; //for timer to determine claps
unsigned short int count = 0; //counter for claps

//set up global variables for IR values
unsigned char ll; 
unsigned char lm;
unsigned char rm;
unsigned char rr;

unsigned char sensor_vals = 0b11111111; //global variable that will be a 4-bit binary representation of all IR values for case testing


//forward declarations
void update_IR();

/* ------------------------------------------------------------ 
**    driveMotors()
**    Parameters:
**        none
**    Return Value:
**        none
**    Description:
**        based on the current state of all 4 IR sensors, either stops, goes forward, goes backward, or turns
** ------------------------------------------------------------ */
void driveMotors(){
    //2 is left, 1 is right
    //2000 for clockwise, 3475 for counterclockwise
    int begin_time = time;
    //if/else statements on different relevant IR states
        if(ll == 0 && lm == 0 && rm == 0 && rr == 0 && time < 1.0){//if all sensors are on the track and it's the start, go forward
            SRV_SetPulseMicroseconds1(2000);
            SRV_SetPulseMicroseconds2(3475);
            right_mode = FORWARD;
            left_mode = FORWARD;
        }else if(ll == 0 && lm == 0 && rm == 0 && rr == 0 && time > 1.0){ //if all sensors are on the track and it's not the start, stop
            while((time - begin_time) < 0.5){
                SRV_SetPulseMicroseconds1(2000);
                SRV_SetPulseMicroseconds2(3475);
                right_mode = FORWARD;
                left_mode = FORWARD;
            }
            update_IR();
            if(ll == 0 && lm == 0 && rm == 0 && rr == 0){
                SRV_SetPulseMicroseconds1(0);
                SRV_SetPulseMicroseconds2(0);
                board_mode = FINISH;
            }
        }else if(ll == 0 && lm == 0 && rm == 0 && rr == 1){ //if the right-most if off the track, turn left until you're at the right state
            while(sensor_vals != 1001){
                SRV_SetPulseMicroseconds1(2000);
                SRV_SetPulseMicroseconds2(2000);
                right_mode = FORWARD;
                left_mode = BACKWARD;
                update_IR();
            }
        }else if(ll == 0 && lm == 0 && rm == 1 && rr == 1){ //if the two right sensors are off the track, turn left until you're at the rights state
            while(sensor_vals != 1001){
                SRV_SetPulseMicroseconds1(2000);
                SRV_SetPulseMicroseconds2(2000);
                right_mode = FORWARD;
                left_mode = BACKWARD;
                update_IR();
            }
        }else if(ll == 0 && lm == 1 && rm == 1 && rr == 0){ //if the three right sensors are off the track, turn left until you're at the right state
            while(sensor_vals != 1001){
                SRV_SetPulseMicroseconds1(2000);
                SRV_SetPulseMicroseconds2(2000);
                right_mode = FORWARD;
                left_mode = BACKWARD;
                update_IR();
            }
        }else if(ll == 1 && lm == 0 && rm == 0 && rr == 0){ //if the left sensor is off the track, turn right until you're at the right state
            while(sensor_vals != 1001){
                SRV_SetPulseMicroseconds1(3475);
                SRV_SetPulseMicroseconds2(3475);
                right_mode = BACKWARD;
                left_mode = FORWARD;
                update_IR();
            }
        }else if(ll == 1 && lm == 0 && rm == 0 && rr == 1){ //if the two middle sensors are on the track and the outside ones are off, go forward
            SRV_SetPulseMicroseconds1(2000);
            SRV_SetPulseMicroseconds2(3475);
            right_mode = FORWARD;
            left_mode = BACKWARD;
        }else if(ll == 1 && lm == 1 && rm == 0 && rr == 0){ //if the two left sensors are off the track, turn right until you're at the right state
            while(sensor_vals != 1001){
                SRV_SetPulseMicroseconds1(3475);
                SRV_SetPulseMicroseconds2(3475);
                right_mode = BACKWARD;
                left_mode = FORWARD;
                update_IR();
            }
        }else if(ll == 1 && lm == 1 && rm == 1 && rr == 0){ //if the three left sensors are off the track, turn right until you're at the right state
            while(sensor_vals != 1001){
                SRV_SetPulseMicroseconds1(3475);
                SRV_SetPulseMicroseconds2(3475);
                right_mode = BACKWARD;
                left_mode = FORWARD;
                update_IR();
            }
        }else if(ll == 1 && lm == 1 && rm ==1 && rr == 1){ //if all four sensors are off the track, go backwards until this is not the case
            while(sensor_vals != 1001){
                SRV_SetPulseMicroseconds1(3475);
                SRV_SetPulseMicroseconds2(2000);
                right_mode = BACKWARD;
                left_mode = BACKWARD;
                update_IR();
            }
        }
}

/* ------------------------------------------------------------ 
**    writeToSSD(float num)
**    Parameters:
**        num - the decimal number represented by the switches/LEDs
**    Return Value:
**        none
**    Description:
**        uses the float passed to it to write that number to the SSD
** ------------------------------------------------------------ */
void writeToSSD(float num){
    // set up the core timer interrupt for timing LED period with a priority of 5 and zero sub-priority
    OpenCoreTimer(CORE_TICK_RATE); //CoreTimer used for tenths of second capture  
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_5 | CT_INT_SUB_PRIOR_0)); 
    INTEnableSystemMultiVectoredInt();
    
    //creates four variables for the four possible digits and initializes them to 0
    int first_digit = 0;
    int second_digit = 0;
    int third_digit = 0;
    int fourth_digit = 0;
    
    //multiplies the float by 10 and casts it to an int for easier math while
    //maintaining the same digits
    int copy = (int) (num * 10);
    
    //if copy < 100, then num < 100, so we know the first digit is 0
    //these lines deal with this case, and find all 4 digits accordingly
    if(copy < 1000){
        first_digit = 0;
        third_digit = (copy % 100) / 10; 
        second_digit = copy - (third_digit * 10);
        second_digit /= 100;
        fourth_digit = copy % 10;
    //these lines deal with the case that copy is between 1000 and 2000
    }else if(copy >= 1000 && copy < 2000){
        first_digit = 1;
        copy -= 1000;
        third_digit = (copy % 100) / 10;
        copy += 1000;
        second_digit = (copy % 1000) - (third_digit * 10);
        second_digit /= 100;
        fourth_digit = copy % 10;
    //these lines deal with the case that copy is between 2000 and 2550, the max value
    }else{
        first_digit = 2;
        copy -= 2000;
        third_digit = (copy % 100) / 10;
        second_digit = copy - (third_digit * 10);
        second_digit /= 100;
        fourth_digit = copy % 10;
    }
   
    //creates 4 characters to represent each of the digits    
    unsigned char a;
    unsigned char b;
    unsigned char c;
    unsigned char d;
    
    if(first_digit == 0){
        a = 0x3f;
    }else{
        a = (char)first_digit;
    }
    if(first_digit == 0 && second_digit == 0){
        a = b = 0x3f;
    }else{
        b = (char)second_digit;
    }
    c = (char)third_digit;
    d = (char)fourth_digit;
    SSD_WriteDigits(d,c,b,a,0,1,0,0);//writes the four digits along with the one decimal point to the SSD
}

/* ------------------------------------------------------------ 
**    writeLEDs(int m)
**    Parameters:
**        int m - value to display on the LEDs
**    Return Value:
**        none
**    Description:
**        takes in the maximum value of the sample buffer, then turns on a certain
 *        number of LEDs depending on what the max value was
** ------------------------------------------------------------ */
void writeLEDs(){
    //the reason we only start turning on LEDs if the value exceeds 510 is to counteract
    //the offset value given when MIC_Val() is 0
    
    int m = MIC_Val();
    
            if(m ==0){//if the max value is between 0 and 510
                LATA = 0x00;//turn on no LEDs
            }
            else if(m > 0  && m <= 128){//if the max value is between 510 and 574
                LATA = 0x01;//turn on only LED0
            }
            else if(m > 128 && m <= 256){//if the max value is between 574 and 638, turn on LED0-LED1
                LATA = 0x03; 
            }
            else if(m > 256 && m <= 384){//if the max value is between 638 and 702, turn on LED0-LED2
                LATA = 0x07;  
            }
            else if(m > 384 && m <= 512){//if the max value is between 702 and 766, turn on LED0-LED3
                LATA = 0x0F;  
            }
            else if(m > 512 && m <= 640){//if the max value is between 766 and 830, turn on LED0-LED4
                LATA = 0x1F;  
            }
            else if(m > 640 && m <= 768){//if the max value is between 830 and 894, turn on LED0-LED5
                LATA = 0x3F;  
            }
            else if(m > 768 && m <= 896){//if the max value is between 894 and 958, turn on LED0-LED6
                LATA = 0x7F;  
            }
            else{//if the max value is between 958 and 1023, turn on all LEDs
                LATA = 0xFF; 
            }
}


/* ------------------------------------------------------------ 
**    display()
**    Parameters:
**        none
**    Return Value:
**        none
**    Description:
**        writes to bottom line of LCD which mode the board is in
** ------------------------------------------------------------ */
void display(){
    switch(board_mode){
        case TEST:
            LCD_WriteStringAtPos("MODE: TEST            ",1,0);
            writeLEDs();
            break;
        case READY:
            LCD_WriteStringAtPos("MODE: READY            ",1,0);
            LATA = 0x00;
            break;
        case GO:
            LCD_WriteStringAtPos("MODE: GO               ",1,0);
            LATA = 0x00;
            break;
        case FINISH:
            LCD_WriteStringAtPos("FINISHED!              ",1,0);
            break;
    } 
}


/* ------------------------------------------------------------ 
**    delay_ms(int ms)
**    Parameters:
**        int ms - the number of ms to delay
**    Return Value:
**        none
**    Description:
**        uses software loop to delay 1ms per argument
** ------------------------------------------------------------ */
void delay_ms(int ms){
    int i = 0, j = 0;
    for(i; i < ms; i++){
        for(j; j < 1426; j++){}
    }
}


/* ------------------------------------------------------------ 
**    update_IR()
**    Parameters:
**        none
**    Return Value:
**        none
**    Description:
**        reads values from all IR sensors and combines into a single integer value for case testing
** ------------------------------------------------------------ */
void update_IR(){
    ll = PMODS_GetValue(0,1); //reads left-most IR sensor
    lm = PMODS_GetValue(0,2); //reads left-middle IR sensor
    rm = PMODS_GetValue(0,3); //reads right-middle IR sensor
    rr = PMODS_GetValue(0,4); //reads right-most IR sensor
    sensor_vals = (ll << 3 + lm << 2 + rm << 1 + rr); //shifts and adds these values to get a single 4-bit binary representation of IR state, with left-most as MSB and right-most as LSB
}

int main(void){
    
    DDPCONbits.JTAGEN = 0; //required to use pin RA0 as I/O
    
    ADC_Init(); //initializes ADC module
    BTN_Init(); //initializes BTN module
    LCD_Init(); //initializes LCD module
    LED_Init(); //initializes LED module
    
    //initialize 4 PMOD pins for IR sensors, from PMODA, as inputs, and with  no pull-up or pull-down
    PMODS_InitPin(0,1,1,0,0); //initializes first PMOD pin
    PMODS_InitPin(0,2,1,0,0); //initializes second PMOD pin
    PMODS_InitPin(0,3,1,0,0); //initializes third PMOD pin
    PMODS_InitPin(0,4,1,0,0); //initializes fourth PMOD pin
    
    MIC_Init(); //initializes MIC module
    SRV_Init(); //initializes SRV module
    SSD_Init(); //initializes SSD module
    SWT_Init(); //initializes SWT module
    
    LCD_WriteStringAtPos("TEAM: 41          ",0,0);
    
    OpenCoreTimer(CORE_TICK_RATE); //opens Core Timer on previously defined tick rate
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_5 | CT_INT_SUB_PRIOR_0)); //configures Core Timer interrupts and sets priority 5 and sub-priority 0
    INTEnableSystemMultiVectoredInt(); //enables multi-vectored interrupts
    
    
    while(1){
        
        int i;
        
        if(board_mode == READY){
            int first_check = MIC_Val();
            if(first_check > 900){
                count = 1;
            }
            if(count && time2 <= 1.0){
                int second_check = MIC_Val();
                if(second_check > 900){
                    board_mode = GO;
                }
            }
            if(time2 > 1.0){
                count = 0;
                time2 = 0.0;
            }
        }
        
        if(board_mode == READY){
            if(BTN_GetValue('U')){
                delay_ms(100);
                while(BTN_GetValue('U')){}
                board_mode = GO;
            }
        }
        
        if(board_mode == TEST){
            if(BTN_GetValue('C')){
                delay_ms(100);
                while(BTN_GetValue('C')){}
                board_mode = READY;
            }
        }
        
        if(board_mode == GO){
            update_IR(); //executes function which reads the state of the IR sensors
            driveMotors(); //executes function to drive servos with PWM
        }
        display(); //displays the appropriate string on the bottom row of the LCD
    }
    
}

void __ISR(_CORE_TIMER_VECTOR,IPL5SOFT) CoreTimerHandler(void){
    mCTClearIntFlag(); //clears interrupt flag
    time += 0.1;
    if(left_mode == STOP && right_mode == STOP){
        time = 0.0;
    }
    if(count){
        time2 += 0.1;
    }
    writeToSSD(time);
    UpdateCoreTimer(CORE_TICK_RATE * 100);
}