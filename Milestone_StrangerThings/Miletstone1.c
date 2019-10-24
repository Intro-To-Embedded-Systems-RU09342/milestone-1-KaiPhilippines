/*
 * Milestone 1: Stranger Things Light Wall
 *
 * MSP430G2553
 *
 * Kyle Limbaga
 * Rowan University
 * Date Created: October 11, 2019
 * Date Updated: October 16, 2019
 *
 *  TI Resources for UARTsetup: msp430g2xx3_uscia0_uart_01_9600.c
 *
 */

#include <msp430.h>

void LEDsetup(){
    /* Port Layout
        P1.6 = Red LED
        P2.1 = Green LED
        P2.5 = Blue LED */

    //Red LED-setup
    P1SEL |= BIT6;  //Enables PWM to P1.6
    P1DIR |= BIT6;  //Set output direction

    //Blue LED-setup
    P2SEL |= BIT5;  //Enables PWM to P2.4
    P2DIR |= BIT5;  //Set output Direction

    //Green LED-setup
    P2SEL |= BIT1;  //Enables PWM to P2.1
    P2DIR |= BIT1;  //Set output Direction
}

// Two Timers because A0 has one compare and control register
void TIMERsetup(){
    //TimerA0 setup for RED LED
    TA0CTL |= TASSEL_2 + MC_1;  // Timer_A0 control = timer clock source select(smclk) + timer module mode control(up mode)
    TA0CCTL1 |= OUTMOD_7;   //Capture and Compare control = Timer_A can perform actions to specific output pins automatically in
                            //hardware (no ISR required). This field sets the desired action (Set/Reset)

    TA0CCR0 = 255;          //Compare and control register 0 = PWM period(max value)
    TA0CCR1 = 0;            //Compare and control register 1 = Initializes RED PWM

    //TimerA1 setup for GREEN and BLUE LED
    TA1CTL |= TASSEL_2 + MC_1;     // Timer_A1 control = timer clock source select(smclk) + timer module mode control(up mode)
    TA1CCTL1 |= OUTMOD_7;   //resets output at CCR0, sets output at CCR1
    TA1CCTL2 |= OUTMOD_7;   //resets output at CCR0, sets output at CCR2
    TA1CCR0 = 255;  //Compare and control register 0 = PWM period(max value)
    TA1CCR1 = 0;    //Compare and control register 1 = Initializes GREEN PWM
    TA1CCR2 = 0;    //Compare and control register 2 = Initializes BLUE PWM

}

void UARTsetup(){   // taken most from Resource explorer uscia0_uart_01_9600
    P1SEL |= BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD WARNING change to |=
    P1SEL2 |= BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD WARNING change to |=
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 104;                            // 1MHz divided by 9600 = 104
    UCA0BR1 = 0;                              // 1MHz 9600
    UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt

}
int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;   // Stop WDT

    LEDsetup(); //Initialize LED setup function
    TIMERsetup();   //Initialize Timer setup function
    UARTsetup();    //Initialize UART setup function

    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled

}


volatile int size = 0;  //For the size of the message
volatile int count = 1; //location of the current byte in transmission
int RED = 0;    //Initializes for RED variable
int GREEN = 0;  //Initializes for GREEN variable
int BLUE = 0;   //Initializes for BLUE variable
//This will run when byte is received
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    while (!(IFG2&UCA0TXIFG));  //Is USCI_A0 TX buffer ready?
    if (count == 1){
        UCA0TXBUF = UCA0RXBUF -3;  //Outputs the length of the response message
        size = UCA0RXBUF - 3; //Set size of the input signal - 3. Need it to check if count and size are the same so to reset
     }
     else if(count<5)   //The count starts at 1, so the count will go to the switches for 2, 3, and 4 counts
     {
         switch(count)  //Switch statements will go to 2,3,4
         {
         case 2:    //2nd byte signal
             RED = UCA0RXBUF;   //Set byte signal = RED pwm
             break;
         case 3:    //3rd byte signal
             GREEN = UCA0RXBUF;  //Set byte signal = GREEN pwm
             break;
         case 4:    //4th byte signal
             BLUE = UCA0RXBUF;   //Set byte signal = BLUE pwm
             break;
         default:
             break;
         }

     }
     else   //If count is not at 1 or any of the RGB values, output the Byte. Sends the Stop Byte or the other bytes to send to another byte
       UCA0TXBUF = UCA0RXBUF;
     if(count == size + 3)  //This is used if board is receiving RGB values from another board
     {
         count = 0; //Resets counter to zero
         TA0CCR1 = RED; //Set RED variable to the Timer register for RED
         TA1CCR1 = GREEN;   //Set GREEN variable to the Timer register for green
         TA1CCR2 = BLUE;    //Set BLUE variable to the Timer register for BLUE
     }

     count += 1;    //Increment count by 1
   }

