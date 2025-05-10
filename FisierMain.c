#include <msp430.h>
#include <stdint.h>
#include <math.h>
#include <stddef.h>
#include "dsp_filters.h"
#include "Project_Generic_functions.h"
#include "Variabile_Globale.h"

#define Intrerupere_P21 (P2IFG & BIT1)
#define Intrerupere_P22 (P2IFG & BIT2)
#define Intrerupere_P23 (P2IFG & BIT3)
#define Intrerupere_P24 (P2IFG & BIT4)
#define Intrerupere_P25 (P2IFG & BIT5)
#define Intrerupere_P26 (P2IFG & BIT6)
#define Intrerupere_P27 (P2IFG & BIT7)

#define Intrerupere_P41 (P4IFG & BIT1)
#define Intrerupere_P42 (P4IFG & BIT2)
#define Intrerupere_P43 (P4IFG & BIT3)
#define Intrerupere_P44 (P4IFG & BIT4)
#define Intrerupere_P45 (P4IFG & BIT5)
#define Intrerupere_P46 (P4IFG & BIT6)
#define Intrerupere_P47 (P4IFG & BIT7)



/********************* FUNCTIA MAIN ******************************/

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // 1.0 Frequency configure
    // operation at 24MHz(beyond 8MHz) _before_ configuring the clock system.

    FRCTL0 = FRCTLPW | NWAITS_2;

    __bis_SR_register(SCG0);
    CSCTL3 |= SELREF__REFOCLK;
    CSCTL0 = 0;
    CSCTL1 &= ~(0x000E);
    CSCTL1 |= DCORSEL_5;
    CSCTL2 = FLLD_0 + 487;
    __delay_cycles(3);
    __bic_SR_register(SCG0);
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;

    CSCTL5 |= DIVM_0 | DIVS_1;                                                   // default DCOCLKDIV as MCLK and SMCLK source

    // Configure SMCLK MCLK pins
    P3DIR |= BIT0 | BIT4;                       // set ACLK SMCLK and LED pin as output
    P3SEL0 |= BIT0 | BIT4;                             // set ACLK and  SMCLK pin as second function

    /** UART **/

    // Configure UART pins



    P4SEL0 |= BIT2 | BIT3;                     // P4.2 = TXD, P4.3 = RXD

    UCA1CTLW0 |= UCSWRST;                      // Hold USCI in reset
    UCA1CTLW0 |= UCSSEL__SMCLK;               // SMCLK = 8 MHz

    UCA1BR0 = 52;                              // Baud rate = 9600 @ 8 MHz SMCLK
    UCA1BR1 = 0;
    UCA1MCTLW = (0x55 << 8) | (5 << 4) | UCOS16; // UCBRSx=0x55, UCBRFx=5, UCOS16=1

    UCA1CTLW0 &= ~UCSWRST;                     // Release USCI from reset
    UCA1IE |= UCRXIE;                          // Enable RX interrupt
    clearTerminalAndDisplayMessages();




    // PIN INTERRUPT P2.3
    // P2.3 ->
    P2DIR &= ~BIT3; // Configuram P2.3 ca intrare
    P2OUT |= BIT3;                          // Configure P2.3 as pulled-up
    P2REN |= BIT3;                          // P2.3 pull-up register enable
    P2IES |= BIT3;                          // P2.3 Hi/Low edge
    P2IE |= BIT3;                           // P2.3 interrupt enabled
    // stergem indicatorul de intreruperi de la P2.3
    P2IFG &= ~BIT3;                         // P2.3 IFG cleared

    // PIN INTERRUPT P2.2
    // P2.2 ->
    P2DIR &= ~BIT2; // Configuram P2.3 ca intrare
    P2OUT |= BIT2;                          // Configure P2.2 as pulled-up
    P2REN |= BIT2;                          // P2.2 pull-up register enable
    P2IES |= BIT2;                          // P2.2 Hi/Low edge
    P2IE |= BIT2;                           // P2.2 interrupt enabled
    // stergem indicatorul de intreruperi de la P2.2
    P2IFG &= ~BIT2;                         // P2.2 IFG cleared

    // PIN INTERRUPT P4.1
    // P4.1 ->
    P4DIR &= ~BIT1; // Configuram P4.1 ca intrare
    P4OUT |= BIT1;                          // Configure P4.1 as pulled-up
    P4REN |= BIT1;                          // P4.1 pull-up register enable
    P4IES |= BIT1;                          // P4.1 Hi/Low edge
    P4IE |= BIT1;                           // P4.1 interrupt enabled
    // stergem indicatorul de intreruperi de la P4.1
    P4IFG &= ~BIT1;                         // P4.1 IFG cleared

    P6DIR |=BIT6; // P6.6 Digital out
    P6OUT = BIT6; // P6.6 -> high

    // ADC
    // Configuram Timer B0 -> Fes=SMCLK/TB0CCR0
    TB0CCTL0 |= CCIE;                                             // TBCCR0 interrupt enabled
    TB0CCR0 = 4705;
    TB0CTL = TBSSEL__SMCLK | MC__UP;                               // ACLK, UP mode

    // PIN ADC8
    P5SEL0 |= BIT0 ;
    P5SEL1 |= BIT0 ;

     // Configure ADC - Pulse sample mode; ADCSC trigger
    ADCCTL0 |= ADCSHT_8 | ADCON;                                  // ADC ON,temperature sample period>30us
    ADCCTL1 |= ADCSHP;                                            // s/w trig, single ch/conv, MODOSC
    ADCCTL2 &= ~ADCRES;                                           // clear ADCRES in ADCCTL
    ADCCTL2 |= ADCRES_2;                                          // 12-bit conversion results
    ADCMCTL0 |= ADCSREF_1 | ADCINCH_8;
    ADCIE |= ADCIE0;                                               // Enable the Interrupt request for a completed ADC_B conversion

    // Configure reference
    PMMCTL0_H = PMMPW_H;                                          // Unlock the PMM registers
    PMMCTL2 |= INTREFEN | REFVSEL_1;                          // Vref_PMM = 2V
    while(!(PMMCTL2 & REFGENRDY));                            // Poll till internal reference settles

    // DAC P1.5
    P1SEL0 |= BIT5;
    P1SEL1 |= BIT5;

    // DAC P3.1
    // Configurează pinul P3.1 ca DACOUT (funcție alternativă)
    P3SEL0 |= BIT1;
    P3SEL1 &= ~BIT1;

    SAC1DAC = DACSREF_1 + DACLSEL_0 + DACIE;
    SAC1DAT = Sinus[0];
    SAC1DAC |= DACEN;

    SAC1OA = NMUXEN + PMUXEN + PSEL_1 + NSEL_1;
    SAC1OA |= OAPM;
    SAC1PGA = MSEL_1;
    SAC1OA |= SACEN + OAEN;

    SAC2DAC = DACSREF_1 + DACLSEL_0 + DACIE;
    SAC2DAT = 1000;
    SAC2DAC |= DACEN;

    SAC2OA = NMUXEN + PMUXEN + PSEL_1 + NSEL_1;
    SAC2OA |= OAPM;
    SAC2PGA = MSEL_1;
    SAC2OA |= SACEN + OAEN;
    TB1CCTL0 |= CCIE;


    TB1CCR0 = 120;
    TB1CTL = TBSSEL__SMCLK | MC_1 | TBCLR;

    __bis_SR_register(GIE); // global interrupts



    PM5CTL0 &= ~LOCKLPM5;                              // Disable the GPIO power-on default high-impedance mode
                                                       // to activate previously configured port settings
}
volatile uint16_t Valoare = 80;
volatile uint16_t ValoareTastatura = 1;

//  RUTINA DE TRATARE A INTRERUPERILOR - MSP -> PC
//  BAUD RATE 9600
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;

    case USCI_UART_UCRXIFG:
    {
      char received = UCA1RXBUF;

      // Echo back character
      while (!(UCA1IFG & UCTXIFG));
      UCA1TXBUF = received;

      if (received == '\r')  // Enter
      {

        uart_buffer[uart_index] = '\0';

        ValoareTastatura = atoi(uart_buffer);  // convertim toată valoarea
        Valoare = (uint16_t)((8000000.0f / ( ValoareTastatura * 100.0f ))-1);;

        // opțional: copiem în command
        for (i = 0; i <= uart_index; i++) {
          command[i] = uart_buffer[i];
        }



        // Trimite cifra cu cifra
        for (i = 0; i < uart_index; i++) {
            while (!(UCA1IFG & UCTXIFG));  // Așteaptă să fie liber bufferul de transmisie
            UCA1TXBUF = uart_buffer[i];    // Trimite caracterul curent din uart_buffer
        }


        // trimite " Hz"
        while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = ' ';
        while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 'H';
        while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 'z';

        // trimitem newline
        while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';
        while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';

        uart_index = 0;
      }
      else if (uart_index < BUFFER_SIZE - 1)
      {
        uart_buffer[uart_index++] = received;
      }

      break;
    }

    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    default: break;
  }
}

// Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{

    if (Intrerupere_P22){
           P2IFG &= ~BIT2;                     // Clear P2.2 IFG
           count++;
    }

    if (Intrerupere_P23){
        P2IFG &= ~BIT3;                     // Clear P2.3 IFG

        switch(TipSemnal){

        case 0:
            // Trimite comanda de escape pentru a curăța terminalul
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 0x1B;  // Escape character (\033)
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '[';    // [
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '2';    // 2
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 'J';    // J

            // Prima linie: "Tip semnal: Sinus"
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
            u32TrimiteMesaj(" Tip semnal: Sinus");
            TipSemnal = 1;
            break;
        case 1:
            // Trimite comanda de escape pentru a curăța terminalul
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 0x1B;  // Escape character (\033)
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '[';    // [
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '2';    // 2
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 'J';    // J

            // Prima linie: "Tip semnal: Dinte Fierastrau"
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
            u32TrimiteMesaj(" Tip semnal: Dinte Fierastrau");
            TipSemnal = 2;
            break;
        case 2:
            // Trimite comanda de escape pentru a curăța terminalul
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 0x1B;  // Escape character (\033)
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '[';    // [
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '2';    // 2
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 'J';    // J

            // Prima linie: "Tip semnal: Rampa"
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
            u32TrimiteMesaj(" Tip semnal: Rampa");
            TipSemnal = 0;
            break;
        default:
            break;
    }
    }

}

// Port 2 interrupt service routine
#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
{
    if (Intrerupere_P41){
    P4IFG &= ~BIT1;

    switch(FrecventaSemnal){
        case 0:
            // Trimite comanda de escape pentru a curăța terminalul
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 0x1B;  // Escape character (\033)
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '[';    // [
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '2';    // 2
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 'J';    // J

            // Prima linie: "Setat automat la 80Hz"
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
            u32TrimiteMesaj(" Setat automat la 80Hz");
            FrecventaSemnal = 1;
            break;
        case 1:
            // Trimite comanda de escape pentru a curăța terminalul
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 0x1B;  // Escape character (\033)
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '[';    // [
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '2';    // 2
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 'J';    // J

            // Prima linie: "Seteaza tu frecventa:"
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
            u32TrimiteMesaj(" Seteaza tu frecventa:");
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
            FrecventaSemnal = 2;
            break;
        case 2:
            // Trimite comanda de escape pentru a curăța terminalul
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 0x1B;  // Escape character (\033)
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '[';    // [
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '2';    // 2
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 'J';    // J

            // Prima linie: "Setat automat la 80Hz"
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
            while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
            u32TrimiteMesaj(" Setat automat la 800Hz");
            FrecventaSemnal = 0;
        default:
            break;
    } }


}


// INTERRUPT ADC TIMER
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer_B (void)
{
    P6OUT ^= BIT6;
    ADCCTL0 |= ADCENC | ADCSC;                                    // Sampling and conversion start

}

// INTERRUPT ADC
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
{
            valADC = ADCMEM0;
            if (valADC <= 1) valADC = 1;
            else valADC = ADCMEM0 - 1;
            //TB1CCR0 = valADC - 1;
            //Frecventa = 12000000 / valADC + 1;

}


uint16_t ValoareLast = 1;
// DAC
// limita de transmisie 1.6 Khz - TB1CCR0 = [2Hz , 1.6KHz]
// INTERRUPT DAC TIMER
#pragma vector = TIMER1_B0_VECTOR
__interrupt void Timer1_B0_ISR(void)
{

    switch(FrecventaSemnal){
        case 0:
            TB1CCR0 = 100; // 800Hz SMCLK/TBCCR * 100
            break;
        case 1:
            TB1CCR0 = 1000;  // 80Hz SMCLK/TBCCR * 100
            break;
        case 2:

                TB1CCR0 = Valoare;
        default:
            break;
    }

    switch(TipSemnal){
        case 0:
            countDac++;
            if(countDac==100)
            countDac=0;

            SAC1DAT = rampa[countDac];
            break;
        case 1:
            countDac++;
            if(countDac==100)
            countDac=0;
            SAC1DAT = Sinus[countDac];
            break;
        case 2:
            countDac++;
            if(countDac==100)
            countDac=0;
            SAC1DAT = DinteFierastrau[countDac];
            break;
        default:
            break;
    }


}




