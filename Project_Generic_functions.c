#include "Project_Generic_functions.h"
#include <msp430.h>
#include <stdint.h>
#include <math.h>
#include <stddef.h>
#include "dsp_filters.h"
#include "Variabile_Globale.h"


/*  Zona Pentru functii  */
/********************************************************************************/
/* Nume Functie: vUART_init      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/****** Baud rate = 115200  ************/
/* P1.6 -> Tx  ~~~  Rx -> P1.7 */
// Inițializare UART (eUSCI_A0)

void vUART_init(void)
{

    UCA0CTLW0 |= UCSWRST;                      // Reset USCI
    UCA0CTLW0 |= UCSSEL__SMCLK;                // SMCLK ca sursă de ceas

    // Baud rate: 115200, cu SMCLK = 24MHz
    UCA0BRW = 208;                             // UCAxBRW = 24MHz / 115200 = 208
    UCA0MCTLW = UCOS16 | UCBRF_0 | 0x5500;     // Modulator (calculat din tabel)

    UCA0CTLW0 &= ~UCSWRST;                     // Scoate USCI din reset
    UCA0IE |= UCRXIE;                          // Activează întreruperea RX

    // Configurare pini P1.6 / P4.2 (TX) și P1.7 / P4.3 (RX)
    P1SEL0 |= BIT6 | BIT7;
    P1SEL1 &= ~(BIT6 | BIT7);
    P4SEL0 |= BIT2 | BIT3;

}
  /* vUART_init Terminat*/
/****************************************************************************** */





/********************************************************************************/
/* Nume Functie: u32TrimiteMesaj      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/****** Baud rate = 115200  ************/
/* Funcție pentru trimiterea unui mesaj */

size_t u32TrimiteMesaj(const char *str)
{
    while (*str) {
       while (!(UCA1IFG & UCTXIFG)); // așteaptă până e liber
       UCA1TXBUF = *str++;
    }
}
  /* u32TrimiteMesaj Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: vIntToString      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/****** Baud rate = 115200  ************/
/* Funcție pentru typecast din int in string */

void vIntToString(unsigned int val, char *str) {
    char temp[10];
    int i = 0;

    if (val == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }

    while (val > 0) {
        temp[i++] = '0' + (val % 10);
        val /= 10;
    }

    int j = 0;
    while (i > 0) {
        str[j++] = temp[--i];
    }
    str[j] = '\0';
}
  /* vIntToString Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: charToInt      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/****** Baud rate = 115200  ************/
/* Funcție pentru typecast din char in int */

int charToInt(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    else
        return -1; // valoare invalidă dacă nu e cifră
}

  /* charToInt Terminat*/
/****************************************************************************** */




/********************************************************************************/
/* Nume Functie: vInit_GPIO      */
/* Argumente : str         */
/* Valoarea returnata : niciuna */
/* Funcție pentru initializarea GPIO */

void vInit_GPIO()
{
    P1DIR = 0xFF; P2DIR = 0xFF;
    P1REN = 0xFF; P2REN = 0xFF;
    P1OUT = 0x00; P2OUT = 0x00;
}

  /* vInit_GPIO Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: vCollectADCData      */
/* Argumente :         */
/* Valoarea returnata : niciuna */
/* Funcție pentru colectarea valorilor din ADC */

void vCollectADCData()
{
    if (buffer_index < MARIME_FFT)
    {
        adc_buffer[buffer_index++] = ADCMEM0; // Citire valoare ADC
    }
    else
    {
        buffer_index = 0; // Reset buffer atunci când se atinge dimensiunea maximă
    }
}
  /* vCollectADCData Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: vFFT      */
/* Argumente : real , imag , n   */
/* Valoarea returnata : niciuna */
/* Funcție pentru transformata Fourier */

void vFFT(float *real, float *imag, int n)
{
    int i, j, k, m, m2;
    float realTemp, imagTemp, cosTemp, sinTemp;

    // Re-arrange the input data
    for (i = 1, j = 0; i < n; i++)
    {
        m = n / 2;
        while (j >= m)
        {
            j -= m;
            m /= 2;
        }
        j += m;

        if (i < j)
        {
            realTemp = real[i];
            imagTemp = imag[i];
            real[i] = real[j];
            imag[i] = imag[j];
            real[j] = realTemp;
            imag[j] = imagTemp;
        }
    }

    // algoritmul FFT
    m = 2;
    while (n > m)
    {
        m2 = m / 2;
        cosTemp = cos(PI / m2);
        sinTemp = -sin(PI / m2);
        for (k = 0; k < n; k += m)
        {
            float wr = 1.0, wi = 0.0;
            for (j = 0; j < m2; j++)
            {
                int i1 = k + j;
                int i2 = i1 + m2;
                realTemp = wr * real[i2] - wi * imag[i2];
                imagTemp = wr * imag[i2] + wi * real[i2];
                real[i2] = real[i1] - realTemp;
                imag[i2] = imag[i1] - imagTemp;
                real[i1] += realTemp;
                imag[i1] += imagTemp;
                wr = wr * cosTemp - wi * sinTemp;
                wi = wi * cosTemp + wr * sinTemp;
            }
        }
        m = m2;
    }
}
  /* vFFT Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: vDoCalculateFFTParameters      */
/* Argumente :  real, imag ,  n  , FrecventaEsantionare    */
/* Valoarea returnata : niciuna */
/* Funcție pentru colectarea valorilor din ADC */

void vDoCalculateFFTParameters(float *real, float *imag, int n, float * FrecventaEsantionare)
{
    int i;
    float amplitudinea_maxima = 0;
    int max_index = 0;

    // Căutăm frecvența dominantă și amplitudinea

    for (i = 0; i < n / 2; i++)  // doar jumătatea pozitivă a spectrului
    {
        float amplitudine = sqrt(real[i] * real[i] + imag[i] * imag[i]);

        if (amplitudine > amplitudinea_maxima)
        {
            amplitudinea_maxima = amplitudine;
            max_index = i;
        }
    }

    // Calculăm frecvența dominantă
    float Frecventa = (float)(max_index * (int)FrecventaEsantionare) / n;

    // Va urma : passband, cutoff, Q-factor etc.

    // Trimiterea prin UART a valorilor calculate
    vTrimiteDateleCatrePC(Frecventa, amplitudinea_maxima);
}
  /* vDoCalculateFFTParameters Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: vTrimiteDateleCatrePC  */
/* Argumente : frecventa        */
/* Valoarea returnata : niciuna */
/* Funcție de trimitere date la PC */

void vTrimiteDateleCatrePC(float frecventa, float amplitudine)
{
    int i;
    char buffer[50];
  //  snprintf(buffer, sizeof(buffer), "Frecventa: %.2f Hz, Amplitudine: %.2f\n", frecventa, amplitudine);
    int n = strlen(buffer);
    for (i = 0; i <= n; i++)
    {
        while(!(UCA1IFG&UCTXIFG));  // Așteaptă până când UART este disponibil
        UCA1TXBUF = buffer[i];
    }
}

  /* vTrimiteDateleCatrePC Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: vResetPlaca      */
/* Argumente :         */
/* Valoarea returnata : niciuna */
/* Funcție de reset */

void vResetPlaca(void) {
    __bis_SR_register(GIE);  // Asigură că întreruperile sunt permise înainte de reset
    __reset();               // Efectuează reset hardware al microcontrolerului
}

  /* vResetPlaca Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: u32Esantionare      */
/* Argumente :         */
/* Valoarea returnata : niciuna */
/* Funcție pentru determinarea frecventei de esantionare a ADC-ului */

void u32Esantionare(uint32_t * FrecventaEsantionare)
{
  if ( (ADCCTL2 & MascaPentru200Kps ) == 0)
    {
        *FrecventaEsantionare = 200000;
    }
    else if ( (ADCCTL2 & MascaPentru50Kps ) == 1)
    {
        *FrecventaEsantionare = 50000;
    }
    else
    {
      vResetPlaca();
      printf("Resetul a avut loc");
    }

}
  /* u32Esantionare Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: cDetectFilterType      */
/* Argumente :         */
/* Valoarea returnata : niciuna */
/* Funcție pentru determinarea filtrului */

// Funcție pentru determinarea filtrului
const char* cDetectieTipFiltru(float *real, float *imag, int n, int sampleRate)
{
    float UltimaMagnitudine = 0.0;
    int FrecventaJoasaIndex = 0;
    int FrecventaRidicataIndex = 0;
    int StartTrecereIndex = 0;
    int StopTrecereIndex = 0;
    int i;
    float magnitude;
    float MagnitudineMax = 0;
    float MagnitudineMin = 1e9; // O valoare mare, care va fi înlocuită cu amplitudinea minimă
    float MagnitudineAverage = 0;

    // Calculăm magnitudinea și analizăm spectrul
    for (i = 1; i < n / 2; i++) // Ignorăm partea negativă a spectrului
    {
        magnitude = sqrt(real[i] * real[i] + imag[i] * imag[i]);
        MagnitudineAverage += magnitude;  // Calculăm media amplitudinii

        if (magnitude > MagnitudineMax)
            MagnitudineMax = magnitude;

        if (magnitude < MagnitudineMin)
            MagnitudineMin = magnitude;

        // Căutăm scăderea amplitudinii pentru detectarea unui filtru trece-jos sau oprește-banda
        if (UltimaMagnitudine > magnitude && FrecventaJoasaIndex == 0)
        {
            FrecventaJoasaIndex = i; // Frecvența unde scade amplitudinea
        }

        // Căutăm frecvența unde amplitudinea începe să crească semnificativ
        if (UltimaMagnitudine < magnitude && FrecventaJoasaIndex > 0)
        {
            FrecventaRidicataIndex = i; // Frecvența de revenire a amplitudinii
            break; // Am găsit banda
        }

        UltimaMagnitudine = magnitude;
    }

    MagnitudineAverage /= (n / 2); // Media amplitudinii

    // Detectarea unui filtru Oprește-Banda (Notch Filter)
    if (MagnitudineMax - MagnitudineMin > 2 * MagnitudineAverage)
    {
        return "Oprește-Bandă (Notch Filter)";
    }

    // Detectare filtru Trecetot (All-Pass Filter) - Amplitudinea constantă pe întreg spectrul
    if (MagnitudineMax - MagnitudineMin < MagnitudineAverage / 2)
    {
        return "Trece-Tot (All-Pass Filter)";
    }

    // Detectare tip filtru pe baza comportamentului amplitudinii
    if (FrecventaJoasaIndex == 0 && FrecventaRidicataIndex > 0)
    {
        return "Trece-Sus (HPF)";
    }
    else if (FrecventaRidicataIndex == 0 && FrecventaJoasaIndex > 0)
    {
        return "Trece-Jos (LPF)";
    }
    else
    {
        return "Trece-Bandă (BPF)";
    }
}


  /* u32Esantionare Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: f32Aplicare_IIR      */
/* Argumente :         */
/* Valoarea returnata : niciuna */
/* Funcție pentru determinarea frecventei de esantionare a ADC-ului */

void f32Aplicare_IIR(float input) {

    // Shift valori vechi
    x[2] = x[1]; x[1] = x[0]; x[0] = input;
    y[2] = y[1]; y[1] = y[0];

    // Aplicarea ecuației diferențiale IIR (Direct Form I)
    y[0] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2]  // Termenii de feedforward
         - a[1]*y[1] - a[2]*y[2];             // Termenii de feedback

    return y[0];
}
  /* f32Aplicare_IIR Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: f32Aplicare_FIR      */
/* Argumente :         */
/* Valoarea returnata : niciuna */
/* Funcție pentru determinarea frecventei de esantionare a ADC-ului */

void f32Aplicare_FIR(float input) {

    // Shift valori vechi
    x[2] = x[1]; x[1] = x[0]; x[0] = input;
    y[2] = y[1]; y[1] = y[0];

    // Aplicarea ecuației diferențiale IIR (Direct Form I)
    y[0] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2] ;  // Termenii de feedforward


    return y[0];
}
  /* f32Aplicare_FIR Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: clearTerminalAndDisplayMessages      */
/* Argumente :         */
/* Valoarea returnata : niciuna */
/* Funcție pentru curatarea terminalului */

void clearTerminalAndDisplayMessages(void)
{
    // Trimite comanda de escape pentru a curăța terminalul
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 0x1B;  // Escape character (\033)
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '[';    // [
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '2';    // 2
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = 'J';    // J

    // Prima linie: "Simulare Filtre Digitale"
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
    u32TrimiteMesaj("Simulare Filtre Digitale");

    // A doua linie: "Setati Frecventa pentru generat semnalul"
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
    u32TrimiteMesaj("Student: Isachi Mihai");

    // A treia linie : " Semnalul va fi in Hz: "
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
    u32TrimiteMesaj("Semnalul va fi in Hz ");

    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
    u32TrimiteMesaj("  ATENTIE! PLACA POATE GENERA PANA LA 1.6KHz ");
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed
    u32TrimiteMesaj("     !SE RECOMANDA FOLOSIREA PANA LA 1.5KHz!  ");

    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\r';   // Carriage return
    while (!(UCA1IFG & UCTXIFG)); UCA1TXBUF = '\n';   // Line feed


    // Linia 3 rămâne ca înainte (se va afișa la fiecare primire de date)
}
  /* clearTerminalAndDisplayMessages Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: send_UART_Filter      */
/* Argumente :         */
/* Valoarea returnata : niciuna */
/* Funcție pentru Trimiterea UART a bufferului */

void send_UART_Filter(float val1, float val2) {
    char buffer[64];
//    sprintf(buffer, "FIR=%.3f\tIIR=%.3f\r\n", val1, val2);

    for (i = 0; i < strlen(buffer); i++) {
        while (!(UCA0IFG & UCTXIFG));  // Așteaptă buffer liber
        UCA0TXBUF = buffer[i];         // Trimite caracter
    }
}
  /* send_UART_Filter Terminat*/
/****************************************************************************** */
