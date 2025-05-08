#include <msp430.h>
#include <stdint.h>
#include <math.h>
#include <stddef.h>

uint32_t numar = 0;
volatile int TipSemnal = 0;
volatile int Frecventa = 0;
int voltage;

unsigned int countDac = 0;
unsigned int FrecventaSemnal = 0;
unsigned int uartIndex;


#define BUFFER_SIZE 64
char uart_buffer[BUFFER_SIZE];
unsigned int uart_index = 0;
char command[BUFFER_SIZE];  // copie finală la ENTER


int valADC = 0;
unsigned int i;
volatile int count = 0;
void int_to_char(unsigned char *temp, unsigned int number);


//pentru IIR butterworth low-pass
#define SIGNAL_LEN 100
#define PI 3.14159265358979
#define MAX_UART_BUFFER 10

// Coeficienți IIR Butterworth Low-pass, ordin 2 (100 Hz, Fs = 1000 Hz)
float b[3] = {0.0675, 0.1349, 0.0675};
float a[3] = {1.0,   -1.14298, 0.4128};

// Istoric semnal
float x[3] = {0};  // intrări x[n], x[n-1], x[n-2]
float y[3] = {0};  // ieșiri  y[n], y[n-1], y[n-2]

// Simulăm un semnal cu zgomot: sinus + zgomot aleator
float input_signal[SIGNAL_LEN];
float output_signal[SIGNAL_LEN];

volatile unsigned int uartBuffer[20];

// Variabile de test


int count2 = 1;
volatile unsigned int unknown_isr_vector = 0;

// Declaratii Static inline

static inline void vResetPlaca(void);
static inline size_t u32NumarCaractere(const char *str);

// Define-uri
#define MARIME_FFT               1024
#define PI                       3.14159265358979323846
#define MascaPentru200Kps        0b000
#define MascaPentru50Kps         0b100
#define TRUE                     0x1
#define FALSE                    0x0
#define MARIME_SEMNAL            100


volatile uint16_t adc_buffer[MARIME_FFT];
volatile uint8_t buffer_index = 0;
volatile uint32_t FEsantionare = 0;
volatile size_t NumarCaractere;
volatile uint8_t bStateDetectie = FALSE;
volatile uint8_t StatusP21;
volatile uint8_t StatusP22;
volatile uint8_t StatusP23;
//volatile uint16_t valADC;

//typedef enum {
//    Semnal_Sinusoidal,
//    Semnal_DinteFierastrau,
//    Semnal_Rampa
//} TipSemnal;

typedef enum {
    Canal0,
    Canal1,
    Canal2,
    Canal3
} Canal_SAC_DAC;

//typedef enum {
//    F200Hz,
//    F500Hz,
//    F1KHz
//} FrecventaSemnal;

typedef enum {
  TreceJos,
  TreceSus,
  TreceBanda,
  OpresteBanda,
  TreceTot
} TipFiltru;

typedef enum {
    STATE_OFF,
    STATE_ON
} LedState;



volatile LedState currentState = STATE_OFF;
//volatile FrecventaSemnal PasFrecventa = F200Hz;
volatile Canal_SAC_DAC PasCanal = Canal0;
//volatile TipSemnal PasulDeSemnal = Semnal_Sinusoidal;

// Defazaj 0 Grade
unsigned int Defazaj_0 = 0 ;

// Defazaj 900 Grade
unsigned int Defazaj_90 = 24 ;

// Defazaj 180 Grade
unsigned int Defazaj_180 = 49 ;

// Defazaj 270 Grade
unsigned int Defazaj_270 = 74 ;

int Numar_Esantioane = 100;

// Valori pentru a genera un Sinus
unsigned int Sinus[100]={2048,2176,2304,2431,2557,2680,2801,2919,3034,3145,
                       3251,3353,3449,3540,3625,3704,3776,3842,3900,3951,
                       3995,4031,4059,4079,4091,4095,4091,4079,4059,4031,
                       3995,3951,3900,3842,3776,3704,3625,3540,3449,3353,
                       3251,3145,3034,2919,2801,2680,2557,2431,2304,2176,
                       2048,1919,1791,1664,1538,1415,1294,1176,1061,950,
                       844,742,646,555,470,391,319,253,195,144,
                       100,64,36,16,4,0,4,16,36,64,
                       100,144,195,253,319,391,470,555,646,742,
                       844,950,1061,1176,1294,1415,1538,1664,1791,1919};

unsigned int DinteFierastrau[100]={82,164,246,328,410,491,573,655,737,819,
                       901,983,1065,1147,1229,1310,1392,1474,1556,1638,
                       1720,1802,1884,1966,2048,2129,2211,2293,2375,2457,
                       2539,2621,2703,2785,2867,2948,3030,3112,3194,3276,
                       3358,3440,3522,3604,3686,3767,3849,3931,4013,4095,
                       4013,3931,3849,3767,3686,3604,3522,3440,3358,3276,
                       3194,3112,3030,2948,2867,2785,2703,2621,2539,2457,
                       2375,2293,2211,2129,2048,1966,1884,1802,1720,1638,
                       1556,1474,1392,1310,1229,1147,1065,983,901,819,
                       737,655,573,491,410,328,246,164,82,0};

unsigned int rampa[100] = {
    0,   41,  82,  123,  164,  205,  246,  287,  328,  369,
    410,  451,  492,  533,  574,  615,  656,  697,  738,  779,
    820,  861,  902,  943,  984,  1025, 1066, 1107, 1148, 1189,
    1230, 1271, 1312, 1353, 1394, 1435, 1476, 1517, 1558, 1599,
    1640, 1681, 1722, 1763, 1804, 1845, 1886, 1927, 1968, 2009,
    2050, 2091, 2132, 2173, 2214, 2255, 2296, 2337, 2378, 2419,
    2460, 2501, 2542, 2583, 2624, 2665, 2706, 2747, 2788, 2829,
    2870, 2911, 2952, 2993, 3034, 3075, 3116, 3157, 3198, 3239,
    3280, 3321, 3362, 3403, 3444, 3485, 3526, 3567, 3608, 3649,
    3690, 3731, 3772, 3813, 3854, 3895, 3936, 3977, 4018, 4059
};

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
/* Nume Functie: vUART_trimite_char      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/****** Baud rate = 115200  ************/
/* Funcție pentru trimiterea unui caracter prin UART */

void vUART_trimite_char(char *str, size_t * nrcat)
{
   int i;
   while (!(UCA0IFG & UCTXIFG)); // Așteaptă bufferul liber
   for(i = 0; i<=(int)nrcat; i++){
       UCA0TXBUF = str[i];                // Trimite caracterul
   }

}
  /* vUART_trimite_char Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: vUART_Trimite_string      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/****** Baud rate = 115200  ************/
/* Funcție pentru trimiterea unui șir de caractere prin UART */

void vUART_Trimite_string(const char *str, size_t * nrcat)
{
    nrcat = u32NumarCaractere(*str);

    while (*str) {
        vUART_trimite_char(*str++, *nrcat);
    }
}
  /* vUART_Trimite_string Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: u32TrimiteMesaj      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/****** Baud rate = 115200  ************/
/* Funcție pentru trimiterea unui mesaj */

static inline size_t u32TrimiteMesaj(const char *str)
{
    while (*str) {
       while (!(UCA1IFG & UCTXIFG)); // așteaptă până e liber
       UCA1TXBUF = *str++;
    }
}
  /* u32NumarCaractere Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: u32NumarCaractere      */
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
  /* u32NumarCaractere Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: u32NumarCaractere      */
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

  /* u32NumarCaractere Terminat*/
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
    snprintf(buffer, sizeof(buffer), "Frecventa: %.2f Hz, Amplitudine: %.2f\n", frecventa, amplitudine);
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

static inline void vResetPlaca(void) {
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
    P2OUT |= BIT3;                          // Configure P1.3 as pulled-up
    P2REN |= BIT3;                          // P1.3 pull-up register enable
    P2IES |= BIT3;                          // P1.3 Hi/Low edge
    P2IE |= BIT3;                           // P1.3 interrupt enabled
    // stergem indicatorul de intreruperi de la P2.3
    P2IFG &= ~BIT3;                         // P1.3 IFG cleared

    // P4.1 ->
    P4DIR &= ~BIT1; // Configuram P4.1 ca intrare
    P4OUT |= BIT1;                          // Configure P1.3 as pulled-up
    P4REN |= BIT1;                          // P1.3 pull-up register enable
    P4IES |= BIT1;                          // P1.3 Hi/Low edge
    P4IE |= BIT1;                           // P1.3 interrupt enabled
    // stergem indicatorul de intreruperi de la P2.3
    P4IFG &= ~BIT1;                         // P1.3 IFG cleared

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

    // DAC
    P1SEL0 |= BIT5;
    P1SEL1 |= BIT5;

    SAC1DAC = DACSREF_1 + DACLSEL_0 + DACIE;
    SAC1DAT = Sinus[0];
    SAC1DAC |= DACEN;

    SAC1OA = NMUXEN + PMUXEN + PSEL_1 + NSEL_1;
    SAC1OA |= OAPM;
    SAC1PGA = MSEL_1;
    SAC1OA |= SACEN + OAEN;
    TB1CCTL0 |= CCIE;


    TB1CCR0 = 120;
    TB1CTL = TBSSEL__SMCLK | MC_1 | TBCLR;

    __bis_SR_register(GIE); // global interrupts



    PM5CTL0 &= ~LOCKLPM5;                              // Disable the GPIO power-on default high-impedance mode
                                                       // to activate previously configured port settings
}
volatile uint16_t Valoare = 80;
volatile uint16_t ValoareTastatura = 1;
//  INTERRUPT ROUTINE UART

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
    P2IFG &= ~BIT3;                         // Clear P1.3 IFG

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

// Port 2 interrupt service routine
#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
{
    if (P4IFG & BIT1){
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




