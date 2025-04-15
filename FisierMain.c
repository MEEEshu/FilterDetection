//******************************************************************************
//  MSP430FR235x Demo - SAC-L3, DAC Buffer Mode
//
//  Description: Configure SAC-L3 for DAC Buffer Mode. Use the 12 bit DAC to
//  output positive ramp. The OA is set in buffer mode to improve DAC output
//  drive strength. Internal 2.5V reference is selected as DAC reference.
//  Observe the output of OA0O pin with oscilloscope.
//  ACLK = n/a, MCLK = SMCLK = default DCODIV ~1MHz.
//
//  configurare buton P2.1
//  selectie tip semnal: sinus/dintefierastrau/rampa
//  configurare buton P2.2
//  selectie canal SAC_DAC_x pentru care sa modificam defajajul
//  configurare buton P2.3
//  incrementarea defazajului fata de SAC_DAC_0
//  configurare buton P2.4
//  decrementarea defazajului fata de SAC_DAC_0
//
//                              MSP430FR2355
//             -----------------------------------------
//         /|\|                                          |
//          | |                                          |
//          --|RST                      P1.1->DAC12->OA0O|--> P1.1 Canal 0 Output
//            |                         P1.5->DAC12->OA1O|--> P1.5 Canal 1 Output
//            |                         P3.3->DAC12->OA2O|--> P3.3 Canal 2 Output
//            |                         P3.5->DAC12->OA3O|--> P3.5 Canal 3 Output
//          --|<-P2.1 - Tip Semnal                       |
//          --|<-P2.2 - Selectam Canal DAC               |
//          --|<-P2.3 - Incrementam Defazajul pentru DAC |
//          --|<-P2.4 - Incrementam Defazajul pentru DAC |
//          --|<-P4.1 - Selectam Frecventa pentru Output |
//            |                                          |
//
//******************************************************************************
//Se doreste frecventa generata = 20KHz
//
#include <msp430.h>
#include <stdint.h>
#include <math.h>
#include <stddef.h>


// Variabile de test

int count = 0;
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


volatile uint16_t adc_buffer[MARIME_FFT];
volatile uint8_t buffer_index = 0;
volatile uint32_t FEsantionare = 0;
volatile size_t NumarCaractere;
volatile uint8_t bStateDetectie = FALSE;

typedef enum {
    Semnal_Sinusoidal,
    Semnal_DinteFierastrau,
    Semnal_Rampa
} TipSemnal;

typedef enum {
    Canal0,
    Canal1,
    Canal2,
    Canal3
} Canal_SAC_DAC;

typedef enum {
    F200Hz,
    F500Hz,
    F1KHz
} FrecventaSemnal;

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
volatile FrecventaSemnal PasFrecventa = F200Hz;
volatile Canal_SAC_DAC PasCanal = Canal0;
volatile TipSemnal PasulDeSemnal = Semnal_Sinusoidal;

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
/* Nume Functie: vUART_trimite_char      */
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
/* Nume Functie: u32NumarCaractere      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/****** Baud rate = 115200  ************/
/* Funcție pentru numararea caracterelor dintr-un sir */

static inline size_t u32NumarCaractere(const char *str)
{
    size_t count = 0;
    while (str[count] != '\0') {
        count++;
    }
    return count;
}
  /* u32NumarCaractere Terminat*/
/****************************************************************************** */


/********************************************************************************/
/* Nume Functie: vInitADC      */
/* Argumente : str         */
/* Valoarea returnata : niciuna */
/* Funcție pentru configurarea ADC-ului */

void vInitADC() {

                                            // Configurează ADC
    P6SEL1 |= BIT0;                         // selectează P6.0 pentru ADC
    ADCCTL0 = ADCENC_1 | ADCSHT_2 | ADCON;  // Enable ADC, ADC clock
    ADCCTL1 = ADCSSEL_2;                    // Sursa ceasului ADC = MCLK
    ADCCTL2 = ADCRES_2;                     // Rezoluție 12 biți

    ADCMCTL0 = ADCINCH_0; // Canalul ADC pe care îl citim (P6.0)

}  /* vInitADC Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: vStartADC      */
/* Argumente : str         */
/* Valoarea returnata : niciuna */
/* Funcție pentru pornirea ADC-ului */

void vStartADC()
{
    ADCCTL0 |= ADCENC_1 | ADCSHS_1 ; // Start ADC conversion
}
  /* vStartADC Terminat*/
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

/********************* FUNCTIA MAIN ******************************/
int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // oprim Watchdog-ul

  FRCTL0 = FRCTLPW | NWAITS_2;              // configuram clock-ul la 24MHz, punem sistemul in Wait

  __bis_SR_register(SCG0);                           // disable FLL
  CSCTL3 |= SELREF__REFOCLK;                         // se configurează FLL (Frequency Locked Loop)
                                                     // pentru a genera un semnal de ceas de 24 MHz folosind un oscilator intern (DCO).
  CSCTL0 = 0;                                        // dam clear DCO and MOD registers
  CSCTL1 |= DCORSEL_7;                               // setam DCO = 24MHz
  CSCTL2 = FLLD_0 + 731;                             // setam DCOCLKDIV = 24MHz
  __delay_cycles(3);
  __bic_SR_register(SCG0);                           // enable FLL
  while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));         // FLL locked

  CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;         // Se selectează DCOCLKDIV ca sursă pentru MCLK și SMCLK
  PM5CTL0 &= ~LOCKLPM5;                              // Disable the GPIO power-on default high-impedance mode
  //3.0 MCLK

  P3DIR |= BIT0;
  P3SEL0 |= BIT0;
  //vInit_GPIO();


  if(P5IN & BIT4)
  {
    bStateDetectie = TRUE; // porneste detectia de filtru
  }
  else
  {
    bStateDetectie = FALSE; // opreste detectia de filtru
  }
  /*vUART_init(); // Configuram UART
  vInit_GPIO();  // Initializam GPIO
  vInitADC();   // Initializam ADC
  u32Esantionare(&FEsantionare); //Stocam frecventa de esantionare a ADC_ului
  int i;
  while(bStateDetectie)
  {
    vStartADC(); //Pornim ADC-ul - incepe esantionarea
    //__bis_SR_register(CPUOFF + GIE); // Intră în low power mode până la întrerupere ADC

    //Colectam Datele de la ADC
    vCollectADCData();
    if (buffer_index == MARIME_FFT)
    {
       // Procesăm datele cu FFT
        float real[MARIME_FFT] = {0};
        float imag[MARIME_FFT] = {0};
        for (i = 0; i < MARIME_FFT; i++)
        {
            real[i] = (float)adc_buffer[i];  // Înlocuiește cu valorile ADC
            imag[i] = 0;
        }

        // Calculăm FFT
        vFFT(real, imag, MARIME_FFT);
        vDoCalculateFFTParameters(real, imag, MARIME_FFT, FEsantionare); // calculam parametrii FFT
        // Detectăm tipul filtrului
        const char *filterType = cDetectieTipFiltru(real, imag, MARIME_FFT, FEsantionare); // detectia de filtru
    }


  }*/

  P5SEL0 |= BIT4;  //Setam pinul P5.4 pentru a porni Detectia Tipului de Filtru
  P5DIR &= ~BIT4;
  P5REN &= ~BIT4;

  // Configurare LED pe P6.6 ca output
  P6DIR |= BIT6;              // P6.6 output
  P6OUT &= ~BIT6;             // LED oprit inițial
  P6SEL0 &= ~BIT6;
  P6SEL1 &= ~BIT6;

  P1DIR |= BIT0;
  P1SEL1 |= BIT0;                             // Se configurează P1.0 ca ieșire pentru SMCLK (pentru diagnosticare).

//configurare DAC -> Se configurează pinii P1.1, P1.5, P3.1 și P3.5 pentru ieșirea semnalului DAC
  P1SEL0 |= BIT1;                           // Selectam P1.1 ca OA0O
  P1SEL1 |= BIT1;                           // OA este folosit ca buffer pentru DAC

  P1SEL0 |= BIT5;                           // Selectam P1.5 ca OA0O
  P1SEL1 |= BIT5;                           // OA este folosit ca buffer pentru DAC

  P3SEL0 |= BIT1;                           // Selectam P3.1 as OA0O
  P3SEL1 &= ~BIT1;                          // OA este folosit ca buffer pentru DAC

  P3SEL0 |= BIT5;                           // Selectam P3.5 as OA0O
  P3SEL1 &= ~BIT5;                          // OA este folosit ca buffer pentru DAC

  // Configurăm P2.1, P2.2, P2.3 și P2.5 ca intrări cu rezistență de pull-up și întrerupere pe flanc descendent
  P2DIR &= ~(BIT1 | BIT2 | BIT3 | BIT5);         // Setăm P2.1, P2.2, P2.3, P2.5 ca intrări
  P2OUT |=  (BIT1 | BIT2 | BIT3 | BIT5);         // Activăm rezistențele de pull-up (1 în OUT)
  P2REN |=  (BIT1 | BIT2 | BIT3 | BIT5);         // Activăm rezistențele interne (pull-up active)
  P2IES |=  (BIT1 | BIT2 | BIT3 | BIT5);         // Detectăm tranziții pe flanc descendent (1 -> 0)
  P2IE  |=  (BIT1 | BIT2 | BIT3 | BIT5);         // Activăm întreruperile pentru acești pini
  P2IFG &= ~(BIT1 | BIT2 | BIT3 | BIT5);         // Curățăm orice flag rămas

  // configuram butonul pentru P4.1

  P4OUT |= BIT1;                          // este configurat asemanator cu P2.3
  P4REN |= BIT1;
  P4IES |= BIT1;
  P4IE  |= BIT1;
  P4IFG &= ~BIT1;

 // __delay_cycles(400);

  // Configuram referinta
  PMMCTL0_H = PMMPW_H;                      // activarea PMM
  PMMCTL2 = INTREFEN | REFVSEL_0;           // Se activeaza referinta de 2V
  while(!(PMMCTL2 & REFGENRDY));            // Se asteapta stabilizarea acestuia

// SAC0_DAC
// SAC0DACSTS=DACIFG;// clear intreupt
// SAC0DAT = sin[0]; // write sample


  SAC0DAC = DACSREF_0 + DACLSEL_2;  // selectam referinta interna ca sursa pentru DAC
  SAC0DAT = rampa[Defazaj_0];               // Initializam  valorile pentru DAC
  SAC0DAC |= DACEN;                         // Activam DAC-ul
 // SAC0DACSTS=DACIFG;// clear intreupt



// SAC1_DAC
  SAC1DAC = DACSREF_0 + DACLSEL_2;  // selectam referinta interna ca sursa pentru DAC
  SAC1DAT = rampa[Defazaj_90];                 // Initializam  valorile pentru DAC
  SAC1DAC |= DACEN;                            // Activam DAC-ul
 // SAC1DACSTS=DACIFG;// clear intreupt


// SAC2_DAC
  SAC2DAC = DACSREF_0 + DACLSEL_2;  // selectam referinta interna ca sursa pentru DAC
  SAC2DAT = Sinus[Defazaj_180];                // Initializam  valorile pentru DAC
  SAC2DAC |= DACEN;                            // Activam DAC-ul
 // SAC2DACSTS=DACIFG;// clear intreupt


// SAC3_DAC
  SAC3DAC = DACSREF_0 + DACLSEL_2;  // selectam referinta interna ca sursa pentru DAC
  SAC3DAT = Sinus[Defazaj_270];                // Initializam  valorile pentru DAC
  SAC3DAC |= DACEN;                            // Activam DAC-ul
 // SAC3DACSTS=DACIFG;// clear intreupt


// SAC0_AO
  SAC0OA = NMUXEN + PMUXEN + PSEL_1 + NSEL_1;  // Selectam inputurile pozitive si negative
  SAC0OA |= OAPM_0;                            // Selectam viteza redusa si LPM
  SAC0PGA = MSEL_1;                            // Setam OA ca buffer
  SAC0OA |= SACEN + OAEN;                      // Activam SAC-ul si AO


// SAC1_AO
  SAC1OA = NMUXEN + PMUXEN + PSEL_1 + NSEL_1;  // Selectam inputurile pozitive si negative
  SAC1OA |= OAPM_0;                            // Selectam viteza redusa si LPM
  SAC1PGA = MSEL_1;                            // Setam OA ca buffer
  SAC1OA |= SACEN + OAEN;                      // Activam SAC-ul si AO


// SAC2_AO
  SAC2OA = NMUXEN + PMUXEN + PSEL_1 + NSEL_1;  // Selectam inputurile pozitive si negative
  SAC2OA |= OAPM_0;                            // Selectam viteza redusa si LPM
  SAC2PGA = MSEL_1;                            // Setam OA ca buffer
  SAC2OA |= SACEN + OAEN;                      // Activam SAC-ul si AO


// SAC3_AO
  SAC3OA = NMUXEN + PMUXEN + PSEL_1 + NSEL_1;  // Selectam inputurile pozitive si negative
  SAC3OA |= OAPM_0;                            // Selectam viteza redusa si LPM
  SAC3PGA = MSEL_1;                            // Setam OA ca buffer
  SAC3OA |= SACEN + OAEN;                      // Activam SAC-ul si AO


 // Folosim TB2.1 ca DAC hardware trigger - pentru a obtine frecventa de actualizare

  TB2CCR0 = 120-1;                             // Perioada setata pentru 120 de cicluri
  TB2CCTL1 = OUTMOD_6;                         // TBCCR1 toggle/set - Modul PWM
  TB2CCR1 = 60;                                // TBCCR1 PWM duty cycle (CCR1/CCR2)
  TB2CTL = TBSSEL__SMCLK | MC_1 | TBCLR;       // SMCLK ca si semnal de tact, se porneste in UP mode

  //P2IFG |= BIT0 | BIT1 | BIT2| BIT3 | BIT4;
 // P4IFG |= BIT1;
  // Activam intreruperile
  PM5CTL0 &= ~LOCKLPM5;
  while(1)
      {
      __bis_SR_register(GIE);         // Enter LPM3, interrupts enabled
      __no_operation();
      P6OUT ^= BIT6;
      }
                       // For debugger
}

// Port 2 interrupt service routine
// ISR pentru Portul 2
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(P2IV, P2IV__P2IFG7))
    {

        case P2IV__P2IFG1:
            P2IFG &= ~BIT1;
            PasCanal++;
            if (PasCanal > Canal3) PasCanal = Canal0;
            __bic_SR_register_on_exit(LPM3_bits);
            break;


        case P2IV__P2IFG2:
            P2IFG &= ~BIT2;
            PasulDeSemnal++;
            if (PasulDeSemnal > Semnal_Rampa) PasulDeSemnal = Semnal_Sinusoidal;
            break;


        case P2IV__P2IFG3:
            P2IFG &= ~BIT3;
            count++;
            switch (PasCanal) {
                case Canal0:
                    Defazaj_0++;
                    if (Defazaj_0 > Numar_Esantioane-1) Defazaj_0 = 0;
                    break;
                case Canal1:
                    Defazaj_90++;
                    if (Defazaj_90 > Numar_Esantioane-1) Defazaj_90 = 0;
                    break;
                case Canal2:
                    Defazaj_180++;
                    if (Defazaj_180 > Numar_Esantioane-1) Defazaj_180 = 0;
                    break;
                case Canal3:
                    Defazaj_270++;
                    if (Defazaj_270 > Numar_Esantioane-1) Defazaj_270 = 0;
                    break;
                default: break;
            }
            break;


        case P2IV__P2IFG4:
            P2IFG &= ~BIT4;
            break;

        case P2IV__P2IFG5:
            P2IFG &= ~BIT5;
            switch (PasCanal) {
                case Canal0:
                    Defazaj_0--;
                    if (Defazaj_0 == 65535) Defazaj_0 = Numar_Esantioane;
                    break;
                case Canal1:
                    Defazaj_90--;
                    if (Defazaj_90 == 65535) Defazaj_90 = Numar_Esantioane;
                    break;
                case Canal2:
                    Defazaj_180--;
                    if (Defazaj_180 == 65535) Defazaj_180 = Numar_Esantioane;
                    break;
                case Canal3:
                    Defazaj_270--;
                    if (Defazaj_270 == 65535) Defazaj_270 = Numar_Esantioane;
                    break;
                default: break;
            }
            break;

        default: break;
    }


    __bic_SR_register_on_exit(LPM3_bits);
}

#pragma vector = TIMER2_B0_VECTOR
__interrupt void Timer2_B0_ISR(void)
{


     Defazaj_0++;    // incrementare index defazaj 0 grade
     Defazaj_90++;   // incrementare index defazaj 90 grade
     Defazaj_180++;  // incrementare index defazaj 180 grade
     Defazaj_270++;  // incrementare index defazaj 270 grade

     //Selectam tipul de semnal
     switch(PasulDeSemnal)

         // semnal sinusoidal
     { case Semnal_Sinusoidal:
                 SAC0DAT = Sinus[Defazaj_0];
                 SAC1DAT = Sinus[Defazaj_90];
                 SAC2DAT = Sinus[Defazaj_180];
                 SAC3DAT = Sinus[Defazaj_270];
                 break;

         // semnal dinte fierastrau
       case Semnal_DinteFierastrau:
                 SAC0DAT = DinteFierastrau[Defazaj_0];
                 SAC1DAT = DinteFierastrau[Defazaj_90];
                 SAC2DAT = DinteFierastrau[Defazaj_180];
                 SAC3DAT = DinteFierastrau[Defazaj_270];
                 break;

         // semnal rampa
       case Semnal_Rampa:
                 SAC0DAT = rampa[Defazaj_0];
                 SAC1DAT = rampa[Defazaj_90];
                 SAC2DAT = rampa[Defazaj_180];
                 SAC3DAT = rampa[Defazaj_270];
                 break;
            // setam pe default sa fie sinus
          default:
                 SAC0DAT = rampa[Defazaj_0];
                 SAC1DAT = rampa[Defazaj_90];
                 SAC2DAT = rampa[Defazaj_180];
                 SAC3DAT = rampa[Defazaj_270];
                 break;
     }
     // verificam indexul de defazaj pentru 0 Grade
     if(Defazaj_0 >= 99)
     {
            // resetam indexul daca a atins ultima valoare din array
         Defazaj_0=0;
     }
     // verificam indexul de defazaj pentru 90 Grade
     else if(Defazaj_90 >= 99)
     {
         // resetam indexul daca a atins ultima valoare din array
         Defazaj_90=0;
     }
     // verificam indexul de defazaj pentru 180 Grade
     else if(Defazaj_180 >= 99)
     {
        // resetam indexul daca a atins ultima valoare din array
        Defazaj_180=0;
     }
     // verificam indexul de defazaj pentru 270 Grade
     else if(Defazaj_270 >= 99)
     {
        // resetam indexul daca a atins ultima valoare din array
        Defazaj_270=0;
     }

}


// Port 4 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT4_VECTOR))) Port_4 (void)
#else
#error Compiler not supported!
#endif
{
    P4IFG &= ~BIT1;                         // Clear P4.1 IFG
    PasFrecventa++;     //0-> 200Hz, 1-> 500Hz, 2->1KHz
    if(PasFrecventa > F1KHz) PasFrecventa = F200Hz;
    switch(PasFrecventa)
    {
    case F200Hz:
                                                    // Folosim TB2.1 ca DAC hardware trigger
         TB2CCR0 = 1200-1;                          // CLK= 20KHz f-semnal=200Hz  ->  PWM Period/2
         TB2CCTL1 = OUTMOD_6;                       // TBCCR1 toggle/set
         TB2CCR1 = 600;                             // TBCCR1 PWM duty cycle
         TB2CTL = TBSSEL__SMCLK | MC_1 | TBCLR;     // SMCLK, up mode, clear TBR

        break;

    case F500Hz:                               // Folosim TB2.1 ca DAC hardware trigger
        TB2CCR0 = 480-1;                           // CLK= 50KHz f-semnal=500Hz ->  PWM Period/2
        TB2CCTL1 = OUTMOD_6;                       // TBCCR1 toggle/set
        TB2CCR1 = 240;                             // TBCCR1 PWM duty cycle
        TB2CTL = TBSSEL__SMCLK | MC_1 | TBCLR;     // SMCLK, up mode, clear TBR

       break;

    case F1KHz:                                // Folosim TB2.1 ca DAC hardware trigger
                                                   // CLK= 100KHz f-semnal=1000Hz   ->  PWM Period/2
        TB2CCR0 = 240-1;                           // PWM Period/2
        TB2CCTL1 = OUTMOD_6;                       // TBCCR1 toggle/set
        TB2CCR1 = 120;                             // TBCCR1 PWM duty cycle
        TB2CTL = TBSSEL__SMCLK | MC_1 | TBCLR;     // SMCLK, up mode, clear TBR

       break;

    default:                                       // Folosim TB2.1 ca DAC hardware trigger
        //PasFrecventa = 0;
        TB2CCR0 = 1200-1;                          // CLK= 20KHz f-semnal=200Hz ->  PWM Period/2
        TB2CCTL1 = OUTMOD_6;                       // TBCCR1 toggle/set
        TB2CCR1 = 600;                             // TBCCR1 PWM duty cycle
        TB2CTL = TBSSEL__SMCLK | MC_1 | TBCLR;     // SMCLK, up mode, clear TBR

       break;
    }
}
