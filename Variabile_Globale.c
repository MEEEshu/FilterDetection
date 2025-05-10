#include "Variabile_Globale.h"
#include <stdint.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>    // pentru printf, sprintf, snprintf
#include <stdlib.h>   // pentru atoi
#include <string.h>   // pentru strlen

float fir_coeffs_2[3] = {0.3, 0.4, 0.3};
float iir_a_2[3] = {1.0, -0.5, 0.25};
float iir_b_2[3] = {0.1, 0.2, 0.1};

float x[3] = {0};
float y[3] = {0};
float b[3] = {0.067455, 0.134911, 0.067455};
float a[3] = {1.0, -1.14298, 0.4128};

uint32_t numar = 0;
volatile int TipSemnal = 0;
volatile int Frecventa = 0;
int voltage = 0;
unsigned int uart_index = 0;
unsigned int countDac = 0;
unsigned int FrecventaSemnal = 0;
unsigned int uartIndex = 0;
char uart_buffer[64] = {0};
char command[64] = {0};
int valADC = 0;
unsigned int i = 0;
volatile int count = 0;

volatile unsigned int uartBuffer[20] = {0};

int count2 = 1;
volatile unsigned int unknown_isr_vector = 0;

volatile uint16_t adc_buffer[1024] = {0};
volatile uint8_t buffer_index = 0;
volatile uint32_t FEsantionare = 0;
volatile size_t NumarCaractere = 0;
volatile uint8_t bStateDetectie = 0;


unsigned int Defazaj_0 = 0;
unsigned int Defazaj_90 = 24;
unsigned int Defazaj_180 = 49;
unsigned int Defazaj_270 = 74;

int Numar_Esantioane = 100;

unsigned int Sinus[100] = {2048,2176,2304,2431,2557,2680,2801,2919,3034,3145,
                           3251,3353,3449,3540,3625,3704,3776,3842,3900,3951,
                           3995,4031,4059,4079,4091,4095,4091,4079,4059,4031,
                           3995,3951,3900,3842,3776,3704,3625,3540,3449,3353,
                           3251,3145,3034,2919,2801,2680,2557,2431,2304,2176,
                           2048,1919,1791,1664,1538,1415,1294,1176,1061,950,
                           844,742,646,555,470,391,319,253,195,144,
                           100,64,36,16,4,0,4,16,36,64,
                           100,144,195,253,319,391,470,555,646,742,
                           844,950,1061,1176,1294,1415,1538,1664,1791,1919};
unsigned int DinteFierastrau[100] = {82,164,246,328,410,491,573,655,737,819,
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
