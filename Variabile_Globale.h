#ifndef VARIABILE_GLOBALE_H_
#define VARIABILE_GLOBALE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>    // pentru printf, sprintf, snprintf
#include <stdlib.h>   // pentru atoi
#include <string.h>   // pentru strlen

//pentru IIR butterworth low-pass
#define SIGNAL_LEN 100
#define PI 3.14159265358979
#define MAX_UART_BUFFER 10

#define BUFFER_SIZE             64         // Dimensiune buffer UART
#define SIGNAL_LEN              100        // Lungime semnal (ADC, filtrare etc.)
#define PI                      3.14159265358979  // (într-un loc) – pi trunchiat
#define MAX_UART_BUFFER         10         // Probabil rezervă UART/command parsing
#define MARIME_FFT              1024       // Dimensiune buffer FFT (adc_buffer)
#define MascaPentru200Kps       0b000      // Masca pentru frecvență transmisie 200 kbps
#define MascaPentru50Kps        0b100      // Masca pentru frecvență transmisie 50 kbps
#define TRUE                    0x1        // Echivalent boolean TRUE
#define FALSE                   0x0        // Echivalent boolean FALSE
#define MARIME_SEMNAL           100        // Semnal sinus/rampa/dinte fierăstrău


extern char uart_buffer[BUFFER_SIZE];
extern unsigned int uart_index ;
extern char command[BUFFER_SIZE];  // copie finală la ENTER


// DOAR declarații:
extern float fir_coeffs_2[3];
extern float iir_a_2[3];
extern float iir_b_2[3];
extern float x[3];
extern float y[3];
extern float b[3];
extern float a[3];

extern uint32_t numar;
extern volatile int TipSemnal;
extern volatile int Frecventa;
extern int voltage;

extern unsigned int countDac;
extern unsigned int FrecventaSemnal;
extern unsigned int uartIndex;
extern char uart_buffer[64];
extern char command[64];
extern int valADC;
extern unsigned int i;
extern volatile int count;
extern volatile unsigned int uartBuffer[20];

extern int count2;
extern volatile unsigned int unknown_isr_vector;

extern volatile uint16_t adc_buffer[1024];
extern volatile uint8_t buffer_index;
extern volatile uint32_t FEsantionare;
extern volatile size_t NumarCaractere;
extern volatile uint8_t bStateDetectie;

extern volatile enum LedState currentState;
extern volatile enum Canal_SAC_DAC PasCanal;

extern unsigned int Defazaj_0;
extern unsigned int Defazaj_90;
extern unsigned int Defazaj_180;
extern unsigned int Defazaj_270;

extern int Numar_Esantioane;
extern unsigned int Sinus[100];
extern unsigned int DinteFierastrau[100];
extern unsigned int rampa[100];



#endif
