#ifndef VARIABILE_GLOBALE_H_
#define VARIABILE_GLOBALE_H_

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* -------------------- Macrodefiniri -------------------- */

#define BUFFER_SIZE             64
#define SIGNAL_LEN              100
#define PI                      3.14159265358979323846
#define MAX_UART_BUFFER         10
#define MARIME_FFT              1024
#define MascaPentru200Kps       0b000
#define MascaPentru50Kps        0b100
#define TRUE                    0x1
#define FALSE                   0x0
#define MARIME_SEMNAL           100

/* -------------------- Tablouri mari -------------------- */

extern volatile uint16_t adc_buffer[1024];
extern unsigned int Sinus[100];
extern unsigned int DinteFierastrau[100];
extern unsigned int rampa[100];
extern volatile unsigned int uartBuffer[20];
extern char uart_buffer[BUFFER_SIZE];
extern char command[BUFFER_SIZE];

/* -------------------- float -------------------- */

extern float fir_coeffs_2[3];
extern float iir_a_2[3];
extern float iir_b_2[3];
extern float x[3];
extern float y[3];
extern float b[3];
extern float a[3];

/* -------------------- int, uint32_t, size_t -------------------- */

extern uint32_t numar;
extern volatile uint32_t FEsantioare;
extern volatile size_t NumarCaractere;

extern int voltage;
extern int valADC;
extern int Numar_Esantioane;
extern int count2;

extern volatile int TipSemnal;
extern volatile int Frecventa;
extern volatile int count;

/* -------------------- unsigned int -------------------- */

extern unsigned int uart_index;
extern unsigned int uartIndex;
extern unsigned int countDac;
extern unsigned int FrecventaSemnal;
extern unsigned int i;

extern unsigned int Defazaj_0;
extern unsigned int Defazaj_90;
extern unsigned int Defazaj_180;
extern unsigned int Defazaj_270;

/* -------------------- Flaguri și control (uint8_t) -------------------- */

extern volatile uint8_t FIR_Activ;
extern volatile uint8_t IIR_Activ;
extern volatile uint8_t buffer_index;
extern volatile uint8_t bStateDetectie;


#endif /* VARIABILE_GLOBALE_H_ */
