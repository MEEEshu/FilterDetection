#ifndef PROJECT_GENERIC_FUNCTIONS_H_
#define PROJECT_GENERIC_FUNCTIONS_H_

#include <stdint.h>
#include <math.h>
#include <stddef.h>



void vUART_init(void);

size_t u32TrimiteMesaj(const char *str);

void vIntToString(unsigned int val, char *str);

int charToInt(char c);

void vInit_GPIO();

void vCollectADCData();

void vFFT(float *real, float *imag, int n);

void vDoCalculateFFTParameters(float *real, float *imag, int n, float * FrecventaEsantionare);

void vTrimiteDateleCatrePC(float frecventa, float amplitudine);

void vResetPlaca(void);

void u32Esantionare(uint32_t * FrecventaEsantionare);

const char* cDetectieTipFiltru(float *real, float *imag, int n, int sampleRate);

void f32Aplicare_IIR(float input);

void f32Aplicare_FIR(float input);

void clearTerminalAndDisplayMessages(void);

void send_UART_Filter(float val1, float val2);


#endif /* PROJECT_GENERIC_FUNCTIONS_H_ */
