#ifndef DSP_FILTERS_H
#define DSP_FILTERS_H

#include <stdint.h>

// Filtru FIR
void fir_filter_init(uint8_t order, float* coeffs);
float fir_filter_process(float input);

// Filtru IIR
void iir_filter_init(uint8_t order, float* a_coeffs, float* b_coeffs);
float iir_filter_process(float input);

#endif
