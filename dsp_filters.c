#include "dsp_filters.h"

#define MAX_ORDER 4

// FIR
static float fir_coeffs[MAX_ORDER + 1];
static float fir_buffer[MAX_ORDER + 1];
static uint8_t fir_order = 0;
/********************************************************************************/
/* Nume Functie: fir_filter_init      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/* Copiază coeficienții din argument în vectorul intern fir_coeffs[] și resetează bufferul fir_buffer[].         */

void fir_filter_init(uint8_t order, float* coeffs) {
    unsigned int i;
    fir_order = order;
    for (i = 0; i <= order; i++) {
        fir_coeffs[i] = coeffs[i];
        fir_buffer[i] = 0;
    }
}
/* fir_filter_init Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: fir_filter_process      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/* Mută eșantioanele anterioare în buffer          */
/* Adaugă eșantionul nou.      */
/* Calculează ieșirea ca sumă ponderată   */

float fir_filter_process(float input) {
    unsigned int i;
    for (i = fir_order; i > 0; i--) {
        fir_buffer[i] = fir_buffer[i - 1];
    }
    fir_buffer[0] = input;

    float result = 0;
    for (i = 0; i <= fir_order; i++) {
        result += fir_coeffs[i] * fir_buffer[i];
    }
    return result;
}

/* fir_filter_process Terminat*/
/****************************************************************************** */

// IIR (Direct Form I)
static float iir_a[MAX_ORDER + 1]; // a0, a1, ..., aN
static float iir_b[MAX_ORDER + 1]; // b0, b1, ..., bN
static float iir_input[MAX_ORDER + 1];
static float iir_output[MAX_ORDER + 1];
static uint8_t iir_order = 0;

/********************************************************************************/
/* Nume Functie: iir_filter_init      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/* Copiază coeficienții în vectorii interni și resetează bufferul de stare.        */


void iir_filter_init(uint8_t order, float* a_coeffs, float* b_coeffs) {
    unsigned int i;
    iir_order = order;
    for (i = 0; i <= order; i++) {
        iir_a[i] = a_coeffs[i];
        iir_b[i] = b_coeffs[i];
        iir_input[i] = 0;
        iir_output[i] = 0;
    }
}

/* iir_filter_init Terminat*/
/****************************************************************************** */

/********************************************************************************/
/* Nume Functie: iir_filter_process      */
/* Argumente : niciunul         */
/* Valoarea returnata : niciuna */
/* Aplică forma directă si normalizeaza a[i] = 1        */

float iir_filter_process(float input) {
    unsigned int i;
    for (i = iir_order; i > 0; i--) {
        iir_input[i] = iir_input[i - 1];
        iir_output[i] = iir_output[i - 1];
    }
    iir_input[0] = input;

    float result = 0;
    for (i = 0; i <= iir_order; i++) {
        result += iir_b[i] * iir_input[i];
    }
    for (i = 1; i <= iir_order; i++) {
        result -= iir_a[i] * iir_output[i];
    }

    iir_output[0] = result;

    return result;
}

/* iir_filter_process Terminat*/
/****************************************************************************** */

