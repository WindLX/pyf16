#ifndef LOFI_F16_AERODATA_H
#define LOFI_F16_AERODATA_H

/* Damping Aero-Coeffs */
void damping(double alpha, double *coeff);
/* Function for relations to control inputs */
void dmomdcon(double alpha, double beta, double *coeff);
void clcn(double alpha, double beta, double *coeff);
/*Cx and Cm aero-coeffs */
void cxcm(double alpha, double dele, double *coeff);
/* Cz aero-coeff */
void cz(double alpha, double beta, double dele, double *coeff);

#endif // LOFI_F16_AERODATA_H