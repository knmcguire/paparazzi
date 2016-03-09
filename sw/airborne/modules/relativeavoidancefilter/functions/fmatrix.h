#ifndef FMATRIX_H
#define FMATRIX_H

// #include <stdio.h>

extern void fmat_add(int n_row, int n_col, float* r, float* a, float* b);
extern void fmat_sub(int n_row, int n_col, float* r, float* a, float* b);
extern void fmat_transpose(int n_row, int n_col, float* r, float* a);
extern void fmat_scal_mult(int n_row, int n_col, float* r, float k, float* a);
extern void fmat_add_scal_mult(int n_row, int n_col, float* r, float*a, float k, float* b);
extern void fmat_mult(int n_rowa, int n_cola, int n_colb, float* r, float* a, float* b);

// extern void fmat_print(int n_row, int n_col, float* a);
extern void fmat_inverse(int n, float* inv, float* a);


#endif /* MATRIX_H */
