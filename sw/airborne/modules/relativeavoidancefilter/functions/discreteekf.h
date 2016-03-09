#ifndef DISCRETEEKF_H
#define DISCRETEEKF_H

struct ekf_filter;

/*
 * Basic functions describing evolution and measure
 */

typedef void (*filter_function)(float*, float*, float*, float*, float*);
typedef void (*measure_function)(float*, float*, float*);

extern struct ekf_filter* ekf_filter_new( 
					unsigned state_dim,
					unsigned measure_dim,
					float* Q,
					float* R,
					filter_function ffun,
					measure_function mfun);
extern void ekf_filter_reset(struct ekf_filter *filter, float *x0, float *P0);

extern void ekf_filter_predict(struct ekf_filter *filter, float *u);
extern void ekf_filter_update(struct ekf_filter *filter, float *y);

extern void ekf_filter_get_state(struct ekf_filter* filter, float *X, float* P);

#endif /* EKF_H */