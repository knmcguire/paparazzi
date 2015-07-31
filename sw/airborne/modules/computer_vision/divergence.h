/*
 * divergence.h
 *
 *  Created on: Apr 28, 2015
 *      Author: knmcguire
 */

#ifndef DIVERGENCE_H_
#define DIVERGENCE_H_


#include "lib/vision/image.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


#define MAX_FLOW 1.0
#define RANSAC 1

#define IMAGE_HEIGHT 240
#define IMAGE_WIDTH 320
//#define EDGE_FLOW
#ifndef MAX_HORIZON
#define MAX_HORIZON 10
#endif

struct edge_hist_t{
	int  horizontal[IMAGE_WIDTH];
    int  vertical[IMAGE_HEIGHT];
};

struct edge_flow_t{
	float horizontal[2];
	float vertical[2];
};

struct displacement_t{
	int  horizontal[IMAGE_WIDTH];
	int  vertical[IMAGE_HEIGHT];
};


int calculate_edge_flow(struct image_t *in,struct image_t* out, struct displacement_t* displacement,struct edge_flow_t* edge_flow, struct edge_hist_t* edge_hist,int front,int rear,int windowsize,int max_distance,int edge_threshold,uint16_t image_width,uint16_t image_height);
void visualize_divergence(struct image_t* in,struct image_t* out,struct displacement_t* displacement,struct edge_hist_t* edge_hist,int front,int rear,float Slope, float Yint,uint16_t image_width, uint16_t image_height,char plot_value);
void visualize_divergence_debug(struct image_t* in,struct image_t* out,struct displacement_t* displacement,int * edge_histogram,int * edge_histogram_prev,int front,int rear,float Slope, float Yint,uint16_t image_width, uint16_t image_height,char plot_value);
void image_draw_color_line(struct image_t *img, struct point_t *from, struct point_t *to, uint8_t color);
int calculate_edge_histogram(struct image_t * in,struct image_t * out,int * edge_histogram, int edge_threshold,int image_width,int image_height,char direction);
int calculate_displacement(int * edge_histogram,int * edge_histogram_prev,int * displacement,int prev_frame_number,int windowsize,int max_distance,int size);
int getMinimum(int* flow_error, int max_ind);
int getMinimumMiddle(int * flow_error, int  max_ind);
int getMaximum(int a[], int n);
int GetMedian(int* daArray, int iSize);
int GetMean(int* daArray, int iSize);
void line_fit_RANSAC( int* displacement, float* Slope, float* Yint,int size);
void line_fit(int* displacement, float* Slope, float* Yint,int size);
void blur_filter(struct image_t *in,struct image_t *out,int Gsize,int sigma);
float simpleKalmanFilter(float* cov,float previous_est, float current_meas,float Q,float R);
#endif /* DIVERGENCE_H_ */
