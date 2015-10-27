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
#define DISP_RANGE_MAX 20
#define WINDOW 9


struct edge_hist_t {
  int32_t horizontal[IMAGE_WIDTH];
  int32_t vertical[IMAGE_HEIGHT];
};

struct edge_flow_t {
  int32_t horizontal_flow;
  int32_t horizontal_div;
  int32_t vertical_flow;
  int32_t vertical_div;
};

struct displacement_t {
  int32_t horizontal[IMAGE_WIDTH];
  int32_t vertical[IMAGE_HEIGHT];
};

struct covariance_t {
  int32_t flow_x;
  int32_t flow_y;
  int32_t div_x;
  int32_t div_y;
};



void divergence_init();

int calculate_edge_flow(struct image_t *in, struct displacement_t *displacement, struct edge_flow_t *edge_flow,
                            struct edge_hist_t edge_hist[], int32_t *avg_disp, uint8_t previous_frame_offset[],
                            uint8_t current_frame_nr, uint8_t window_size, uint8_t disp_range, uint16_t edge_threshold,
                            uint16_t image_width, uint16_t image_height, uint16_t RES);
void calculate_edge_histogram(struct image_t *in, int32_t *edge_histogram, uint16_t image_width, uint16_t image_height,
                              char direction, char side, uint16_t edge_threshold);
void calculate_displacement(int32_t *edge_histogram, int32_t *edge_histogram_prev, int32_t *displacement,
                            uint16_t size, uint8_t window, uint8_t disp_range);
int32_t calculate_displacement_fullimage(int32_t *edge_histogram, int32_t *edge_histogram_2, uint16_t size, uint8_t disp_range);

void line_fit(int32_t *displacement, int32_t *Slope, int32_t *Yint, uint32_t image_width, uint32_t border, uint16_t RES);
void line_fit_RANSAC(int32_t *displacement, int32_t *slope, int32_t *yInt, uint16_t size, uint32_t border, uint32_t RES);
int ipow(int base, int exp);
void totalKalmanFilter(struct covariance_t *coveriance, struct edge_flow_t *prev_edge_flow,
                       struct edge_flow_t *edge_flow, uint32_t Q, uint32_t R, uint32_t RES);
int32_t simpleKalmanFilter(int32_t *cov, int32_t previous_est, int32_t current_meas, int32_t Q, int32_t R, int32_t RES);

void visualize_divergence(uint8_t *in, int32_t *displacement, int32_t slope, int32_t yInt, uint32_t image_width,
                          uint32_t image_height);
#endif /* DIVERGENCE_H_ */
