
#include "lib/vision/image.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>




void sobel_edge_filter(struct image_t *input,struct image_t *output,uint32_t* edge_histogram);
void calculate_edge_histogram_displacement(uint32_t* edge_histogram,uint32_t* edge_histogram_prev,int32_t* displacement,uint16_t image_width,uint16_t image_height);
void line_fit(int32_t* displacement, float* Slope, float* Yint,uint16_t image_width);
void visualize_divergence(struct image_t* in,struct image_t* out,int32_t* displacement,float Slope, float Yint,uint16_t image_width, uint16_t image_height);
uint32_t getMinimum(uint32_t* flow_error, uint32_t max_ind);
