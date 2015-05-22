
#include "lib/vision/image.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


struct edge_hist_t {
uint32_t* horizontal;
uint32_t* vertical;
};

void sobel_edge_filter(struct image_t *input,struct image_t *output,struct edge_hist_t* edge_histogram);
void blur_filter(struct image_t *input,struct image_t *output);
void calculate_edge_histogram_displacement(struct edge_hist_t edge_histogram[],uint8_t previous_frame_number,int32_t* displacement,uint16_t image_width,uint16_t image_height);
void line_fit(int32_t* displacement, float* Slope, float* Yint,uint16_t image_width);
void visualize_divergence(struct image_t* in,struct image_t* out,uint32_t* edge_histogram,uint32_t* edge_histogram_prev,int32_t* displacement,float Slope, float Yint,uint16_t image_width, uint16_t image_height);
void image_draw_color_line(struct image_t *img, struct point_t *from, struct point_t *to, uint8_t color);

uint32_t getMinimum(uint32_t* flow_error, uint32_t max_ind);
