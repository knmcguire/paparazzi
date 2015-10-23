/*
 * divergence.cpp
 *
 *  Created on: Apr 28, 2015
 *      Author: knmcguire
 */
#include "divergence.h"




int calculate_edge_flow(struct image_t *in, struct displacement_t *displacement, struct edge_flow_t *edge_flow,
                            struct edge_hist_t edge_hist[], int32_t *avg_disp, uint8_t previous_frame_offset[],
                            uint8_t current_frame_nr, uint8_t window_size, uint8_t disp_range, uint16_t edge_threshold,
                            uint16_t image_width, uint16_t image_height, uint16_t RES)
{
	// check that inputs within allowable ranges
	  if (disp_range > DISP_RANGE_MAX)
	    disp_range = DISP_RANGE_MAX;

	  // Define arrays and pointers for edge histogram and displacements
	  int32_t *edge_histogram_x = edge_hist[current_frame_nr].horizontal;
	  int32_t *prev_edge_histogram_x;
	  int32_t edge_histogram_x_right[IMAGE_WIDTH];

	  int32_t *edge_histogram_y = edge_hist[current_frame_nr].vertical;
	  int32_t *prev_edge_histogram_y;


	  // TODO confirm below
	  if (MAX_HORIZON > 1) {
	    uint32_t flow_mag_x, flow_mag_y;
	    flow_mag_x = abs(edge_flow->horizontal_flow);
	    flow_mag_y = abs(edge_flow->vertical_flow);

	    // TODO check which image we should pick
	    // TODO I think you should switch when you go over the RES / flow_mag_x/(disparity_range/2) boundary
	    if (4*flow_mag_x * (MAX_HORIZON - 1) > RES*disp_range) {
	      previous_frame_offset[0] = (RES*disp_range) / (4*flow_mag_x) + 1;
	    } else {
	      previous_frame_offset[0] = MAX_HORIZON - 1;
	    }

	    if (4*flow_mag_y * (MAX_HORIZON - 1) > RES*disp_range) {
	      previous_frame_offset[1] = (RES*disp_range)/ (4*flow_mag_y) + 1;
	    } else {
	      previous_frame_offset[1] = MAX_HORIZON - 1;
	    }
	  }


	  // the previous frame number relative to dynamic parameters
	  uint8_t previous_frame_x = (current_frame_nr - previous_frame_offset[0] + MAX_HORIZON) %
	                             MAX_HORIZON; // wrap index
	  uint8_t previous_frame_y = (current_frame_nr - previous_frame_offset[1] + MAX_HORIZON) %
	                             MAX_HORIZON; // wrap index

	  // copy previous edge histogram based on previous frame number
	  prev_edge_histogram_x = edge_hist[previous_frame_x].horizontal;
	  prev_edge_histogram_y = edge_hist[previous_frame_y].vertical;


	  // Calculate Edge Histogram
	  calculate_edge_histogram(in, edge_histogram_x, image_width, image_height, 'x', 'l', edge_threshold);
	  calculate_edge_histogram(in, edge_histogram_y, image_width, image_height, 'y', 'l', edge_threshold);
	  //if stereoboard is used
	  //calculate_edge_histogram(in, edge_histogram_x_right, image_width, image_height, 'x', 'r', edge_threshold);

	  // Calculate displacement
	  calculate_displacement(edge_histogram_x, prev_edge_histogram_x, displacement->horizontal, image_width, window_size,
	                         disp_range);
	  calculate_displacement(edge_histogram_y, prev_edge_histogram_y, displacement->vertical, image_height, window_size,
	                         disp_range);
	  //*avg_disp = calculate_displacement_fullimage(edge_histogram_x, edge_histogram_x_right, image_width, disp_range);



	  // Fit a linear line
	/*#ifdef RANSAC
	  line_fit_RANSAC(displacement->horizontal, &edge_flow->horizontal_div, &edge_flow->horizontal_flow, image_width, window_size + disp_range, RES);
	  line_fit_RANSAC(displacement->vertical, &edge_flow->vertical_div, &edge_flow->vertical_flow, image_height, window_size + disp_range, RES);
	#else*/
	  line_fit(displacement->horizontal, &edge_flow->horizontal_div, &edge_flow->horizontal_flow, image_width, window_size + disp_range, RES);
	  line_fit(displacement->vertical, &edge_flow->vertical_div, &edge_flow->vertical_flow, image_height, window_size + disp_range,RES);
	//#endif



	  // Correct Divergence slope and translation by the amount of frames skipped
	  edge_flow->horizontal_flow  /= previous_frame_offset[0];
	  edge_flow->horizontal_div   /= previous_frame_offset[0];
	  edge_flow->vertical_flow    /= previous_frame_offset[1];
	  edge_flow->vertical_div     /= previous_frame_offset[1];



/* todo check if adaptive thresolding is nesaserry
#if ADAPTIVE_THRES_EDGE

    //blur_filter(in,out,3,1);

    int mean_x=GetMean(prev_edge_histogram_x_p,image_width);
    int mean_y=GetMean(prev_edge_histogram_y_p,image_height);

    int max_ind_x=getMaximum(prev_edge_histogram_x_p,image_width);
    int max_ind_y=getMaximum(prev_edge_histogram_y_p,image_height);
    int max_x=prev_edge_histogram_x_p[max_ind_x];
    int max_y=prev_edge_histogram_y_p[max_ind_y];


    int edge_thres_x=mean_x+(max_x+mean_x)/edge_threshold;
    int edge_thres_y=mean_x+(max_y+mean_y)/edge_threshold;
;
#else
    int edge_thres_x=0;
    int edge_thres_y=0;
#endif

    //Calculculate current edge_histogram
    calculate_edge_histogram(in,out,edge_histogram_x_p,edge_thres_x,image_width,image_height,'x');
    calculate_edge_histogram(in,out,edge_histogram_y_p,edge_thres_y,image_width,image_height,'y');
*/

    int median_x=GetMedian(edge_histogram_x,image_width);
    int median_y=GetMedian(edge_histogram_y,image_height);

/*
    //calculate displacement based on histogram
    int sum_disx=calculate_displacement(edge_histogram_x_p,prev_edge_histogram_x_p,displacement->horizontal,previous_frame_number[0],windowsize,max_distance,image_width);
    int sum_disy=calculate_displacement(edge_histogram_y_p,prev_edge_histogram_y_p,displacement->vertical,previous_frame_number[1],windowsize,max_distance,image_height);



    //Line fit of the displacement by least square estimation or Ransac
#ifdef RANSAC
    line_fit_RANSAC(displacement->horizontal, &slope_x,&trans_x,image_width);
    line_fit_RANSAC(displacement->vertical, &slope_y,&trans_y,image_height);

#else
    line_fit(displacement->horizontal, &slope_x,&trans_x,image_width);
    line_fit(displacement->vertical, &slope_y,&trans_y,image_height);
#endif
*/

    return (median_x+median_y)/2;
}

void calculate_edge_histogram(struct image_t *in, int32_t *edge_histogram, uint16_t image_width, uint16_t image_height,
                              char direction, char side, uint16_t edge_threshold)
{
	 uint8_t *in_buf = (uint8_t *)in->buf;
  // TODO use arm_conv_q31()
  int32_t sobel_sum = 0;
  int32_t Sobel[3] = { -1, 0, 1};

  uint32_t y = 0, x = 0;
  int32_t c = 0;

  uint32_t idx = 0;

  // set pixel offset based on which image needed from interlaced image
  uint32_t px_offset = 0;
  if (side == 'l')
    px_offset = 0;
  else if (side == 'r')
    px_offset = 1;
  else
    while(1); // let user know something is wrong

  // compute edge histogram
  if (direction == 'x') {

    // set values that are not visited
    edge_histogram[0] = edge_histogram[image_width-1] = 0;
    for (x = 1; x < image_width-1; x++) {
      edge_histogram[x] = 0;
      for (y = 0; y < image_height; y++) {
        sobel_sum = 0;

        for (c = -1; c <= 1; c++) {
          idx = 2*(image_width * y + (x + c));  // 2 for interlace

          sobel_sum += Sobel[c + 1] * (int32_t)in_buf[idx + px_offset];

        }
        sobel_sum = abs(sobel_sum);
        if (sobel_sum > edge_threshold)
          edge_histogram[x] += sobel_sum;


      }
    }
  }
  else if (direction == 'y'){
    // set values that are not visited
    edge_histogram[0] = edge_histogram[image_height-1] = 0;
    for (y = 1; y < image_height-1; y++) {
      edge_histogram[y] = 0;
      for (x = 0; x < image_width; x++) {
        sobel_sum = 0;

        for (c = -1; c <= 1; c++) {
          idx = 2*(image_width * (y + c) + x);  // 2 for interlace

          sobel_sum += Sobel[c + 1] * (int32_t)in_buf[idx + px_offset];
        }
        sobel_sum = abs(sobel_sum);
        if (sobel_sum > edge_threshold)
          edge_histogram[y] += sobel_sum;
      }
    }
  }
  else
    while(1);   // hang to show user something isn't right

}

// Calculate_displacement calculates the displacement between two histograms
// D should be half the search disparity range
// W is local search window
void calculate_displacement(int32_t *edge_histogram, int32_t *edge_histogram_prev, int32_t *displacement, uint16_t size,
                            uint8_t window, uint8_t disp_range)
{
  int32_t c = 0,r = 0;
  uint32_t x = 0;
  uint32_t SAD_temp[2*DISP_RANGE_MAX+1];    // size must be at least 2*D + 1

  int32_t W = window;
  int32_t D = disp_range;

  memset(displacement, 0, size);

  // TODO: replace with arm offset subtract
  for (x = W + D; x < size - W - D; x++) {
    displacement[x] = 0;
    for (c = -D; c <= D; c++) {
      SAD_temp[c + D] = 0;
      for (r = -W; r <= W; r++) {
        SAD_temp[c + D] += abs(edge_histogram[x + r] - edge_histogram_prev[x + r + c]);
      }
    }
    displacement[x] = (int32_t)getMinimum(SAD_temp, 2*D+1) - D;
  }
}

// Calculate_displacement calculates the displacement between two histograms
// D should be disparity range
// TODO: for height should always look to the positive disparity range, can ignore negative
int32_t calculate_displacement_fullimage(int32_t *edge_histogram, int32_t *edge_histogram_2, uint16_t size, uint8_t disp_range)
{
  int32_t c = 0;
  uint32_t x = 0;
  uint32_t SAD_temp[2*DISP_RANGE_MAX+1];    // size must be at least 2*D + 1

  int32_t D = disp_range;

  // TODO check if one loop can be replaced by line diff
  for (c = -D; c <= D; c++) {
    SAD_temp[c + D] = 0;
    for (x = D; x < size - D; x++) {
      SAD_temp[c + D] += abs(edge_histogram[x] - edge_histogram_2[x + c]);
    }
  }

  return D - (int32_t)getMinimum(SAD_temp, 2*D+1);
}


// Line_fit fits a line using least squares to the histogram disparity map
void line_fit(int32_t *displacement, int32_t *divergence, int32_t *flow, uint32_t size, uint32_t border, uint16_t RES)
{
  int32_t x;

  int32_t count = 0;
  int32_t sumY = 0;
  int32_t sumX = 0;
  int32_t sumX2 = 0;
  int32_t sumXY = 0;
  int32_t xMean = 0;
  int32_t yMean = 0;

  *divergence = 0;
  *flow = 0;

  // compute fixed sums
  int32_t xend = size - border - 1;
  sumX = xend*(xend+1)/2 - border*(border+1)/2 + border;
  sumX2 = xend * (xend+1) * (2*xend+1) / 6;
  xMean = (size-1) / 2;
  count = size - 2*border;

  for (x = border; x < size - border; x++) {
    sumY += displacement[x];
    sumXY += x * displacement[x];
  }

  yMean = RES * sumY / count;

  *divergence = (RES * sumXY - sumX * yMean) / (sumX2 - sumX * xMean);    // compute slope of line ax + b
  *flow = yMean - *divergence * xMean;  // compute b (or y) intercept of line ax + b
}



void line_fit_RANSAC(int32_t *displacement, int32_t *divergence, int32_t *flow, uint16_t size, uint32_t border, uint32_t RES)
{
  //Fit a linear line with RANSAC (from Guido's code)
  uint8_t ransac_iter = 20;
  int32_t it;
  uint32_t ind1, ind2, tmp, entry;
  uint32_t total_error, best_ind;
  int32_t dx, dflow, predicted_flow;
  // flow = a * x + b
  int32_t a[ransac_iter];
  int32_t b[ransac_iter];
  uint32_t errors[ransac_iter];

  uint16_t entries = size - 2*border;

  for (it = 0; it < ransac_iter; it++) {
    ind1 = rand() % entries + border;
    ind2 = rand() % entries + border;

    while (ind1 == ind2) {
      ind2 = rand() % entries + border;
    }

    // TODO: is this really necessary?
    if (ind1 > ind2) {
      tmp = ind2;
      ind2 = ind1;
      ind1 = tmp;
    }

    dx = ind2 - ind1;   // never zero
    dflow = displacement[ind2] - displacement[ind1];

    // Fit line with two points
    a[it] = RES * dflow / dx;
    b[it] = RES * displacement[ind1] - (a[it] * ind1);
    // evaluate fit:
    total_error = 0;
    for (entry = border; entry < size-border; entry++) {
      predicted_flow = a[it] * entry + b[it];
      total_error += ipow(RES*displacement[entry] - predicted_flow,2);
    }
    errors[it] = total_error;
  }
  // select best fit:
  best_ind = getMinimum(errors, ransac_iter);

  *divergence = a[best_ind];
  *flow = b[best_ind];
}


/* todo fix visualization
 void visualize_divergence(struct image_t* in,struct image_t* out,struct displacement_t* displacement,struct edge_hist_t* edge_hist,int front,int rear,float Slope, float Yint,uint16_t image_width, uint16_t image_height,char plot_value)
{
    uint32_t x;
    image_copy(in,out);

    struct point_t  linedraw;
    struct point_t linedraw_prev;


    for( x = 0; x < image_width-1; x++)
    {


        switch(plot_value){

        case 'e':

            linedraw.y =(uint16_t)(  edge_hist[front].horizontal[x+1]/40+image_height/2);
            linedraw.x=(uint16_t)x+1;

            linedraw_prev.y =(uint16_t)( edge_hist[front].horizontal[x]/40+image_height/2);
            linedraw_prev.x=(uint16_t)x;

            image_draw_color_line(out, &linedraw_prev,&linedraw,255);

            linedraw.y =(uint16_t)( edge_hist[rear].horizontal[x+1]/40+image_height/2);
            linedraw.x=(uint16_t)x+1;

            linedraw_prev.y =(uint16_t)( edge_hist[rear].horizontal[x]/40+image_height/2);
            linedraw_prev.x=(uint16_t)x;

            image_draw_color_line(out, &linedraw_prev,&linedraw,100);

            break;
        case 'd':
            linedraw.y =(uint16_t)(displacement->horizontal[x+1]+image_height/2);
            linedraw.x=(uint16_t)x+1;

            linedraw_prev.y =(uint16_t)(displacement->vertical[x]+image_height/2);
            linedraw_prev.x=(uint16_t)x;
            image_draw_color_line(out, &linedraw_prev,&linedraw,1);
            break;
        case 'l':
            linedraw.y =(uint16_t)(Slope*(x+1)+Yint+image_height/2);
            linedraw.x=(uint16_t)x+1;

            linedraw_prev.y =(uint16_t)(Slope*(x)+Yint+image_height/2);
            linedraw_prev.x=(uint16_t)x;
            image_draw_color_line(out, &linedraw_prev,&linedraw,1);

            break;
        default:
            break;

        }
    }

}


void visualize_divergence_debug(struct image_t* in,struct image_t* out,struct displacement_t* displacement,int * edge_histogram,int * edge_histogram_prev,int front,int rear,float Slope, float Yint,uint16_t image_width, uint16_t image_height,char plot_value)
{
    uint32_t x;
    image_copy(in,out);

    struct point_t  linedraw;
    struct point_t linedraw_prev;

    for( x = 0; x < image_width-1; x++)
    {


        switch(plot_value){

        case 'e':

            linedraw.y =(uint16_t)( edge_histogram[x+1]/40+image_height/2);
            linedraw.x=(uint16_t)x+1;

            linedraw_prev.y =(uint16_t)( edge_histogram[x]/40+image_height/2);
            linedraw_prev.x=(uint16_t)x;

            image_draw_color_line(out, &linedraw_prev,&linedraw,255);

            linedraw.y =(uint16_t)( edge_histogram_prev[x+1]/40+image_height/2);
            linedraw.x=(uint16_t)x+1;

            linedraw_prev.y =(uint16_t)(  edge_histogram_prev[x]/40+image_height/2);
            linedraw_prev.x=(uint16_t)x;

            image_draw_color_line(out, &linedraw_prev,&linedraw,100);

            break;
        case 'd':
            linedraw.y =(uint16_t)(displacement->horizontal[x+1]+image_height/2);
            linedraw.x=(uint16_t)x+1;

            linedraw_prev.y =(uint16_t)(displacement->horizontal[x]+image_height/2);
            linedraw_prev.x=(uint16_t)x;
            image_draw_color_line(out, &linedraw_prev,&linedraw,1);
            break;
        case 'l':
            linedraw.y =(uint16_t)(Slope*(x+1)+Yint+image_height/2);
            linedraw.x=(uint16_t)x+1;

            linedraw_prev.y =(uint16_t)(Slope*(x)+Yint+image_height/2);
            linedraw_prev.x=(uint16_t)x;
            image_draw_color_line(out, &linedraw_prev,&linedraw,1);

            break;
        default:
            break;

        }
    }

}*/

void image_draw_color_line(struct image_t *img, struct point_t *from, struct point_t *to, uint8_t color)
{
    int xerr = 0, yerr = 0;
    uint8_t *img_buf = (uint8_t *)img->buf;
    uint8_t pixel_width = (img->type == IMAGE_YUV422) ? 2 : 1;
    uint16_t startx = from->x;
    uint16_t starty = from->y;

    /* compute the distances in both directions */
    int32_t delta_x = to->x - from->x;
    int32_t delta_y = to->y - from->y;

    /* Compute the direction of the increment,
     an increment of 0 means either a horizontal or vertical
     line.
  */
    int8_t incx, incy;
    if (delta_x > 0) { incx = 1; }
    else if (delta_x == 0) { incx = 0; }
    else { incx = -1; }

    if (delta_y > 0) { incy = 1; }
    else if (delta_y == 0) { incy = 0; }
    else { incy = -1; }

    /* determine which distance is greater */
    uint16_t distance = 0;
    delta_x = abs(delta_x);
    delta_y = abs(delta_y);
    if (delta_x > delta_y) { distance = delta_x * 20; }
    else { distance = delta_y * 20; }

    /* draw the line */
    for (uint16_t t = 0; starty >= 0 && starty < img->h && startx >= 0 && startx < img->w && t <= distance + 1; t++) {
        img_buf[img->w * pixel_width * starty + startx * pixel_width] = (t <= 3) ? 0 :  color;

        if (img->type == IMAGE_YUV422) {
            img_buf[img->w * pixel_width * starty + startx * pixel_width + 1] = 255;

            if (startx + 1 < img->w) {
                img_buf[img->w * pixel_width * starty + startx * pixel_width + 2] = (t <= 3) ? 0 :  color;
                img_buf[img->w * pixel_width * starty + startx * pixel_width + 3] = 255;
            }
        }

        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            startx += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            starty += incy;
        }
    }
}

int getMinimum(int * flow_error, int  max_ind)
{
    uint32_t i;
    uint32_t min_ind = 0;
    uint32_t min_err = flow_error[0];
    for(i = 1; i < max_ind; i++)
    {
        if(flow_error[i] < min_err)
        {
            min_ind = i;
            min_err = flow_error[i];
        }
    }
    return min_ind;
}
int getMinimumMiddle(int * flow_error, int  max_ind)
{
    uint32_t i;
    uint32_t min_ind_left = max_ind/2;
    uint32_t min_ind_right = max_ind/2;

    uint32_t min_err_left = flow_error[max_ind/2];
    uint32_t min_err_right = flow_error[max_ind/2];

    for(i = max_ind/2; i >0; i--)
    {

        if(flow_error[i] < min_err_left)
        {
            min_ind_left = i;
            min_err_left = flow_error[i];
        }
    }
    for(i = max_ind/2; i < max_ind; i++)
    {
        if(flow_error[i] < min_err_right)
        {
            min_ind_right = i;
            min_err_right = flow_error[i];
        }
    }
    if(min_err_left>min_err_right)
        return min_ind_right;
    else
        return min_ind_left;



}

int getMaximum(int a[], int n) {
    int c, max, index;

    max = a[0];
    index = 0;

    for (c = 1; c < n; c++) {
        if (a[c] > max) {
            index = c;
            max = a[c];
        }
    }

    return index;
}

int GetMedian(int* daArray, int iSize) {
    // Allocate an array of the same size and sort it.
    int dpSorted[iSize];
    for (int i = 0; i < iSize; ++i) {
        dpSorted[i] = daArray[i];
    }
    for (int i = iSize - 1; i > 0; --i) {
        for (int j = 0; j < i; ++j) {
            if (dpSorted[j] > dpSorted[j+1]) {
                int dTemp = dpSorted[j];
                dpSorted[j] = dpSorted[j+1];
                dpSorted[j+1] = dTemp;
            }
        }
    }

    // Middle or average of middle values in the sorted array.
    int dMedian = 0;
    if ((iSize % 2) == 0) {
        dMedian = (dpSorted[iSize/2] + dpSorted[(iSize/2) - 1])/2.0;
    } else {
        dMedian = dpSorted[iSize/2];
    }
    return dMedian;
}

int GetMean(int* daArray, int iSize) {
    int dSum = daArray[0];
    for (int i = 1; i < iSize; ++i) {
        dSum += daArray[i];
    }
    return dSum/iSize;
}

int ipow(int base, int exp)
{
    int result = 1;
    while (exp)
    {
        if (exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }

    return result;
}

void blur_filter(struct image_t *in,struct image_t *out,int Gsize,int sigma)
{



    double  Gaussian[Gsize][Gsize];
    int G_hsize=(Gsize-1)/2;
    double radius;
    // sum is for normalization
    double sum = 0.0;
    // generate  kernel
    for (int k = -G_hsize; k <= G_hsize; k++)
    {
        for(int m = -G_hsize; m <= G_hsize; m++)
        {

            radius = sqrt(k*k + m*m);
            Gaussian[k + G_hsize][m + G_hsize] = exp(-(radius*radius)/(2*sigma*sigma));
            sum += Gaussian[k + G_hsize][m + G_hsize];
        }    //printf("%d",(size-1)/2);

    }

    // normalize the Kernel
    for(int i = 0; i < Gsize; ++i){
        for(int j = 0; j < Gsize; ++j){
            Gaussian[i][j] /= sum;
        }
    }
    //double  Gaussian[5][5] = {{0.0232,0.0338,0.0383,0.0338,0.0232},{0.0338 ,0.0492,0.0558 ,0.0492,0.0338},{0.0383 ,0.0558 ,0.0632,0.0558 ,0.0383},{0.0338,0.0492,0.0558,0.0492,0.0338}, {0.0232,0.0338,0.0383,0.0338 ,0.0232}};
    int8_t r, c;
    uint32_t  gaussian;
    uint8_t *source = (uint8_t *)in->buf;
    uint8_t *dest = (uint8_t *)out->buf;


    for(uint16_t y = 0; y < in->h; y++) {
        for(uint16_t x = 0; x < in->w; x++) {
            uint32_t idx = in->w*y*2 + (x)*2;

            //Convolution
            if(y>G_hsize&&y<in->h-G_hsize&&x>G_hsize&&x<in->w-G_hsize)
            {
                gaussian=0;
                for(r = -G_hsize; r <=G_hsize; r++)
                {
                    for(c = -G_hsize; c <= G_hsize; c++)
                    {
                        uint32_t idx_filter = in->w*(y+r)*2 + (x+c)*2;
                        gaussian += (uint32_t)(Gaussian[r+G_hsize][c+G_hsize] * (source[idx_filter+1]));
                    }
                }
                gaussian=abs(gaussian);
                dest[idx+1] = gaussian;//abs(-1*source[idx_left+1]+source[idx_right+1]);
                dest[idx]=127;
            }else{
                dest[idx]=127;

            }

        }
    }
}

int32_t simpleKalmanFilter(int32_t *cov, int32_t previous_est, int32_t current_meas, int32_t Q, int32_t R, int32_t RES)
{
  int32_t predict_cov = *cov + Q;
  int32_t K = RES * predict_cov / (*cov + R);

  *cov = ((RES - K) * predict_cov) / RES;

  return (previous_est + (K * (current_meas - previous_est)) / RES);
}



