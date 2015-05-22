#include "edgefilter.h"
#include "lib/vision/image.h"

#define MAX_HORIZON_FLOW 5
#define MAX_FLOW 5

void edge_flow(struct image_t *input,struct edge_hist_t* edge_histogram,float* Slope, float* Yint)
{
    int32_t displacement[input->w];
    uint8_t previous_frame_number[2];

    //ADAPTIVE FLOW
    if(abs(Yint)<3.0){
        previous_frame_number[1]=(uint8_t)((MAX_HORIZON_FLOW-1)*(MAX_FLOW-abs(Yint[1]))/MAX_FLOW)+1;
        previous_frame_number[2]=(uint8_t)((MAX_HORIZON_FLOW-1)*(MAX_FLOW-abs(Yint[2]))/MAX_FLOW)+1;}

    else{
        previous_frame_number[1]=1;
        previous_frame_number[2]=1;}

    //Create Edge_filter
   // sobel_edge_filter(&img_small, &img_sobel,&edge_hist[0]);

   // calculate_edge_histogram_displacement(edge_hist,previous_frame_number,&displacement,input->w,input->h);


}

void sobel_edge_filter(struct image_t *input,struct image_t *output,struct edge_hist_t* edge_histogram)
{
    //Function to calulate the image gradient and create an ege histogram
    uint32_t  Sobel[3] = {-1, 0, 1};
    int8_t r, c;
    uint32_t  sobel_hor=0;
    uint32_t  sobel_ver=0;
    uint8_t *source = (uint8_t *)input->buf;
    uint8_t *dest = (uint8_t *)output->buf;
    uint32_t* edge_histogram_hor=(uint32_t*) edge_histogram->horizontal;
    uint32_t* edge_histogram_ver=(uint32_t*) edge_histogram->vertical;

    uint32_t edge_histogram_temp_hor=0;
    uint32_t edge_histogram_temp_ver=0;


    for(uint16_t j=0;j<input->w;j++)
        edge_histogram_hor[j]=0;

    for(uint16_t x = 1; x < input->w-1; x++) {
        for(uint16_t y = 1; y < input->h-1; y++) {

            uint32_t idx = input->w*y*2 + (x)*2;
            sobel_hor=0;
            sobel_ver=0;

            //Convolution of the box filter
            if(y>1&&y<input->h-1&&x>1&&x<input->w-1)
            {
                //horizontal
                for(c = -1; c <= 1; c++)
                {
                    uint32_t idx_filter = input->w*(y)*2 + (x+c)*2;
                    sobel_hor += Sobel[c+1] * source[idx_filter+1];
                }
                //vertical
                for(r = -1; r <= 1; r++)
                {
                    uint32_t idx_filter = input->w*(y+r)*2 + (x)*2;
                    sobel_ver += Sobel[r+1] * source[idx_filter+1];
                }

            }

            sobel_hor=abs(sobel_hor);
            sobel_ver=abs(sobel_ver);

            //making image for horizontal edges
            dest[idx+1]=sobel_hor;
            dest[idx]=127;

            edge_histogram_temp_hor += (uint32_t)sobel_hor;
            edge_histogram_temp_ver += (uint32_t)sobel_ver;


        }

        //edge_histogram_vor
        edge_histogram_hor[x]=(uint32_t)edge_histogram_temp_hor;
        edge_histogram_ver[x]=(uint32_t)edge_histogram_temp_ver;

        edge_histogram_temp_hor=0;
        edge_histogram_temp_ver=0;

    }
}


void calculate_edge_histogram_displacement(struct edge_hist_t edge_histogram[],int8_t previous_frame_number, int8_t dynamic_idx,int32_t* displacement,uint16_t image_width,uint16_t image_height)
{
    //Calculating the edge histogram displacement with SAD
    int32_t  c,r;
    uint16_t i,y,x,flow_ind;
    int32_t W=10;
    int32_t D=10;
    uint32_t SAD_temp[20];
    uint16_t edge_histogram_tot=0;
    uint32_t min_index=0;
    uint32_t SAD_tot=0;

    uint32_t* edge_histogram_hor= edge_histogram[0].horizontal;
    uint32_t* edge_histogram_prev_hor=edge_histogram[previous_frame_number].horizontal;


    for(x=0; x<image_width;x++)
    {
        SAD_tot=0;
        if(x>=W+D&&x<=image_width-W-D)
        {
            min_index=0;
            for(c=-D;c<D;c++)
            {
                //Sum of Absolute differences
                SAD_temp[D+c]=0;
                for(r=-W;r<W;r++){

                    SAD_temp[c+D]+=abs(edge_histogram_hor[x+r]-edge_histogram_prev_hor[x+r+c]);
                }
                SAD_tot+=SAD_temp[D+c];
            }
            // The index with the minimum sum is the best fit
            min_index=getMinimum(SAD_temp,20);
            if(SAD_tot>10000)
                displacement[x]=(int32_t)((min_index-W));
            else
                displacement[x]=0;
        }else{
            displacement[x]=0;
        }
    }

}

void line_fit(int32_t* displacement, float* Slope, float* Yint,uint16_t image_width)
{

    //Fitting a linear line over the displacement(flow) vector
    int16_t x;

    uint32_t Count=0;
    float SumY=0;
    float  SumX=0;
    float SumX2=0;
    float SumXY=0;
    float XMean=0;
    float YMean=0;
    float Slope_temp,Yint_temp;


    for(x=0;x<image_width;x++){

        if(displacement[x]!=0){

            SumX+=x;
            SumY+=displacement[x];

            SumX2+=x *x;
            SumXY+=x *displacement[x];
            Count++;
        }
    }

    XMean=SumX/(float)Count;
    YMean=SumY/(float)Count;

    Slope_temp=((float)(SumXY-SumX*YMean)/(float)(SumX2-SumX*XMean));
    Yint_temp= (YMean-Slope_temp* (float)XMean);

    if(Count!=0){
        *Slope=Slope_temp;
        *Yint=Yint_temp;
    }else{
        *Slope=0;
        *Yint=0;}
}

void line_fit_RANSAC( int32_t* displacement, float* Slope, float* Yint,uint16_t size)
{

    //Fit a linear line with RANSAC (from Guido's code)
    uint8_t ransac_iter=20;
    uint8_t it;
    uint16_t k;
    uint32_t ind1, ind2, tmp, entry, total_error, best_ind;
    int32_t dx, dflow, predicted_flow;
    // flow = a * x + b
    float a[ransac_iter];
    float b[ransac_iter];
    float a_temp,b_temp;
    uint32_t errors[ransac_iter];

    uint16_t X[size];

    for(k=0;k<size;k++)
        X[k]=k;

    for(it = 0; it < ransac_iter; it++)
    {

        errors[it] = 0;
        total_error=0;

        ind1 = rand() % size;
        ind2 = rand() % size;

        while(ind1 == ind2)
            ind2 = rand() % size;

        if(X[ind1] > X[ind2])
        {
            tmp = ind2;
            ind2 = ind1;
            ind1 = tmp;
        }


        dx=X[ind2]-X[ind1];
        dflow = displacement[ind2] - displacement[ind1];

    //Fit line with two points

        a[it] = (float)dflow/(float)dx;
        b[it] = (float)displacement[ind1]- (a[it] *(float)(X[ind1] - size/2));

        // evaluate fit:
        for (entry = 0; entry < size; entry++)
        {
            predicted_flow = (int32_t)(a[it] * (float)(X[entry] - size/2) + b[it]);
            total_error += (uint32_t) abs(displacement[entry] - predicted_flow);
        }
        errors[it] = total_error;
    }

    // select best fit:
    best_ind = getMinimum(errors, 20);
    //printf("%d\n",best_ind);
    (*Slope) = a[best_ind] ;
    (*Yint) = b[best_ind];

}


void visualize_divergence(struct image_t* in,struct image_t* out,int32_t* displacement,struct edge_hist_t edge_hist[],float Slope, float Yint,uint16_t image_width, uint16_t image_height)
{
    uint32_t y,x;



    image_copy(in,out);

    struct point_t  linedraw;
    struct point_t linedraw_prev;


    uint32_t* edge_histogram_hor= edge_hist[0].horizontal;
    uint32_t* edge_histogram_prev_hor=edge_hist[1].horizontal;

    for( x = 0; x < image_width-1; x++)
    {

       /* linedraw.y =(uint16_t)(displacement[x+1]+image_height/2);
        linedraw.x=(uint16_t)x+1;

        linedraw_prev.y =(uint16_t)(displacement[x]+image_height/2);
        linedraw_prev.x=(uint16_t)x;*/

       linedraw.y =(uint16_t)(  edge_histogram_hor[x+1]/50+image_height/2);
         linedraw.x=(uint16_t)x+1;

         linedraw_prev.y =(uint16_t)( edge_histogram_hor[x]/50+image_height/2);
         linedraw_prev.x=(uint16_t)x;

        image_draw_color_line(out, &linedraw_prev,&linedraw,1);

        linedraw.y =(uint16_t)( edge_histogram_prev_hor[x+1]/50+image_height/2);
          linedraw.x=(uint16_t)x+1;

          linedraw_prev.y =(uint16_t)( edge_histogram_prev_hor[x]/50+image_height/2);
          linedraw_prev.x=(uint16_t)x;

         image_draw_color_line(out, &linedraw_prev,&linedraw,254);

    }

}

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


uint32_t getMinimum(uint32_t* flow_error, uint32_t max_ind)
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

