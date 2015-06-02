/*
 * divergence.cpp
 *
 *  Created on: Apr 28, 2015
 *      Author: knmcguire
 */
#include "divergence.h"



void calculate_edge_flow(struct image_t *in,struct image_t* out, struct displacement_t* displacement,struct edge_flow_t* edge_flow, struct edge_hist_t* edge_hist,int front,int rear,uint16_t image_width,uint16_t image_height)
{



    //Define arrays and pointers for edge histogram and displacements
    int edge_histogram_x[image_width],prev_edge_histogram_x[image_width];
    int * edge_histogram_x_p=edge_histogram_x;
    int * prev_edge_histogram_x_p=prev_edge_histogram_x;

    int edge_histogram_y[image_height],prev_edge_histogram_y[image_height];
    int * edge_histogram_y_p=edge_histogram_y;
    int * prev_edge_histogram_y_p=prev_edge_histogram_y;

    float slope_x=0.0;
    float trans_x=0.0;
    float slope_y=0.0;
    float trans_y=0.0;

    int previous_frame_number;
            if(fabs(edge_flow->horizontal[0])<MAX_FLOW&&!isnan(edge_flow->horizontal[0]))
            {
                previous_frame_number=(int)((MAX_HORIZON-2)*((float)MAX_FLOW-fabs(edge_flow->horizontal[0]))/(float)MAX_FLOW)+1;
            }
            else
                previous_frame_number=1;

            //previous_frame_number=1;

            // the previous frame number relative to dynamic parameters
            int previous_frame_number_rel=front-previous_frame_number;
            if(previous_frame_number_rel<0)
                previous_frame_number_rel=rear+(MAX_HORIZON-1-abs(previous_frame_number));

            // Copy previous edge gram to pointer
            printf("%d %d %d %d \n",front,previous_frame_number,previous_frame_number_rel,rear);


            memcpy(prev_edge_histogram_x_p,&edge_hist[previous_frame_number_rel].horizontal,sizeof(int)*image_width);
            memcpy(prev_edge_histogram_y_p,&edge_hist[previous_frame_number_rel].vertical,sizeof(int)*image_height);


            //Calculculate current edge_histogram
            calculate_edge_histogram(in,out,edge_histogram_x_p,image_width,image_height,'x');
            calculate_edge_histogram(in,out,edge_histogram_y_p,image_width,image_height,'y');

            //Copy new edge histogram to the structure
            memcpy(edge_hist[front].horizontal,edge_histogram_x_p,sizeof(int)*image_width);
            memcpy(edge_hist[front].vertical,edge_histogram_y_p,sizeof(int)*image_height);

            //calculate displacement based on histogram
            calculate_displacement(edge_histogram_x_p,prev_edge_histogram_x_p,displacement->horizontal,previous_frame_number,image_width);
            calculate_displacement(edge_histogram_y_p,prev_edge_histogram_y_p,displacement->vertical,previous_frame_number,image_height);



            //Line fit of the displacement by least square estimation or Ransac
    #ifdef RANSAC
            line_fit_RANSAC(displacement->horizontal, &slope_x,&trans_x,image_width);
            line_fit_RANSAC(displacement->vertical, &slope_y,&trans_y,image_height);

    #else
            line_fit(displacement->horizontal, &slope_x,&trans_x,image_width);
            line_fit(displacement->vertical, &slope_y,&trans_y,image_height);
    #endif


            //Correct Divergence slope and translation by the amount of frames skipped
            slope_x=slope_x/(float)previous_frame_number;
            trans_x=trans_x/(float)previous_frame_number;

            slope_y=slope_y/(float)previous_frame_number;
            trans_y=trans_y/(float)previous_frame_number;
            printf("check end\n");

            edge_flow->horizontal[0]=slope_x;
            edge_flow->horizontal[1]=trans_x;
            edge_flow->vertical[0]=slope_y;
            edge_flow->vertical[1]=trans_y;




}


void calculate_edge_histogram(struct image_t * in,struct image_t * out,int * edge_histogram,int image_width,int image_height,char direction)
{


    int  sobel;
    int Sobel[3] = {-1, 0, 1};
    int y,x;
    int edge_histogram_temp;
    int  c,r;
    int idx;
     int idx_filter;
     uint8_t *source = (uint8_t *)in->buf;
     uint8_t *dest = (uint8_t *)out->buf;

    if(direction=='x')
    for( x = 0; x < image_width; x++)
    {
        edge_histogram_temp=0;
        for( y = 0; y < image_height; y++)
        {


            idx = image_width*y + (x);
            sobel=0;

            for(c = -1; c <=1; c++)
            {

                    idx_filter= image_width*(y) + (x+c);


                sobel += Sobel[c+1] * (int)(source[idx_filter]);


            }


            sobel=abs(sobel);
            edge_histogram_temp += sobel;

            dest[idx]=sobel;

        }

        edge_histogram[x]=(int)edge_histogram_temp;

    }
    else if(direction=='y')

    for( y = 0; y < image_height; y++)
    {
        edge_histogram_temp=0;
        for( x = 0;x < image_width; x++)
        {

            int idx = image_width*y + (x);
            sobel=0;

            for(c = -1; c <=1; c++)
            {

                    idx_filter = image_width*(y+c) + (x);

                sobel += Sobel[c+1] * (int)(source[idx_filter]);
            }

            sobel=abs(sobel);

            edge_histogram_temp += sobel;
            dest[idx]=sobel;

        }

        edge_histogram[y]=(int)edge_histogram_temp;

    }
    else
        printf("direction is wrong!!\n");

}

void calculate_displacement(int * edge_histogram,int * edge_histogram_prev,int * displacement,int prev_frame_number,int size)
{


    int  c,r;
int y,x,flow_ind;

int W=20;
int D=10;
int d;
int SAD_temp[2*D];
int i;
int min_cost;
int  min_index;

int mean_displacement;

int  minimum=D*2;
int sum_dis, avg;
sum_dis=0;


for(x=0; x<size;x++)
{
    minimum=D*2;

    //Sad_temp to 0;

    if(x>D&&x<size-D)
    {
        for(c=-D;c<D;c++)
        {
            SAD_temp[c+D]=0;
            for(r=-W;r<W;r++)
                SAD_temp[c+D]+=abs(edge_histogram[x+r]-edge_histogram_prev[x+r+c]);
        }

        min_index=getMinimum(SAD_temp,2*D);

        displacement[x]=(int)((min_index-D));
    }else
        displacement[x]=0;


}

}

void line_fit(int* displacement, float* Slope, float* Yint,int size)
{
    int x;

    int Count=192-20;
    int SumY,SumX,SumX2,SumXY;
    int XMean,YMean;
    int k;
    int count_disp;

    for(k=0;k<size;k++){
        count_disp+=displacement[k];    }

    double Slope_temp,Yint_temp;

    //double Slope,Yint;

    for(x=10;x<192-10;x++){

        SumX+=x;
        SumY+=displacement[x];

        SumX2+=x *x;
        SumXY+=x *displacement[x];

    }
    XMean=SumX/Count;
    YMean=SumY/Count;

    Slope_temp=((double)(SumXY-SumX*YMean)/(double)(SumX2-SumX*XMean));
    Yint_temp= (YMean-Slope_temp* (double)XMean);

    *Slope=Slope_temp;
    *Yint=Yint_temp;
    //printf("%f %f\n", Slope, Yint);


}

void line_fit_RANSAC( int* displacement, float* Slope, float* Yint,int size)
{

    //Fit a linear line with RANSAC (from Guido's code)
    int ransac_iter=20;
    int it;
    int k;
    int ind1, ind2, tmp, entry, total_error, best_ind;
    int dx, dflow, predicted_flow;
    // flow = a * x + b
    float a[ransac_iter];
    float b[ransac_iter];
    float a_temp,b_temp;
    int errors[ransac_iter];

    int X[size];
    int count_disp=0;

    for(k=0;k<size;k++){
        X[k]=k;
        count_disp+=displacement[k];    }

    if(count_disp!=0)
    {
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
            while(displacement[ind1]==0)
                ind1 = rand() % size;
            while(displacement[ind2]==0)
                ind2 = rand() % size;




            dx=X[ind2]-X[ind1];
            dflow = displacement[ind2] - displacement[ind1];

            //Fit line with two points

            a[it] = (float)dflow/(float)dx;
            b[it] = (float)displacement[ind1]- (a[it] *(float)(X[ind1]));

            // evaluate fit:
            for (entry = 0; entry < size; entry++)
            {
                predicted_flow = (int32_t)(a[it] * (float)(X[entry] ) + b[it]);
                total_error += (uint32_t) abs(displacement[entry] - predicted_flow);
            }
            errors[it] = total_error;
        }
        // select best fit:
        best_ind = getMinimum(errors, 20);
        //printf("%d\n",best_ind);
        (*Slope) = (float)a[best_ind] ;
        (*Yint) = (float)b[best_ind];
    }
    else
    {
        (*Slope) = 0.0 ;
        (*Yint) = 0.0;
    }




}

void visualize_divergence(struct image_t* in,struct image_t* out,struct displacement_t* displacement,struct edge_hist_t* edge_hist,int front,int rear,float Slope, float Yint,uint16_t image_width, uint16_t image_height)
{
    uint32_t y,x;



    image_copy(in,out);

    struct point_t  linedraw;
    struct point_t linedraw_prev;


   // uint32_t* edge_histogram_hor= edge_hist[0].horizontal;
    //uint32_t* edge_histogram_prev_hor=edge_hist[1].horizontal;

    for( x = 0; x < image_width-1; x++)
    {

       linedraw.y =(uint16_t)(-displacement->horizontal[x+1]+image_height/2);
        linedraw.x=(uint16_t)x+1;

        linedraw_prev.y =(uint16_t)(-displacement->vertical[x]+image_height/2);
        linedraw_prev.x=(uint16_t)x;
        image_draw_color_line(out, &linedraw_prev,&linedraw,1);


       linedraw.y =(uint16_t)(  edge_hist[front].horizontal[x+1]/20+image_height/2);
         linedraw.x=(uint16_t)x+1;

         linedraw_prev.y =(uint16_t)( edge_hist[front].horizontal[x]/20+image_height/2);
         linedraw_prev.x=(uint16_t)x;

        image_draw_color_line(out, &linedraw_prev,&linedraw,255);

        linedraw.y =(uint16_t)( edge_hist[rear].horizontal[x+1]/20+image_height/2);
          linedraw.x=(uint16_t)x+1;

          linedraw_prev.y =(uint16_t)( edge_hist[rear].horizontal[x]/20+image_height/2);
          linedraw_prev.x=(uint16_t)x;

         image_draw_color_line(out, &linedraw_prev,&linedraw,100);

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






