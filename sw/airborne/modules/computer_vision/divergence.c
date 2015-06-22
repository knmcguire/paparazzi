/*
 * divergence.cpp
 *
 *  Created on: Apr 28, 2015
 *      Author: knmcguire
 */
#include "divergence.h"



int calculate_edge_flow(struct image_t *in,struct image_t* out, struct displacement_t* displacement,struct edge_flow_t* edge_flow, struct edge_hist_t* edge_hist,int front,int rear,int windowsize,int max_distance,int edge_threshold,uint16_t image_width,uint16_t image_height)
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


    //determine previous frame number (adapt time
   int previous_frame_number[2];

    if(fabs(edge_flow->horizontal[1])<MAX_FLOW&&!isnan(edge_flow->horizontal[1])&&MAX_HORIZON!=1)
    {
        previous_frame_number[0]=(int)((MAX_HORIZON-2)*((float)MAX_FLOW-fabs(edge_flow->horizontal[1]))/(float)MAX_FLOW)+1;
    }
    else
        previous_frame_number[0]=1;
    if(fabs(edge_flow->vertical[1])<MAX_FLOW&&!isnan(edge_flow->vertical[1])&&MAX_HORIZON!=1)
    {
        previous_frame_number[1]=(int)((MAX_HORIZON-2)*((float)MAX_FLOW-fabs(edge_flow->vertical[1]))/(float)MAX_FLOW)+1;
    }
    else
        previous_frame_number[1]=1;
/*
     int previous_frame_number[2];

    if(fabs(edge_flow->horizontal[1])!=0&&1/fabs(edge_flow->horizontal[1])>MAX_HORIZON)
        previous_frame_number[0]=MAX_HORIZON;
    else
        previous_frame_number[0]=(int)(1/fabs(edge_flow->horizontal[1]))+1;

    if(fabs(edge_flow->vertical[1])!=0&&1/fabs(edge_flow->vertical[1])>MAX_HORIZON)
        previous_frame_number[1]=MAX_HORIZON;
    else
        previous_frame_number[1]=(int)(1/fabs(edge_flow->vertical[1]))+1;*/

    // the previous frame number relative to dynamic parameters
    int previous_frame_number_rel[2];
    previous_frame_number_rel[0]=front-previous_frame_number[0];
    previous_frame_number_rel[1]=front-previous_frame_number[1];
    if(previous_frame_number_rel[0]<0)
        previous_frame_number_rel[0]=rear+(MAX_HORIZON-1-abs(previous_frame_number[0]));
    if(previous_frame_number_rel[1]<0)
        previous_frame_number_rel[1]=rear+(MAX_HORIZON-1-abs(previous_frame_number[1]));

    // Copy previous edge gram to pointer
    // printf("%d %d %d %d \n",front,previous_frame_number,previous_frame_number_rel,rear);


    memcpy(prev_edge_histogram_x_p,&edge_hist[previous_frame_number_rel[0]].horizontal,sizeof(int)*image_width);
    memcpy(prev_edge_histogram_y_p,&edge_hist[previous_frame_number_rel[1]].vertical,sizeof(int)*image_height);



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
    //printf("%d,%d\n",median_x,median_y);

    //int edge_thres_x=mean_x+(median_x-mean_x)/4;
    // int edge_thres_y=mean_y+(median_y-mean_y)/4;

    // int edge_thres_x=median_x/2;
    //int edge_thres_y=median_y/2;
#else
    int edge_thres_x=0;
    int edge_thres_y=0;
#endif

    //Calculculate current edge_histogram
    calculate_edge_histogram(in,out,edge_histogram_x_p,edge_thres_x,image_width,image_height,'x');
    calculate_edge_histogram(in,out,edge_histogram_y_p,edge_thres_y,image_width,image_height,'y');


    int median_x=GetMedian(edge_histogram_x_p,image_width);
    int median_y=GetMedian(edge_histogram_y_p,image_height);


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

    //if((median_x+median_y)/2>400){//To avoid detecting flow from the noisy bottom camera

        //Correct Divergence slope and translation by the amount of frames skipped
        slope_x=slope_x/(float)previous_frame_number[0];
        trans_x=trans_x/(float)previous_frame_number[0];

        slope_y=slope_y/(float)previous_frame_number[1];
        trans_y=trans_y/(float)previous_frame_number[1];

        if(isnan(fabs(slope_x)))
            slope_x=0.0;

        if(isnan(fabs(trans_x)))
            trans_x=0.0;

        if(isnan(fabs(slope_y)))
            slope_y=0.0;

        if(isnan(fabs(trans_y)))
            trans_y=0.0;


        /* if(abs(trans_x-edge_flow->horizontal[0])>1)
           trans_x=(trans_x+edge_flow->horizontal[0])/2;*/
        //Smoothing
        /*if (abs(trans_x-edge_flow->horizontal[1])>10)
        trans_x=edge_flow->horizontal[1];


    if (abs(trans_y-edge_flow->vertical[1])>10)
        trans_y=edge_flow->vertical[1];*/

        //To eliminate noise peaks
        if(sum_disx>0){
            edge_flow->horizontal[1]=trans_x;}
        else edge_flow->horizontal[1]=0.0;

        if(sum_disy>0)
            edge_flow->vertical[1]=trans_y;
        else
            edge_flow->vertical[1]=0.0;


        edge_flow->horizontal[0]=slope_x;
        edge_flow->vertical[0]=slope_y;
    /*}else{
        edge_flow->horizontal[1]=0.0;
        edge_flow->vertical[1]=0.0;
        edge_flow->horizontal[0]=0;
        edge_flow->vertical[0]=0;
    }*/

    visualize_divergence_debug(in,out,displacement,edge_histogram_x_p,prev_edge_histogram_x_p,front,rear,edge_flow->horizontal[0],edge_flow->horizontal[1],image_width,image_height,'l');


    //Copy new edge histogram to the structure
    memcpy(edge_hist[front].horizontal,edge_histogram_x_p,sizeof(int)*image_width);
    memcpy(edge_hist[front].vertical,edge_histogram_y_p,sizeof(int)*image_height);

    return (median_x+median_y)/2;

}




calculate_edge_histogram(struct image_t * in,struct image_t * out,int * edge_histogram, int thres,int image_width,int image_height,char direction)
{


    int  sobel;
    int Sobel[3] = {-1.0, 0, 1.0};
    int y,x;
    int edge_histogram_temp;
    int  c,r;
    int idx;
    int idx_filter;
    uint8_t *source = (uint8_t *)in->buf;
    uint8_t *dest = (uint8_t *)out->buf;


    if(direction=='x'){
        for( x = 1; x < image_width-1; x++)
        {
            edge_histogram_temp=0;
            for( y = 1; y < image_height-1; y++)
            {


                idx = image_width*y*2 + (x)*2;
                sobel=0;

                for(c = -1; c <=1; c++)
                {

                    idx_filter= image_width*(y)*2 + (x+c)*2;


                    sobel += Sobel[c+1] * (int)(source[idx_filter+1]);


                }


                sobel=abs(sobel);

                edge_histogram_temp += sobel;

                dest[idx+1]=sobel;
                dest[idx]=127;

            }

            if((int)edge_histogram_temp>thres)
                edge_histogram[x]=(int)edge_histogram_temp;
            else
                edge_histogram[x]=0;
        }
    }
    else if(direction=='y')
    {
        for( y = 1; y < image_height-1; y++)
        {
            edge_histogram_temp=0;
            for( x = 1;x < image_width-1; x++)
            {

                idx = image_width*y*2 + (x)*2;
                sobel=0;

                for(c = -1; c <=1; c++)
                {

                    idx_filter= image_width*(y+c)*2 + (x)*2;

                    sobel += Sobel[c+1] * (int)(source[idx_filter+1]);
                }

                sobel=abs(sobel);

                edge_histogram_temp += sobel;
                dest[idx+1]=sobel;
                dest[idx]=127;


            }

            //edge_histogram[y]=(int)edge_histogram_temp;
            if((int)edge_histogram_temp>thres)
                edge_histogram[y]=(int)edge_histogram_temp;
            else
                edge_histogram[y]=0;
        }
    }
    else
        printf("direction is wrong!!\n");

}

int calculate_displacement(int * edge_histogram,int * edge_histogram_prev,int * displacement,int prev_frame_number,int windowsize,int max_distance,int size)
{


    int  c,r;
    int x;
    int W=windowsize;
    int D=max_distance;
    int d;
    int SAD_temp[2*D];
    int i;
    int  min_index;
    int sum_dis;
    sum_dis=0;

    //  printf("edgehistogram: ");

    for(x=0; x<size;x++)
    {

        //Sad_temp to 0;
        //  printf("%d ",edge_histogram[x]);

        if(x>D+W&&x<size-D-W)
        {
            \
            for(c=-D;c<D;c++)
            {
                SAD_temp[c+D]=0;

                for(r=-W;r<W;r++)
                    SAD_temp[c+D]+=abs(edge_histogram[x+r]-edge_histogram_prev[x+r+c]);
            }


            min_index=getMinimumMiddle(SAD_temp,D*2);
            if (abs(min_index-D)<D-1)
                displacement[x]=(int)((min_index-D));
            else
                displacement[x]=0;

        }else
            displacement[x]=0;

        sum_dis+=abs(displacement[x]);



    }
    // printf("\n ");

    return sum_dis;

}

void line_fit(int* displacement, float* Slope, float* Yint,int size)
{
    int x;

    int Count=size-40;
    int SumY,SumX,SumX2,SumXY;
    int XMean,YMean;
    int k;
    int count_disp;

    for(k=0;k<size;k++){
        count_disp+=displacement[k];    }

    double Slope_temp,Yint_temp;

    //double Slope,Yint;

    for(x=20;x<size-20;x++){

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
        best_ind = getMinimum(errors, ransac_iter);
        //printf("%d\n",best_ind);
        (*Slope) = (float)a[best_ind] ;
        (*Yint) = (float)b[best_ind]+a[best_ind]*size/2;
    }
    else
    {
        (*Slope) = 0.0 ;
        (*Yint) = 0.0;
    }




}

void visualize_divergence(struct image_t* in,struct image_t* out,struct displacement_t* displacement,struct edge_hist_t* edge_hist,int front,int rear,float Slope, float Yint,uint16_t image_width, uint16_t image_height,char plot_value)
{
    uint32_t y,x;



    image_copy(in,out);

    struct point_t  linedraw;
    struct point_t linedraw_prev;


    // uint32_t* edge_histogram_hor= edge_hist[0].horizontal;
    //uint32_t* edge_histogram_prev_hor=edge_hist[1].horizontal;

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
    uint32_t y,x;



    image_copy(in,out);

    struct point_t  linedraw;
    struct point_t linedraw_prev;


    // uint32_t* edge_histogram_hor= edge_hist[0].horizontal;
    //uint32_t* edge_histogram_prev_hor=edge_hist[1].horizontal;

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

float simpleKalmanFilter(float* cov,float previous_est, float current_meas,float Q,float R)
{

    float predict_state=previous_est;
    float predict_cov=*cov+Q;
    float K=predict_cov*(1/(*cov+R));

    float new_est=predict_state+K*(current_meas-previous_est);
    *cov=(1-K)*predict_cov;

    return new_est;
}

/*void kalmanFilter(float current_state,float previous_state,float previous_P,float fps,float new_state)
{
    w=1;
    v=1;
    float predict_state[2];
    float predict_P[2][2];
    float phi={{1,1},{0,1}}; //processmodel
    float phi_t={{1,1},{0,1}}; //processmodel transposed
    float H={{0,0},{1,0}}; //Observation model
    float R={{w,0},{0,w}}; //Observation Noise
    float Q={{v,0},{0,v}};  //Process Noise
    float K;

    //Predicted state
    for(int x=0;x<2;x++)
        for(int y=0;y<2;y++)
            predict_state[x]+=phi[x][y]*previous_state[x];


    //error covariance
    for(int x=0;x<2;x++)
        for(int y=0;y<2;y++)
            predict_P[x]+=phi[x][y]*previous_P[x]*phi_t[x][y]+Q[x][y];

    //Kalman Gain

    for(int x=0;x<2;x++)
        for(int y=0;y<2;y++)
           K[x]+=predict_P[x]*



}*/

