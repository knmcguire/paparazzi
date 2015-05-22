#include "edgefilter.h"
#include "lib/vision/image.h"



int Gsize=7;
double sigma=3.0;
int thres=30;
//int stereo_nav_status=0;
int thres_verticalcount=90;
int thres_disparity=20;
int multiply_disparity=3;

void sobel_edge_filter(struct image_t *input,struct image_t *output,struct edge_hist_t* edge_histogram)
{

    uint32_t  Sobel[3] = {-1, 0, 1};
    int8_t r, c;
    uint32_t  sobel=0;



    uint8_t *source = (uint8_t *)input->buf;
    uint8_t *dest = (uint8_t *)output->buf;

    uint32_t* edge_histogram_hor=(uint32_t*) edge_histogram->horizontal;
    uint32_t edge_histogram_temp=0;

    for(uint16_t j=0;j<input->w;j++)
        edge_histogram_hor[j]=0;

    for(uint16_t x = 1; x < input->w-1; x++) {
        for(uint16_t y = 1; y < input->h-1; y++) {

            uint32_t idx = input->w*y*2 + (x)*2;
            sobel=0;
            //Convolution
            if(y>1&&y<input->h-1&&x>1&&x<input->w-1)
            {

                for(c = -1; c <= 1; c++)
                {
                    uint32_t idx_filter = input->w*(y)*2 + (x+c)*2;
                    sobel += Sobel[c+1] * source[idx_filter+1];
                }
            }
            sobel=abs(sobel);

            edge_histogram_temp += (uint32_t)sobel;
            dest[idx+1]=sobel;

            dest[idx]=127;
        }

        edge_histogram_hor[x]=(uint32_t)edge_histogram_temp;
        edge_histogram_temp=0;

    }

}

void blur_filter(struct image_t *input,struct image_t *output)
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
    uint8_t *source = input->buf;
    uint8_t *dest = output->buf;


    for(uint16_t y = 0; y < input->h; y++) {
        for(uint16_t x = 0; x < input->w; x++) {
            uint32_t idx = input->w*y*2 + (x)*2;

            //Convolution
            if(y>G_hsize&&y<input->h-G_hsize&&x>G_hsize&&x<input->w-G_hsize)
            {
                gaussian=0;
                for(r = -G_hsize; r <=G_hsize; r++)
                {
                    for(c = -G_hsize; c <= G_hsize; c++)
                    {
                        uint32_t idx_filter = input->w*(y+r)*2 + (x+c)*2;
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

void calculate_edge_histogram_displacement(struct edge_hist_t edge_histogram[],uint8_t previous_frame_number,int32_t* displacement,uint16_t image_width,uint16_t image_height)
{
    int32_t  c,r;
    uint16_t i,y,x,flow_ind;

    int32_t W=10;
    int32_t D=10;
    uint32_t SAD_temp[20];
    uint16_t edge_histogram_tot=0;
    uint32_t min_index=0;

    uint32_t SAD_tot=0;

    uint32_t* edge_histogram_hor= edge_histogram[0].horizontal;
    uint32_t* edge_histogram_prev_hor=edge_histogram[1].horizontal;


    for(x=0; x<image_width;x++)
    {
        SAD_tot=0;
        if(x>=W+D&&x<=image_width-W-D)
        {
            min_index=0;
            for(c=-D;c<D;c++)
            {
                SAD_temp[D+c]=0;

                for(r=-W;r<W;r++){
                    //printf("%dcheck\n",x+r+c);

                    SAD_temp[c+D]+=abs(edge_histogram_hor[x+r]-edge_histogram_prev_hor[x+r+c]);
                }
                SAD_tot+=SAD_temp[D+c];

            }

            min_index=getMinimum(SAD_temp,20);
            printf("%d\n",SAD_tot);
            //printf("%d,check\n",previous_frame_number);
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
    int16_t x;

    uint32_t Count=0;
    float SumY=0;
    float  SumX=0;
    float SumX2=0;
    float SumXY=0;
    float XMean=0;
    float YMean=0;
    float Slope_temp,Yint_temp;

    //double Slope,Yint;



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


    //printf("%f %f\n", Slope, Yint);

}

void visualize_divergence(struct image_t* in,struct image_t* out,uint32_t* edge_histogram,uint32_t* edge_histogram_prev,int32_t* displacement,float Slope, float Yint,uint16_t image_width, uint16_t image_height)
{
    uint32_t y,x;
    uint32_t line_check1=0;
    uint32_t line_check2=0;

    //uint8_t *source = (uint8_t *)in->buf;
    //uint8_t *dest = (uint8_t *)out->buf;

    image_copy(in,out);

    struct point_t  linedraw;
    struct point_t linedraw_prev;


    for( x = 0; x < image_width-1; x++)
    {
        //line_check1=(uint32_t)(Slope*(float)x+(Yint)+(float)image_height/2);
        //line_check2=(uint32_t)(displacement[x]+image_height/2);
        //line_check1=(uint32_t)(edge_histogram[x]/50+image_height/2);
        //line_check2=(uint32_t)(edge_histogram_prev[x]/50+image_height/2);

        linedraw.y =(uint16_t)(displacement[x+1]+image_height/2);
        linedraw.x=(uint16_t)x+1;

        linedraw_prev.y =(uint16_t)(displacement[x]+image_height/2);
        linedraw_prev.x=(uint16_t)x;

        image_draw_color_line(out, &linedraw_prev,&linedraw,1);


        /*linedraw.y =(uint16_t)(edge_histogram[x+1]/50+image_height/2);
        linedraw.x=(uint16_t)x+1;

        linedraw_prev.y =(uint16_t)(edge_histogram[x]/50+image_height/2);
        linedraw_prev.x=(uint16_t)x;

        image_draw_color_line(out, &linedraw_prev,&linedraw,1);

        linedraw.y =(uint16_t)(edge_histogram_prev[x+1]/50+image_height/2);
        linedraw.x=(uint16_t)x+1;

        linedraw_prev.y =(uint16_t)(edge_histogram_prev[x]/50+image_height/2);
        linedraw_prev.x=(uint16_t)x;
        image_draw_color_line(out, &linedraw_prev,&linedraw,255);*/


        /*for( y = 0; y < image_height; y++)
        {
            //line_check=(uint32_t)(Slope*x+Yint+image_height/2);

            uint32_t idx = image_width*y*2 + (x)*2;

            if(y==line_check1)
                dest[idx]=0;
            else if(y==line_check2)
                dest[idx]=255;

            else
                dest[idx]=127;

            //out[idx]=(uint32_t)(10*Slope*Slope);

            dest[idx+1]=source[idx+1];

            //else
            //	out[idx+1]=in[idx+1];

            //out[idx]=0;




        }*/



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
