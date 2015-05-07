#include "edgefilter.h"



int Gsize=7;
double sigma=3.0;
int thres=30;
//int stereo_nav_status=0;
int thres_verticalcount=90;
int thres_disparity=20;
int multiply_disparity=3;

void sobel_edge_filter(struct image_t *input,struct image_t *output,uint32_t* edge_histogram)
{

    uint32_t  Sobel[3][3] = {{-1, 0, 1},{-2,0,2},{-1,0,1}};
    int8_t r, c;
    uint32_t  sobel;
    uint8_t *source = (uint8_t *)input->buf;
    uint8_t *dest = (uint8_t *)output->buf;

    uint32_t edge_histogram_temp=0;


    for(uint16_t x = 1; x < input->w-1; x++) {
        for(uint16_t y = 1; y < input->h-1; y++) {

            uint32_t idx = input->w*y*2 + (x)*2;
            sobel=0;
            //Convolution
            if(y>1&&y<input->h-1&&x>1&&x<input->w-1)
            {
                for(r = -1; r <=1; r++)
                {
                    for(c = -1; c <= 1; c++)
                    {
                        uint32_t idx_filter = input->w*(y+r)*2 + (x+c)*2;
                        sobel += Sobel[r+1][c+1] * (source[idx_filter]);
                    }
                }}
            sobel=abs(sobel);

            edge_histogram_temp += (uint32_t)sobel;
            dest[idx+1]=sobel;

            dest[idx]=127;
        }

        edge_histogram[x]=(uint32_t)edge_histogram_temp;
        edge_histogram_temp=0;
    }
}

void calculate_edge_histogram_displacement(uint32_t* edge_histogram,uint32_t* edge_histogram_prev,int32_t* displacement,uint16_t image_width,uint16_t image_height)
{
    int32_t  c,r;
uint16_t y,x,flow_ind;

int32_t W=20;
int32_t D=20;
uint32_t SAD_temp[40];

uint32_t min_index=0;




for(x=0; x<image_width;x++)
{

    if(x>=W+D&&x<=image_width-W-D)
    {
        for(c=-D;c<W;c++)
        {
            SAD_temp[D+c]=0;

            for(r=-W;r<W;r++)
                SAD_temp[c+D]+=abs(edge_histogram[x+r]-edge_histogram_prev[x+r+c]);


        }

        min_index=getMinimum(SAD_temp,40);

        displacement[x]=(int32_t)(min_index-W);
    }else{
        displacement[x]=0;
    }

    // displacement[x]=(int32_t)(-1*x+10);
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

        //if(displacement[x]!=0){

            SumX+=x;
            SumY+=displacement[x];

            SumX2+=x *x;
            SumXY+=x *displacement[x];
            Count++;
    //}

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

void visualize_divergence(struct image_t* in,struct image_t* out,int32_t* displacement,float Slope, float Yint,uint16_t image_width, uint16_t image_height)
{
    uint32_t y,x;
    uint32_t line_check1=0;
    uint32_t line_check2=0;

    uint8_t *source = (uint8_t *)in->buf;
    uint8_t *dest = (uint8_t *)out->buf;

    for( x = 0; x < image_width; x++)
    {
        line_check1=(uint32_t)(Slope*(float)x+(Yint)+(float)image_height/2);
        line_check2=(uint32_t)(displacement[x]+image_height/2);
        for( y = 0; y < image_height; y++)
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
