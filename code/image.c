#include "zf_common_headfile.h"

uint8 Gray_filter[MT9V03X_H][MT9V03X_W];
uint8 Gray_zip[MT9V03X_H/2][MT9V03X_W/2];
uint8 image_sobel[MT9V03X_H/2][MT9V03X_W/2];


uint8 thresholds;


void zip()//图像压缩
{
    for(int i=0;i<MT9V03X_H/2;i++)
    {
        for(int j=0;j<MT9V03X_W/2;j++)
        {
            Gray_zip[i][j]=Gray_filter[i*2][j*2];
        }
    }
}

void fileOverview()//高斯滤波
{
    for(uint16_t i = 1;i<MT9V03X_H-1;i++)
            {
                for(uint16 j =1;j<MT9V03X_W-1;j++)
                {
                    Gray_filter[i][j] =(94*mt9v03x_image[i-1][j-1]+118*mt9v03x_image[i-1][j  ]+94*mt9v03x_image[i-1][j+1]
                                       +118*mt9v03x_image[i  ][j-1]+147*mt9v03x_image[i  ][j  ]+118*mt9v03x_image[i  ][j+1]
                                       +94*mt9v03x_image[i+1][j-1]+118*mt9v03x_image[i+1][j  ]+94*mt9v03x_image[i+1][j+1])/1000;
                }
            }
}

uint8 otsuThreshold(uint8 *image, uint16 width, uint16 height)//大津法
{
    #define GrayScale 256
    int pixelCount[GrayScale] = {0};//每个灰度值所占像素个数
    float pixelPro[GrayScale] = {0};//每个灰度值所占总像素比例
    int i,j;
    int Sumpix = width * height;   //总像素点
    uint8 threshold = 0;
    uint8* data = image;  //指向像素数据的指针


    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
          //   pixelCount[(int)image[i][j]]++;    若不用指针用这个
        }
    }
    float u = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / Sumpix;   //计算每个像素在整幅图像中的比例
        u += i * pixelPro[i];  //总平均灰度
    }


    float maxVariance=0.0;  //最大类间方差
    float w0 = 0, avgValue  = 0;  //w0 前景比例 ，avgValue 前景平均灰度
    for(int i = 0; i < 256; i++)     //每一次循环都是一次完整类间方差计算 (两个for叠加为1个)
    {
        w0 += pixelPro[i];  //假设当前灰度i为阈值, 0~i 灰度像素所占整幅图像的比例即前景比例
        avgValue  += i * pixelPro[i];

        float variance = pow((avgValue/w0 - u), 2) * w0 /(1 - w0);    //类间方差
        if(variance > maxVariance)
        {
            maxVariance = variance;
            threshold = (uint8)i;
        }
    }


    return threshold;

}

void erzhihua()//图像二值化
{
    for(int i=0;i<MT9V03X_H/2;i++)
    {
        for(int j=0;j<MT9V03X_W/2;j++)
        {
            if(Gray_zip[i][j]>=thresholds)//thresholds,&&i!=0&&i!=50&&j!=0&&j!=59
            {
                image_sobel[i][j]=255;
            }
            else image_sobel[i][j]=0;
        }
    }
}

void sobel()
{
    for(uint16_t i = 1;i<59;i++)
            {
                for(uint16 j =1;j<79;j++)
                {
                    int16 x = Gray_zip[i-1][j+1]+2*Gray_zip[i][j+1]+Gray_zip[i+1][j+1]
                              - Gray_zip[i-1][j-1]-2*Gray_zip[i][j-1]-Gray_zip[i+1][j-1];
                    int16 y = Gray_zip[i-1][j-1]+2*Gray_zip[i-1][j]+Gray_zip[i-1][j+1]
                              - Gray_zip[i+1][j-1]-2*Gray_zip[i+1][j]-Gray_zip[i+1][j+1];
                    if(abs(x)+abs(y)>160)
                    {
                        image_sobel[i][j] = 0;
                    }
                    else
                    {
                        image_sobel[i][j] = 255;
                    }
                }
            }
}



