#include "morphology.h"

#include <iostream>

using namespace cv;
using namespace std;

Morphology::Morphology()
{
}

void Morphology::open(cv::Mat &output,const cv::Mat &input, double r)
{
    Mat doutput(input.size(),CV_32FC1);
    Mat output1(input.size(),CV_8UC1);

    //invert image
    output = (uchar)255 - input;

    //dilation
    distanceTransform(output,doutput,CV_DIST_L2,3);

    threshold(doutput,output,r,255,THRESH_BINARY_INV);

    output.convertTo(output1,CV_8UC1);

    //erosion
    distanceTransform(output1,doutput,CV_DIST_L2,3);

    threshold(doutput,output1,r,255,THRESH_BINARY);

    output1.convertTo(output,CV_8UC1);

}

void Morphology::close(cv::Mat &output, const cv::Mat &input, double r)
{
    Mat doutput(input.size(),CV_32FC1);
    Mat output1(input.size(),CV_8UC1);

    //erosion
    distanceTransform(input,doutput,CV_DIST_L2,3);

    threshold(doutput,output1,r,255,THRESH_BINARY_INV);

    output1.convertTo(output,CV_8UC1);

    //dilation
    distanceTransform(output,doutput,CV_DIST_L2,3);

    threshold(doutput,output1,r,255,THRESH_BINARY_INV);

    output1.convertTo(output,CV_8UC1);
}

void Morphology::dilate(cv::Mat &output, const cv::Mat &input, double r)
{
    Mat doutput;
    Mat output1;

    //invert image
    output = (uchar)255 - input;

    //dilation
    distanceTransform(output,doutput,CV_DIST_L2,3);

    threshold(doutput,output,r,255,THRESH_BINARY_INV);

    output.convertTo(output1,CV_8UC1);
}

void Morphology::erode(cv::Mat &output, const cv::Mat &input, double r)
{
    Mat doutput;
    Mat output1;

    //erosion
    distanceTransform(input,doutput,CV_DIST_L2,3);

    threshold(doutput,output1,r,255,THRESH_BINARY);

    output1.convertTo(output,CV_8UC1);
}
