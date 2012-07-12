#include "skeleton.h"

using namespace cv;

Skeleton::Skeleton()
{
}

void Skeleton::findSkeleton(Mat &skeleton, const Mat &input)
{
    //distance image
    Mat doutput(input.size(),CV_32FC1);
    Mat output(input.size(),CV_8UC1);
    double minVal,maxVal;

    distanceTransform(input,doutput,CV_DIST_L2,3);
    minMaxLoc(doutput,&minVal,&maxVal);
    doutput.convertTo(output,CV_8UC1,255.0/(maxVal-minVal),-minVal*255.0/(maxVal-minVal));

    adaptiveThreshold(output,skeleton,255,ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,3,-2);

}
