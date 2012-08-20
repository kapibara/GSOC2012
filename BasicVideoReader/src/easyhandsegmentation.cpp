#include "easyhandsegmentation.h"

#include <iostream>

using namespace std;

EasyHandSegmentation::EasyHandSegmentation(int maxObjectSize):_pixels(maxObjectSize)
{
    CV_Assert(maxObjectSize>0);

    _maxObjectSize = maxObjectSize;
    _depthThr = 40;
    _lt = 0;
    _ut = pow(2,14);
}

EasyHandSegmentation::~EasyHandSegmentation()
{
    //no dynamic memory was allocated
}

void EasyHandSegmentation::segmentHand(cv::Mat &mask, cv::Rect3D &region, const cv::Mat &rgb, const cv::Mat &depth)
{
    segmentHand(mask,region,depth);
}

void EasyHandSegmentation::init()
{
    _pixels.clear();
}

void EasyHandSegmentation::segmentHand(cv::Mat &mask, cv::Rect3D &region, const cv::Mat &depth)
{
    CV_Assert(mask.type() == CV_8UC1);
    CV_Assert(depth.type() == CV_16UC1);

    CV_Assert(mask.rows == depth.rows);
    CV_Assert(mask.cols == depth.cols);

    mask.setTo(EasyHandSegmentation::EMPTY);

    point current = searchNearestPixel(region,depth);
    if (current.first < 0){
        region.isIni = false;
        return;
    }

    int rowcount = depth.rows, colcount = depth.cols;

    _pixels.clear();
    double mean = depth.at<unsigned short>(current.first,current.second);
    int minx=depth.cols,miny=depth.rows,maxx=0,maxy=0,minz = pow(2,15),maxz = 0;
    unsigned short dv = 0;
    int depthMinDiff = 50;
    int pixelcount = 1;
    _pixels.push(current);

    while(!_pixels.empty() & pixelcount < _maxObjectSize)
    {
        current = _pixels.front();
        _pixels.pop();

        dv = depth.at<unsigned short>(current.first,current.second);

        if (current.first < minx) minx = current.first;
                else if (current.first > maxx) maxx = current.first;
        if (current.second < miny) miny = current.second;
                else if (current.second > maxy) maxy = current.second;
        if (dv < minz) minz = dv;
                else if (dv > maxz) maxz = dv;

        if ( current.first + 1 < rowcount ){
            processNeighbor(pixelcount,mean,mask,current.first + 1,current.second,depth);
        }

        if ( current.first - 1 > -1 ){
            processNeighbor(pixelcount,mean,mask,current.first - 1,current.second,depth);
        }

        if ( current.second + 1 < colcount ){
            processNeighbor(pixelcount,mean,mask,current.first,current.second + 1,depth);
        }

        if( current.second - 1 > -1 ){
            processNeighbor(pixelcount,mean,mask,current.first,current.second - 1,depth);
        }

    }

    region.width = maxy - miny; //cols range
    region.height = maxx - minx; //rows range

    if(region.width<=0 | region.height <=0){
        region.isIni = false;
    }
    else{
        region.isIni = true;
        region.x = miny; //cols
        region.y = minx; //rows
        region.z = minz;
        region.depth = max(maxz - minz,0);
 /*       if(region.depth < depthMinDiff){
            region.z -= depthMinDiff/2;
            region.depth = depthMinDiff;
        }*/
    }
}

bool EasyHandSegmentation::processNeighbor(int &pixelcount, double &mean, cv::Mat &mask, const short first, const short second, const cv::Mat &depth)
{
    unsigned short d = depth.at<unsigned short>(first,second );

    if ( mask.at<uchar>(first,second ) == EasyHandSegmentation::EMPTY &
         fabs(d-mean/pixelcount) < _depthThr & d > _lt & d <= _ut)
    {
        pixelcount++;
        mean += d;
        mask.at<uchar>(first,second ) = EasyHandSegmentation::HAND;
        _pixels.push(point(first,second));
    }
}

EasyHandSegmentation::point EasyHandSegmentation::searchNearestPixel(const cv::Rect3D &region, const cv::Mat &depth)
{

    const unsigned short *depthptr;
    int lowerBoundary = region.z;
    int upperBoundary = region.z + region.depth;
    unsigned short minval = pow(2,15); //not nessecerely maximum, but not minimum for sure
    point result(-1,-1);

    if (region.z < _lt)
        lowerBoundary = _lt;

    if (upperBoundary > _ut)
        upperBoundary = _ut;

    //search for nearest pixel
    for(int i=region.y; i< (region.y+region.height); i++)
    {
        depthptr = depth.ptr<const unsigned short>(i);

        for(int j=region.x; j<(region.x+region.width); j++)
        {
            if(depthptr[j]> lowerBoundary & depthptr[j] <= upperBoundary & depthptr[j]<minval)
            {
                minval = depthptr[j];
                result.first = i;
                result.second = j;
            }
        }
    }

    return result;
}
