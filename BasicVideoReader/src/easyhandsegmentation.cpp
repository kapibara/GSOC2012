#include "easyhandsegmentation.h"

#include <iostream>

using namespace std;

EasyHandSegmentation::EasyHandSegmentation(int maxObjectSize):_pixels(maxObjectSize)
{
    CV_Assert(maxObjectSize>0);

    _maxObjectSize = maxObjectSize;
    _depthThr = 50;
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

    //calculate the number of pixels depending on the distance from the KINECT
    int rowcount = depth.rows, colcount = depth.cols;

    //queue can be much smaller, but it is to be on the safe side
    //allocate it once?
    _pixels.clear();
    double mean = depth.at<unsigned short>(current.first,current.second);
    int minx=depth.cols,miny=depth.rows,maxx=0,maxy=0,minz = pow(2,15),maxz = 0;
    unsigned short dv = 0;
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

        if ( current.first + 1 < rowcount )
        {
            if ( current.second + 1 < colcount )
            {
                if ( mask.at<uchar>(current.first + 1,current.second + 1 ) == EasyHandSegmentation::EMPTY &
                     fabs(depth.at<unsigned short>(current.first + 1,current.second + 1 )-mean/pixelcount) < _depthThr
                     & depth.at<unsigned short>(current.first + 1,current.second + 1 ) > 0)
                {
                    pixelcount++;
                    mean += depth.at<unsigned short>(current.first + 1,current.second + 1 );
                    mask.at<uchar>(current.first + 1,current.second + 1 ) = EasyHandSegmentation::HAND;
                    _pixels.push(point(current.first + 1,current.second + 1));
                }
            }

            if( current.second - 1 > -1 )
            {
                if ( mask.at<uchar>(current.first + 1,current.second - 1 ) == EasyHandSegmentation::EMPTY &
                     fabs(depth.at<unsigned short>(current.first + 1,current.second - 1 )-mean/pixelcount) < _depthThr
                     & depth.at<unsigned short>(current.first + 1,current.second + 1 ) > 0)
                {
                    pixelcount++;
                    mean += depth.at<unsigned short>(current.first + 1,current.second - 1 );
                    mask.at<uchar>(current.first + 1,current.second - 1 ) = EasyHandSegmentation::HAND;
                    _pixels.push(point(current.first + 1,current.second - 1));
                }
            }
        }

        if ( current.first - 1 > -1 )
        {
            if ( current.second + 1 < colcount )
            {
                if ( mask.at<uchar>(current.first - 1,current.second + 1 ) == EasyHandSegmentation::EMPTY &
                     fabs(depth.at<unsigned short>(current.first - 1,current.second + 1 )-mean/pixelcount) < _depthThr
                     & depth.at<unsigned short>(current.first - 1,current.second + 1 ) > 0)
                {
                    pixelcount++;
                    mean += depth.at<unsigned short>(current.first - 1,current.second + 1 );
                    mask.at<uchar>(current.first - 1,current.second + 1 ) = EasyHandSegmentation::HAND;
                    _pixels.push(point(current.first - 1,current.second + 1));
                }
            }

            if( current.second - 1 > -1 )
            {
                if ( mask.at<uchar>(current.first - 1,current.second - 1 ) == EasyHandSegmentation::EMPTY &
                     fabs(depth.at<unsigned short>(current.first - 1,current.second - 1 )-mean/pixelcount) < _depthThr
                     & depth.at<unsigned short>(current.first - 1,current.second - 1 ) > 0)
                {
                    pixelcount++;
                    mean += depth.at<unsigned short>(current.first - 1,current.second - 1 );
                    mask.at<uchar>(current.first - 1,current.second - 1 ) = EasyHandSegmentation::HAND;
                    _pixels.push(point(current.first - 1,current.second - 1));
                }
            }
        }
    }



    if(region.width<0)
        region.isIni = false;
    else{
        region.x = miny; //cols
        region.y = minx; //rows
        region.z = minz;
        region.width = maxy - miny; //cols range
        region.height = maxx - minx; //rows range
        region.depth = maxz - minz;
        region.isIni = true;
    }
}

EasyHandSegmentation::point EasyHandSegmentation::searchNearestPixel(const cv::Rect3D &region, const cv::Mat &depth)
{

    const unsigned short *depthptr;
    int upperBoundary = region.z + region.depth;
    unsigned short minval = pow(2,15); //not nessecerely maximum, but not minimum for sure
    point result(-1,-1);

    //search for nearest pixel
    for(int i=region.y; i< (region.y+region.height); i++)
    {
        depthptr = depth.ptr<const unsigned short>(i);

        for(int j=region.x; j<(region.x+region.width); j++)
        {
            if(depthptr[j]>region.z & depthptr[j] <= upperBoundary & depthptr[j]<minval)
            {
                minval = depthptr[j];
                result.first = i;
                result.second = j;
            }
        }
    }

    return result;
}
