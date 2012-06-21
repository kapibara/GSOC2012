#include "easyhandtracker.h"
#include "easyhandsegmentation.h"

#include <iostream>

using namespace cv;
using namespace std;

EasyHandTracker::EasyHandTracker()
{
    _delta = 20;
    _segmentator = new EasyHandSegmentation();
}

void EasyHandTracker::init()
{
    _position._isInit = false;
}

bool EasyHandTracker::track(cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb)
{
    Rect rect;
    Mat subRect;
    unsigned short INFTY = pow(2,15);

    CV_Assert(mask.type() == CV_8UC1);
    CV_Assert(depth.type() == CV_16UC1);
    CV_Assert(rgb.type() == CV_8UC3);

    CV_Assert(mask.rows == depth.rows);
    CV_Assert(mask.cols == depth.cols);

    mask.setTo(0);

    //cut a part of depth image - hand can not move too fast
    if(_position._isInit)
    {/*
        subRect = depth(Rect(_position.x , _position.y, _position.width, _position.height ));
        //make a copy not to change the original
        subRect = subRect.clone();

        //filter by depth
        int rowcount = subRect.rows, colcount = subRect.cols;
        unsigned short *subRectPtr;

        if (subRect.isContinuous())
        {
            colcount*= rowcount;
            rowcount = 1;
        }

        // may be i dont need it, no i need this -> handle occlusions -> can track 2 hands
        for(int i=0; i<rowcount; i++)
        {
            subRectPtr = subRect.ptr<unsigned short>(i);

            for(int j=0; j<colcount; j++)
            {
                if(subRectPtr[j]<_position._z | subRectPtr[j]> (_position._z + _position._depth))
                {
                    //out of range - too near or too far
                    subRectPtr[j] = 0;
                }
            }
        }
*/
        Mat subMask = mask(Rect(_position._x , _position._y, _position._width, _position._height ));

        _segmentator->segmentHand(subMask,rect,subRect);

        //find new range
        uchar *subMaskPtr;
        int maxDepth = 0;
        int minDepth = INFTY;
        rowcount = subRect.rows;
        colcount = subRect.cols;

        for(int i=0; i<rowcount; i++)
        {
            subRectPtr = subRect.ptr<unsigned short>(i);
            subMaskPtr = subMask.ptr<uchar>(i);

            for(int j=0; j<colcount; j++)
            {

                if(subRectPtr[j]<minDepth & subMaskPtr[j] > 0)
                {
                    minDepth = subRectPtr[j];
                }

                if(subRectPtr[j]>maxDepth & subMaskPtr[j] > 0)
                {
                    maxDepth = subRectPtr[j];
                }
            }
        }

        if(minDepth > maxDepth){
            cout << "minDepth:maxDepth" << minDepth << ":" << maxDepth << endl;
            minDepth = maxDepth = 0;
            return false;
        }

        //define new rect
        _position._x = std::max(_position._x + rect.x-_delta,0);
        _position._y = std::max(_position._y + rect.y-_delta,0);
        _position._z = std::max(minDepth -_delta,0);
        _position._width = std::min(rect.width + 2*_delta,mask.size().width-_position._x );
        _position._height = std::min(rect.height + 2*_delta,mask.size().height-_position._y);
        _position._depth = maxDepth - minDepth + 2*_delta;
        _position._isInit = true;

        rectangle(mask,Rect(_position._x , _position._y, _position._width, _position._height ),100);
    }
    else{
        _segmentator->segmentHand(mask,rect,depth);

        int rowcount = depth.rows, colcount = depth.cols;
        int maxDepth = 0;
        int minDepth = INFTY;
        unsigned char *maskPtr;
        const unsigned short *depthPtr;

        if (depth.isContinuous() & mask.isContinuous());
        {
            colcount*= rowcount;
            rowcount = 1;
        }

        for(int i=0; i<rowcount; i++)
        {
            depthPtr = depth.ptr<const unsigned short>(i);
            maskPtr = mask.ptr<unsigned char>(i);

            for(int j=0; j<colcount; j++)
            {
                if(depthPtr[j]<minDepth & maskPtr[j] > 0)
                {
                    minDepth = depthPtr[j];
                }

                if(depthPtr[j]>maxDepth & maskPtr[j] > 0)
                {
                    maxDepth = depthPtr[j];
                }
            }
        }

        //define new rect
        _position._x = std::max(rect.x-_delta,0);
        _position._y = std::max(rect.y-_delta,0);
        _position._z = std::max(minDepth -_delta,0);
        _position._width = std::min(rect.width + 2*_delta,mask.size().width-_position._x );
        _position._height = std::min(rect.height + 2*_delta,mask.size().height-_position._y);
        _position._depth = maxDepth - minDepth + 2*_delta;
        _position._isInit = true;

        if(minDepth > maxDepth){
            minDepth = maxDepth = 0;
            _position._isInit = false; //because we have not started to track yet!
        }
    }

    cout << "frame size: [" << mask.rows << ":" << mask.cols << "]" << endl;
    cout << "dimensions: [" << _position._x << "," << _position._y << "," << _position._z << "]["
                           << _position._width << "," << _position._height << "," << _position._depth << "]" << endl;

    return true;
}
