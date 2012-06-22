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

EasyHandTracker::~EasyHandTracker()
{
    delete _segmentator;
}

void EasyHandTracker::init()
{
    _position.isIni = false;
    _isLost = false;
    _segmentator->init();
}

void EasyHandTracker::init(cv::Mat &mask)
{

}

bool EasyHandTracker::track(cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb)
{
    if(_isLost){
        cerr << "WARNING: tracker lost the object; reinitialize" << endl;
        return false;
    }

    CV_Assert(mask.type() == CV_8UC1);
    CV_Assert(depth.type() == CV_16UC1);
    CV_Assert(rgb.type() == CV_8UC3);

    CV_Assert(mask.rows == depth.rows);
    CV_Assert(mask.cols == depth.cols);

    mask.setTo(0);

    //cut a part of depth image - hand can not move too fast
    if(!(_position.isIni))
    {
        _position = Rect3D(0,0,0,mask.size().width,mask.size().height,pow(2,15));
    }

    _segmentator->segmentHand(mask,_position,depth);

    if (!_position.isIni){
        _isLost = true;
        return false;
    }

    //make the rect a little bigger
    _position.x = std::max(_position.x-_delta,0);
    _position.y = std::max(_position.y-_delta,0);
    _position.z = std::max(_position.z -_delta,0);
    _position.width = std::min(_position.width + 2*_delta,mask.size().width-_position.x );
    _position.height = std::min(_position.height + 2*_delta,mask.size().height-_position.y);
    _position.depth = _position.depth + 2*_delta;
    _position.isIni = true;

    rectangle(mask,Rect(_position.x,_position.y,_position.width,_position.height),100);
/*
    cout << "frame size: [" << mask.rows << ":" << mask.cols << "]" << endl;
    cout << "dimensions: [" << _position.x << "," << _position.y << "," << _position.z << "]["
                           << _position.width << "," << _position.height << "," << _position.depth << "]" << endl;
*/
    return true;
}
