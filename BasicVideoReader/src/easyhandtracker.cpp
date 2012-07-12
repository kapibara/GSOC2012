#include "easyhandtracker.h"
#include "easyhanddetector.h"
#include "easyhandsegmentation.h"

#include <iostream>

using namespace cv;
using namespace std;

EasyHandTracker::EasyHandTracker()
{
    _delta = 20;
    _detector = new EasyHandDetector(100,1000); //close-range detector; values are in mm
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
    _isDetected = false;
    _segmentator->init();
    _detector->reset();
}

void EasyHandTracker::init(cv::Mat &mask)
{
//implement!
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

    //starting from here we have some idea about position
    mask.setTo(0);


    if(!_position.isIni)
    {
        //hand was not detected from previous frames
        if(!_detector->detect(_position,depth,rgb)){
            //no hand detected
            return false;
        }else{
            _detector->reset();
            _isDetected = true;
        }
    }

    _segmentator->segmentHand(mask,_position,depth);

    if (_isDetected & !_position.isIni){
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

    //rectangle(mask,Rect(_position.x+1,_position.y+1,_position.width-1,_position.height-1),255);

    return true;
}

bool EasyHandTracker::track(cv::Rect3D &position, cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb)
{
    bool result = track(mask,depth,rgb);
    position = _position;

    return result;
}
