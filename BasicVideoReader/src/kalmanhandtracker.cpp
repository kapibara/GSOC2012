#include "kalmanhandtracker.h"

#include "easyhanddetector.h"
#include "easyhandsegmentation.h"

#include <iostream>

#define KFtype float

#if KFtype == float
#define Mtype CV_32FC1
#elif KFtype == double
#define Mtype CV_64FC1
#else
#error 'Unknown KF type!'
#endif

using namespace cv;
using namespace std;

/*TODO: _isDetectedCount can probabli replace _isDetected*/

KalmanHandTracker::KalmanHandTracker(float timestep): _filter(6,3,0,CV_32FC1)
{
    _segmentator = new EasyHandSegmentation();
    _detector = new EasyHandDetector();
    _timestep = timestep;
    _minInterval = 50;
    _accThr = 10;
}

KalmanHandTracker::~KalmanHandTracker()
{
    delete _segmentator;
}

void KalmanHandTracker::init()
{
    _position.isIni = false;
    _isLost = false;
    _isDetected = false;
    _isDetectedCount = 0;
    _segmentator->init();
    _detector->reset();
    _filter.init(6,3,0,Mtype);
    _filter.transitionMatrix =
            ((Mat_<KFtype>(6,6)) <<
            1, 0, 0, _timestep, 0, 0,
            0, 1, 0, 0, _timestep, 0,
            0, 0, 1, 0, 0, _timestep,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1);
    _filter.measurementMatrix =
            ((Mat_<KFtype>(3,6)) <<
            1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0);
    _filter.measurementNoiseCov = Mat::eye(3,3,Mtype);
    _filter.processNoiseCov = Mat::eye(6,6,Mtype);
}

bool KalmanHandTracker::track(cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb)
{
    if(_isLost){
        cerr << "WARNING: tracker lost the object; reinitialize" << endl;
        cerr << "position: " <<  _position << endl;
        return false;
    }

    CV_Assert(mask.type() == CV_8UC1);
    CV_Assert(depth.type() == CV_16UC1);
    CV_Assert(rgb.type() == CV_8UC3);

    CV_Assert(mask.rows == depth.rows);
    CV_Assert(mask.cols == depth.cols);

    //Rect3D position
    _oldPosition = _position;

    //starting from here we have some idea about position
    mask.setTo(0);

    if(!_position.isIni)
    {
        //hand was not detected from previous frames
        if(!_detector->detect(_position,depth,rgb)){
            //no hand detected
            return false;
        }else{
            //set initial state
            _filter.statePre = (Mat_<KFtype>(6,1) << (_position.x + _position.width/2.0),
                                                     (_position.y + _position.height/2.0),
                                                     (_position.z + _position.depth/2.0),0,0,0);
            _detector->reset();
            _isDetected = true;
            _isDetectedCount++;
        }
    }

    //position is already predicted with KF, if it is not the first frame)

    _segmentator->segmentHand(mask,_position,depth);

    if (_isDetected & !_position.isIni){
        cerr << "position: " <<  _position << endl;
        _isLost = true;
        return false;
    }

    //draw raw position
    //rectangle(mask,Rect(_position.x,_position.y,_position.width,_position.height),100);

    Point3D center = _position.center();

    //correct the position with current measurement
    Mat toDisplay = _filter.correct((Mat_<KFtype>(3,1) << center.x,center.y,center.z));

    if (_isDetectedCount>_accThr){
        //correct and predict if we already saw some measurements
        correctPosition(toDisplay,mask.size().width, mask.size().height);
    }else{
        _isDetectedCount++;
    }

    //draw (smoothed) position
    //rectangle(mask,Rect(_position.x,_position.y,_position.width,_position.height),255);

    //predict position in the next frame
    toDisplay = _filter.predict();

    if (_isDetectedCount>_accThr){
        //correct and predict if we already saw some measurements
        correctPosition(toDisplay,mask.size().width, mask.size().height);
    }else{
        _isDetectedCount++;
    }

    return true;
}

void KalmanHandTracker::correctPosition(const cv::Mat &correction, int width, int height)
{
    Point3D center = _position.center();

    _position.x += (correction.at<KFtype>(0,0) >= center.x)?
              floor(correction.at<KFtype>(0,0) - center.x):
               ceil(correction.at<KFtype>(0,0) - center.x) ;
    _position.y += (correction.at<KFtype>(1,0) >= center.y)?
              floor(correction.at<KFtype>(1,0) - center.y):
               ceil(correction.at<KFtype>(1,0) - center.y);
    _position.z += (correction.at<KFtype>(2,0) >= center.z)?
              floor(correction.at<KFtype>(2,0) - center.z):
               ceil(correction.at<KFtype>(2,0) - center.z);

    _position.x = max(_position.x,0);
    _position.y = max(_position.y,0);
    _position.z = max(_position.z,0);
    _position.x = min(_position.x,width-1);
    _position.y = min(_position.y,height-1);
    _position.width = min(_position.width,width - _position.x - 1);
    _position.height = min(_position.height,height - _position.y - 1);

    if (_position.depth < _minInterval){
        _position.z = max(1, _position.z + _position.depth/2 - _minInterval /2);
        _position.depth = min(_minInterval, _position.z + _minInterval /2);
    }

}

bool KalmanHandTracker::track(cv::Rect3D &position, cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb)
{
    bool result = track(mask,depth,rgb);
    position = _position;

    return result;
}

void KalmanHandTracker::init(cv::Mat &mask)
{
//implement!
}
