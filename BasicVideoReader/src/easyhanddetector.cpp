#include "easyhanddetector.h"

using namespace cv;
using namespace std;

EasyHandDetector::EasyHandDetector(short minDistance,short maxDistance)
{
    CV_Assert(minDistance < maxDistance);

    _minDist = minDistance;
    _maxDist = maxDistance;
    _frameWidth = -1;
    _frameHeight = -1;
    _stub = 0;
    _segmentation.setDepthThreashold(_minDist,_maxDist);
}

void EasyHandDetector::reset()
{
    /*it is easy hand detector.. nothing to do here*/
    _frameWidth = -1;
    _frameHeight = -1;
}

bool EasyHandDetector::detect(cv::Rect3D &position,const cv::Mat &depth,const cv::Mat &rgb)
{
    CV_Assert(depth.type() == CV_16UC1);

    if (_frameWidth < 0){
        _frameWidth = depth.cols;
        _frameHeight = depth.rows;
        if (_stub != 0)
        {
            if (_stub->cols != depth.cols | _stub->rows != depth.rows)
            {
                //detector is very likely to be used on the same video
                //so deallocate stub only if needed
                delete _stub;
                _stub = new cv::Mat(depth.size(),CV_8UC1);
            }
        }
        else{
            _stub = new cv::Mat(depth.size(),CV_8UC1);
        }
    }

    //position is not initialized
    //init, since segmentation algorithm expects initialized position
    if (!position.isIni){
        position = cv::Rect3D(0,0,_minDist,depth.cols,depth.rows,_maxDist-_minDist);
    }

    _segmentation.segmentHand(*_stub,position,depth);

    return position.isIni;
}

EasyHandDetector::~EasyHandDetector()
{
    if (_stub!=0)
        delete _stub;
}
