#ifndef EASYHANDDETECTOR_H
#define EASYHANDDETECTOR_H

#include "handdetector.h"
#include "easyhandsegmentation.h"

/*detects every object between [minDistance,maxDistace]*/

class EasyHandDetector : public HandDetector
{
public:
    EasyHandDetector(short minDistance=0,short maxDistance=pow(2,14));
    ~EasyHandDetector();

    void reset();
    bool detect(cv::Rect3D &position,const cv::Mat &depth, const cv::Mat &rgb);
private:

    int _minDist,_maxDist;
    short _frameWidth,_frameHeight;
    EasyHandSegmentation _segmentation;
    cv::Mat *_stub;
};

#endif // EASYHANDDETECTOR_H
