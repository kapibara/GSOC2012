#ifndef HANDTRACKER_H
#define HANDTRACKER_H

#include <opencv2/opencv.hpp>

#include "handsegmentation.h"

class HandTracker
{
public:
    virtual ~HandTracker(){}
    virtual void init() = 0;
    virtual void init(cv::Mat &mask) = 0;
    virtual bool track(cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb) = 0;
};

#endif // HANDTRACKER_H
