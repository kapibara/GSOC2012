#ifndef HANDTRACKER_H
#define HANDTRACKER_H

#include <opencv2/opencv.hpp>

class HandTracker
{
public:
    virtual void init() = 0;
    virtual bool track(cv::Mat &Mask, const cv::Mat &depth, const cv::Mat &rgb) = 0;
};

#endif // HANDTRACKER_H
