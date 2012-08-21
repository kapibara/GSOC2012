#ifndef HANDTRACKER_H
#define HANDTRACKER_H

#include <opencv2/opencv.hpp>

#include "handdetector.h"

/*
  Hand tracker class allows to track the hand;
  It has at least one componenent:
  -hand detector
  Hand tracker takes the ownership of its hand detector, e.g. it disposes it;
  Do not deallocate detector when it is used by Hand tracker
  Hand tracker expects frames to come in the right order;
  Hand tracker is statefull, which means that it uses information about previous hand position
  to compute the current hand position;
  If hand tracker lost hand, it should be reinitialized
*/

class HandTracker
{
public:
    virtual ~HandTracker(){if (_detector!=0) delete _detector;}
    virtual void init() = 0;
    virtual void init(cv::Mat &mask) = 0;
    virtual bool track(cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb) = 0;
    virtual bool isTracking() = 0;

    /*set own hand detector; init() function should be called after setting hand detector*/
    virtual void setHandDetector(HandDetector *detector)
    {
        if (_detector!=0) delete _detector;
        _detector = detector;
        _detector->reset();
    }

protected:
    HandDetector *_detector;
};

#endif // HANDTRACKER_H
