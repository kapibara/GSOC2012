#ifndef EASYHANDTRACKER_H
#define EASYHANDTRACKER_H

#include "handtracker.h"
#include "handsegmentation.h"

#include "rect3d.hpp"

class EasyHandTracker: public HandTracker
{

public:
    EasyHandTracker();

    void init();
    bool track(cv::Mat &Mask, const cv::Mat &depth, const cv::Mat &rgb);

    void setDistanceThreashold(int delta)
    {
        _delta = delta;
    }

private:
    Rect3D _position;
    HandSegmentation *_segmentator;

    int _delta;
};

#endif // EASYHANDTRACKER_H
