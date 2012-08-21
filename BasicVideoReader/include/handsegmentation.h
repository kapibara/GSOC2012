#ifndef HANDSEGMENTATION_H
#define HANDSEGMENTATION_H

#include <opencv2/opencv.hpp>
#include "rect3d.hpp"

/*segment hand interface*/

class HandSegmentation
{
public:
    virtual ~HandSegmentation(){}
    virtual void init() = 0;
    virtual void segmentHand(cv::Mat &mask, cv::Rect3D &region, const cv::Mat &rgb, const cv::Mat &depth) = 0;
    virtual void segmentHand(cv::Mat &mask, cv::Rect3D &region, const cv::Mat &depth) = 0;
};

#endif // HANDSEGMENTATION_H
