#ifndef EASYHANDSEGMENTATION_H
#define EASYHANDSEGMENTATION_H

#include <utility>

#include <handsegmentation.h>
#include "fastqueue.hpp"

class EasyHandSegmentation : public HandSegmentation
{

public:
    enum {EMPTY=0, HAND=255};
    typedef std::pair<short, short> point;

    EasyHandSegmentation(int maxObjectSize = 100000);
    ~EasyHandSegmentation();
    void init();
    void segmentHand(cv::Mat &mask, cv::Rect3D &region,  const cv::Mat &rgb, const cv::Mat &depth);
    void segmentHand(cv::Mat &mask, cv::Rect3D &region,  const cv::Mat &depth);

private:
    point searchNearestPixel(const cv::Rect3D &probableLocation, const cv::Mat &depth);

    int _maxObjectSize;
    double _depthThr;
    FastQueue<point> _pixels;

};

#endif // EASYHANDSEGMENTATION_H
