#ifndef EASYHANDSEGMENTATION_H
#define EASYHANDSEGMENTATION_H

#include <utility>

#include <handsegmentation.h>

class EasyHandSegmentation : public HandSegmentation
{

public:
    enum {EMPTY=0, HAND=255};
    typedef std::pair<short, short> point;

    EasyHandSegmentation(int maxHandSize = 10000, double distance = 1000);
    void segmentHand(cv::Mat &mask, cv::Rect3D &region,  const cv::Mat &rgb, const cv::Mat &depth);
    void segmentHand(cv::Mat &mask, cv::Rect3D &region,  const cv::Mat &depth);

private:
    point searchNearestPixel(const cv::Rect3D &probableLocation, const cv::Mat &depth);

    int _maxHandSizeCoeff;
    double _depthThr;
};

#endif // EASYHANDSEGMENTATION_H
