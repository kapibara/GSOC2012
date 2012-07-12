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
    void setDepthThreashold(const short lowerT, const short upperT)
    {
        CV_Assert(lowerT < upperT);

        _lt = lowerT;
        _ut = upperT;
    }

    void segmentHand(cv::Mat &mask, cv::Rect3D &region,  const cv::Mat &rgb, const cv::Mat &depth);
    void segmentHand(cv::Mat &mask, cv::Rect3D &region,  const cv::Mat &depth);

private:
    point searchNearestPixel(const cv::Rect3D &probableLocation, const cv::Mat &depth);
    inline bool processNeighbor(int &pixelcount, double &mean, cv::Mat &mask, const short first, const short second, const cv::Mat &depth);

    short _lt,_ut;
    int _maxObjectSize;
    double _depthThr;
    FastQueue<point> _pixels;

};

#endif // EASYHANDSEGMENTATION_H
