#ifndef EASYHANDSEGMENTATION_H
#define EASYHANDSEGMENTATION_H

#include <utility>

#include <handsegmentation.h>
#include "fastqueue.hpp"


/*class implementing fast region growing for hand segmentation*/
class EasyHandSegmentation : public HandSegmentation
{

public:
    enum {EMPTY=0, HAND=255};
    typedef std::pair<short, short> point;

    EasyHandSegmentation(int maxObjectSize = 100000);
    ~EasyHandSegmentation();

    /*init method; should be called after class creation*/
    void init();

    /*Set b-depth threashold for region growing algorithm*/
    void setUnionThreashold(int uniThr)
    {
        _depthThr = uniThr;
    }

    /*set the area used for segmentation -  by depth*/
    void setDepthThreashold(const short lowerT, const short upperT)
    {
        CV_Assert(lowerT < upperT);

        _lt = lowerT;
        _ut = upperT;
    }

    /*set maximal allowed object size in pixels*/
    void setMaxObjectSize(int maxObjectSize){
        _maxObjectSize = maxObjectSize;
    }

    /*segment hand using both types of input;
      rgb part is thrown away;
      for compatibility with HandSegmentation interface*/
    void segmentHand(cv::Mat &mask, cv::Rect3D &region,  const cv::Mat &rgb, const cv::Mat &depth);
    /*segmen hand using depth only; the real algorithm*/
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
