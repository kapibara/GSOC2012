#ifndef TRACKDRAWER_H
#define TRACKDRAWER_H

#include <vector>
#include "rect3d.hpp"

#include <opencv2/opencv.hpp>

class TrackDrawer
{
public:
    TrackDrawer();
    ~TrackDrawer();

    void reset();
    template<class DT>
    void drawTrack(cv::Mat &mask,const cv::Rect3D &position);
    void drawTrack(cv::Mat &mask,const cv::Point &center);
    void setColor(const cv::Scalar &color)
    {
        _color = color;
    }

private:
    std::vector<cv::Point> _path;

    cv::Scalar _color;

    void plotLine(cv::Mat &mask);
};

#endif // TRACKDRAWER_H
