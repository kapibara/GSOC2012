#ifndef CONTOURBASEDFINGERDETECTOR_H
#define CONTOURBASEDFINGERDETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "rect3d.hpp"
#include "dynamictimewarping.h"

class ContourBasedFingerDetector
{
public:
    ContourBasedFingerDetector();
    ~ContourBasedFingerDetector();

    void reset() {
        _orderedTips.clear();
    }

    void setNumberOfScales(int scalesCount) {_scalesCount=scalesCount;}
    void setMinCosAllowed(float minCos) {_cosThr =  minCos;}
    void setSqrDistance(int sqrDistance) {_sqrDistance = sqrDistance;}

    void saveContour(const std::string &filename);
    void detectFingerTipsSuggestions(std::vector<cv::Point> &tips, const cv::Mat &patch);
    void locateFingerTips(std::vector<cv::Point> &tips);
    void orderFingerTips(std::vector<cv::Point> &tips, const cv::Point &palmCenter);
    const std::vector<cv::Point> & getOrderedTips(){
        return _orderedTips;
    }

private:

    std::vector<cv::Point> _contour;
    std::vector<int> _ids;
    std::vector<cv::Point> _orderedTips;
    cv::Size _patchSize;

    int _scalesCount;
    float _cosThr;
    int _sqrDistance;
};

#endif // CONTOURBASEDFINGERDETECTOR_H


