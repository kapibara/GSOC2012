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

    bool extractContour(std::vector<cv::Point> &contour,cv::Mat &mask,bool simplifyContour = false);
    bool initTemplate(cv::Mat &mask,bool simplifyContour = false);
    double compareToTemplate(cv::Mat &mask);
    bool isInitialized() {return (_template.size()>0);}
    void saveContour(const std::string &filename);
    void detectFingerTips(std::vector<cv::Point> &tips, const cv::Rect3D &location, const cv::Mat &mask);
    void rejectNonFingers(std::vector<cv::Point> &realTips, const cv::Rect3D &location);

private:
    bool pixelwiseContourTracking(std::vector<cv::Point> &contour,const cv::Rect3D &location, const cv::Mat &mask);

    std::vector<cv::Point> _template;
    std::vector<cv::Point> _contour;
    std::vector<int> _ids;

    bool _isSimplified;
};

#endif // CONTOURBASEDFINGERDETECTOR_H
