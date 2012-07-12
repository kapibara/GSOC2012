#ifndef PALMCENTERDETECTOR_H
#define PALMCENTERDETECTOR_H

#include <opencv2/opencv.hpp>
#include "rect3d.hpp"

class PalmCenterDetector
{
public:
    PalmCenterDetector(float timestep = 1.0/30.0);

    void reset();
    void detect(cv::Point &p, cv::Point &ps, double &r, double &rs,const cv::Rect3D &pos, const cv::Mat &mat);

private:
    void computeCenter(cv::Point &p, double &r, const cv::Mat &mat);

    void checkState(cv::Mat &state, int width, int height);

    cv::KalmanFilter _filter; //2D + 1 Kalman filter
    int _iniCount;
    float _timestep;
    float _prop;
};

#endif // PALMCENTERDETECTOR_H
