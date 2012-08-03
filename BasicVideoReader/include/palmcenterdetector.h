#ifndef PALMCENTERDETECTOR_H
#define PALMCENTERDETECTOR_H

#include <opencv2/opencv.hpp>
#include "rect3d.hpp"

class PalmCenterDetector
{
public:
    PalmCenterDetector(float timestep = 1.0/30.0);

    void setUseRobust(bool useRobust){
        _useRobust = useRobust;
    }

    void reset();
    void detect(cv::Point &ps, double &rs,const cv::Rect3D &pos, const cv::Mat &mat);

    cv::Rect getPredictBox(int mwidth,int mheight);

private:
    void computeCenter(cv::Point &p, double &r, const cv::Mat &mat);

    void computeCenterRobust(cv::Point &p, double &r, const cv::Point &lastP,  const cv::Mat &mat);

//    void checkState(cv::Mat &state, int width, int height);

    cv::KalmanFilter _filter; //2D + 1 Kalman filter
    cv::Point _lastPosition;
    int _iniCount;
    float _timestep;
    float _prop;
    bool _useRobust;
};


#endif // PALMCENTERDETECTOR_H
