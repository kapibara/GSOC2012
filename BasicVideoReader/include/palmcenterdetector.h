#ifndef PALMCENTERDETECTOR_H
#define PALMCENTERDETECTOR_H

#include <opencv2/opencv.hpp>
#include "rect3d.hpp"

/*Class detects palm center based on fast distance transformation as the widest part of the hand;
  Kalman filter is used to smooth the output*/

class PalmCenterDetector
{
public:
    PalmCenterDetector(float timestep = 1.0/30.0);

    /*experimental function; smooth the track a little more;
      however provokes slower responce; default is true*/
    void setUseRobust(bool useRobust){ _useRobust = useRobust;}

    /*set search window increase factor for predicted window: value in [0,1] will cause the search window to decrease;
      values>1 will increase search window and make tracking more stable*/

    void setIncFactor(double inc) {_prop = inc;}

    /*set search window increase factor for predicted window: value in [0,1] will cause the search window to decrease;
      values>1 will increase search window and make tracking more stable; default is 1.5*/
    void setAccThr(int accThr) {_accThr = accThr;}

    /*set decrease distance threashold, which is allowed by robust center estimation
      (in percentage from maximum distance; default is 0.1)*/
    void setDecreasePercentage(float p) {_percentage = p;}

    /* reset() the detector*/
    void reset();
    void detect(cv::Point &ps, double &rs,const cv::Rect3D &pos, const cv::Mat &mat);
    void detect(cv::Point &ps, double &rs, const cv::Mat &patch);

    cv::Rect getPredictBox(int mwidth,int mheight);

private:
    void computeCenter(cv::Point &p, double &r, const cv::Rect &region,const cv::Mat &mat);

    /*robust center detection algorithm;
      try to find center as close to the one in the previous frame as possible;
      it is not necesserily maximum of distance transform, but the difference from maximum should not be greater
      then max*_percentage*/
    void computeCenterRobust(cv::Point &p, double &r, const cv::Point &lastP,  const cv::Mat &mat);

    cv::KalmanFilter _filter; //2D + 1 Kalman filter
    cv::Point _lastPosition;
    int _iniCount;

    float _timestep;
    float _percentage;
    float _prop;
    bool _useRobust;
    int _accThr;
};


#endif // PALMCENTERDETECTOR_H
