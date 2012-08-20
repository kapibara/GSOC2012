#ifndef KALMANHANDTRACKER_H
#define KALMANHANDTRACKER_H

#include "handtracker.h"
#include "rect3d.hpp"

#include "handsegmentation.h"

class KalmanHandTracker: public HandTracker
{
public:
    KalmanHandTracker(float timestep = 1.0/30.0);

    ~KalmanHandTracker();
    void init();
    void init(cv::Mat &mask);
    bool track(cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb);
    bool isTracking()
    {
        return _isDetected & !_isLost;
    }

    bool track(cv::Rect3D &position, cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb);

    const cv::Rect3D &lastPosition(){ return _oldPosition;}

private:

    inline void correctPosition(const cv::Mat &correction, int width, int height);

    cv::KalmanFilter _filter;
    HandSegmentation *_segmentator;

    cv::Rect3D _position;
    cv::Rect3D _oldPosition;

    const int CUMTHRESH;

    bool _isLost;
    bool _isDetected;
    int _isDetectedCount;
    float _timestep;

};

#endif // KALMANHANDTRACKER_H
