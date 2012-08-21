#ifndef KALMANHANDTRACKER_H
#define KALMANHANDTRACKER_H

#include "handtracker.h"
#include "rect3d.hpp"

#include "handsegmentation.h"

/*Class implements Kalman filter tracking*/

class KalmanHandTracker: public HandTracker
{
public:
    KalmanHandTracker(float timestep = 1.0/30.0);

    ~KalmanHandTracker();

    /*set number of detections before start tracking (estimate corresponding Kalman filter matrices;
      default is 10;*/
    void setAccThr(int accThr) {_accThr = accThr;}

    /*this is minimal search interval in depth;
      it is motivates by the fact that since depth interval can practically become non-existent,
      too small value can make the tracker to loose track all the time;
      default is 50;*/
    void setMinInterval(int minInterval) {_minInterval = minInterval;}

    void init();
    void init(cv::Mat &mask);
    bool track(cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb);

    /*set another hand segmentation algorithm;
      the ownership on the hs in taken by KalmanHandTracker class;
      do not delete it */
    void setHandSegmentation(HandSegmentation *hs) {
        delete _segmentator; //delete the old one
        _segmentator = hs;
    }

    /*check object state: returns true, if an object is tracked and false otherwise*/
    bool isTracking()
    {
        return _isDetected & !_isLost;
    }

    /*the function receives depth and rgb images and calculates new hand mask;
      the return value is true, if hand is depected and false otherwise*/
    bool track(cv::Rect3D &position, cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb);

    /*returns tha last predicted position;
      done for experimental purposes*/
    const cv::Rect3D &lastPosition(){ return _oldPosition;}

private:

    inline void correctPosition(const cv::Mat &correction, int width, int height);

    cv::KalmanFilter _filter;
    HandSegmentation *_segmentator;

    cv::Rect3D _position;
    cv::Rect3D _oldPosition;

    int _accThr;

    bool _isLost;
    bool _isDetected;
    int _isDetectedCount;
    float _timestep;

    int _minInterval;

};

#endif // KALMANHANDTRACKER_H
