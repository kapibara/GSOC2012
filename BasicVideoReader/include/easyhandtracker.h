#ifndef EASYHANDTRACKER_H
#define EASYHANDTRACKER_H

#include "handtracker.h"
#include "handsegmentation.h"

#include "rect3d.hpp"

class EasyHandTracker: public HandTracker
{

public:
    EasyHandTracker();
    ~EasyHandTracker();

    /*Hand Tracker interface method;
      should be called before start tracking*/
    void init();

    /*Hand Tracker interface method;
      should be called before start tracking*/
    void init(cv::Mat &mask);

    /*interface method; checks if the object is detected and if yes, starts tracking*/
    bool track(cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb);

    /*interface method; checks if the object is detected and if yes, starts tracking*/
    bool track(cv::Rect3D &position, cv::Mat &mask, const cv::Mat &depth, const cv::Mat &rgb);

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

    /*set threashold, how near the object is to the camera*/
    void setDistanceThreashold(int delta)
    {
        _delta = delta;
    }

private:
    cv::Rect3D _position;
    HandSegmentation *_segmentator;

    int _delta;
    bool _isLost;
    bool _isDetected;
};

#endif // EASYHANDTRACKER_H
