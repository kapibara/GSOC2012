#ifndef HANDDETECTOR_H
#define HANDDETECTOR_H

#include <opencv2/opencv.hpp>
#include "rect3d.hpp"

/*Hand detector detects hand from a sequence;
  it is assumed to be statefull
  after a hand is detected (detectHand returned true), reinit hand detector,
  otherwise next detection is not gauranteed to be correct!
  */

class HandDetector
{
public:
    virtual ~HandDetector() {}

    /*reset all parameters*/
    virtual void reset() = 0;

    /*detect the object using depth man and rgb*/
    virtual bool detect(cv::Rect3D &position,const cv::Mat &depth,const cv::Mat &rgb) = 0;

};

#endif // HANDDETECTOR_H
