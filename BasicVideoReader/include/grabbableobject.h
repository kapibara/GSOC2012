#ifndef GRABBEDOBJECT_H
#define GRABBEDOBJECT_H

#include <opencv2/opencv.hpp>

class GrabbableObject{
public:

    bool isGrabbed() {return _grabbed;} //get state

    virtual void reset() = 0;
    virtual void draw(cv::Mat &image) = 0; //draw object on the image
    virtual void setCursorPosition(const cv::Point3i &position, bool state) = 0; //changes state

protected:
    bool _grabbed;
};

#endif // GRABBEDOBJECT_H
