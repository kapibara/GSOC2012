#ifndef GRABBEDOBJECT_H
#define GRABBEDOBJECT_H

#include <opencv2/opencv.hpp>

/*common interface for grabbed objects*/

class GrabbableObject{
public:

    /*return state of the object*/
    bool isGrabbed() {return _grabbed;} //get state

    /*reset to initial state*/
    virtual void reset() = 0;

    /*draw object on the image*/
    virtual void draw(cv::Mat &image) = 0; //draw object on the image

    /*update object position and state, if necessary*/
    virtual void setCursorPosition(const cv::Point3i &position, bool state) = 0; //changes state

protected:
    bool _grabbed;
};

#endif // GRABBEDOBJECT_H
