#ifndef GRABBABLEBALL_H
#define GRABBABLEBALL_H

#include <string>
#include "grabbableobject.h"

/*The class implements an object, that can be dragged around using coursor;
  the object has state and 3D position*/

class GrabbableBall: public GrabbableObject
{
public:
    GrabbableBall();

    /*set object and text color;
      default is green*/
    void setColor(const cv::Scalar &color) {_color = color;}
    /*set original object radius; default is 10*/
    void setIniRadius(double R) {_r=_iniR = R;}
    /*set initial object 2D position; default is (100,100)*/
    void setIniPosition(const cv::Point &iniPosition) {_pos = iniPosition; }
    /*set how much object size changes depending on depth;
      specifying negative values will lead to object size increase when
      moving closer to the screen; default is 0.1*/
    void setAlpha(double alpha) {_alpha = alpha;}
    /*threshold as a number of setCursorPosition() calls before grabbing the object;
      default is 5*/
    void setGrabbWaitThr(int thr) { _waitThr = thr;}
    /*set text written on the ball*/
    void setString(const std::string &text) {_text = text;_ts = cv::getTextSize(_text,cv::FONT_HERSHEY_SIMPLEX,1.5,2,0);}

    /*reset the state*/
    void reset();

    /* draw the ball with text in the right position on the image*/
    void draw(cv::Mat &image);

    /*report current cursor position and state of the hand:
      position - 3D position
      state == true, if the hand posture is recognized as grabbed
      else state == false*/
    void setCursorPosition(const cv::Point3i &position, bool state);

private:

    cv::Scalar _color;
    cv::Point _pos;

    double _R0;
    double _alpha;
    double _r;

    double _iniR;
    int _grabbWait;
    int _waitThr;
    int _freeWait;

    std::string  _text;
    cv::Size _ts;

};

#endif // GRABBABLEBALL_H
