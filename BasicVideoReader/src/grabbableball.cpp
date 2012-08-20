#include "grabbableball.h"

#include <iostream>

using namespace cv;

GrabbableBall::GrabbableBall()
{
    _r = _iniR = 10;
    _color = Scalar(0,1,0);
    _pos = cv::Point(100,100);
    _alpha = 0.1;
    _grabbed = false;
    _waitThr = 5;
}

void GrabbableBall::draw(cv::Mat &image){


    if (!_grabbed)
        circle(image,_pos,_r,_color*0.5,-1); //-1 means fill the circle
    else {
        circle(image,_pos,_r, _color*0.5, 2);
    }
    putText(image,_text,cv::Point(_pos.x - _ts.width/2, _pos.y+_ts.height/2),FONT_HERSHEY_SIMPLEX,1.5,_color,2);
}

void GrabbableBall::reset(){
    _grabbed = false;
    _r = _iniR;
    _grabbWait = 0;
    _freeWait = 0;
}

void GrabbableBall::setCursorPosition(const cv::Point3i &position, bool state){
    if (!state){
        //free object
        _grabbWait = 0;
        _freeWait++;
        if (_freeWait >= _waitThr){
            _grabbed = false;
            return;
        }
    }
    else{
        _freeWait = 0;
         if (_grabbed){
            //recompute size
            _r = _R0 - _alpha*position.z;
            _pos = cv::Point(position.x, position.y);
        }else{
            //check if possible to grab
            if ((position.x - _pos.x)*(position.x - _pos.x) + (position.y - _pos.y)*(position.y - _pos.y) <= _r*_r){
                _grabbWait++;
                if (_grabbWait >= _waitThr){
                    _grabbed = true;
                    _R0 = _r + _alpha*position.z; //because alpha should be negative
                    _pos = cv::Point(position.x, position.y);
                    _grabbWait = 0;
                }
            }
        }
    }
}
