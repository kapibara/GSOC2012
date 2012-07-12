#include "palmcenterdetector.h"
#include <iostream>

using namespace std;
using namespace cv;

typedef unsigned short ushort;

PalmCenterDetector::PalmCenterDetector(float timestep):_filter(6,3,0,CV_32FC1)
{
    _timestep = timestep;
    _prop = 1.5;
}

void PalmCenterDetector::reset()
{
    cout << "timestep: " << _timestep << endl;
    _iniCount = 0;
    _filter.init(6,3,0,CV_32FC1);
    _filter.transitionMatrix =
            (Mat_<float>(6,6) <<
             1, 0, 0, _timestep, 0, 0,
             0, 1, 0, 0, _timestep, 0,
             0, 0, 1, 0, 0, _timestep,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1);
    _filter.measurementMatrix =
            ((Mat_<float>(3,6)) <<
            1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0);
    _filter.measurementNoiseCov = Mat::eye(3,3,CV_32FC1);
    _filter.processNoiseCov = Mat::eye(6,6,CV_32FC1);
}

void PalmCenterDetector::computeCenter(Point &maxLoc, double &r, const Mat &mat)
{
    Mat output(mat.size(),CV_32FC1);

    //compute fast distace transform
    distanceTransform(mat,output,CV_DIST_L2,3);

    minMaxLoc(output,0,&r,0,&maxLoc);
}

void PalmCenterDetector::detect(Point &p, Point &ps, double &r, double &rs, const Rect3D &pos, const Mat &mat)
{
    if(_iniCount < 10){
        //cut mat part and compute EDT on it
        Rect pos2D(pos.x,pos.y,pos.width,pos.height);
        Mat patch = mat(pos2D);

        computeCenter(p,r,patch);

        p.x += pos.x;
        p.y += pos.y;

        if (_iniCount == 0){
            _filter.statePost = (Mat_<float>(6,1) << p.x, p.y, r, 0, 0, 0);
        }else{
            _filter.predict();
            _filter.correct((Mat_<float>(3,1)<< p.x,p.y,r));
        }

        ps = p;
        rs = r;
        _iniCount++;
    }
    else{

        double rproportion = 1.5;
        Mat state = _filter.predict();
        int x = std::max((int)(state.at<float>(0,0)-_prop*state.at<float>(2,0)),0),
            y = std::max((int)(state.at<float>(1,0)-_prop*state.at<float>(2,0)),0),
            width = std::min(2*_prop*state.at<float>(2,0),mat.size().width - (x+_prop*state.at<float>(2,0))-1),
            height = std::min(2*_prop*state.at<float>(2,0),mat.size().height - (y+_prop*state.at<float>(2,0))-1);

        if (width <= 0 | height <= 0 | x < 0 | y < 0 | x + width >= mat.size().width | y + height >= mat.size().height){
            _iniCount = 0;
            reset();
        }


        Rect pos2D(x,y,width,height);
        Mat patch = mat(pos2D);

        computeCenter(p,r,patch);

        p.x += pos2D.x;
        p.y += pos2D.y;

        //smooth
        state = _filter.correct((Mat_<float>(3,1) << p.x, p.y, r));
        ps.x = state.at<float>(0,0);
        ps.y = state.at<float>(1,0);
        rs = state.at<float>(2,0);
    }
}

void PalmCenterDetector::checkState(Mat &state, int width, int height){
    state.at<float>(0,0) = max(state.at<float>(0,0),_prop*state.at<float>(2,0)+1);
    state.at<float>(1,0) = max(state.at<float>(1,0),_prop*state.at<float>(2,0)+1);
    state.at<float>(0,0) = min(state.at<float>(0,0),width-_prop*state.at<float>(2,0)-1);
    state.at<float>(1,0) = min(state.at<float>(1,0),height-_prop*state.at<float>(2,0)-1);
}
