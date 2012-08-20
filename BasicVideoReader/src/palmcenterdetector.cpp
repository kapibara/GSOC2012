#include "palmcenterdetector.h"
#include <iostream>

using namespace std;
using namespace cv;

typedef unsigned short ushort;

PalmCenterDetector::PalmCenterDetector(float timestep):_filter(6,3,0,CV_32FC1)
{
    _timestep = timestep;
    _prop = 1.5;
    _useRobust = true;
    _lastPosition = Point(0,0);
    _accThr = 10;
}

void PalmCenterDetector::reset()
{
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

void PalmCenterDetector::computeCenterRobust(cv::Point &p, double &r, const cv::Point &lastP, const Mat &mat)
{
    Mat output(mat.size(),CV_32FC1);
    double max, threshold, smallestDist;
    Point maxLoc;
    float *imgptr;

    //compute fast distace transform
    distanceTransform(mat,output,CV_DIST_L2,3);

    minMaxLoc(output,0,&max,0,&maxLoc);

    p = maxLoc;
    r = max;

    threshold = 0.1*max;

    smallestDist = (lastP.x - maxLoc.x)*(lastP.x - maxLoc.x) + (lastP.y - maxLoc.y)*(lastP.y - maxLoc.y);

    //now search for all pixels that differ from maximum a little
    //some kind of general score as a concensus between "no noise" and as big as possible" needed!
    for(int i=0; i<output.rows; i++){
        imgptr = output.ptr<float>(i);
        for (int j=0; j<output.cols; j++){
            if(max - imgptr[j] < threshold){

            // that is not too far from maximum
                if((lastP.x - j)*(lastP.x - j) + (lastP.y - i)*(lastP.y - i) < smallestDist){
                    p.x = j;
                    p.y = i;
                    r= imgptr[j];
                    smallestDist = (lastP.x - j)*(lastP.x - j) + (lastP.y - i)*(lastP.y - i);
                }
            }
        }
    }
}

void PalmCenterDetector::computeCenter(Point &maxLoc, double &r, const cv::Rect &region, const Mat &patch)
{
    Mat output(patch.size(),CV_32FC1);

    //compute fast distace transform on the whole segmented region
    distanceTransform(patch,output,CV_DIST_L2,3);

    //cut a part of segmented region
    Mat submat = output(region);

    //search minMax in the part region
    minMaxLoc(submat,0,&r,0,&maxLoc);

    //translate found in the region coordinate system
    maxLoc.x += region.x;
    maxLoc.y += region.y;
}

cv::Rect PalmCenterDetector::getPredictBox(int mwidth,int mheight)
{
    Mat state = _filter.predict();

    int x = std::max((int)(state.at<float>(0,0)-_prop*state.at<float>(2,0)),0),
        y = std::max((int)(state.at<float>(1,0)-_prop*state.at<float>(2,0)),0),
        width = std::min(2*_prop*state.at<float>(2,0),mwidth- (x+_prop*state.at<float>(2,0))-1),
        height = std::min(2*_prop*state.at<float>(2,0),mheight - (y+_prop*state.at<float>(2,0))-1);

    return Rect(x,y,width,height);
}

void PalmCenterDetector::detect(Point &ps, double &rs, const Mat &patch)
{
    ps.x = ps.y = 0;

    if(_iniCount <= _accThr){

        if (_useRobust == false | _lastPosition.x < 0 | _lastPosition.y < 0 | _lastPosition.x >= patch.size().width | _lastPosition.y >= patch.size().height)
        {
            computeCenter(ps,rs,Rect(0,0,patch.size().width-1,patch.size().height-1),patch);
        }
        else{
            computeCenterRobust(ps,rs,_lastPosition,patch);
        }

        if (_iniCount == 0){
            _filter.statePost = (Mat_<float>(6,1) << ps.x, ps.y, rs, 0, 0, 0);
        }else{
            _filter.predict();
            _filter.correct((Mat_<float>(3,1)<< ps.x,ps.y,rs));
        }

        _iniCount++;
    }
    else{
        Mat state = _filter.predict();

        int x = std::max((int)(state.at<float>(0,0)-_prop*state.at<float>(2,0)),0),
            y = std::max((int)(state.at<float>(1,0)-_prop*state.at<float>(2,0)),0),
            width = std::min(2*_prop*state.at<float>(2,0),patch.size().width - (x+_prop*state.at<float>(2,0))-1),
            height = std::min(2*_prop*state.at<float>(2,0),patch.size().height - (y+_prop*state.at<float>(2,0))-1);

        if (width <= 0 | height <= 0 | x < 0 | y < 0 | x + width >= patch.size().width | y + height >= patch.size().height){
            _iniCount = 0;
            cerr << "Hand lost; reinitialize" << endl;
            reset();
            return;
        }

        if (_useRobust == false | _lastPosition.x < 0 | _lastPosition.y < 0 | _lastPosition.x >= width | _lastPosition.y >= height){
            computeCenter(ps,rs,Rect(x,y,width,height),patch);
        }
        else{
            computeCenterRobust(ps,rs,_lastPosition,patch);
        }

        //smooth
        state = _filter.correct((Mat_<float>(3,1) << ps.x, ps.y, rs));

        ps.x = state.at<float>(0,0);
        ps.y = state.at<float>(1,0);
        rs = state.at<float>(2,0);
    }

    _lastPosition = ps;
}

void PalmCenterDetector::detect(Point &ps, double &rs, const Rect3D &pos, const Mat &mat)
{
    Rect pos2D(pos.x,pos.y,pos.width,pos.height);
    Mat patch = mat(pos2D);

    if(_iniCount < _accThr){
        //cut mat part and compute EDT on it

        Point oldPoint = _lastPosition-Point(pos.x,pos.y);

        if (_useRobust == false | oldPoint.x < 0 | oldPoint.y < 0 | oldPoint.x >= patch.size().width | oldPoint.y >= patch.size().height){
            computeCenter(ps,rs,Rect(0,0,patch.size().width-1,patch.size().height-1),patch);
        }
        else{
            computeCenterRobust(ps,rs,oldPoint,patch);
        }

        ps.x += pos.x;
        ps.y += pos.y;

        if (_iniCount == 0){
            _filter.statePost = (Mat_<float>(6,1) << ps.x, ps.y, rs, 0, 0, 0);
        }else{
            _filter.predict();
            _filter.correct((Mat_<float>(3,1)<< ps.x,ps.y,rs));
        }

        _iniCount++;
    }
    else{
        Mat state = _filter.predict();

        //translate predicted position in relative coordinates
        int x = std::max((int)(state.at<float>(0,0)-_prop*state.at<float>(2,0))-pos.x,0),
            y = std::max((int)(state.at<float>(1,0)-_prop*state.at<float>(2,0))-pos.y,0),
            width = std::min(2*_prop*state.at<float>(2,0),patch.size().width - (x+_prop*state.at<float>(2,0))-1),
            height = std::min(2*_prop*state.at<float>(2,0),patch.size().height - (y+_prop*state.at<float>(2,0))-1);

        if (width <= 0 | height <= 0 | x < 0 | y < 0 | x + width >= patch.size().width | y + height >= patch.size().height){
            _iniCount = 0;
            cerr << "Hand lost; reinitialize" << endl;
            reset();
            return;
        }

        Rect pos2Dpredicted = Rect(x,y,width,height);

        Point oldPoint = _lastPosition-Point(x,y);

        if (_useRobust == false | oldPoint.x < 0 | oldPoint.y < 0 | oldPoint.x >= width | oldPoint.y >= height){
            computeCenter(ps,rs,pos2Dpredicted,patch);
        }
        else{
            computeCenterRobust(ps,rs,oldPoint,patch);
        }

        ps.x += pos2D.x;
        ps.y += pos2D.y;


        //smooth
        state = _filter.correct((Mat_<float>(3,1) << ps.x, ps.y, rs));

        ps.x = state.at<float>(0,0);
        ps.y = state.at<float>(1,0);
        rs = state.at<float>(2,0);
    }

    _lastPosition = ps;
}
