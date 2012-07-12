#include "contourbasedfingerdetector.h"

#include "fastqueue.hpp"

#include <queue>
#include <iostream>
#include <fstream>
#include <iterator>

using namespace std;
using namespace cv;

ContourBasedFingerDetector::ContourBasedFingerDetector()
{
}

ContourBasedFingerDetector::~ContourBasedFingerDetector()
{

}

bool ContourBasedFingerDetector::extractContour(vector<Point> &contour, cv::Mat &mask, bool simplifyContour)
{
    CV_Assert(mask.type() == CV_8UC1);

    vector<vector <Point> > contours;
    int mode = CV_RETR_LIST, method = CV_CHAIN_APPROX_SIMPLE;

    //this function actually changes mask..
    findContours(mask,contours,mode,method);

    if(contours.size()>0){

        //take a contour with the biggest number of pixels - should be hand
        int size = contours[0].size(), index = 0;

        for(int i=1; i<contours.size(); i++)
        {

            if (contours[i].size()>size){
                size = contours[i].size();
                index = i;
            }
        }

        if (simplifyContour)
            approxPolyDP(contours[index],contour,2,true);
        else
            contour = contours[index];
    }
}

bool ContourBasedFingerDetector::initTemplate(cv::Mat &mask, bool simplifyContour)
{
    _template.clear();

    extractContour(_template,mask,simplifyContour);
    _isSimplified =  simplifyContour;

    return (_template.size()>0);
}

void ContourBasedFingerDetector::saveContour(const string &filename)
{
    ofstream out(filename.c_str());

    out << _contour.size() << endl;

    for(int i=0; i<_contour.size(); i++){
        out << _contour[i].x << " " << _contour[i].y << endl;
    }

    out << _ids.size() << endl;

    for(int i=0; i<_ids.size(); i++){
        out << _ids[i] << endl;
    }

    out.close();
}

double ContourBasedFingerDetector::compareToTemplate(cv::Mat &mask)
{
    if(_template.size()==0){
        cerr << "WARNING: template is not initialized" << endl;
        return 0;
    }

    vector <Point> contour;
    int method = CV_CONTOURS_MATCH_I2;

    extractContour(contour,mask,_isSimplified);

    return matchShapes(contour,_template,method,0);
}

void ContourBasedFingerDetector::detectFingerTips(std::vector<cv::Point> &tips, const cv::Rect3D &location, const cv::Mat &mask)
{
    //track contour, calculate curvature at different scales...
    //find starting point:in

    vector<vector <Point> > contours;
    vector<Vec4i> hierarchy;
    Mat objectRef = mask(Rect(location.x,location.y,location.width,location.height));
    Mat object = objectRef.clone();
    int mode = CV_RETR_CCOMP,method = CV_CHAIN_APPROX_NONE;

    //find inner and outer contours
    findContours(object,contours,hierarchy,mode,method);
    int outerContour = -1;

    //now take outer contour
    for(int i=0; i< contours.size(); i++){
        if(hierarchy[i][3]<0) //no parent contour
        {
            outerContour = i; //assume only one outer contour
            break;
        }
    }

    cout << "outerContour: " << outerContour << endl;

    if (outerContour >= 0){
        int scales[] = {15, 18, 21, 24, 27};
        _contour = contours[outerContour];
        int scaleLength = 5, contourLength = _contour.size();
        queue<int> locmins[scaleLength];
        int first,start;
        int minidx;
        Point pp, pn;
        double pnorm,nnorm,cos,sin,minvalue,firstminvalue,thr = -0.1;
        int pNi,nNi;

        cout << "Contour length: " << _contour.size() << endl;

        for(int i=0; i< scaleLength; i++){
            pNi = contourLength - scales[i] + 1;
            nNi = scales[i];
            first = -1;
            start = -1;
            firstminvalue = -1;

            for(int j=0; j< contourLength; j++){
                pp = _contour[j] - _contour[pNi];
                pn = _contour[nNi] - _contour[j];

                pnorm = sqrt(pp.x*pp.x + pp.y*pp.y);
                nnorm = sqrt(pn.x*pn.x + pp.y*pp.y);

                cos = (pp.x*pn.x+pp.y*pn.y)/(pnorm*nnorm);
                sin = -(pp.x*pn.y - pp.y*pn.x);

                if(first >= 0 & sin > 0 & cos < thr){

                //state is the same; search min
                    if(cos < minvalue){
                        minvalue = cos;
                        minidx = j;
                    }
                }

                if(first< 0 & sin > 0 & cos < thr){
                //state change
                    first = j;
                    minvalue = cos;
                    minidx = j;
                    if (start < 0){
                        start = j;
                    }
                }

                if(first >= 0 & (sin < 0 | cos > thr)){
                    //state change
                    if (locmins[i].empty()){
                        firstminvalue = minvalue;
                    }
                    locmins[i].push(minidx);
                    first = -1;
                    minidx = -1;
                    minvalue = 1; //cos - less then that it is imposible
                }
                pNi = (pNi + 1)%contourLength;
                nNi = (nNi + 1)%contourLength;
            }

            //process the last part
            if(first > 0 & start == 0){
                //continious interval throught the starting point
                if (minvalue < firstminvalue){
                    locmins[i].front() = minidx;
                }
            }
        }

        _ids.clear();

        //transform indices to actual points
        for(int i=0; i<scaleLength; i++){
            while(!locmins[i].empty()){
                _ids.push_back(locmins[i].front());
                tips.push_back(_contour[locmins[i].front()]+Point(location.x,location.y));
                locmins[i].pop();
            }
        }
    }
}

void ContourBasedFingerDetector::rejectNonFingers(std::vector<cv::Point> &realTips, const cv::Rect3D &location)
{

    if (_ids.empty() | _contour.empty()){
        cerr << "contour was not detected! call detectFingerTips first!" << endl;
    }

    int count_thr_l = 1;
    int count_thr_h = 10;
    int last = _ids[0],length=1,firstlength = (_ids[0]==0 & _ids[_ids.size()-1]== (_contour.size()-1))?0:-1;
    Point mean,firstmean;

    sort(_ids.begin(),_ids.end());

    mean = _contour[_ids[0]];

    //search for intervals
    for(int i=1; i< _ids.size(); i++){

        cout << "deltaL " << (_ids[i] - last) << endl;

        if((_ids[i] - last ) <= 1){
            mean = mean + _contour[_ids[i]];
            length++;
        }else{

            cout << "length_in: " << length << endl;

            //cluster ended
            if(firstlength == 0){
                firstlength = length;
                firstmean = mean;
            }

            if (length > count_thr_l & length < count_thr_h){

                //that is finger

                realTips.push_back(Point(((double)mean.x)/length,((double)mean.y)/length)+Point(location.x,location.y));
                length = 1;
                mean = _contour[_ids[i]];
            }
        }
        last = _ids[i];
    }

    if(firstlength > 0){
        length = length + firstlength;
        //last ids points to contour last element
        if (length > count_thr_l & length < count_thr_h){
            mean = mean + firstmean;

            realTips[0] = Point(((double)mean.x)/length,((double)mean.y)/length)+Point(location.x,location.y);
        }
    }
}
