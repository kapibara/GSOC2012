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

void ContourBasedFingerDetector::detectFingerTipsSuggestions(std::vector<cv::Point> &tips, const cv::Mat &patch)
{
    //track contour, calculate curvature at different scales...
    //find starting point:in

    vector<vector <Point> > contours;
    vector<Vec4i> hierarchy;
    Mat object = patch.clone();
    int mode = CV_RETR_CCOMP,method = CV_CHAIN_APPROX_NONE;

    _patchSize = patch.size();

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

    if (outerContour >= 0){
        _contour = contours[outerContour];
        int scaleLength = 20, contourLength = _contour.size();
        int scales[scaleLength];

        queue<int> locmins[scaleLength];
        int first,start;
        int minidx;
        Point pp, pn;
        double pnorm,nnorm,cos,sin,minvalue,firstminvalue,thr = -0.1;
        int pNi,nNi;

        //set up scales
        for(int i=0; i< scaleLength; i++){
            scales[i]=floor(0.01*contourLength+i*((0.03*contourLength)/scaleLength));
            if (scales[i]==0){
                scales[i]=1;
            }
        }

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
                    //state change; start to search for local minimum
                    first = j;
                    minvalue = cos;
                    minidx = j;
                    if (start < 0){
                        start = j;
                    }
                }

                if(first >= 0 & (sin < 0 | cos > thr)){
                    //state change; save local minimum
                    if (locmins[i].empty()){
                        firstminvalue = minvalue;
                    }
                    locmins[i].push(minidx);
                    first = -1;
                    minidx = -1;
                    minvalue = 1; //cos - bigger then that it is imposible
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
                tips.push_back(_contour[locmins[i].front()]);
                locmins[i].pop();
            }
        }
    }
}

void ContourBasedFingerDetector::locateFingerTips(vector<Point> &tips)
{
    if (!_ids.empty()){

        sort(_ids.begin(),_ids.end());
        Point currentMean = _contour[_ids[0]];
        int currentElemCount=1;
        vector<int> lengths;
        int thr = 10; //index difference allowed

        for(vector<int>::iterator i=(_ids.begin()+1); i!=_ids.end(); i++){
            if ((*i - *(i-1)) < thr){
                currentMean = currentMean + _contour[*i];
                currentElemCount++;
            }else{
                tips.push_back(currentMean);
                lengths.push_back(currentElemCount);
                currentMean = _contour[*i];
                currentElemCount = 1;
            }
        }

        //push the last point
        tips.push_back(currentMean);
        lengths.push_back(currentElemCount);

        //relabel the first class if there is continious segment
        if(_contour.size()-(_ids.back()+1)+_ids.front()<thr){
            tips[0] = tips[0]+tips[tips.size()-1];
            lengths[0] = lengths[0]+lengths[lengths.size()-1];
            tips.pop_back(); //remove the last element, since it is in the first already
        }

        for(int i=0; i<tips.size(); i++){
            tips[i] = Point(tips[i].x/lengths[i],tips[i].y/lengths[i]);
        }
    }
}


void ContourBasedFingerDetector::orderFingerTips(std::vector<cv::Point> &tips, const cv::Point &palmCenter)
{
    int distanceThr = 2500;
    int minDist,cc,cDist;

    vector<pair<int,int> > correspondence;

    if (!_orderedTips.empty()){
        for(int i=0; i< tips.size(); i++){
            minDist =distanceThr+1;
            cc = -1;
            for(int j=0; j<_orderedTips.size(); j++){
                cDist = (_orderedTips[j].x - tips[i].x)*(_orderedTips[j].x - tips[i].x) + (_orderedTips[j].y - tips[i].y)*(_orderedTips[j].y - tips[i].y);
                if (cDist < minDist )
                {
                    cc = j;
                    minDist = cDist;
                }
            }
            if (minDist <= distanceThr){
                correspondence.push_back(pair<int, int>(i,cc));
            }
        }

        for(int i=0; i<_orderedTips.size(); i++){
            _orderedTips[i] = Point(-1,-1);
        }

        for(int i=0; i<correspondence.size(); i++){
            _orderedTips[correspondence[i].second] = tips[correspondence[i].first];
        }
    }
    else{
        int i=0;

        while(i<std::min<int>(tips.size(),5)){
            _orderedTips.push_back(tips[i]);
            i++;
        }

        while(i<5){
            _orderedTips.push_back(Point(-1,-1));
            i++;
        }
    }


}


