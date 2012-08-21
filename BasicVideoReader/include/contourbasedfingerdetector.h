#ifndef CONTOURBASEDFINGERDETECTOR_H
#define CONTOURBASEDFINGERDETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "rect3d.hpp"
#include "dynamictimewarping.h"

/*This class implements contour-based fingertips detection;
  the algorithm for initial fingertips detection suggestions
  is described in
  "Vision-Based Interpretation of Hand Gestures for Remote Control of Computer Mouse" by A.A. Argyros, Manolis Laurakis
  (formula for curvature is changed, sinse there is misprint in the article) */

class ContourBasedFingerDetector
{
public:
    ContourBasedFingerDetector();
    ~ContourBasedFingerDetector();

    void reset() {
        _orderedTips.clear();
    }


    /* Sets the number of scales; default value 20;
       scale is the distance (in pixels) between point where the curvature is computed with its successor and predessor;
       this distance is important in curvature calculation:
       curvature_i = cos((x_{i+scale_j}-x_i)^(x_i - x_{i-scale_j}))

       when scalesCount is specified, scale_j is computed using formula:
       scale_j = 0.01*contourLength+j*((0.03*contourLength)/scaleLength);
       the formula is empirical*/
    void setNumberOfScales(int scalesCount) {_scalesCount=scalesCount;}
    /* Threshold on cos value (defined by  curvature_i = cos((x_{i+scale_j}-x_i)^(x_i - x_{i-scale_j})))
       default value is -0.01; it is adviced to set in negative*/
    void setMinCosAllowed(float minCos) {_cosThr =  minCos;}
    /* Square distance to order detected finger tips to finger tips detected in the previous frame;
       default value is 2500 (px); experimental function */
    void setSqrDistance(int sqrDistance) {_sqrDistance = sqrDistance;}
    /* Union threshold defines how far finger tips should be from each other in the contour to still be
      related to the same finger tip; default is 10*/
    void setUnionThreshold(int thr) {_unionThr = thr;}

    /*save conour to a text file*/
    void saveContour(const std::string &filename);
    /*detects initial fingertips detection suggestions;
      suggestions are written into tips;
      patch is segmented hand mask*/
    void detectFingerTipsSuggestions(std::vector<cv::Point> &tips, const cv::Mat &patch);
    /*locates real finger tips based on finger tips suggestions;
      method detectFingerTipsSuggestions should be called first;
      tips array is output array, it should be empty*/
    void locateFingerTips(std::vector<cv::Point> &tips);
    /*order fingertips in the right order (e.g. if the thumb was first it will be placed first in the array of tips;
      tips array should contain output from locateFingerTips function;
      result of the ordering can be accuired with getOrderedTips() function;
      experimental function; does not work too good*/
    void orderFingerTips(std::vector<cv::Point> &tips, const cv::Point &palmCenter);

    const std::vector<cv::Point> & getOrderedTips(){
        return _orderedTips;
    }

private:

    std::vector<cv::Point> _contour;
    std::vector<int> _ids;
    std::vector<cv::Point> _orderedTips;
    cv::Size _patchSize;

    int _scalesCount;
    float _cosThr;
    int _sqrDistance;
    int _unionThr;
};

#endif // CONTOURBASEDFINGERDETECTOR_H


