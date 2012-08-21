#ifndef MORPHOLOGY_H
#define MORPHOLOGY_H

#include <opencv2/opencv.hpp>

/* The class is a container for a set of morphological operations*/

class Morphology
{
public:
    Morphology();

    static void open(cv::Mat &output,const cv::Mat &input, double r);
    static void close(cv::Mat &output,const cv::Mat &input, double r);
    static void dilate(cv::Mat &output, const cv::Mat &input, double r);
    static void erode(cv::Mat &output, const cv::Mat &input, double r);
};

#endif // MORPHOLOGY_H
