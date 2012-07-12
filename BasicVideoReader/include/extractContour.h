#ifndef EXTRACTCONTOUR_H
#define EXTRACTCONTOUR_H

#include <vector>
#include <opencv2/opencv.hpp>

//void extractContour(std::vector<cv::Point> &hand, const cv::Mat &input);

void extractContour(std::vector<cv::Point> &hand, cv::Mat &image);


#endif // EXTRACTCONTOUR_H
