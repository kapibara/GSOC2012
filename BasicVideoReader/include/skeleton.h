#ifndef SKELETON_H
#define SKELETON_H

#include <opencv2/opencv.hpp>

class Skeleton
{
public:
    Skeleton();

    static void findSkeleton(cv::Mat &skeleton,const cv::Mat &input);
};

#endif // SKELETON_H
