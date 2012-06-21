#include <opencv2/opencv.hpp>

#include <queue>

using namespace cv;
using namespace std;

#define point pair<unsigned short, unsigned short>

#define EMPTY 0
#define REGION 255

#define HAND_SIZE 100

//mask image: mask, where
//depth image: stays const, used to determine hand position and segment it
void segmentHandEasy(Mat &mask,const Mat &depth)
{
    CV_Assert(mask.type() == CV_8UC1);
    CV_Assert(depth.type() == CV_16UC1);

    CV_Assert(mask.rows == depth.rows);
    CV_Assert(mask.cols == depth.cols);

    mask.setTo(EMPTY);

    int rowcount = depth.rows, colcount = depth.cols;

    const unsigned short *depthptr;
    unsigned short minval = pow(2,15); //not nessecerely maximum, but not minimum for sure
    unsigned short ri=0,ci=0;

    //search for nearest pixel
    for(int i=0; i<rowcount; i++)
    {
        depthptr = depth.ptr<const unsigned short>(i);

        for(int j=0; j<colcount; j++)
        {
            if(depthptr[j]>0 & depthptr[j]<minval)
            {
                minval = depthptr[j];
                ri = i;
                ci = j;
            }
        }

    }

    //take its neighbours
    queue<point > pixels;
    point current;
    double mean = minval;
    double thr = HAND_SIZE*0.5;
    unsigned short dv = 0;
    int pixelcount = 1;
    pixels.push(point(ri,ci));

    while(!pixels.empty())
    {
        //go through neighbours
        //4 Neighbourhood
        current = pixels.front();
        pixels.pop();

        dv = depth.at<unsigned short>(current.first,current.second);

        if ( current.first + 1 < rowcount )
        {
            if ( current.second + 1 < colcount )
            {
                if ( mask.at<uchar>(current.first + 1,current.second + 1 ) == EMPTY &
                     fabs(depth.at<unsigned short>(current.first + 1,current.second + 1 )-mean/pixelcount) < thr
                     & depth.at<unsigned short>(current.first + 1,current.second + 1 ) > 0)
                {
                    pixelcount++;
                    mean += depth.at<unsigned short>(current.first + 1,current.second + 1 );
                    mask.at<uchar>(current.first + 1,current.second + 1 ) = REGION;
                    pixels.push(point(current.first + 1,current.second + 1));
                }
            }

            if( current.second - 1 > -1 )
            {
                if ( mask.at<uchar>(current.first + 1,current.second - 1 ) == EMPTY &
                     fabs(depth.at<unsigned short>(current.first + 1,current.second - 1 )-mean/pixelcount) < thr
                     & depth.at<unsigned short>(current.first + 1,current.second + 1 ) > 0)
                {
                    pixelcount++;
                    mean += depth.at<unsigned short>(current.first + 1,current.second - 1 );
                    mask.at<uchar>(current.first + 1,current.second - 1 ) = REGION;
                    pixels.push(point(current.first + 1,current.second - 1));
                }
            }
        }

        if ( current.first - 1 > -1 )
        {
            if ( current.second + 1 < colcount )
            {
                if ( mask.at<uchar>(current.first - 1,current.second + 1 ) == EMPTY &
                     fabs(depth.at<unsigned short>(current.first - 1,current.second + 1 )-mean/pixelcount) < thr
                     & depth.at<unsigned short>(current.first - 1,current.second + 1 ) > 0)
                {
                    pixelcount++;
                    mean += depth.at<unsigned short>(current.first - 1,current.second + 1 );
                    mask.at<uchar>(current.first - 1,current.second + 1 ) = REGION;
                    pixels.push(point(current.first - 1,current.second + 1));
                }
            }

            if( current.second - 1 > -1 )
            {
                if ( mask.at<uchar>(current.first - 1,current.second - 1 ) == EMPTY &
                     fabs(depth.at<unsigned short>(current.first - 1,current.second - 1 )-mean/pixelcount) < thr
                     & depth.at<unsigned short>(current.first - 1,current.second - 1 ) > 0)
                {
                    pixelcount++;
                    mean += depth.at<unsigned short>(current.first - 1,current.second - 1 );
                    mask.at<uchar>(current.first - 1,current.second - 1 ) = REGION;
                    pixels.push(point(current.first - 1,current.second - 1));
                }
            }
        }
    }

}
