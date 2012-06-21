#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

void extractContour(vector<Point> &hand, Mat &image)
{

    CV_Assert(image.type() == CV_8UC1);

    cerr << "Extract contours " << endl;

    vector<vector <Point> > contours;
    int mode = CV_RETR_LIST, method = CV_CHAIN_APPROX_SIMPLE;

    findContours(image,contours,mode,method);

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

        approxPolyDP(contours[index],hand,5,true);
    }
}
