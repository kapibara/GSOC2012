#include "trackdrawer.h"

using namespace std;
using namespace cv;

TrackDrawer::TrackDrawer()
{
    //do nothind
}

TrackDrawer::~TrackDrawer()
{
    //do nothing
}

void TrackDrawer::reset()
{
    _path.clear();
}

template<class DT>
void TrackDrawer::drawTrack(cv::Mat &mask,const cv::Rect3D &position)
{
    if (!position.isIni)
        return;

    //compute center of mass
    double meanX=0,meanY=0;
    int pc=0;
    DT *maskPtr;

    for(int i=position.y; i< position.y + position.height; i++)
    {
        maskPtr = mask.ptr<DT>(i);
        for(int j=position.x; j<position.x + position.width; j++)
        {
            if(maskPtr[j]>0){
                meanX += j;
                meanY += i;
                pc++;
            }
        }
    }

    meanX /= pc;
    meanY /= pc;

    drawTrack(mask,Point(meanX,meanY));
}

void TrackDrawer::drawTrack(cv::Mat &mask,const cv::Point &center)
{
    _path.push_back(center);

    plotLine(mask);
}

void TrackDrawer::plotLine(cv::Mat &mask)
{
    for(int i=1; i<_path.size(); i++)
    {
        line(mask,_path[i-1],_path[i],_color);
    }
}
