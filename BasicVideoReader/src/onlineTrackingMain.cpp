#include <iostream>
#include <opencv2/opencv.hpp>

#include "easyhandtracker.h"
#include "trackdrawer.h"
#include "morphology.h"
#include "palmcenterdetector.h"
#include "contourbasedfingerdetector.h"
#include "kalmanhandtracker.h"

using namespace cv;
using namespace std;

void saveTips(const string &filename,const vector<cv::Point> &tips)
{


}

void copyTo3Chanels(Mat &output,const Mat &input)
{
    int rowcount = input.rows;
    int colcount = input.cols;
    const uchar *idataptr;
    uchar* odataptr;

    if (input.isContinuous() & output.isContinuous())
    {
        colcount*= rowcount;
        rowcount = 1;
    }

    for(int i=0; i<rowcount; i++){

        idataptr = input.ptr<const uchar>(i);
        odataptr = output.ptr<uchar>(i);

        for(int j=0; j<colcount; j++){
            odataptr[3*j] = idataptr[j];
            odataptr[3*j+1] = idataptr[j];
            odataptr[3*j+2] = idataptr[j];
        }
    }
}

//XnSensorServer

int main(int argc, char **argv)
{
    cout << "OpenCV version installed: " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << endl;
    cout << "Starting capturing data" << endl;

    try
    {
        VideoCapture capture( CV_CAP_OPENNI );

        Mat depthMap;
        Mat rgbImage;

        int key = 0;

        cout << "Capture device: isOpened: " << capture.isOpened() << endl;

        capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
        capture.grab();

        capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );

        cout << "Frame size: " << depthMap.size().width << ";" << depthMap.size().height << endl;
        cout << "FPS: " << capture.get(CV_CAP_PROP_FPS) << endl;

        Mat mask = Mat(depthMap.size(),CV_8UC1);
        Mat opened = Mat(depthMap.size(),CV_8UC1);
        Mat toDisplay(depthMap.size(),CV_8UC3);
        KalmanHandTracker tracker;
        Rect3D position;
        vector<Point> tips;
        int saveCount = 1;
        stringstream ss;//create a stringstream
        string filebase("output");
        PalmCenterDetector pdetector;
        ContourBasedFingerDetector fd;
        TrackDrawer drawer;
        TrackDrawer noFilterDrawer;
        Point p,ps;
        double r,rs;
        bool trackerResult;
        tracker.init();
        int i=0;

        pdetector.reset();

        drawer.setColor(Scalar(0,255,0));
        noFilterDrawer.setColor(Scalar(0,0,255));

        while(true)
        {
            capture.grab();

            cout << capture.retrieve( rgbImage, CV_CAP_OPENNI_BGR_IMAGE ) << endl;
            cout << capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP ) << endl;

            cout << "depth map type" << (depthMap.type() == CV_16UC1) << endl;

            cout << "i=" << i<< endl;

            i = (i+1)%10;


            if(trackerResult=tracker.track(position,mask,depthMap,rgbImage)){
                //perform morphology
                Morphology::close(opened,mask,5);

                copyTo3Chanels(toDisplay,opened);

                pdetector.detect(p,ps,r,rs,position,opened);

                circle(toDisplay,ps,rs,Scalar(0,0,255));

                drawer.drawTrack(toDisplay,ps);
                noFilterDrawer.drawTrack(toDisplay,p);
            }

            if(trackerResult){
                tips.clear();

                fd.detectFingerTips(tips,position,opened);

                for(int i=0; i<tips.size(); i++){
                    circle(toDisplay,tips[i],2,Scalar(0,255,0));
                }

                //tips.clear();

                //fd.rejectNonFingers(tips,position);

                //cout << "number of remaining finger tips: " << tips.size() << endl;

                //for(int i=0; i<tips.size(); i++){
                //    circle(rgbImage,tips[i],2,Scalar(0,0,255));
                //}
            }


            if(trackerResult){
                rectangle(toDisplay,Rect(position.x,position.y,position.width,position.height),Scalar(255,0,0));
            }

            imshow("Depth",depthMap);
            imshow("RGBImg",toDisplay);

            key = waitKey(10);
            if (key == 'q')
                break;
            if (key == 's'){
                tracker.init();
                pdetector.reset();
                drawer.reset();
            }

            if (key == 'd'){
                //save current contour as template
                cout << "Save file number " << saveCount << endl;

                ss.seekp(0);
                ss << saveCount;
                fd.saveContour(filebase+ss.str()+".txt");
                saveCount++;
            }

        }
    }
    catch(...)
    {
        cerr <<" exception caught" << endl;
    }
}
