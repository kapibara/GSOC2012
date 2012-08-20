#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <time.h>

#include "easyhandtracker.h"
#include "trackdrawer.h"
#include "morphology.h"
#include "palmcenterdetector.h"
#include "contourbasedfingerdetector.h"
#include "kalmanhandtracker.h"
#include "grabbableball.h"

using namespace cv;
using namespace std;


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

int main(int argc, char **argv)
{
    cout << "OpenCV version installed: " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << endl;
    cout << "Starting capturing data" << endl;

    srand(time(NULL));

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
        Mat patch;
        KalmanHandTracker tracker;
        Rect3D position;
        vector<Point> tips;
        PalmCenterDetector pdetector;
        ContourBasedFingerDetector fd;
        stringstream info;
        GrabbableBall ball;
        Point ps;
        double rs;
        bool trackerResult;

        ball.setIniPosition(Point(rand()%(mask.size().width-200)+100,rand()%(mask.size().height-200)+100));
        ball.setColor(Scalar(0,0,255));
        ball.setIniRadius(50);

        pdetector.reset();
        pdetector.setUseRobust(false);

        while(true)
        {
            capture.grab();

            capture.retrieve( rgbImage, CV_CAP_OPENNI_BGR_IMAGE );
            capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );

            if(trackerResult=tracker.track(position,mask,depthMap,rgbImage)){
                //perform morphology
                Morphology::open(opened,mask,3);
                mask.setTo(0);
                Morphology::close(mask,opened,3);

                patch = mask(Rect(position.x,position.y,position.width,position.height));
                copyTo3Chanels(toDisplay,mask);
                //toDisplay.setTo(0);

                pdetector.detect(ps,rs,patch);
                //to relative coordinates
                ps.x += position.x;
                ps.y += position.y;
                circle(toDisplay,ps,rs,Scalar(0,255,0));

                /*detect and display finger tips*/
                tips.clear();
                fd.detectFingerTipsSuggestions(tips,patch);

                if(!tips.empty()){

                    tips.clear();

                    fd.locateFingerTips(tips);

                    info.str("");
                    info << "fingers: " << tips.size() << endl;

                    putText(toDisplay,info.str(),Point(depthMap.size().width*0.75,depthMap.size().height*0.75),FONT_HERSHEY_PLAIN,1,Scalar(100,100,100));
                }else{
                    info.str("");
                    info << "fingers: " << 0 << endl;

                    putText(toDisplay,info.str(),Point(depthMap.size().width*0.75,depthMap.size().height*0.75),FONT_HERSHEY_PLAIN,1,Scalar(100,100,100));

                }

                ball.setCursorPosition(cv::Point3i(ps.x,ps.y,position.z + position.depth/2),(tips.size() <= 1));

            }

            ball.draw(toDisplay);

            imshow("RGBImg",toDisplay);

            key = waitKey(10);

            if (key == 'q')
                break;

            if (key == 's'){
                tracker.init();

                pdetector.reset();

                fd.reset();

                ball.reset();
            }

        }
    }
    catch(...)
    {
        cerr <<" exception caught" << endl;
    }
}
