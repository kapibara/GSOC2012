#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <cstdio>

#include "DDConvert.h"

//#include "easyhandtracker.h"
#include "kalmanhandtracker.h"
#include "fastqueue.hpp"
#include "trackdrawer.h"
#include "palmcenterdetector.h"
#include "contourbasedfingerdetector.h"
#include "morphology.h"



#define first_byte(v) (v & 0x00FF)
#define second_byte(v) (v >> 8)
#define short_from_2char(v1,v2) ((((unsigned short)v2) << 8) + v1)

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    cout << "OpenCV version installed: " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << endl;

    if(argc < 3)
    {
        cerr << "Not enough arguments: exec <depth input file> <rgb input file>" << endl;
        exit(-1);
    }

    try
    {
        VideoCapture sourceDepth(argv[1]);
        VideoCapture sourceRGB(argv[2]);

        //get frames number
        int frameCount = sourceRGB.get(CV_CAP_PROP_FRAME_COUNT);
//        int frameCount = 100;
        int frameWidth = sourceDepth.get(CV_CAP_PROP_FRAME_WIDTH),
            frameHeight = sourceDepth.get(CV_CAP_PROP_FRAME_HEIGHT);


        Mat imageRGB,imageDepthR,
            imageDepth(Size(frameWidth,frameHeight),CV_16UC1),
            handMask(Size(frameWidth,frameHeight),CV_8UC1);

        cout << sourceDepth.isOpened() << endl;
        cout << sourceDepth.isOpened() << endl;

        if (!(sourceDepth.isOpened() & sourceRGB.isOpened()))
        {
            cerr << "Unable to open video files: depth file: " << sourceDepth.isOpened() << "rgb file: " << sourceRGB.isOpened() << endl;
            exit(-1);
        }

        //get codec

        double fps = sourceDepth.get(CV_CAP_PROP_FPS);
        //EasyHandTracker tracker;
        vector<Point> tips;
        KalmanHandTracker tracker(1.0/30.0);
        Point p,ps;
        double r,rs;
        double sim;
        stringstream ss;//create a stringstream
        string filebase("output");
        int savedCount = 0;
        int key;
        PalmCenterDetector pdetector;
        ContourBasedFingerDetector fd;
        Mat morph(Size(frameWidth,frameHeight),CV_8UC1);
        Rect3D position;
        TrackDrawer drawer, drawerS;
        ofstream stream("contours.txt");
        bool trackerResult;

        cout << "number of frames: " << frameCount << endl
             << "frame width: " << frameWidth << endl
             << "frame height: " << frameHeight << endl
             << "fps: " << fps << endl;

        drawer.setColor(Scalar(0,0,200));
        drawerS.setColor(Scalar(0,200,0));

        while(true)
        {
            sourceRGB.set(CV_CAP_PROP_POS_FRAMES,0);
            sourceDepth.set(CV_CAP_PROP_POS_FRAMES,0);
            tracker.init();
            drawer.reset();
            drawerS.reset();
            pdetector.reset();

            for(int i=0; i <  frameCount -1 ; i++ )
            {
                cout << "frame count: " << i << endl;

                sourceDepth >> imageDepthR;
                sourceRGB >> imageRGB;

                convertFrom8UC3Format(imageDepth,imageDepthR);

                trackerResult = tracker.track(position,handMask,imageDepth,imageRGB);

                if (tracker.isTracking() & !trackerResult){
                    cout << "Track lost!" << endl;
                    //exit(0);
                }

                if (trackerResult){
                    pdetector.detect(p,ps,r,rs,position,handMask);
                    cout << "p: " << p << "ps: " << ps << endl;
                    circle(imageRGB,p,r,Scalar(0,0,200));
                    circle(imageRGB,ps,rs,Scalar(0,200,0));
                    drawer.drawTrack(imageRGB,p);
                    drawerS.drawTrack(imageRGB,ps);
                }

                tips.clear();

                Morphology::erode(morph,handMask,2);


                if(trackerResult){
                    fd.detectFingerTips(tips,position,morph);  

                    cout << "number of finger tips: " << tips.size() << endl;

                    for(int i=0; i<tips.size(); i++){
                        circle(imageRGB,tips[i],2,Scalar(0,0,200));
                    }

                    tips.clear();

                    fd.rejectNonFingers(tips,position);
                }
                if (fd.isInitialized() && trackerResult){
                    sim = fd.compareToTemplate(handMask);
                    cout << "Similarity value: " << sim << endl;

                }

                //cout << "Opening!" << endl;
                //Morphology::close(morph,handMask,1);

                //imshow("Depth",imageDepth);
                imshow("RGB",imageRGB);
                imshow("hand mask", handMask);

                key = waitKey(10);

                if (key == 'd'){
                    //save current contour as template
                    //fd.initTemplate(handMask);
                }
                if(key == 's'){
                    cout << "Save file number " << savedCount << endl;

                    ss.seekp(0);
                    ss << savedCount;
                    fd.saveContour(filebase+ss.str()+".txt");
                    savedCount++;
                }
            }

            stream.close();
        }
    }
    catch(...)
    {
        cerr <<" exception caught" << endl;
    }
}
