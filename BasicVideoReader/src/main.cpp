#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <cstdio>

#include "DDConvert.h"

#include "easyhandtracker.h"
#include "extractContour.h"
#include "fastqueue.hpp"

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


        Mat imageRGB, imageDepthR,
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
        EasyHandTracker tracker;
        bool trackerResult;

        cout << "number of frames: " << frameCount << endl
             << "frame width: " << frameWidth << endl
             << "frame height: " << frameHeight << endl
             << "fps: " << fps << endl;

        tracker.init();

        while(true)
        {
            sourceRGB.set(CV_CAP_PROP_POS_FRAMES,0);
            sourceDepth.set(CV_CAP_PROP_POS_FRAMES,0);

            for(int i=0; i < frameCount -1 ; i++ )
            {
                sourceDepth >> imageDepthR;
                sourceRGB >> imageRGB;

                convertFrom8UC3Format(imageDepth,imageDepthR);

                trackerResult = tracker.track(handMask,imageDepth,imageRGB);

                if (!trackerResult){
                    cerr << "Track lost!" << endl;
                    tracker.init();
                    //exit(0);
                }

                imshow("Depth",imageDepth);
                imshow("RGB",imageRGB);
                imshow("hand mask", handMask);

                waitKey(10);
            }
        }
    }
    catch(...)
    {
        cerr <<" exception caught" << endl;
    }
}
