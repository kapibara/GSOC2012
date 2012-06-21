#include <iostream>
#include <opencv2/opencv.hpp>

#include "easyhandtracker.h"

using namespace cv;
using namespace std;

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
        capture.retrieve( depthMap, CV_CAP_OPENNI_BGR_IMAGE );

        cout << "Frame size: " << depthMap.size().width << ";" << depthMap.size().height << endl;
        cout << "FPS: " << capture.get(CV_CAP_PROP_FPS) << endl;

        Mat mask = Mat(depthMap.size(),CV_8UC1);
        EasyHandTracker tracker;
        bool trackResult;
        tracker.init();

        while(true)
        {
            capture.grab();


            cout << "retrieve depth image: " << capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP ) << endl;
            cout << "retrieve rgb image: " << capture.retrieve( rgbImage, CV_CAP_OPENNI_BGR_IMAGE ) << endl;

            trackResult = tracker.track(mask,depthMap,rgbImage);

            cout << "Track result: " << trackResult << endl;

            imshow("RGB",rgbImage);
            imshow("Depth",depthMap);
            imshow("Mask",mask);

            key = waitKey(10);
            if (key == 'q')
                break;

            //restart tracking
            if (key == 's')
                tracker.init();
        }
    }
    catch(...)
    {
        cerr <<" exception caught" << endl;
    }
}
