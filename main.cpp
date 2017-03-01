#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <iostream>
#include <cyusb.h>
#include <opencv2/opencv.hpp>

#include "MsgQueue.h"
//#include "maker_binocular.h"
#include "CaptureData.h"
#include "CMutex.h"


int main()
{

    CaptureData *CaptureData_p=new CaptureData;
    USBElements sensorCapture;
    long long OutImgTimeStamp=0;
    int ret=1;
    int First_Run=0;
    while(1)
    {
        if(First_Run==0)
        {
            sleep(2);
            First_Run=1;
        }
        while(ret==1)
        ret=CaptureData_p->Get_CaptureData(&sensorCapture,OutImgTimeStamp);

        if(ret==0)
        {
            printf("get capturedata =0\n");
        }

        cv::Mat frame(480,640,CV_8UC1,sensorCapture.LeftImgBuffer);
        cv::Mat frame1(480,640,CV_8UC1,sensorCapture.RightImgBuffer);

        cv::imshow("Right",frame1);
        cv::imshow("Left",frame);

        cv::waitKey(1);
        ret=1;

    }

    return 0;
}