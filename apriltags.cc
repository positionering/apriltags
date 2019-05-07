#include <iostream>

#include "opencv2/opencv.hpp"

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
#include <apriltag/common/getopt.h>
#include <apriltag/apriltag_pose.h>
}

using namespace std;
using namespace cv;


int main(int argc, char *argv[])
{
    // Initialize camera
    VideoCapture cap(1);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }
    
    cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);

    // Initialize tag detector with options
    apriltag_family_t *tf = tag36h11_create();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    
    td->quad_decimate = 2.0;
    td->quad_sigma = 0;
    td->refine_edges = 1;
    td->decode_sharpening = 0.25;
    
    apriltag_pose_t pose;
    
    Mat frame, gray;
    while (true) {
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
                    
            
            // fabriks kalibrerad d435 kameran
          apriltag_detection_info_t info;
          info.det = det;
         info.tagsize = 0.1615;  // size april tag (88 mm liten, 185 mm stor) (0.1615 stor?)
        info.fx = 1660.70; // 1394.33;     // 1983.97376;
        info.fy = 1668.19; // 1394.95;     // 1981.62916;
        info.cx = 886.07; // 964.117;     // 998.341216;
        info.cy = 522.40; // 537.659;     // 621.618227;
          
          // Then call estimate_tag_pose.
          double err = estimate_tag_pose(&info, &pose);
         
         cout << pose.t->data[0] << "  " << pose.t->data[1] << "  " << pose.t->data[2]<<std::endl; 
       // cout << pose.t->data[0] << " x " << pose.t->data[1] << " y " << err << " err " << det->hamming << " hamming " << det->decision_margin << " decision_margin"  << endl;
            
        }
        apriltag_detections_destroy(detections);
    }

    apriltag_detector_destroy(td);

   
        tag36h11_destroy(tf);
    return 0;
}
