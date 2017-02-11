#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

class ImageProcess {
public:
  ImageProcess();
  int* getBoard(Mat frame);
}

#endif
