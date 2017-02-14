#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"


using namespace std;
using namespace cv;

Mat DLTHomography(vector<Point2f>, vector<Point2f>);

Mat CreateMatrixA(Point2f, Point2f);

#endif 
