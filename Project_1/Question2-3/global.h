#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdio.h>
#include <iostream>
#include <cmath>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"


using namespace std;
using namespace cv;

Mat AffineHomography(vector<Point2f>);

Mat MetricHomography(vector<Point2f>, Mat);

Mat CreateMatrixA(Mat, Mat, Mat, Mat);

Mat PointsToTransLine(Point2f, Point2f, Mat);

Mat PointsToLine(Point2f, Point2f);

Mat Point2fToMat(Point2f);

Mat Cholesky (Mat);

Mat WarpScale(Mat, Mat);

#endif 
