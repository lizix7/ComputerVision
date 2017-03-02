#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <random>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "lmmin.h"
#include "lmstruct.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

typedef struct {
    vector<cv::Point2f> src_points;
    vector<cv::Point2f> dst_points;
} data_struct; 

typedef struct {
    int n_inliers;
    vector<cv::Point2f> src_inliers;
    vector<cv::Point2f> dst_inliers;
} inlier_struct; 

Mat DLT(vector<Point2f>, vector<Point2f>);

Mat Normalized_DLT(vector<Point2f>, vector<Point2f>);

Mat MLE(vector<Point2f>, vector<Point2f>);

Mat MLE_with_H(vector<Point2f>, vector<Point2f>, Mat);

Mat Ransac(vector<Point2f>, vector<Point2f>);

inlier_struct Stablize(vector<Point2f>, vector<Point2f>, Mat);

bool isCollinear(vector<Point2f>); 

void Evaluate_Sampson( const double *, int, const void *, double *, int *);

Mat Jacobian(pair<Point2f,Point2f>, Mat);

Mat Epsilon(pair<Point2f,Point2f>, Mat);

Mat Sampson(Mat, Mat); 

Mat Sampson_error(Mat, Mat); 

void ProjectPoints(vector<Point2f>, vector<Point2f> &, Mat);

Point2f MatToPoint2f(Mat);

Mat Point2fToMat(Point2f);

Mat Normalize(vector<Point2f>); 

Mat CreateMatrixA(Point2f, Point2f);

#endif 
