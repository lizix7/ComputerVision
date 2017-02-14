/*
 * @file dlt.cpp
 * @brief Direct Linear Transformation algorithm implemenation 
 * @author Zhixing Li
 */

#include "global.h"

// src_points are the original points
// dst_points are the destination points
// DLTHomography is the function using direct linear transformation algorithm to calculate H
Mat DLTHomography(vector<Point2f> src_points, vector<Point2f> dst_points) { 
    if (src_points.size()!=dst_points.size()) {
	cout<< " --(!) Size of the corresponding points is not equal" << endl; 
    }
    Mat A(0,9,CV_32FC1);
    for (int i=0; i<src_points.size(); i++) {
	Mat A_i = CreateMatrixA(src_points[i], dst_points[i]);
	A.push_back(A_i);
    }
    // SVD decomposition of A 
    Mat w, u, vt;
    // Use FULL_UV, or v is only a 9x8 matrix, not 9x9
    SVD::compute(A,w,u,vt,SVD::FULL_UV);
    // v = vt'
    // Mat v.copyTo(vt.row(8)); 
    Mat h(0,9,CV_32FC1);
    h.push_back(vt.row(8));
    // h is the last column of v, last row of vt
    // Normalize h into h33 = 1
    h = h/h.at<float>(0,h.cols-1);
    // Reshape h into H
    Mat H = h.reshape(0,3);

    // Print out matrix to check
    //cout << "w = "<< endl << " " << w << endl << endl;
    //cout << "u = "<< endl << " " << u << endl << endl;
    //cout << "vt = "<< endl << " " << vt << endl << endl;
    //cout << "h = "<< endl << " " << h << endl << endl;
	
    return H; 

}

Mat CreateMatrixA(Point2f src, Point2f dst) {
    // Matrix A_i is a 2x9 matrix initialized with zeros
    Mat A_i = Mat::zeros(2,9,CV_32FC1);
    // Assign the first row of matrix A_i
    A_i.at<float>(0,3)=-src.x;
    A_i.at<float>(0,4)=-src.y;
    A_i.at<float>(0,5)=-1;
    A_i.at<float>(0,6)=dst.y*src.x;
    A_i.at<float>(0,7)=dst.y*src.y;
    A_i.at<float>(0,8)=dst.y*1;
    // Assign the second row of matrix A_i
    A_i.at<float>(1,0)=src.x;
    A_i.at<float>(1,1)=src.y;
    A_i.at<float>(1,2)=1;
    A_i.at<float>(1,6)=-dst.x*src.x;
    A_i.at<float>(1,7)=-dst.x*src.y;
    A_i.at<float>(1,8)=-dst.x*1;

    // Matrix A_i is prepared
    return A_i; 
}

