/*
 * @file affine.cpp
 * @brief Affine rectification using line at infinity
 * @author Zhixing Li
 */

#include "global.h"

// src_points are the original points
// dst_points are the destination points
// AffineHomography is calculating the homography for affinity
Mat AffineHomography (vector<Point2f> src_points) { 
    if (src_points.size()!=4) {
	cout<< " --(!) Size of point set is not 4" << endl; 
    }
    // Initialize matrix A
    Mat A = Mat::eye(3,3,CV_32FC1);
    // Calculate 2 sets of parallel lines: l1 l2 m1 m2
    // In the vector, [3] is bottom right [2] is bottom left [1] is top right [0] is top left
    // l1 = [0]x[1] l2 = [2]x[3]
    Mat l_1 = PointsToLine(src_points[0],src_points[1]);
    Mat l_2 = PointsToLine(src_points[2],src_points[3]);
    // m1 = [0]x[2] m2 = [1]x[3]
    Mat m_1 = PointsToLine(src_points[0],src_points[2]);
    Mat m_2 = PointsToLine(src_points[1],src_points[3]);
    // Vanishing points: v1 v2
    // v1 = l1 x l2	v2 = m1 x m2
    Mat v_1 = l_1.cross(l_2);
    Mat v_2 = m_1.cross(m_2);
    // Vanishing line: vl
    // vl = v1 x v2
    Mat v_l = v_1.cross(v_2);
    // Transpose v_l into a row vector 
    Mat v_l_t = v_l.t();
    // Normalize v_l_t by setting last element as 1
    v_l_t = v_l_t/v_l_t.at<float>(0,v_l_t.cols-1);

    // Update the homography matrix A
    v_l_t.copyTo(A.row(2));    

    // Print out matrix to check
    cout << "v_l = "<< endl << " " << v_l << endl << endl;
    
    
    return A; 

}

Mat MetricHomography (vector<Point2f> src_points, Mat H) {
    // Calculate 2 sets of orthogonal lines: l1 l2 m1 m2
    // l1 = [0]x[1] m1 = [1]x[3]
    Mat l_1 = PointsToTransLine(src_points[0],src_points[1],H);
    Mat m_1 = PointsToTransLine(src_points[1],src_points[3],H);
    // l2 = [0]x[3] m2 = [1]x[2]
    Mat l_2 = PointsToTransLine(src_points[0],src_points[3],H);
    Mat m_2 = PointsToTransLine(src_points[1],src_points[2],H);

    // l3 = [4]x[5] m3 =[5]x[6]
    //Mat l_3 = PointsToTransLine(src_points[4],src_points[5],H);
    //Mat m_3 = PointsToTransLine(src_points[5],src_points[6],H);

    //cout << "l1 = "<< endl << " " << l_1 << endl << endl;
    //cout << "m1 = "<< endl << " " << m_1 << endl << endl;
    // As = 0
    Mat A = CreateMatrixA(l_1,l_2,m_1,m_2);
    
    // SVD decomposition of A 
    Mat w, u, vt;
    SVD::compute(A,w,u,vt,SVD::FULL_UV);

    Mat s = Mat::zeros(2,2,CV_32FC1);
    s.at<float>(0,0)=vt.at<float>(2,0);
    s.at<float>(0,1)=vt.at<float>(2,1);
    s.at<float>(1,0)=vt.at<float>(2,1);
    s.at<float>(1,1)=vt.at<float>(2,2);
    // Verify the result
    //cout << "A S = "<< endl << " " << A*(-1*vt.row(2).t()) << endl << endl;
    
    // s should be positive definite
    s = s/s.at<float>(1,1);

    // S=KK^T
    // Use cholesky decomposition to calculate K
    // K is an upper-triangular matrix normalized as det K = 1
    Mat k = Cholesky(s);

    Mat M = Mat::eye(3,3,CV_32FC1);
    M.at<float>(0,0)=k.at<float>(0,0);
    M.at<float>(0,1)=k.at<float>(0,1);
    M.at<float>(1,0)=k.at<float>(1,0);
    M.at<float>(1,1)=k.at<float>(1,1);

    Mat C = Mat::zeros(3,3,CV_32FC1);
    C.at<float>(0,0)=s.at<float>(0,0);
    C.at<float>(0,1)=s.at<float>(0,1);
    C.at<float>(1,0)=s.at<float>(1,0);
    C.at<float>(1,1)=s.at<float>(1,1);

    cout << "Cinf = "<< endl << " " << C << endl << endl;

    // Print out matrix to check
    
    //cout << "w = "<< endl << " " << w << endl << endl;
    //cout << "u = "<< endl << " " << u << endl << endl;
    //cout << "vt = "<< endl << " " << vt << endl << endl;
    //cout << "A = "<< endl << " " << A << endl << endl;
    //cout << "S = "<< endl << " " << s << endl << endl;
    //cout << "k = "<< endl << " " << k << endl << endl;
    
    return M.inv();

}

Mat CreateMatrixA(Mat l_1, Mat l_2, Mat m_1, Mat m_2) {
    // Matrix A is a 3x3 matrix initialized with zeros
    // Usually we make A a square matrix
    Mat A = Mat::zeros(3,3,CV_32FC1);
    A.at<float>(0,0)=l_1.at<float>(0,0)*m_1.at<float>(0,0);
    A.at<float>(0,1)=l_1.at<float>(0,0)*m_1.at<float>(1,0)+l_1.at<float>(1,0)*m_1.at<float>(0,0);
    A.at<float>(0,2)=l_1.at<float>(1,0)*m_1.at<float>(1,0);
    A.at<float>(1,0)=l_2.at<float>(0,0)*m_2.at<float>(0,0);
    A.at<float>(1,1)=l_2.at<float>(0,0)*m_2.at<float>(1,0)+l_2.at<float>(1,0)*m_2.at<float>(0,0);
    A.at<float>(1,2)=l_2.at<float>(1,0)*m_2.at<float>(1,0);
    //A.at<float>(1,0)=l_3.at<float>(0,0)*m_3.at<float>(0,0);
    //A.at<float>(1,1)=l_3.at<float>(0,0)*m_3.at<float>(1,0)+l_3.at<float>(1,0)*m_3.at<float>(0,0);
    //A.at<float>(1,2)=l_3.at<float>(1,0)*m_3.at<float>(1,0);

    return A;
}

Mat PointsToTransLine (Point2f point_1, Point2f point_2, Mat H) {
    Mat p_1 = H*Point2fToMat(point_1);
    Mat p_2 = H*Point2fToMat(point_2);
    Mat t_l = p_1.cross(p_2);

    return t_l;
}	

Mat PointsToLine (Point2f point_1, Point2f point_2) {
    // Matrix l is a column vector --> line
    Mat line(3,1,CV_32FC1);
    // l = point x point
    Mat p_1 = Point2fToMat(point_1);
    Mat p_2 = Point2fToMat(point_2);
    line = p_1.cross(p_2);

    // Matrix l is prepared
    return line; 
}

Mat Point2fToMat (Point2f point) {
    // Change point from Point2f into Mat
    Mat p(3,1,CV_32FC1);
    p.at<float>(0,0)=point.x;
    p.at<float>(1,0)=point.y;
    p.at<float>(2,0)=1;
 
    return p;
}

Mat Cholesky (Mat A) {
    Mat S = Mat::zeros(A.rows,A.cols,CV_32FC1);
    S.at<float>(1,1)=sqrt(A.at<float>(1,1));
    S.at<float>(0,1)=A.at<float>(0,1)/S.at<float>(1,1);
    S.at<float>(0,0)=sqrt(A.at<float>(0,0)-S.at<float>(0,1)*S.at<float>(0,1));

    float det = sqrt(S.at<float>(0,0)*S.at<float>(1,1));
    S = S/det;

    return S;
}

Mat WarpScale(Mat image, Mat H) {
    // Corner points of original image
    Point2f corner_1(0,0);
    Point2f corner_2(image.size().width-1,0);
    Point2f corner_3(0,image.size().height-1);
    Point2f corner_4(image.size().width-1,image.size().height-1);

    Mat c_1 = Point2fToMat(corner_1);
    Mat c_2 = Point2fToMat(corner_2);
    Mat c_3 = Point2fToMat(corner_3);
    Mat c_4 = Point2fToMat(corner_4);

    // New corner points after transformation
    Mat new_c_1 = H*c_1;
    Mat new_c_2 = H*c_2;
    Mat new_c_3 = H*c_3;
    Mat new_c_4 = H*c_4;
	
    // Normalize the point
    new_c_1 = new_c_1/new_c_1.at<float>(2,0);
    new_c_2 = new_c_2/new_c_2.at<float>(2,0);
    new_c_3 = new_c_3/new_c_3.at<float>(2,0);
    new_c_4 = new_c_4/new_c_4.at<float>(2,0);

    vector<int> Widths;
    vector<int> Heights;

    Widths.push_back(static_cast<int>(new_c_1.at<float>(0,0)));
    Widths.push_back(static_cast<int>(new_c_2.at<float>(0,0)));
    Widths.push_back(static_cast<int>(new_c_3.at<float>(0,0)));
    Widths.push_back(static_cast<int>(new_c_4.at<float>(0,0)));
    Heights.push_back(static_cast<int>(new_c_1.at<float>(1,0)));
    Heights.push_back(static_cast<int>(new_c_2.at<float>(1,0)));
    Heights.push_back(static_cast<int>(new_c_3.at<float>(1,0)));
    Heights.push_back(static_cast<int>(new_c_4.at<float>(1,0)));

    int width = Widths[0];
    int height = Heights[0];

    for(int i=1; i<4; i++) {
	if (width<Widths[i]) {
	    width = Widths[i];
	}
	if (height<Heights[i]) {
	    height = Heights[i];
	}
    }
        
    
    //cout << "width = "<< endl << " " << width << endl << endl;
    //cout << "height = "<< endl << " " << height << endl << endl;

    Mat new_image;
    
    warpPerspective(image, new_image, H, Size(width,height), 0);
    resize(new_image, new_image, image.size());

    return new_image;

}
