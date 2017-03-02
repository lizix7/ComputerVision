/*
 * @file dlt.cpp
 * @brief Direct Linear Transformation algorithm implemenation 
 * @author Zhixing Li
 */

#include "global.h"

// src_points are the original points
// dst_points are the destination points
// DLTHomography is the function using direct linear transformation algorithm to calculate H
Mat DLT(vector<Point2f> src_points, vector<Point2f> dst_points) { 
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

Mat Normalized_DLT(vector<Point2f> src_points, vector<Point2f> dst_points) { 
    vector<Point2f> new_src_points;
    vector<Point2f> new_dst_points;
    // Compute the similarity transformation T and T'
    Mat T_1 = Normalize(src_points);
    Mat T_2 = Normalize(dst_points);
    // Compute the projected interest points
    ProjectPoints (src_points, new_src_points, T_1);
    ProjectPoints (dst_points, new_dst_points, T_2);
    // Compute the homography H_tilt using DLT
    Mat H_tilt = DLT(new_src_points,new_dst_points);
    // Denormalization H = T'.inv() H_tilt T
    Mat H = T_2.inv()*H_tilt*T_1;
    // Normalized Matrix H
    H = H/H.at<float>(2,2);

    return H;
}

Mat MLE(vector<Point2f> src_points, vector<Point2f> dst_points) {
    // Initialized guess of homography H 
    Mat H_init = Normalized_DLT(src_points, dst_points);
    // Parse H into par vector as requested by limfit
    int n_par = H_init.cols*H_init.rows;
    double par[n_par];
    for (int i=0; i<3; i++) {
	for (int j=0;j<3; j++) {
	    par[i*3+j]=H_init.at<float>(i,j);
	}
    } 
    // Parse all pairs of keypoints into data as requested by limfit
    int m_dat = 4*src_points.size();
    data_struct data ={src_points, dst_points};
    // Control parameters
    lm_control_struct control = lm_control_float;
    lm_status_struct status;
    //cout << "my m_dat = "<< endl << " " << m_dat << endl << endl;
    // Lmfit
    lmmin(n_par, par, m_dat, &data, Evaluate_Sampson, &control, &status);
    // After the iteration, par has the approached H values
    Mat H = Mat::zeros(3,3,CV_32FC1);
    for (int i=0; i<3; i++) {
	for (int j=0; j<3; j++) {
	    H.at<float>(i,j)=par[i*3+j];
	}
    }
    // Normalized Matrix H
    H = H/H.at<float>(2,2);

    return H;
}

Mat MLE_with_H(vector<Point2f> src_points, vector<Point2f> dst_points, Mat H_init) {
    // Parse H into par vector as requested by limfit
    int n_par = H_init.cols*H_init.rows;
    double par[n_par];
    for (int i=0; i<3; i++) {
	for (int j=0;j<3; j++) {
	    par[i*3+j]=H_init.at<float>(i,j);
	}
    } 
    // Parse all pairs of keypoints into data as requested by limfit
    int m_dat = 4*src_points.size();
    data_struct data ={src_points, dst_points};
    // Control parameters
    lm_control_struct control = lm_control_float;
    lm_status_struct status;
    //cout << "my m_dat = "<< endl << " " << m_dat << endl << endl;
    // Lmfit
    lmmin(n_par, par, m_dat, &data, Evaluate_Sampson, &control, &status);
    // After the iteration, par has the approached H values
    Mat H = Mat::zeros(3,3,CV_32FC1);
    for (int i=0; i<3; i++) {
	for (int j=0; j<3; j++) {
	    H.at<float>(i,j)=par[i*3+j];
	}
    }
    // Normalized Matrix H
    H = H/H.at<float>(2,2);

    return H;
}

Mat Ransac(vector<Point2f> src_points, vector<Point2f> dst_points) {
    // Initialize number of sampling: N
    // We are expecting proportion of outliers are around 50%.
    int N = 100;
    // Sample size is 4 because of four point correspondences
    int s = 4;
    int sample_count = 0;
    int number_total = src_points.size();
    // p usually set as 0.99
    float p = 0.99;
    // Verify the collinearity 
    bool collinear_flag = false;
    // Sample vectors
    vector<Point2f> sample_src;
    vector<Point2f> sample_dst;
    // The best picked homography H with its standard deviation and inliers number
    Mat H;
    float H_standard_deviation = 0;
    int H_inliers = 0;
    // Inliers vector
    vector<int> random_numbers;
    vector<Point2f> src_inliers, dst_inliers;
    vector<int> H_inliers_status(src_points.size(),0);
    while(N>sample_count) {
	// Randomly pick four sample src points and dst points for correspondences
	random_device rd;
	mt19937 generator(rd());	
	uniform_int_distribution<int> distribution(0,number_total-1);
	do {
	    random_numbers.clear();
	    sample_src.clear();
	    sample_dst.clear();
	    for (int i=0; i<s; i++) {
		int random_i = distribution(generator);
		sample_src.push_back(src_points[random_i]);
		sample_dst.push_back(dst_points[random_i]);
		random_numbers.push_back(random_i);
		//cout << "random_numbers = "<< endl << " " << random_i << endl << endl;
	    }
	    // Check whether the points are collinear or not
	    collinear_flag = isCollinear(sample_src);
	} while (collinear_flag);  
	// Calculate normalized H based on four pairs of points
	Mat h = Normalized_DLT(sample_src,sample_dst);
	// Calcualte d for each correspondence in the whole data
	float d_sum = 0;
	vector<float> distances;
	float distance = 0;
	for (int i=0; i<number_total; i++) {
	    pair<Point2f,Point2f> X = make_pair(src_points[i],dst_points[i]);
	    Mat e = Epsilon(X,h);
	    Mat j = Jacobian(X,h);
	    Mat s_e = Sampson_error(j,e);   
	    distance = sqrt(s_e.at<float>(0,0));
	    d_sum = d_sum + distance;
	    distances.push_back(distance);
	}
	// Calculate standard deviation for the whole data set
	float d_mean = d_sum/number_total;
	float dv_sum = 0;
	for (int i=0; i<number_total; i++) {
	    dv_sum = dv_sum + pow(distances[i]-d_mean,2);
	}
	//float std_dv = sqrt(dv_sum/number_total);
	//cout << "standard_deviation = "<< endl << " " << std_dv << endl << endl;
	// Calculate threshold t = sqrt(5.99)std_dv
	//float t = sqrt(5.99)*std_dv;
	// After real tests, t is around 50, which is really a large threshold.
	// Here, we set t to 7, which makes the output almost same with openCV.
	int t = 7;
	// Calculate number of inliers
	vector<int> inliers_status(src_points.size(),0);
	int n_inliers = 0;
	for (int i=0; i<number_total; i++) {
	    if (distances[i]<t) {
		n_inliers++;
		// Update the inlier status vector
		inliers_status[i]=1;
	    }	
	}
	// Calculate standard deviation for inliers
	float d_in_sum = 0;
	float dv_in_sum = 0;
	for (int i=0; i<number_total; i++) {
	    if (inliers_status[i]==1) {
		d_in_sum = d_in_sum + distances[i];
	    }
	}
	float d_in_mean = d_in_sum/n_inliers;
	for (int i=0; i<number_total; i++) {
	    if (inliers_status[i]==1) {
		dv_in_sum = dv_in_sum + pow(distances[i]-d_in_mean,2);
	    }
	}
	
	float std_dv_in = sqrt(dv_in_sum/n_inliers);

	// Pick the better homography
	// if this is not the first time to calculate H
	if (sample_count!=0) {
	    if (n_inliers>H_inliers) {
		H=h;
		H_standard_deviation = std_dv_in;
		H_inliers = n_inliers;
		H_inliers_status = inliers_status;
	    }
	    else if (n_inliers==H_inliers) {
		if (std_dv_in<H_standard_deviation) {
		    H=h;
		    H_standard_deviation = std_dv_in;
		    H_inliers_status = inliers_status;
		}
	    }
	}
	else {
	    H=h;
	    H_standard_deviation = std_dv_in;
	    H_inliers = n_inliers;
	    H_inliers_status = inliers_status;
	}   
	// Update N
	//cout << "inliers_number = "<< endl << " " << H_inliers << endl << endl;
	double eps = 1-static_cast<double>(H_inliers)/static_cast<double>(number_total);
	//cout << "point number = "<< endl << " " << number_total << endl << endl;
	//cout << "eps = "<< endl << " " << eps << endl << endl;
	N = static_cast<int>(log(1-p)/log(1-pow(1-eps,s)));
	cout << "N = "<< endl << " " << N << endl << endl;
	sample_count++;
	//cout << "sample_count = "<< endl << " " << sample_count << endl << endl;
	// After N times sampling, RANSAC picks the best homography 
	// and reserves all the inliers
    }
    //cout << "total number = "<< endl << " " << number_total << endl << endl;
    // Then we need to do the optimal estimation using LM 
    // First of all, we need to prepare the true inliers vectors
    for (int i=0; i<number_total; i++) {
	if (H_inliers_status[i]==1) {
	    src_inliers.push_back(src_points[i]);
	    dst_inliers.push_back(dst_points[i]);
	}	    
    }
    inlier_struct data = {H_inliers, src_inliers, dst_inliers};
    // Optimal estimation
    do {
	H_inliers = data.n_inliers;
	Mat H_est = MLE_with_H(data.src_inliers, data.dst_inliers, H);
	H = H_est;
	data = Stablize(src_points, dst_points, H);

    } while (data.n_inliers != H_inliers);
    // Output the number of inliers for RANSAC test
    cout << "inliers_number = "<< endl << " " << H_inliers << endl << endl;
    return H;
}

inlier_struct Stablize(vector<Point2f> src_points, vector<Point2f> dst_points, Mat H) {
    int number_total = src_points.size();
    // Inliers vector
    vector<Point2f> src_inliers, dst_inliers;
    // Calcualte d for each correspondence in the whole data
    float d_sum = 0;
    vector<float> distances;
    float distance = 0;
    for (int i=0; i<number_total; i++) {
	pair<Point2f,Point2f> X = make_pair(src_points[i],dst_points[i]);
	Mat e = Epsilon(X,H);
	Mat j = Jacobian(X,H);
	Mat s_e = Sampson_error(j,e);   
	distance = sqrt(s_e.at<float>(0,0));
	d_sum = d_sum + distance;
	distances.push_back(distance);
    }
    // Set threshold to 7
    int t = 7;
    // Calculate number of inliers
    int n_inliers = 0;
    for (int i=0; i<number_total; i++) {
	if (distances[i]<t) {
	    n_inliers++;
	    // Update the inlier status vector
	    src_inliers.push_back(src_points[i]);
	    dst_inliers.push_back(dst_points[i]);

	}	
    }
    inlier_struct data ={n_inliers, src_inliers, dst_inliers};
    return data;
}


bool isCollinear(vector<Point2f> sample_points) {
    for (int i=0; i<sample_points.size()-2; i++) {
	Mat A = Point2fToMat(sample_points[i]);
	for (int j=i+1; j<sample_points.size()-1; j++) {
	    Mat B = Point2fToMat(sample_points[j]);
	    Mat AB = A.cross(B);
	    for (int k=j+1; k<sample_points.size(); k++) {
		Mat C = Point2fToMat(sample_points[k]);
		// Check whether point C lies on line AB
		// x^T l = 0
		float result = C.dot(AB);
		if (abs(result) < 0.01) {
		    return true;
		}
	    }
	}
    }
    return false;
}

void Evaluate_Sampson( const double *par, int m_dat, const void *data, double *fvec, int *info ) {
    data_struct *D = (data_struct*)data;
    // Initialize homography matrix H from parameters par
    Mat H = Mat::zeros(3,3,CV_32FC1);
    for (int i=0; i<3; i++) {
	for (int j=0; j<3; j++) {
	    H.at<float>(i,j)=par[i*3+j];
	}
    }
    // For the lmfit's sake, we need to let it calculate the norm of Sampson's error
    // limfit is not written for OpenCV. It is really hard to use.
    // However, the LM solver in OpenCV is not available to use either.
    int size_pair = D->src_points.size();
    for	(int i=0; i< size_pair; i++) {
	pair<Point2f,Point2f> X = make_pair(D->src_points[i],D->dst_points[i]);
	Mat e = Epsilon(X, H);
	Mat j = Jacobian(X, H);
	Mat d = Sampson(j, e);
	fvec[4*i] = d.at<float>(0,0);
	fvec[4*i+1] = d.at<float>(1,0);
	fvec[4*i+2] = d.at<float>(2,0);
	fvec[4*i+3] = d.at<float>(3,0);
    }
}

Mat Jacobian(pair<Point2f,Point2f> X, Mat H) {
    // Jacobian is a 2x4 matrix
    Mat J = Mat::zeros(2,4,CV_32FC1);
    J.at<float>(0,0)=-H.at<float>(1,0)+X.second.y*H.at<float>(2,0);
    J.at<float>(0,1)=-H.at<float>(1,1)+X.second.y*H.at<float>(2,1);
    J.at<float>(0,3)=X.first.x*H.at<float>(2,0)+X.first.y*H.at<float>(2,1)+H.at<float>(2,2);

    J.at<float>(1,0)=H.at<float>(0,0)-X.second.x*H.at<float>(2,0);
    J.at<float>(1,1)=H.at<float>(0,1)-X.second.x*H.at<float>(2,1);
    J.at<float>(1,2)=-(X.first.x*H.at<float>(2,0)+X.first.y*H.at<float>(2,1)+H.at<float>(2,2));

    return J;
}

Mat Epsilon(pair<Point2f,Point2f> X, Mat H) {
    Mat A = CreateMatrixA(X.first, X.second);
    // Reshape H into a column vector
    Mat h = H.reshape(0,9);
    Mat E = A*h;

    return E;
}

Mat Sampson(Mat J, Mat E) {
    Mat JJ = J*J.t();
    Mat d_x = -J.t()*JJ.inv()*E;
    Mat S = E.t()*JJ.inv()*E;

    return d_x;
}

Mat Sampson_error(Mat J, Mat E) {
    Mat JJ = J*J.t();
    Mat S = E.t()*JJ.inv()*E;

    return S;
}

void ProjectPoints (vector<Point2f> points, vector<Point2f> &new_points, Mat H) {
    for (int i=0; i<points.size(); i++) {
	// Calculate the projected point
	Mat p = H*Point2fToMat(points[i]);
	// Transfer the point from Mat format into Point2f
	// After projection, it needs to be transferred to inhomogeneous coordinates
	// which means X_3 = 1
	new_points.push_back(MatToPoint2f(p));
    }
}

Point2f MatToPoint2f (Mat p) {
    Point2f point(0,0);
    // Normalize the Mat p to make X_3 = 1
    p = p/p.at<float>(2,0);
    point.x = p.at<float>(0,0);
    point.y = p.at<float>(1,0);

    return point;
}

Mat Point2fToMat (Point2f point) {
    // Change point from Point2f into Mat
    Mat p(3,1,CV_32FC1);
    p.at<float>(0,0)=point.x;
    p.at<float>(1,0)=point.y;
    p.at<float>(2,0)=1;
 
    return p;
}

Mat Normalize (vector<Point2f> points) {
    float sum_x = 0; 
    float sum_y = 0;
    // Calculate mean for x and y
    for (int i=0; i<points.size(); i++) {
	sum_x = sum_x + points[i].x;
	sum_y = sum_y + points[i].y;
    }
    float x_mean = sum_x /points.size();
    float y_mean = sum_y /points.size();
    // Calculate average distance between X and origin
    float distance = 0;
    for (int i=0; i<points.size(); i++) {
	distance = distance + sqrt( pow(points[i].x-x_mean,2) + pow(points[i].y-y_mean,2) );
    }
    // Calculate scale s for T
    float scale = sqrt(2)/(distance/points.size());
    // Calculate translation t for T
    float t_x = -scale * x_mean;
    float t_y = -scale * y_mean;
    // Create Matrix T and fill in with s and t
    Mat T = Mat::eye(3,3,CV_32FC1);
    T.at<float>(0,0)=scale;
    T.at<float>(1,1)=scale;
    T.at<float>(0,2)=t_x;
    T.at<float>(1,2)=t_y;
    
    return T;
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

