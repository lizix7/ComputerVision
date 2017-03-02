/*
 * @file main.cpp 
 * @brief use DLT algorithm to stitch the images
 * @author Zhixing Li
 */

#include "global.h"

void readme();
/*
 * @function main
 * @brief Main function
 */

int main( int argc, char** argv ) {
    // Check the input command line    
    if( argc != 3 ) { 
	readme(); return -1; 
    }
    // Read the original image
    Mat img_1 = imread( argv[1]);
    Mat img_2 = imread( argv[2]);
    // Conver image to gray-scale
    Mat gray_img_1;
    Mat gray_img_2;
    cvtColor(img_1, gray_img_1, CV_RGB2GRAY);
    cvtColor(img_2, gray_img_2, CV_RGB2GRAY);
    // Check image reading
    if( !img_1.data || !img_2.data ) { 
	cout<< " --(!) Error reading images " << endl; return -1; 
    }
    // Declare the distance
    double max_dist = 0; double min_dist = 100;
    // Declare keypoints
    vector<KeyPoint> keypoints_1, keypoints_2;
    // Declare descriptors
    Mat descriptors_1, descriptors_2;

    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(minHessian);
    detector->detectAndCompute( gray_img_1, Mat(), keypoints_1, descriptors_1 );
    detector->detectAndCompute( gray_img_2, Mat(), keypoints_2, descriptors_2 );

    /*//-- Step 2: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    vector<DMatch> matches;
    matcher.match( descriptors_1, descriptors_2, matches );
     */

    //-- Step 2: Matching descriptor vectors using brute force matcher
    BFMatcher matcher(NORM_L2);
    vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    //-- Step 3: Quick calculation of max and min distances between keypoints
    for(int i=0; i<descriptors_1.rows; i++ ) { 
	double dist = matches[i].distance;
	if( dist < min_dist ) min_dist = dist;
	if( dist > max_dist ) max_dist = dist;
    }

    //-- Step 4: Draw "good" matches
    vector< DMatch > good_matches;
    for(int i=0; i<descriptors_1.rows; i++ ) { 
	if( matches[i].distance <= max(2*min_dist, 0.02) ) { 
	    good_matches.push_back( matches[i]); 
	}
    }
    Mat img_matches;
    drawMatches( gray_img_1, keypoints_1, gray_img_2, keypoints_2, 
		 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Show detected matches
    //imshow( "Good Matches", img_matches );
    //-- Step 5: Calculate Homography that project image 2 to image 1
    vector<Point2f> src_points, dst_points;
    for (int i=0; i<good_matches.size(); i++) {
	dst_points.push_back(keypoints_1[good_matches[i].queryIdx].pt);
	src_points.push_back(keypoints_2[good_matches[i].trainIdx].pt);
    }

    
    // Test module for RANSAC
    // Random generator for the first image
    double mean = 400.0;
    double std_dev = 3;
    random_device rd_1;
    mt19937 g_1(rd_1());	
    normal_distribution<> d_1(mean,std_dev);
    random_device rd_2;
    mt19937 g_2(rd_2());	
    normal_distribution<> d_2(mean,std_dev);

    int N = 300;
    for (int i=0; i<N; i++) {
	src_points.push_back(Point2f(d_1(g_1),d_2(g_2)));
	dst_points.push_back(Point2f(d_1(g_1),d_2(g_2)));
    }
    
    
    double t = (double)getTickCount();

    //Mat DLT_H = DLT(src_points, dst_points);
    //cout << "my DLT_H = "<< endl << " " << DLT_H << endl << endl;
    //Mat N_DLT_H = Normalized_DLT(src_points, dst_points);
    //cout <<"my N_DLT_H = "<< endl << " " << N_DLT_H << endl << endl;
    //Mat MLE_H = MLE(src_points, dst_points);
    //cout << "my MLE_H = "<< endl << " " << MLE_H << endl << endl;
    Mat RANSAC_H = Ransac(src_points, dst_points);
    cout << "my RANSAC_H = "<< endl << " " << RANSAC_H << endl << endl;
    //Mat cvHomography = findHomography(src_points, dst_points, CV_RANSAC);
    //cout << "true H = "<< endl << " " << cvHomography << endl << endl;
 
    t = ((double)getTickCount() - t)/getTickFrequency();
    cout << "Times passed in seconds: " << t << endl;

    //-- Step 6: Warp the image
    //Mat cv_rectified_image, my_rectified_image;
    Mat result;
    //warpPerspective(img_2, result, cvHomography, Size(img_2.cols+img_1.cols,img_2.rows));
    //warpPerspective(img_2, result, DLT_H, Size(img_2.cols+img_1.cols,img_2.rows));
    //warpPerspective(img_2, result, N_DLT_H, Size(img_2.cols+img_1.cols,img_2.rows));
    //warpPerspective(img_2, result, MLE_H, Size(img_2.cols+img_1.cols,img_2.rows));
    warpPerspective(img_2, result, RANSAC_H, Size(img_2.cols+img_1.cols,img_2.rows));
    Mat half(result,Rect(0,0,img_1.cols,img_1.rows));
    img_1.copyTo(half);

    // Step 5: Display the rectified image
    //imshow("Stitched Image", result);
    //imwrite("Stitched_image_OpenCV.jpg", result);
    //imwrite("feature_match.jpg", img_matches);

    //cout << "Times passed in seconds: " << t << endl;

    waitKey(0);
    return 0;
}
/*
 * @function readme
 */
void readme() { 
    cout << " Usage: ./four_point_rectification <img1> " << endl;
}
