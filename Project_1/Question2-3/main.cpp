/*
 * @file main.cpp 
 * @brief use four point correspondence to rectify the image
 * @author Zhixing Li
 */

#include "global.h"

void readme();
/*
 * @function main
 * @brief Main function
 */

int main( int argc, char** argv ) {
    //double t = (double)getTickCount();
    // Check the input command line    
    if( argc != 2 ) { 
	readme(); return -1; 
    }
    // Read the original image
    Mat original_image = imread( argv[1] );
    // Check image reading
    if( !original_image.data ) { 
	cout<< " --(!) Error reading images " << endl; return -1; 
    }
    // Step 1: Manually select four points in the original image
    // in opencv image coordinate system, x-axis and y-axis looks like the following:
    // ------------------------------> x
    // |
    // |	_____________
    // |	|	    |
    // |	|	    |
    // |	|   img	    |
    // |	|	    |
    // |	+-----------+
    // | y
    // v

    vector<Point2f> src_points;
	
    // Point set of image (a)
    /*
    Point2f src_point_top_left(1015,448);
    Point2f src_point_top_right(1198,443);
    Point2f src_point_bottom_left(1017,735);
    Point2f src_point_bottom_right(1200,705);
    */
    
    // Point set of image (b)
    Point2f src_point_top_left(450,735);
    Point2f src_point_top_right(662,745);
    Point2f src_point_bottom_left(453,987);
    Point2f src_point_bottom_right(660,1029);
    
    src_points.push_back(src_point_top_left);
    src_points.push_back(src_point_top_right);
    src_points.push_back(src_point_bottom_left);
    src_points.push_back(src_point_bottom_right);

    // Mark the lines l1 l2 m1 m2 o1 o2
    line(original_image, src_point_top_left, src_point_top_right,CV_RGB(0,255,0),3);
    line(original_image, src_point_bottom_left, src_point_bottom_right,CV_RGB(0,255,0),3);
    line(original_image, src_point_top_left, src_point_bottom_left,CV_RGB(0,255,0),3);
    line(original_image, src_point_top_right, src_point_bottom_right,CV_RGB(0,255,0),3);
    line(original_image, src_point_top_left, src_point_bottom_right,CV_RGB(0,255,0),3);
    line(original_image, src_point_top_right, src_point_bottom_left,CV_RGB(0,255,0),3);

    // Step 3: Calculate Homography
    //Mat Homography = getPerspectiveTransform(src_points, dst_points);
    //Mat Homography = findHomography(src_points, dst_points);

    Mat H_p = AffineHomography(src_points);
    cout << "my HP = "<< endl << " " << H_p << endl << endl;
    Mat H_a = MetricHomography(src_points,H_p);
    cout << "my HA = "<< endl << " " << H_a << endl << endl;
        
    Mat H = H_a*H_p;
    // H(0,1)=0.25
    /* If this is set to 0, then UCinfUT = 0, H should be perfect
       However, after the experiment, the perfect H will not rectify the image correctly
       Oppositely, the not-perfect H can rectify the image correctly, but not perfect
    */
    //H.at<float>(0,1)=0;
    cout << "my H = "<< endl << " " << H << endl << endl;


    // Step 4: Warp the image
    Mat projective_rectified_image, rectified_image, my_rectified_image;;
    projective_rectified_image=WarpScale(original_image,H_p);
    rectified_image=WarpScale(projective_rectified_image,H_a);
    my_rectified_image=WarpScale(original_image,H);
    
    // Verify the dual conics
    Mat C = Mat::eye(3,3,CV_32FC1);
    C.at<float>(2,2)=0;
    cout << "my Cinf = "<< endl << " " << H*C*H.t() << endl << endl;

    //t = ((double)getTickCount() - t)/getTickFrequency();
    // Step 5: Display the rectified image
    //imshow("Original Image", original_image);
    //imshow("Rectified Image", my_rectified_image);
    imwrite("original_image.jpg", original_image);
    imwrite("rectified_image.jpg", rectified_image);
    imwrite("projective_rectified_image.jpg", projective_rectified_image);
    imwrite("my_rectified_image.jpg", my_rectified_image);

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
