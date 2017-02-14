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
    //Point2f src_point_top_left(525,45);
    //Point2f src_point_top_right(1010,100);
    //Point2f src_point_bottom_left(580,1230);
    //Point2f src_point_bottom_right(1020,1085);
    
    // Point set of image (b)
    Point2f src_point_top_left(665,400);
    Point2f src_point_top_right(1115,350);
    Point2f src_point_bottom_left(660,1375);
    Point2f src_point_bottom_right(1060,1520);

    
    src_points.push_back(src_point_top_left);
    src_points.push_back(src_point_top_right);
    src_points.push_back(src_point_bottom_left);
    src_points.push_back(src_point_bottom_right);

    // Mark the selected points
    circle(original_image, src_point_top_left,2,CV_RGB(0,255,0),3);
    circle(original_image, src_point_top_right,2,CV_RGB(0,255,0),3);
    circle(original_image, src_point_bottom_left,2,CV_RGB(0,255,0),3);
    circle(original_image, src_point_bottom_right,2,CV_RGB(0,255,0),3);

    // Step 2: Manually select four points in the rectified image
    vector<Point2f> dst_points;
    
    // Point set of image (a)
    //Point2f dst_point_top_left(500,100);
    //Point2f dst_point_top_right(1000,100);
    //Point2f dst_point_bottom_left(500,1000);
    //Point2f dst_point_bottom_right(1000,1000);

     // Point set of image (a)
    Point2f dst_point_top_left(700,400);
    Point2f dst_point_top_right(1200,400);
    Point2f dst_point_bottom_left(700,1400);
    Point2f dst_point_bottom_right(1200,1400);

   
    dst_points.push_back(dst_point_top_left);
    dst_points.push_back(dst_point_top_right);
    dst_points.push_back(dst_point_bottom_left);
    dst_points.push_back(dst_point_bottom_right);

    // Step 3: Calculate Homography
    //Mat Homography = getPerspectiveTransform(src_points, dst_points);
    //Mat Homography = findHomography(src_points, dst_points);

    Mat myHomography = DLTHomography(src_points, dst_points);
    cout << "my H = "<< endl << " " << myHomography << endl << endl;
    Mat cvHomography = findHomography(src_points, dst_points);
    cout << "true H = "<< endl << " " << cvHomography << endl << endl;

 
    // Step 4: Warp the image
    Mat cv_rectified_image, my_rectified_image;
    warpPerspective(original_image, cv_rectified_image, cvHomography, original_image.size());
    warpPerspective(original_image, my_rectified_image, myHomography, original_image.size());

    //t = ((double)getTickCount() - t)/getTickFrequency();
    // Step 5: Display the rectified image
    //imshow("Original Image", original_image);
    //imshow("Rectified Image", my_rectified_image);
    imwrite("original_image.jpg", original_image);
    imwrite("cv_rectified_image.jpg", cv_rectified_image);
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
