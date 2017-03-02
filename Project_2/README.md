In this project, H matrix is asked to be computed using different algorithms.

1. Basic DLT algorithm.
2. Normalized DLT algorithm.
3. Gold Standard algorithm using Maximum Likelihood Estimation.
4. RANSAC algorithm

There are two types of errors to choose. One is reprojection error, the other is Sampson error. 

Reprojection error has a slightly better accuracy compared to Sampson error, however, it requires much more parameters: 2n+9. Sampson error only needs 9 parameters, which is the size of homography matrix. 

Therefore, in further consideration, Sampson error is adapted in this gold standard MLE algorithm. 
Levenberg-Marquardt algorithm is used to minimize the Sampson error. 

In the implementation code, a math package called [lmfit](http://apps.jcns.fz-juelich.de/doku/sc/lmfit) is in charge of the calculation.  

Last but not least, we will compare our own results with those generated from OpenCV findHomography function with RANSAC flag turned on. 
