//#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

//#include <mgl2/mgl.h>

#include <iostream>
#include <queue>
#include <stdio.h>

#include "constants.h"
#include "helpers.h"
#include "findEyeCenter.h"

using namespace cv;
using namespace std;
// Pre-declarations
cv::Mat floodKillEdges(cv::Mat &mat);

#pragma mark Helpers

cv::Point unscalePoint(cv::Point p, cv::Rect origSize) {
    float ratio = (((float)FAST_EYE_WIDTH)/origSize.width);
    int x = round(p.x / ratio);
    int y = round(p.y / ratio);
    return cv::Point(x,y);
}

void scaleToFastSize(const cv::Mat &src,cv::Mat &dst) {
    cv::resize(src, dst, cv::Size(FAST_EYE_WIDTH,(((float)FAST_EYE_WIDTH)/src.cols) * src.rows));
}

cv::Mat computeMatXGradient(const cv::Mat &mat, cv::Mat mask) {
    cv::Mat out(mat.rows,mat.cols,CV_64F);
    out = cv::Scalar(0);
    for (int y = 0; y < mat.rows; ++y) {
        const uchar *Mr = mat.ptr<uchar>(y);
        double *Or = out.ptr<double>(y);
        uint8_t *maskRow = mask.ptr<uint8_t>(y);
        //Or[0] = Mr[1] - Mr[0];
        for (int x = 1; x < mat.cols - 1; ++x) {
            if (maskRow[x+10] <255 || maskRow[x-10] <255 ){
                // continue;
            }
            Or[x] = (Mr[x + 1] - Mr[x - 1]) / 2.0;

        }
        Or[mat.cols-1] = Mr[mat.cols-1] - Mr[mat.cols-2];
    }

    return out;
}

#pragma mark Main Algorithm

void testPossibleCentersFormula(int x, int y, const cv::Mat &weight,double gx, double gy, cv::Mat &out, cv::Mat &mask, cv::Mat &ref, int &numMultiplications) {
    // for all possible centers
    for (int cy = 0; cy < out.rows; ++cy) {
        double *Or = out.ptr<double>(cy);
        uint8_t *refRow = ref.ptr<uint8_t>(cy);
        uint8_t *maskRow = mask.ptr<uint8_t>(cy);
        const unsigned char *Wr = weight.ptr<unsigned char>(cy);
        for (int cx = 0; cx < out.cols; ++cx) {
            if (maskRow[cx] <255){
                continue;
            }
            if (x == cx && y == cy) {
                continue;
            }
            if (refRow[cx] > 50){
                continue;
            }

            // create a vector from the possible center to the gradient origin
            double dx = x - cx;
            double dy = y - cy;

            // normalize d
            double magnitude = sqrt((dx * dx) + (dy * dy));
            dx = dx / magnitude;
            dy = dy / magnitude;
            double dotProduct = dx*gx + dy*gy;
            dotProduct = std::max(0.0,dotProduct);

            // square and multiply by the weight
            if (ENABLE_WEIGHT) {
                Or[cx] += dotProduct * dotProduct * (Wr[cx]/WEIGHT_DIVISOR);
            } else {
                Or[cx] += dotProduct * dotProduct;
            }
            numMultiplications ++;
        }
    }
}

void equalizeForTube(cv::Mat eyeROI, cv::Mat mask){
    int sum = 0;
    for (int y = 0; y < eyeROI.rows; ++y) {
        uint8_t *eyeRow = eyeROI.ptr<uint8_t>(y);
        uint8_t *maskRow = mask.ptr<uint8_t>(y);
        for (int x = 0; x < eyeROI.cols; ++x) {
            if (maskRow[x] >0){
                continue;
            }
            sum = sum + eyeRow[x];
        }
    }
    int averageBrightness = sum/(eyeROI.rows * eyeROI.cols);

    for (int y = 0; y < eyeROI.rows; ++y) {
        uint8_t *eyeRow = eyeROI.ptr<uint8_t>(y);
        uint8_t *maskRow = mask.ptr<uint8_t>(y);
        for (int x = 0; x < eyeROI.cols; ++x) {
            if (maskRow[x] >0){
                continue;
            }
            eyeRow[x] = averageBrightness;
        }
    }
    equalizeHist(eyeROI, eyeROI);
}

cv::Point findEyeCenter(cv::Mat eyeROI, cv::Mat mask, float* confidence) {
    equalizeForTube(eyeROI, mask);
    // draw eye region
    //-- Fd the gradient
    cv::Mat gradientX = computeMatXGradient(eyeROI, mask);
    cv::Mat gradientY = computeMatXGradient(eyeROI.t(), mask.t()).t();
    //-- Normalize and threshold the gradient
    // compute all the magnitudes
    cv::Mat mags = matrixMagnitude(gradientX, gradientY);
    //compute the threshold
    double gradientThresh = computeDynamicThreshold(mags, GRADIENT_THRESHOLD);

    for (int y = 0; y < eyeROI.rows; ++y) {
        double *Xr = gradientX.ptr<double>(y), *Yr = gradientY.ptr<double>(y);
        const double *Mr = mags.ptr<double>(y);
        uint8_t *maskRow = mask.ptr<uint8_t>(y);
        for (int x = 0; x < eyeROI.cols; ++x) {
            if (maskRow[x] <255){
                continue;
            }
            double gX = Xr[x], gY = Yr[x];
            double magnitude = Mr[x];

            if (magnitude > gradientThresh) {
                Xr[x] = gX/magnitude;
                Yr[x] = gY/magnitude;
            } else {
                Xr[x] = 0.0;
                Yr[x] = 0.0;
            }
        }
    }

    //-- Create a blurred and inverted image for weighting
    cv::Mat weight;
    GaussianBlur( eyeROI, weight, cv::Size( WEIGHT_BLUR_SIZE, WEIGHT_BLUR_SIZE ), 0, 0 );
    for (int y = 0; y < weight.rows; ++y) {
        unsigned char *row = weight.ptr<unsigned char>(y);
        for (int x = 0; x < weight.cols; ++x) {
            row[x] = (255 - row[x]);
        }
    }

    //-- Run the algorithm!
    cv::Mat outSum = cv::Mat::zeros(eyeROI.rows,eyeROI.cols,CV_64F);

    // for each possible gradient location
    // Note: these loops are reversed from the way the paper does them
    // it evaluates every possible center for each gradient location instead of
    // every possible gradient location for every center.
    //
    int numMultiplications = 0;
    for (int y = 0; y < weight.rows; ++y) {
        const double *Xr = gradientX.ptr<double>(y), *Yr = gradientY.ptr<double>(y);
        uint8_t *maskRow = mask.ptr<uint8_t>(y);
        for (int x = 0; x < weight.cols; ++x) {
            double gX = Xr[x], gY = Yr[x];
            if (maskRow[x] <255){
                continue;
            }
            if (gX == 0.0 && gY == 0.0) {
                continue;
            }
            testPossibleCentersFormula(x, y, weight, gX, gY, outSum, mask,eyeROI, numMultiplications);
        }
    }

   // printf("EyeRoi dims: %ix%i\n",eyeROI.cols, eyeROI.rows);
   // printf("OUT dims: %ix%i\n",outSum.cols, outSum.rows);
    //printf("Number of multiplications: %i\n",numMultiplications);


    // scale all the values down, basically averaging them
    double numGradients = (weight.rows*weight.cols);
    cv::Mat out;
    cv::Mat out8;
    outSum.convertTo(out, CV_32F,100/numGradients);
    outSum.convertTo(out8, CV_8U);

    //cv::imshow("EyeROI",eyeROI);
    //cv::imshow("GradientX",gradientX);
    //cv::imshow("GradientY",gradientX);
    //cv::imshow("Centers",out);
    //-- Find the maximum point
    cv::Point maxP;
    double maxVal;
    cv::minMaxLoc(out, NULL,&maxVal,NULL,&maxP);

    //-- Flood fill the edges
    if(ENABLE_POST_PROCESS) {
        cv::Mat floodClone;
        //double floodThresh = computeDynamicThreshold(out, 1.5);
        double floodThresh = maxVal * POST_PROCESS_THRESHOLD;
        cv::threshold(out, floodClone, floodThresh, 0.0f, cv::THRESH_TOZERO);
        cv::Mat mask = floodKillEdges(floodClone);
        //imshow(debugWindow + " Mask",mask);
        //imshow(debugWindow,out);
        // redo max
        cv::minMaxLoc(out, NULL,&maxVal,NULL,&maxP,mask);
    }
    return maxP;
}

cv::Point findCenterWithBlobs(cv::Mat eye, cv::Mat mask){
    //equalizeForTube(eye, mask);

    vector<vector<Point> > contours;
    Mat eye2;
    eye.copyTo(eye2);
    findContours(eye, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    Mat cimage = Mat::zeros(eye.size(), CV_8UC3);

    for(size_t i = 0; i < contours.size(); i++)
    {
        size_t count = contours[i].size();
        if( count < 6 )
            continue;

        Mat pointsf;
        Mat(contours[i]).convertTo(pointsf, CV_32F);
        RotatedRect box = fitEllipse(pointsf);

        if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
            continue;
        drawContours(eye2, contours, (int)i, Scalar::all(255), 1, 8);

        ellipse(eye2, box, Scalar(0,0,255), 1, CV_AA);
        ellipse(eye2, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, CV_AA);
        //ellipse(cimage, box, Scalar(0,0,255), 1, CV_AA);
        //ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, CV_AA);
        Point2f vtx[4];
        box.points(vtx);
        for( int j = 0; j < 4; j++ ) {
            //line(cimage, vtx[j], vtx[(j + 1) % 4], Scalar(0, 255, 0), 5, CV_AA);
            line(eye2, vtx[j], vtx[(j + 1) % 4], Scalar(255, 255, 255), 1, CV_AA);
        }
    }

    imshow("eye_ellipse", eye2);
    imshow("result", cimage);


   /* std::vector<cv::KeyPoint> keypoints;
    detector->detect(eye, keypoints);
    cv::Mat im_with_keypoints;
    //cv::drawKeypoints( eye, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    imshow("Keypoints",im_with_keypoints);*/
    return cv::Point(0,0);
}

#pragma mark Postprocessing

bool floodShouldPushPoint(const cv::Point &np, const cv::Mat &mat) {
    return inMat(np, mat.rows, mat.cols);
}

// returns a mask
cv::Mat floodKillEdges(cv::Mat &mat) {
    rectangle(mat,cv::Rect(0,0,mat.cols,mat.rows),255);

    cv::Mat mask(mat.rows, mat.cols, CV_8U, 255);
    std::queue<cv::Point> toDo;
    toDo.push(cv::Point(0,0));
    while (!toDo.empty()) {
        cv::Point p = toDo.front();
        toDo.pop();
        if (mat.at<float>(p) == 0.0f) {
            continue;
        }
        // add in every direction
        cv::Point np(p.x + 1, p.y); // right
        if (floodShouldPushPoint(np, mat)) toDo.push(np);
        np.x = p.x - 1; np.y = p.y; // left
        if (floodShouldPushPoint(np, mat)) toDo.push(np);
        np.x = p.x; np.y = p.y + 1; // down
        if (floodShouldPushPoint(np, mat)) toDo.push(np);
        np.x = p.x; np.y = p.y - 1; // up
        if (floodShouldPushPoint(np, mat)) toDo.push(np);
        // kill it
        mat.at<float>(p) = 0.0f;
        mask.at<uchar>(p) = 0;
    }
    return mask;
}
