//
// Created by etienne on 9/28/16.
//
#include <opencv2/imgproc/imgproc.hpp>

#ifndef PROJECT_KALMANFILTER_H
#define PROJECT_KALMANFILTER_H

class KalmanFilter{
    int x_measured   [2];
    int x_est   [2];
    int x_last [2];
    float gk   [2];
    float pk   [2];
    float pk_last [2];
    float confidence;
    bool initialized;

    void predict(){
        for (int i = 0; i <= 2; ++i) {
         x_est[i] = x_last[i];
         pk[i]= pk_last[i];
        }

    }

    void update(){
        for (int i = 0; i <= 2; i++) {
            gk[i] = pk_last[i]/(pk_last[i] +confidence);
            x_est[i] = x_est[i] + gk[i]*(x_measured[i] - x_est[i]);
            pk[i] = (1-gk[i])*pk_last[i];
            x_last[i] = x_est[i];
        }
    }
public:
    KalmanFilter(){
        initialized = false;
    }
    bool isInit(){ return initialized;};
    void initialize(cv::Mat frame){
        x_last[0] = frame.cols/2;
        x_last[1]= frame.rows/2;
        pk_last[0] = 1.0f;
        pk_last[1] = 1.0f;
        initialized = true;
    }
    cv::Point filterPosition(cv::Point p, float r) {
        confidence = r;
        x_measured[0] = p.x;
        x_measured[1] = p.y;
        predict();
        update();
        cv::Point ret;
        ret.x = x_est[0];
        ret.y= x_est[1];
        return ret;
    }

};

#endif //PROJECT_KALMANFILTER_H
