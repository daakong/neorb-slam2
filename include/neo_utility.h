//
// Created by da on 2021/11/24.
//

//include LOGURU stuffs
#define LOGURU_WITH_STREAMS 1
#include "src/loguru.hpp"
#include "armadillo"
#include<opencv2/core/core.hpp>


//#define TIME_DEBUG // FOR TIME DEBUG


namespace  ORB_SLAM2 {
//
//    inline bool log_flag(bool flag);
//
//    inline bool log_flag(bool flag, char success_target);
//
//    inline bool log_flag(bool flag, char success_target, int frameID);



    float localEntropy4uv(cv::Mat img, int u_to_entropy, int v_to_entropy);
    void computeGradImg(const cv::Mat & gray_img_in, cv::Mat & grad_img_out);
    float uvScore_uni(float entropy_in, float grad_in);


}


