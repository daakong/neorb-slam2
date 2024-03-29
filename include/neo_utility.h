//
// Created by da on 2021/11/24.
//

//include LOGURU stuffs
#pragma once
#define LOGURU_WITH_STREAMS 1
#include "src/loguru.hpp"
#include "armadillo"
#include<opencv2/core/core.hpp>

#define NEO_FLIP_IMG

#define TIME_DEBUG

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

typedef struct FrameLog{
    int idLog;
    int inliers;
    float scoreLog;
    std::string timestamp_string;
};

bool saveLogFile(std::vector<FrameLog> &LogVec);

