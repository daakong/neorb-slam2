//
// Created by da on 2021/11/24.
//
#include "neo_utility.h"
#include "loguru.hpp"

namespace  ORB_SLAM2 {
//    inline bool log_flag(bool flag) {
//        if (flag) {
//            LOG_S(INFO) << "Success.";
//        } else {
//            LOG_S(WARNING) << "something went WRONG!!";
//        }
//    }
//
//    inline bool log_flag(bool flag, char success_target) {
//        if (flag) {
//            LOG_S(INFO) << success_target << "Success.";
//        } else {
//            LOG_S(WARNING) << success_target << "something went WRONG!!";
//        }
//    }
//
//    inline bool log_flag(bool flag, char success_target, int frameID) {
//        if (flag) {
//            LOG_S(INFO) << success_target << "Success. Frame" << frameID;
//        } else {
//            LOG_S(WARNING) << success_target << "something went WRONG!!!!Frame" << frameID;
//        }
//    }

    float localEntropy4uv(cv::Mat img, int u_to_entropy, int v_to_entropy){

        //from CSDN, 做了重要调整，源代码bug很多无法运行: https://blog.csdn.net/u011268787/article/details/80426789

        //模板大小 半径是3  模板大小9*9  //9*9非常扯淡？
        int w = 5; // 半径
        int m1 = img.rows;
        int n1 = img.cols;
        cv::Mat img_padding;
        //先对图片的四周进行补零
        cv::Mat Hist1 = cv::Mat::zeros(1, 256, CV_32F);
        cv::copyMakeBorder(img, img_padding, w, w, w, w, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0,0));


        cv::Mat imgn = cv::Mat::zeros(m1+6, n1+6,CV_64F);

        int m = m1 + 2 * w;
        int n = n1 + 2 * w;

//        int x00 = img_padding.rows;
//        int y00 = img_padding.cols;
////        LOG_S(INFO) << "PADDING X:" << x00 << "PADDING Y:" <<y00;

//        for (int i = w; i < m - w; i++) {
//            for (int j = w; j < n - w;j++) {
        cv::Mat Hist = cv::Mat::zeros(1, 400,CV_32F);
        int m_loop = u_to_entropy;
        int n_loop = v_to_entropy;
        for (int p = m_loop - w; p <= m_loop + w;p++)
        {
            for (int q = n_loop - w; q <= n_loop + w;q++) {
                int tmp_int = (int) img_padding.at<uchar>(p, q);
                if(tmp_int < 400){
                    //cout << "  " << int(img_padding.at<float>(p, q));
                    Hist.at<float>(int(img_padding.at<uchar>(p, q))) = int (Hist.at<float>(int(img_padding.at<uchar>(p, q)))) + 1;  //这个地方有问题 // 注问题根本不在这里。。。
                    //cout << int(Hist.at<float>(int(img_padding.at<float>(p, q)))) << " ";
                }
//                        else {
                // neo: 不想打印出来。。。
//                            cout << p << " " << q<<" ";
//                            cout << int(img_padding.at<uchar>(p, q));
//                            cout << "-----------------------------------------";
//                        }

            }
        }
//            Hist = Hist / (cv::sum(cv::sum(Hist)));    // 将直方图归一化
        //cout << "???:" << Hist.at<float>(0, 0);
        normalize(Hist, Hist,1.0000, 0.0000, cv::NORM_MINMAX);

        /*for (int i = 0; i < 256; i++) {
            if (Hist.at<float>(0,i) != 0) {
                cout << Hist.at<float>(0, i) << " ";
            }
        }*/
        float out_entropy;

        //cout << double(imgn.at<float>(i,j)) << endl;
        //cout << "什么数值：" << Hist.at<float>(20) << " ";
        for (int k = 0; k <256;k++) {
            if (float(Hist.at<float>(k)) > 0.0001 ) {

                out_entropy = out_entropy + float(Hist.at<float>(k))* (float(log2(float(1.0 / Hist.at<float>(k))))); // 局部熵

            }

        }
//            }
//        }
//        cv::Rect rect(3, 3, 424, 87);
//        imgn = imgn(rect);
//        return imgn;
        return out_entropy;
    }




}