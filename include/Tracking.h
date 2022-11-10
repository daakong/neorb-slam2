/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>
#include <armadillo>

namespace ORB_SLAM2
{




class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageStereo(const int frame_n, const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp,
                              const cv::Mat &imLastframe, const cv::Mat &imRightLastframe,
                              cv::Mat &imgray_LastKeyframe);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageRGBD(const int frame_n, const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp,
                          const cv::Mat &imLastframe, const cv::Mat &imDepthLastframe,
                          cv::Mat &imgray_LastKeyframe); // this is the neo version
        cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal length should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal length
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    typedef struct DrawKeyNeo {
        arma::rowvec position;
        double score;
    }neodraw;


    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    bool neoTrackReferenceKeyFrame(bool if_has_exframe, const cv::Mat & lastimRGB, const cv::Mat & lastimDepth,
                                   const cv::Mat & imgray_keyframe);
    void UpdateLastFrame();
    bool TrackWithMotionModel();
    bool neoTrackWithMotionModel(bool if_has_exframe, const cv::Mat & lastimRGB, const cv::Mat & lastimDepth);


    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    ///////neo function
    // sig_uv 应当是一个2元素rowvec。 sig_p 应当是一个三元素rowvec。

    inline void synthInfoMat(const Frame *F, const int &kptIdx, const MapPoint *pMP,
                             const arma::mat &H_meas, const float &res_u, const float &res_v,
                             const arma::mat &H_proj, arma::mat &H_rw_row,
                             const arma::rowvec & sig_uv, const arma::rowvec & sig_p);

    // sig_uv 应当是一个2元素rowvec。 sig_p 应当是一个三元素rowvec。
    inline void computerSigma(const Frame *F, const int &kptIdx, const MapPoint *pMP,
                              const arma::mat &H_meas, const float &res_u, const float &res_v,
                              const arma::mat &H_proj, arma::mat &H_rw,
                              arma::rowvec & sig_uv, arma::rowvec & sig_p, const cv::Mat & score_3d_homo,
                              const cv::Mat & img_grad_current);
    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    void neoRGBD_Track(bool if_has_exframe, const cv::Mat & exframe_rgb, const cv::Mat & exframe_depth,
                       cv::Mat & ex_keyframe_gray);


    bool neoBuildInfoMat(Frame &inFrame, bool call_from_motion_model,
                                   double& score, vector<neodraw>& neodraw_vec);
    bool neoBuildInfoMat_new(bool if_has_exframe, Frame &inFrame, Frame &exFrame, bool call_from_motion_model, arma::mat & infoMat, vector<MapPointWithScore>& mp_exframe_withScore,
                         vector<neodraw> &neodraw_vec, int & nmatches);

    bool
    Computer_H_subBlock(const cv::Mat &Tcw, const arma::Row<double> &yi, arma::Mat<double> &H13,
                         arma::Mat<double> &H47,
                         arma::Mat<double> &dhu_dhrl, const bool check_viz, float &u, float &v);

    //comes from good feature
    inline void reWeightInfoMat(const Frame *F, const int &kptIdx, const MapPoint *pMP,
                                const arma::mat &H_meas, const float &res_u, const float &res_v,
                                const arma::mat &H_proj, arma::mat &H_rw)
    {

        int measSz = H_meas.n_rows;
        arma::mat Sigma_r(measSz, measSz), W_r(measSz, measSz);
        Sigma_r.eye();

        if (F != NULL && kptIdx >= 0 && kptIdx < F->mvKeysUn.size())
        {
            //
            float Sigma2 = F->mvLevelSigma2[F->mvKeysUn[kptIdx].octave];
            Sigma_r = Sigma_r * Sigma2;

//            std::cout << Sigma_r ;
//            std::cout << "sigma2:" << Sigma2 << std::endl;


        }

        // cholesky decomp of diagonal-block scaling matrix W
        if (arma::chol(W_r, Sigma_r, "lower") == true)
        {
            // scale the meas. Jacobian with the scaling block W_r
            H_rw = arma::inv(W_r) * H_meas;
//            std::cout << H_rw;
        }
        else{
            // do nothing
                                std::cout << "chol failed!" << std::endl;
//                                std::cout << "oct level =" << kpUn.octave << "; invSigma2 = " << invSigma2 << std::endl;
        }

        //#ifdef WITH_QUALITY_WEIGHTED_JACOBIAN
        //        double quality_max = double(ORBmatcher::TH_HIGH);
        //#ifdef OBS_DEBUG_VERBOSE
        //        std::cout << F->mvpMatchScore[i] << std::endl;
        //        std::cout << H << std::endl;
        //#endif
        //        double weight_qual = std::max(0.0, double(quality_max - F->mvpMatchScore[i]) / double(quality_max));
        //        weight_qual = weight_qual * (1.0 - BASE_WEIGHT_QUAL) + BASE_WEIGHT_QUAL;
        //        H = H * weight_qual;
        //#ifdef OBS_DEBUG_VERBOSE
        //        std::cout << weight_qual << std::endl;
        //        std::cout << H << std::endl << std::endl << std::endl;
        //#endif
        //#endif

    }

    inline void convert_to_rainbow(double score, cv::Vec3b& pixel){
        unsigned char grayValue = (unsigned char) score;
//    cv::Vec3b pixel;
        if (grayValue <= 51) {
            pixel[0] = 255;
            pixel[1] = grayValue * 5;
            pixel[2] = 0;
        }
        else if (grayValue <= 102)
        {
            grayValue -= 51;
            pixel[0] = 255 - grayValue * 5;
            pixel[1] = 255;
            pixel[2] = 0;
        }
        else if (grayValue <= 153)
        {
            grayValue -= 102;
            pixel[0] = 0;
            pixel[1] = 255;
            pixel[2] = grayValue * 5;
        }
        else if (grayValue <= 204)
        {
            grayValue -= 153;
            pixel[0] = 0;
            pixel[1] = 255 - static_cast
                    <unsigned char>(grayValue
                                    * 128.0 / 51 + 0.5);
            pixel[2] = 255;
        }
        else if (grayValue <= 255)
        {
            grayValue -= 204;
            pixel[0] = 0;
            pixel[1] = 127 - static_cast
                    <unsigned char>(grayValue
                                    * 127.0 / 51 + 0.5);
            pixel[2] = 255;
        }
    }

    bool
    neoGet_H_subBlock_using_score(const cv::Mat &Tcw, const arma::rowvec &yi, arma::mat &H_out, arma::mat &dh_dp,
                      const bool check_viz, float &u, float &v, const cv::Mat & score);
    bool
    neoGet_H_subBlock_using_score(const cv::Mat &Tcw, const arma::rowvec &yi, arma::mat &H13, arma::mat &H47, arma::mat &dhu_dhrl,
                      const bool check_viz, float &u, float &v, const arma::rowvec & score);




    bool
    neoGet_H_subBlock(const cv::Mat &Tcw, const arma::rowvec &yi, arma::mat &H13, arma::mat &H47, arma::mat &dhu_dhrl,
                      const bool check_viz, float &u, float &v);

    bool neoComputeLastFrameScore(bool if_has_exframe, const cv::Mat &lastimRGB,
                                  vector<MapPointWithScore> &lastMp_score, const cv::Mat & Tcw_exframe,
                                  const bool if_for_KF);

    void drawPointsWrap(vector<neodraw> &neodraw_inframe, int matches0, float  score0);

//    bool computeGradImg(const cv::Mat &gray_img_in, cv::Mat &grad_img_out);
};

} //namespace ORB_SLAM

#endif // TRACKING_H
