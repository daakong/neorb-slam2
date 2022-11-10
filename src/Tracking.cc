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


#include "Tracking.h"
#include "neo.h"
#include "neo_utility.h"

#include <src/loguru.cpp>



#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <unistd.h>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>

#include "gf_Util.hpp"

using namespace std;

namespace ORB_SLAM2 {


    Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap,
                       KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor) :
            mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
            mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer *>(NULL)), mpSystem(pSys), mpViewer(NULL),
            mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0) {
        // Load camera parameters from settings file

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0) {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        float fps = fSettings["Camera.fps"];
        if (fps == 0)
            fps = 30;

        // Max/Min Frames to insert keyframes and to check relocalisation
        mMinFrames = 0;
        mMaxFrames = fps;

        cout << endl << "Camera Parameters: " << endl;
        cout << "- fx: " << fx << endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if (DistCoef.rows == 5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;
        cout << "- fps: " << fps << endl;


        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;

        if (mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        // Load ORB parameters

        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];

        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::STEREO)
            mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::MONOCULAR)
            mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        cout << endl << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        if (sensor == System::STEREO || sensor == System::RGBD) {
            mThDepth = mbf * (float) fSettings["ThDepth"] / fx;
            cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }

        if (sensor == System::RGBD) {
            mDepthMapFactor = fSettings["DepthMapFactor"];
            if (fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        }

    }

    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
        mpLocalMapper = pLocalMapper;
    }

    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
        mpLoopClosing = pLoopClosing;
    }

    void Tracking::SetViewer(Viewer *pViewer) {
        mpViewer = pViewer;
    }


    cv::Mat Tracking::GrabImageStereo(const int frame_n, const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp,
                                      const cv::Mat &imLastframe, const cv::Mat &imRightLastframe,
                                      cv::Mat &imgray_LastKeyframe
                                      ) {
        mImGray = imRectLeft;
        cv::Mat imGrayRight = imRectRight;

        if (mImGray.channels() == 3) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            }
        } else if (mImGray.channels() == 4) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            }
        }

        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary,
                              mK, mDistCoef, mbf, mThDepth);

//        Track();
//        neoTrack();
        if(frame_n == 0){
            neoRGBD_Track(false, imLastframe, imRightLastframe, imgray_LastKeyframe);
        } else{
            neoRGBD_Track(true, imLastframe, imRightLastframe, imgray_LastKeyframe);
        }

        return mCurrentFrame.mTcw.clone();
    }

    cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp) {
        mImGray = imRectLeft;
        cv::Mat imGrayRight = imRectRight;

        if (mImGray.channels() == 3) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            }
        } else if (mImGray.channels() == 4) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            }
        }

        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary,
                              mK, mDistCoef, mbf, mThDepth);

//        Track();
//        neoTrack();
        return mCurrentFrame.mTcw.clone();
    }


    cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp) {
        mImGray = imRGB;
        cv::Mat imDepth = imD;

        if (mImGray.channels() == 3) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        } else if (mImGray.channels() == 4) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
            imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                              mThDepth);

//    Track();
//        neoRGBD_Track(false, imRGB, imDepth, imRGB);

        LOG_S(ERROR) << "rUNING WRONG CODE!!!";
        return mCurrentFrame.mTcw.clone();
    }

    cv::Mat Tracking::GrabImageRGBD( const int frame_n, const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp,
                                     const cv::Mat &imLastframe, const cv::Mat &imDepthLastframe,
                                     cv::Mat &imgray_LastKeyframe
    ) {
        mImGray = imRGB;
        cv::Mat imDepth = imD;

        if (mImGray.channels() == 3) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        } else if (mImGray.channels() == 4) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
            imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                              mThDepth);

//    Track();

        if(frame_n == 0){
            neoRGBD_Track(false, imLastframe, imDepthLastframe, imgray_LastKeyframe);
        } else{
            neoRGBD_Track(true, imLastframe, imDepthLastframe, imgray_LastKeyframe);
        }

        return mCurrentFrame.mTcw.clone();
    }

    cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp) {
        mImGray = im;

        if (mImGray.channels() == 3) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        } else if (mImGray.channels() == 4) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
        else
            mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                                  mThDepth);

        Track();

        return mCurrentFrame.mTcw.clone();
    }

    void Tracking::neoRGBD_Track(bool if_has_exframe, const cv::Mat &exframe_rgb, const cv::Mat & exframe_depth,
                                 cv::Mat & ex_keyframe_gray) {
        if (mState == NO_IMAGES_YET) {
            mState = NOT_INITIALIZED;
        }

        mLastProcessedState = mState;
#ifndef NEO_RGBD_NOR_STEREO

#endif

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        if (mState == NOT_INITIALIZED) {
            if (mSensor == System::STEREO || mSensor == System::RGBD)
                StereoInitialization();
            else
                MonocularInitialization();

            mpFrameDrawer->Update(this);

            if (mState != OK)
                return;
        } else {
            // System is initialized. Track Frame.
            bool bOK;
//            LOG_S(INFO) << "Entering0." << mCurrentFrame.mnId << " state" << !(mbOnlyTracking || DISABLE_LOCALMAP);

            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            if (!(mbOnlyTracking || DISABLE_LOCALMAP)) {
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.
                LOG_S(INFO) << "Entering here." << mCurrentFrame.mnId;

                if (mState == OK) {
                    // Local Mapping might have changed some MapPoints tracked in last frame
                    CheckReplacedInLastFrame();

                    if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
//                        LOG_S(INFO) << "To track reference frame here." << mCurrentFrame.mnId;
//                        bOK = TrackReferenceKeyFrame();
                        bOK = neoTrackReferenceKeyFrame(if_has_exframe, exframe_rgb, exframe_depth, ex_keyframe_gray);
                    } else {
//                        LOG_S(INFO) << "To track motion model here." << mCurrentFrame.mnId;
//                        bOK = TrackWithMotionModel();
                        bOK = neoTrackWithMotionModel(if_has_exframe, exframe_rgb, exframe_depth);
                        if (!bOK){
//                            bOK = TrackReferenceKeyFrame();
                            bOK = neoTrackReferenceKeyFrame(if_has_exframe, exframe_rgb, exframe_depth, ex_keyframe_gray);
                        }
                    }
                } else {
                    bOK = Relocalization();
                }
            } else {
                // Localization Mode: Local Mapping is deactivated

                if (mState == LOST) {
                    bOK = Relocalization();
                } else {
                    if (!mbVO) {
                        // In last frame we tracked enough MapPoints in the map

                        if (!mVelocity.empty()) {
//                            LOG_S(INFO) << "To track motion model.";
//                            bOK = TrackWithMotionModel();
                            bOK = neoTrackWithMotionModel(if_has_exframe, exframe_rgb, exframe_depth);
                        } else {
//                            LOG_S(INFO) << "To track key frame.";
                            bOK = neoTrackReferenceKeyFrame(if_has_exframe, exframe_rgb, exframe_depth,ex_keyframe_gray);
//                            bOK = TrackReferenceKeyFrame();
                        }


                    } else {
                        // In last frame we tracked mainly "visual odometry" points.

                        // We compute two camera poses, one from motion model and one doing relocalization.
                        // If relocalization is sucessfull we choose that solution, otherwise we retain
                        // the "visual odometry" solution.

                        bool bOKMM = false;
                        bool bOKReloc = false;
                        vector<MapPoint *> vpMPsMM;
                        vector<bool> vbOutMM;
                        cv::Mat TcwMM;
                        if (!mVelocity.empty()) {
                            bOKMM = TrackWithMotionModel();
                            vpMPsMM = mCurrentFrame.mvpMapPoints;
                            vbOutMM = mCurrentFrame.mvbOutlier;
                            TcwMM = mCurrentFrame.mTcw.clone();
                        }
                        bOKReloc = Relocalization();

                        if (bOKMM && !bOKReloc) {
                            mCurrentFrame.SetPose(TcwMM);
                            mCurrentFrame.mvpMapPoints = vpMPsMM;
                            mCurrentFrame.mvbOutlier = vbOutMM;

                            if (mbVO) {
                                for (int i = 0; i < mCurrentFrame.N; i++) {
                                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
                                        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                    }
                                }
                            }
                        } else if (bOKReloc) {
                            mbVO = false;
                        }

                        bOK = bOKReloc || bOKMM;
                    }
                }
            }

            mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // If we have an initial estimation of the camera pose and matching. Track the local map.
            if ((!mbOnlyTracking || DISABLE_LOCALMAP)) {
                if (bOK)
                    bOK = TrackLocalMap();
            } else {
                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                // the camera we will use the local map again.
                if (bOK && !mbVO)
                    bOK = TrackLocalMap();
            }

            if (bOK)
                mState = OK;
            else
                mState = LOST;

            // Update drawer
            mpFrameDrawer->Update(this);

            // If tracking were good, check if we insert a keyframe
            if (bOK) {
                // Update motion model
                if (!mLastFrame.mTcw.empty()) {
                    cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                    mVelocity = mCurrentFrame.mTcw * LastTwc;
                } else
                    mVelocity = cv::Mat();

                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

                // Clean VO matches
                for (int i = 0; i < mCurrentFrame.N; i++) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (pMP)
                        if (pMP->Observations() < 1) {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                        }
                }

                // Delete temporal MapPoints
                for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end();
                     lit != lend; lit++) {
                    MapPoint *pMP = *lit;
                    delete pMP;
                }
                mlpTemporalPoints.clear();

                // Check if we need to insert a new keyframe
                if (NeedNewKeyFrame()) {
                    CreateNewKeyFrame();
                    mImGray.copyTo(ex_keyframe_gray);
                }
                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame.
                for (int i = 0; i < mCurrentFrame.N; i++) {
                    if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
            }

            // Reset if the camera get lost soon after initialization
            if (mState == LOST) {
                if (mpMap->KeyFramesInMap() <= 5) {
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    mpSystem->Reset();
                    return;
                }
            }

            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            mLastFrame = Frame(mCurrentFrame);
        }

        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if (!mCurrentFrame.mTcw.empty()) {
            cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState == LOST);
        } else {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState == LOST);
        }

    }


    void Tracking::Track() {
        if (mState == NO_IMAGES_YET) {
            mState = NOT_INITIALIZED;
        }

        mLastProcessedState = mState;

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        if (mState == NOT_INITIALIZED) {
            if (mSensor == System::STEREO || mSensor == System::RGBD)
                StereoInitialization();
            else
                MonocularInitialization();

            mpFrameDrawer->Update(this);

            if (mState != OK)
                return;
        } else {
            // System is initialized. Track Frame.
            bool bOK;

            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            if (!(mbOnlyTracking || DISABLE_LOCALMAP)) {
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.

                if (mState == OK) {
                    // Local Mapping might have changed some MapPoints tracked in last frame
                    CheckReplacedInLastFrame();

                    if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
                        bOK = TrackReferenceKeyFrame();
                    } else {
                        bOK = TrackWithMotionModel();
                        if (!bOK)
                            bOK = TrackReferenceKeyFrame();
                    }
                } else {
                    bOK = Relocalization();
                }
            } else {
                // Localization Mode: Local Mapping is deactivated


                if (mState == LOST) {
                    bOK = Relocalization();
                } else {
                    if (!mbVO) {
                        // In last frame we tracked enough MapPoints in the map

                        if (!mVelocity.empty()) {
                            bOK = TrackWithMotionModel();
                        } else {
                            bOK = TrackReferenceKeyFrame();
                        }
                    } else {
                        // In last frame we tracked mainly "visual odometry" points.

                        // We compute two camera poses, one from motion model and one doing relocalization.
                        // If relocalization is sucessfull we choose that solution, otherwise we retain
                        // the "visual odometry" solution.

                        bool bOKMM = false;
                        bool bOKReloc = false;
                        vector<MapPoint *> vpMPsMM;
                        vector<bool> vbOutMM;
                        cv::Mat TcwMM;
                        if (!mVelocity.empty()) {
                            bOKMM = TrackWithMotionModel();
                            vpMPsMM = mCurrentFrame.mvpMapPoints;
                            vbOutMM = mCurrentFrame.mvbOutlier;
                            TcwMM = mCurrentFrame.mTcw.clone();
                        }
                        bOKReloc = Relocalization();

                        if (bOKMM && !bOKReloc) {
                            mCurrentFrame.SetPose(TcwMM);
                            mCurrentFrame.mvpMapPoints = vpMPsMM;
                            mCurrentFrame.mvbOutlier = vbOutMM;

                            if (mbVO) {
                                for (int i = 0; i < mCurrentFrame.N; i++) {
                                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
                                        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                    }
                                }
                            }
                        } else if (bOKReloc) {
                            mbVO = false;
                        }

                        bOK = bOKReloc || bOKMM;
                    }
                }
            }

            mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // If we have an initial estimation of the camera pose and matching. Track the local map.
            if (!mbOnlyTracking) {
                if (bOK)
                    bOK = TrackLocalMap();
            } else {
                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                // the camera we will use the local map again.
                if (bOK && !mbVO)
                    bOK = TrackLocalMap();
            }

            if (bOK)
                mState = OK;
            else
                mState = LOST;

            // Update drawer
            mpFrameDrawer->Update(this);

            // If tracking were good, check if we insert a keyframe
            if (bOK) {
                // Update motion model
                if (!mLastFrame.mTcw.empty()) {
                    cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                    mVelocity = mCurrentFrame.mTcw * LastTwc;
                } else
                    mVelocity = cv::Mat();

                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

                // Clean VO matches
                for (int i = 0; i < mCurrentFrame.N; i++) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (pMP)
                        if (pMP->Observations() < 1) {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                        }
                }

                // Delete temporal MapPoints
                for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end();
                     lit != lend; lit++) {
                    MapPoint *pMP = *lit;
                    delete pMP;
                }
                mlpTemporalPoints.clear();

                // Check if we need to insert a new keyframe
                if (NeedNewKeyFrame())
                    CreateNewKeyFrame();

                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame.
                for (int i = 0; i < mCurrentFrame.N; i++) {
                    if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
            }

            // Reset if the camera get lost soon after initialization
            if (mState == LOST) {
                if (mpMap->KeyFramesInMap() <= 5) {
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    mpSystem->Reset();
                    return;
                }
            }

            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            mLastFrame = Frame(mCurrentFrame);
        }

        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if (!mCurrentFrame.mTcw.empty()) {
            cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState == LOST);
        } else {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState == LOST);
        }

    }


    void Tracking::StereoInitialization() {
        if (mCurrentFrame.N > 500) {
            // Set Frame pose to the origin
            mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

            // Create KeyFrame
            KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

            // Insert KeyFrame in the map
            mpMap->AddKeyFrame(pKFini);

            // Create MapPoints and asscoiate to KeyFrame
            for (int i = 0; i < mCurrentFrame.N; i++) {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0) {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap);
                    pNewMP->AddObservation(pKFini, i);
                    pKFini->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                }
            }

            cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

            mpLocalMapper->InsertKeyFrame(pKFini);

            mLastFrame = Frame(mCurrentFrame);
            mnLastKeyFrameId = mCurrentFrame.mnId;
            mpLastKeyFrame = pKFini;

            mvpLocalKeyFrames.push_back(pKFini);
            mvpLocalMapPoints = mpMap->GetAllMapPoints();
            mpReferenceKF = pKFini;
            mCurrentFrame.mpReferenceKF = pKFini;

            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

            mpMap->mvpKeyFrameOrigins.push_back(pKFini);

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            mState = OK;
        }
    }

    void Tracking::MonocularInitialization() {

        if (!mpInitializer) {
            // Set Reference Frame
            if (mCurrentFrame.mvKeys.size() > 100) {
                mInitialFrame = Frame(mCurrentFrame);
                mLastFrame = Frame(mCurrentFrame);
                mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
                for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

                if (mpInitializer)
                    delete mpInitializer;

                mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

                return;
            }
        } else {
            // Try to initialize
            if ((int) mCurrentFrame.mvKeys.size() <= 100) {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
                return;
            }

            // Find correspondences
            ORBmatcher matcher(0.9, true);
            int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches,
                                                           100);

            // Check if there are enough correspondences
            if (nmatches < 100) {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                return;
            }

            cv::Mat Rcw; // Current Camera Rotation
            cv::Mat tcw; // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

            if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
                for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
                    if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
                        mvIniMatches[i] = -1;
                        nmatches--;
                    }
                }

                // Set Frame Poses
                mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
                Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
                tcw.copyTo(Tcw.rowRange(0, 3).col(3));
                mCurrentFrame.SetPose(Tcw);

                CreateInitialMapMonocular();
            }
        }
    }

    void Tracking::CreateInitialMapMonocular() {
        // Create KeyFrames
        KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
        KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);


        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // Insert KFs in the map
        mpMap->AddKeyFrame(pKFini);
        mpMap->AddKeyFrame(pKFcur);

        // Create MapPoints and asscoiate to keyframes
        for (size_t i = 0; i < mvIniMatches.size(); i++) {
            if (mvIniMatches[i] < 0)
                continue;

            //Create MapPoint.
            cv::Mat worldPos(mvIniP3D[i]);

            MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap);

            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, mvIniMatches[i]);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            //Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            //Add to Map
            mpMap->AddMapPoint(pMP);
        }

        // Update Connections
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        // Bundle Adjustment
        cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

        Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

        // Set median depth to 1
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth = 1.0f / medianDepth;

        if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100) {
            cout << "Wrong initialization, reseting..." << endl;
            Reset();
            return;
        }

        // Scale initial baseline
        cv::Mat Tc2w = pKFcur->GetPose();
        Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale points
        vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
            if (vpAllMapPoints[iMP]) {
                MapPoint *pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
            }
        }

        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        mLastFrame = Frame(mCurrentFrame);

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mState = OK;
    }

    void Tracking::CheckReplacedInLastFrame() {
        for (int i = 0; i < mLastFrame.N; i++) {
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (pMP) {
                MapPoint *pRep = pMP->GetReplaced();
                if (pRep) {
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
    }

    bool Tracking::neoBuildInfoMat(Frame &inFrame, bool call_from_motion_model,
                                   double &score, vector<neodraw> &neodraw_vec) {
        // old frame-to-local-map archieve...
        //this code is abandoned.
        LOG_S(ERROR)<< "Your are using abandoned code!!";
        return false;
//    LOG_S(INFO) << "entering neo info map";

        arma::mat H13, H47, H_meas, H_proj, H_disp, H_rw;
        float res_u = 0, res_v = 0, u, v;

        vector<arma::mat> pointsInfoMatrices;
        for (int i = 0; i < inFrame.mvpMapPoints.size(); i++) {
            MapPoint *pMp = inFrame.mvpMapPoints[i];

            if (pMp) {
                if (inFrame.mvbOutlier[i] == false) {

                    neodraw drawPoint;
                    // for feature positions
                    arma::rowvec featurePosi = arma::zeros<arma::rowvec>(4);
                    cv::Mat pos = pMp->GetWorldPos();
                    featurePosi[0] = pos.at<float>(0);
                    featurePosi[1] = pos.at<float>(1);
                    featurePosi[2] = pos.at<float>(2);
                    featurePosi[3] = 1;

                    //for Measurement positions
                    arma::rowvec measurePosi = arma::zeros<arma::rowvec>(2);
                    cv::KeyPoint kpUn = inFrame.mvKeysUn[i];
                    measurePosi[0] = kpUn.pt.x;
                    measurePosi[1] = kpUn.pt.y;
                    drawPoint.position = measurePosi;

                    bool flag = false;

                    // insert h subblock compute
                    flag = Tracking::Computer_H_subBlock(inFrame.mTcw, featurePosi.subvec(0, 2), H13, H47, H_proj,
                                                         true, u, v);
                    res_u = measurePosi[0] - u;
                    res_v = measurePosi[1] - v;

                    // assemble into H matrix
                    H_meas = arma::join_horiz(H13, H47);

                    if (flag) {
//                   LOG_S(INFO) << "H computing success. Frame" << inFrame.mnId;
                    } else {
                        LOG_S(WARNING) << "H computing: something wrong.....frame" << inFrame.mnId;
                    }

                    reWeightInfoMat(&mCurrentFrame, i, pMp, H_meas, res_u, res_v, H_proj, H_rw);
//                LOG_S(INFO) << "H_measure:" << endl << H_meas << endl << "H_rw:" << endl << H_rw;
                    arma::mat point_infoMat = H_rw.t() * H_rw;
                    double single_point_score = logDet(point_infoMat);
                    drawPoint.score = single_point_score;
                    pointsInfoMatrices.push_back(point_infoMat);
                    neodraw_vec.push_back(drawPoint);
                }
            }
        }
        arma::mat H_c;
        for (vector<arma::mat>::iterator iter = pointsInfoMatrices.begin(); iter != pointsInfoMatrices.end(); iter++) {
            if (iter == pointsInfoMatrices.begin()) {
                H_c = *iter;
            } else {
                H_c = arma::join_vert(H_c, *iter);
            }
        }
//    LOG_S(INFO) << "H_C" << endl << H_c;
        arma::mat infoMat = H_c.t() * H_c;
        score = logDet(infoMat);
//        LOG_S(INFO) << "info Mat" << endl << infoMat;

//    LOG_S(INFO) << "Score computing finished. Frame" << inFrame.mnId << " Score:" << score ;


        return true;
    }


    bool Tracking::neoBuildInfoMat_new(bool if_has_exframe, Frame &inFrame, Frame &exFrame, bool call_from_motion_model, arma::mat & info_mat,
                                   vector<MapPointWithScore>& mp_exframe_withScore, vector<neodraw> &neodraw_vec,
                                   int & nmatches) {

        if(!if_has_exframe){
            LOG_S(WARNING) << "No exframe!";
            info_mat = arma::zeros<arma::mat>(6,6);
            return false;

        }
//    LOG_S(INFO) << "entering neo info map";

        arma::mat  H_meas, H_proj, H_disp, H_rw;
        float res_u = 0, res_v = 0, u, v;

        vector<arma::mat> pointsInfoMatrices;
        vector<arma::mat> h_wr_row_vec;
        arma::mat infoMat_ = arma::zeros<arma::mat>(6,6);
        int matched_id = 0;
#ifdef  TIME_DEBUG
        LOG_S(INFO) << "Timer 3.1";
#endif
        cv::Mat img_gradient;
        computeGradImg(mImGray, img_gradient);


        for (int i = 0; i < inFrame.mvpMapPoints.size(); i++) {
            MapPoint *pMp = inFrame.mvpMapPoints[i];

            if (pMp) {

                ////ngc detection ... don't need anymore.
//                if(pMp == mp_exframe_withScore[matched_id].target_pMp){
//                    matched_id++;
//                }


                if (inFrame.mvbOutlier[i] == false) {

                    neodraw drawPoint;
                    // for feature positions
                    arma::rowvec featurePosi = arma::zeros<arma::rowvec>(4);
                    cv::Mat pos = pMp->GetWorldPos();
                    featurePosi[0] = pos.at<float>(0);
                    featurePosi[1] = pos.at<float>(1);
                    featurePosi[2] = pos.at<float>(2);
                    featurePosi[3] = 1;

                    //for Measurement positions
                    arma::rowvec measurePosi = arma::zeros<arma::rowvec>(2);
                    cv::KeyPoint kpUn = inFrame.mvKeysUn[i];
                    measurePosi[0] = kpUn.pt.x;
                    measurePosi[1] = kpUn.pt.y;
                    drawPoint.position = measurePosi;

                    //// 判断可见吗？
                    if(measurePosi[0] < mCurrentFrame.mnMinX || measurePosi[0] > mCurrentFrame.mnMaxX)
                        continue;
                    if( measurePosi[1] < mCurrentFrame.mnMinY || measurePosi[1] > mCurrentFrame.mnMaxY)
                        continue;

//                    measurePosi[0] +
//                    measurePosi[1] +

                        // arma 模式的向量，有点浪费事情，转为cv：：Mat类型
//                    arma::rowvec score_3d_lastp = arma::zeros<arma::rowvec>(3);
////                    float score_from_local_illum =  mp_exframe_withScore[i].GetScore_arma(score_3d_lastp);
//                    mp_exframe_withScore[i].GetScore_arma(score_3d_lastp);

                    // CVMAT 类型的score值
                    cv::Mat score_3d_homo(4,1,CV_32F);
                    mp_exframe_withScore[i].GetScore_cvmat_homo(score_3d_homo);
//                    LOG_S(INFO) << "SCORE 3D:" << score_3d_lastp[0] << "," <<score_3d_lastp[1];

                    // insert h subblock compute
                    bool flag = Tracking::neoGet_H_subBlock_using_score(inFrame.mTcw, featurePosi.subvec(0, 2), H_meas, H_proj,
                                                                        true, u, v, score_3d_homo);
                    res_u = measurePosi[0] - u;
                    res_v = measurePosi[1] - v;

//                    // assemble into H matrix
//                    H_meas = arma::join_horiz(H13, H47);


                    if (flag) {
//                   LOG_S(INFO) << "H computing success. Frame" << inFrame.mnId;
                    } else {
                        LOG_S(WARNING) << "H computing: something wrong.....frame" << inFrame.mnId;
                    }

                    arma::rowvec sigma_uv, sigma_p;

                    computerSigma(&mCurrentFrame, i, pMp, H_meas, res_u, res_v, H_proj, H_rw,
                                  sigma_uv, sigma_p, score_3d_homo,img_gradient);
//                    LOG_S(INFO) << "SCORE 3D HOMO:" << score_3d_homo;
                    synthInfoMat(&mCurrentFrame, i, pMp, H_meas, res_u, res_v, H_proj, H_rw, sigma_uv, sigma_p);
//                    reWeightInfoMat(&mCurrentFrame, i, pMp, H_meas, res_u, res_v, H_proj, H_rw);
//                LOG_S(INFO) << "H_measure:" << endl << H_meas << endl << "H_rw:" << endl << H_rw;
                    arma::mat point_infoMat = H_rw.t() * H_rw;
//                    if(infoMat_.size()==point_infoMat.size()){
//                        infoMat_ = infoMat_ + point_infoMat;
//                    } else{
//                        // doing nothing
//                    }
//                    float single_point_score = logDet(point_infoMat);
                    float single_point_score = log(arma::det(point_infoMat));
                    drawPoint.score = single_point_score;
                    pointsInfoMatrices.push_back(point_infoMat);
                    h_wr_row_vec.push_back(H_rw);
                    neodraw_vec.push_back(drawPoint);

                    /////// This is for de-slam. To remove points.
#ifdef DE_SLAM_REMOVE_POINTS
                    if(single_point_score < DE_SLAM_SCORE_THRE){
                        inFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                        inFrame.mvbOutlier[i] = false;
                        pMp->mbTrackInView = false;
                        pMp->mnLastFrameSeen = inFrame.mnId;
                        nmatches--;
                    }
#endif

                }
            }

        }
//        LOG_S(INFO) << "important!! matched" << matched_id << ", size:" << mp_exframe_withScore.size();

        arma::mat H_c;
        for (vector<arma::mat>::iterator iter = h_wr_row_vec.begin(); iter != h_wr_row_vec.end(); iter++) {
            if (iter == h_wr_row_vec.begin()) {
                H_c = *iter;
            } else {
                H_c = arma::join_vert(H_c, *iter);
            }
        }
#ifdef  TIME_DEBUG
        LOG_S(INFO) << "Timer 3.4";
#endif

//    LOG_S(INFO) << "H_C" << endl << H_c;
        infoMat_ = H_c.t() * H_c;

        info_mat = infoMat_;
        float score = logDet(infoMat_);
//        LOG_S(INFO) << "info Mat score:" << score << ":"<< endl << infoMat_ ;

//    LOG_S(INFO) << "Score computing finished. Frame" << inFrame.mnId << " Score:" << score ;
#ifdef  TIME_DEBUG
        LOG_S(INFO) << "Timer 3.5";
#endif


        return true;
    }

    inline bool Tracking::neoGet_H_subBlock_using_score(const cv::Mat &Tcw,
                                                        const arma::rowvec &yi,
                                                        arma::mat &H_out,
                                                        arma::mat &dh_dp,
                                                        const bool check_viz,
                                                        float &u, float &v, const cv::Mat & score) {

        cv::Mat idMat = cv::Mat::eye(4, 4, CV_32F);
        arma::rowvec Xv;

//        cv::Mat score_proj(4,1, CV_32F);
//        score_proj = Tcw * score;

        convert_Homo_Pair_To_PWLS_Vec(0, idMat, 1, Tcw, Xv);
        arma::rowvec q_wr = Xv.subvec(3, 6);
        arma::mat R_rw = arma::inv(q2r(q_wr));
        arma::rowvec t_rw = yi - Xv.subvec(0, 2);

        float fu = mCurrentFrame.fx;
        float fv = mCurrentFrame.fy;

        // dhu_dhrl
        // lmk @ camera coordinate
        arma::mat hrl = R_rw * t_rw.t();

        cv::Mat Twc(4,1,CV_32F);
        cv::invert(Tcw, Twc);
        cv::Mat homo_p(4,1,CV_32F);
        homo_p.at<float>(0,0) = yi(0,0);
        homo_p.at<float>(1,0) = yi(0,1);
        homo_p.at<float>(2,0) = yi(0,2);
        homo_p.at<float>(3,0) = 1.0f;
        homo_p = Tcw * homo_p;


        float px = homo_p.at<float>(0,0);
        float py = homo_p.at<float>(1,0);
        float pz = homo_p.at<float>(2,0);
        arma::mat dh_dp_ = arma::zeros<arma::mat>(2,3);
        if(homo_p.at<float>(2,0) < 1e-6){
            // 如果深度太小，则输出0
            //todo
        } else{
            dh_dp_ = {
                    {fu / homo_p.at<float>(2,0), 0.0f, -homo_p.at<float>(0,0) * fu / (homo_p.at<float>(2,0) * homo_p.at<float>(2,0))},
                    {0.0f, fv/homo_p.at<float>(2,0),  -homo_p.at<float>(1,0) * fu / (homo_p.at<float>(2,0) * homo_p.at<float>(2,0))}
            };
        }

//        if (fabs(hrl(2, 0)) < 1e-6) {
//            dhu_dhrl = arma::zeros<arma::mat>(2, 3);
//        } else {
//            //        dhu_dhrl << fu/(hrl(2,0))   <<   0.0  <<   -hrl(0,0)*fu/( std::pow(hrl(2,0), 2.0)) << arma::endr
//            //                 << 0.0    <<  fv/(hrl(2,0))   <<  -hrl(1,0)*fv/( std::pow(hrl(2,0), 2.0)) << arma::endr;
//            dhu_dhrl = {{fu / (hrl(2, 0)), 0.0,              -hrl(0, 0) * fu / (std::pow(hrl(2, 0), 2.0))},
//                        {0.0,              fv / (hrl(2, 0)), -hrl(1, 0) * fv / (std::pow(hrl(2, 0), 2.0))}};
//        }

        arma::mat  dp_dxi_ = arma::zeros<arma::mat>(3,6);
        dp_dxi_ = {
                {1.f, 0.f, 0.f, 0.f,pz , -py },
                {0.f, 1.f, 0.f, -pz, 0.f, px },
                {0.f, 0.f, 1.f, py, -px, 0.f}
        };

//        arma::rowvec qwr_conj = qconj(q_wr);
//        // H matrix subblock (cols 1~3): H13
//        H13 = -1.0 * (dhu_dhrl * R_rw);
//
//        arma::colvec v2;
//        v2 << 1.0 << -1.0 << -1.0 << -1.0 << arma::endr;
//        arma::mat dqbar_by_dq = arma::diagmat(v2);
//
//        // H matrix subblock (cols 4~7): H47
//        H47 = dhu_dhrl * (dRq_times_a_by_dq(qwr_conj, t_rw) * dqbar_by_dq);

        dh_dp =dh_dp_;

        H_out = dh_dp_ * dp_dxi_;
        return true;

    }


    inline bool Tracking::neoGet_H_subBlock_using_score(const cv::Mat &Tcw,
                                                        const arma::rowvec &yi,
                                                        arma::mat &H13, arma::mat &H47,
                                                        arma::mat &dhu_dhrl,
                                                        const bool check_viz,
                                                        float &u, float &v, const arma::rowvec & score) {

        // 这是使用arma格式传递分数值的版本，暂时废弃。。。
        cv::Mat idMat = cv::Mat::eye(4, 4, CV_32F);
        arma::rowvec Xv;

        arma::rowvec score_projected = arma::zeros<arma::rowvec>(4);
        score_projected.subvec(0,2) = score.subvec(0,2);
        score_projected[3] = 1.f;
        cv::Mat score_proj(4,1, CV_32F);
        score_proj.at<float>(0,0) = score[0];
        score_proj.at<float>(1,0) = score[1];
        score_proj.at<float>(2,0) = score[2];
        score_proj.at<float>(3,0) = 1.f;
        score_proj = Tcw * score_proj;

        convert_Homo_Pair_To_PWLS_Vec(0, idMat, 1, Tcw, Xv);
        arma::rowvec q_wr = Xv.subvec(3, 6);
        arma::mat R_rw = arma::inv(q2r(q_wr));
        arma::rowvec t_rw = yi - Xv.subvec(0, 2);

        float fu = mCurrentFrame.fx;
        float fv = mCurrentFrame.fy;
        // dhu_dhrl
        // lmk @ camera coordinate
        arma::mat hrl = R_rw * t_rw.t();

        if (fabs(hrl(2, 0)) < 1e-6) {
            dhu_dhrl = arma::zeros<arma::mat>(2, 3);
        } else {
            //        dhu_dhrl << fu/(hrl(2,0))   <<   0.0  <<   -hrl(0,0)*fu/( std::pow(hrl(2,0), 2.0)) << arma::endr
            //                 << 0.0    <<  fv/(hrl(2,0))   <<  -hrl(1,0)*fv/( std::pow(hrl(2,0), 2.0)) << arma::endr;
            dhu_dhrl = {{fu / (hrl(2, 0)), 0.0,              -hrl(0, 0) * fu / (std::pow(hrl(2, 0), 2.0))},
                        {0.0,              fv / (hrl(2, 0)), -hrl(1, 0) * fv / (std::pow(hrl(2, 0), 2.0))}};
        }

        arma::rowvec qwr_conj = qconj(q_wr);

        // H matrix subblock (cols 1~3): H13
        H13 = -1.0 * (dhu_dhrl * R_rw);

        arma::colvec v2;
        v2 << 1.0 << -1.0 << -1.0 << -1.0 << arma::endr;
        arma::mat dqbar_by_dq = arma::diagmat(v2);

        // H matrix subblock (cols 4~7): H47
        H47 = dhu_dhrl * (dRq_times_a_by_dq(qwr_conj, t_rw) * dqbar_by_dq);

        return true;

    }
    inline bool Tracking::neoGet_H_subBlock(const cv::Mat &Tcw,
                                            const arma::rowvec &yi,
                                            arma::mat &H13, arma::mat &H47,
                                            arma::mat &dhu_dhrl,
                                            const bool check_viz,
                                            float &u, float &v) {

        cv::Mat idMat = cv::Mat::eye(4, 4, CV_32F);
        arma::rowvec Xv;

        convert_Homo_Pair_To_PWLS_Vec(0, idMat, 1, Tcw, Xv);
        arma::rowvec q_wr = Xv.subvec(3, 6);
        arma::mat R_rw = arma::inv(q2r(q_wr));
        arma::rowvec t_rw = yi - Xv.subvec(0, 2);

        float fu = mCurrentFrame.fx;
        float fv = mCurrentFrame.fy;
        // dhu_dhrl
        // lmk @ camera coordinate
        arma::mat hrl = R_rw * t_rw.t();

        if (fabs(hrl(2, 0)) < 1e-6) {
            dhu_dhrl = arma::zeros<arma::mat>(2, 3);
        } else {
            //        dhu_dhrl << fu/(hrl(2,0))   <<   0.0  <<   -hrl(0,0)*fu/( std::pow(hrl(2,0), 2.0)) << arma::endr
            //                 << 0.0    <<  fv/(hrl(2,0))   <<  -hrl(1,0)*fv/( std::pow(hrl(2,0), 2.0)) << arma::endr;
            dhu_dhrl = {{fu / (hrl(2, 0)), 0.0,              -hrl(0, 0) * fu / (std::pow(hrl(2, 0), 2.0))},
                        {0.0,              fv / (hrl(2, 0)), -hrl(1, 0) * fv / (std::pow(hrl(2, 0), 2.0))}};
        }

        arma::rowvec qwr_conj = qconj(q_wr);

        // H matrix subblock (cols 1~3): H13
        H13 = -1.0 * (dhu_dhrl * R_rw);

        arma::colvec v2;
        v2 << 1.0 << -1.0 << -1.0 << -1.0 << arma::endr;
        arma::mat dqbar_by_dq = arma::diagmat(v2);

        // H matrix subblock (cols 4~7): H47
        H47 = dhu_dhrl * (dRq_times_a_by_dq(qwr_conj, t_rw) * dqbar_by_dq);

        return true;

    }

    inline bool Tracking::Computer_H_subBlock(const cv::Mat &Tcw,
                                              const arma::rowvec &yi,
                                              arma::mat &H13, arma::mat &H47,
                                              arma::mat &dhu_dhrl,
                                              const bool check_viz,
                                              float &u, float &v) {
        /// THis is the original code from good feature.
        // This part is abandoned now.!
        LOG_S(ERROR) << "You are running abandoned code!!";

        cv::Mat idMat = cv::Mat::eye(4, 4, CV_32F);
        arma::rowvec Xv;

        convert_Homo_Pair_To_PWLS_Vec(0, idMat, 1, Tcw, Xv);
        arma::rowvec q_wr = Xv.subvec(3, 6);
        arma::mat R_rw = arma::inv(q2r(q_wr));
        arma::rowvec t_rw = yi - Xv.subvec(0, 2);

        float fu = mCurrentFrame.fx;
        float fv = mCurrentFrame.fy;
        // dhu_dhrl
        // lmk @ camera coordinate
        arma::mat hrl = R_rw * t_rw.t();

        if (fabs(hrl(2, 0)) < 1e-6) {
            dhu_dhrl = arma::zeros<arma::mat>(2, 3);
        } else {
            //        dhu_dhrl << fu/(hrl(2,0))   <<   0.0  <<   -hrl(0,0)*fu/( std::pow(hrl(2,0), 2.0)) << arma::endr
            //                 << 0.0    <<  fv/(hrl(2,0))   <<  -hrl(1,0)*fv/( std::pow(hrl(2,0), 2.0)) << arma::endr;
            dhu_dhrl = {{fu / (hrl(2, 0)), 0.0,              -hrl(0, 0) * fu / (std::pow(hrl(2, 0), 2.0))},
                        {0.0,              fv / (hrl(2, 0)), -hrl(1, 0) * fv / (std::pow(hrl(2, 0), 2.0))}};
        }

        arma::rowvec qwr_conj = qconj(q_wr);

        // H matrix subblock (cols 1~3): H13
        H13 = -1.0 * (dhu_dhrl * R_rw);

        arma::colvec v2;
        v2 << 1.0 << -1.0 << -1.0 << -1.0 << arma::endr;
        arma::mat dqbar_by_dq = arma::diagmat(v2);

        // H matrix subblock (cols 4~7): H47
        H47 = dhu_dhrl * (dRq_times_a_by_dq(qwr_conj, t_rw) * dqbar_by_dq);

        return true;

    }

    bool Tracking::neoTrackReferenceKeyFrame(bool if_has_exframe, const cv::Mat & lastimRGB, const cv::Mat & lastimDepth,
                                             const cv::Mat & imgray_keyframe) {
        // Compute Bag of Words vector
        mCurrentFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, true);
        vector<MapPoint *> vpMapPointMatches;
        vector<MapPointWithScore> mpwithScore_lastKey;
        mpwithScore_lastKey.resize(mCurrentFrame.N, MapPointWithScore(static_cast<MapPoint*>(NULL), -1, 0, 0));

        int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches, mpwithScore_lastKey);

        if (nmatches < 15)
            return false;

        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.mTcw);
#ifdef TIME_DEBUG
        LOG_S(INFO) << "Timer 1";
#endif
        int mpscore_size = mpwithScore_lastKey.size();
        //        LOG_S(INFO) << "mpscore size:" << mpscore_size;
        neoComputeLastFrameScore(if_has_exframe, lastimRGB, mpwithScore_lastKey, mLastFrame.mTcw, true);
#ifdef TIME_DEBUG
        LOG_S(INFO) << "Timer 2";
#endif


        vector<neodraw> neodraw_inframe;
        float infoScore = 0;
        arma::mat infoMat;
#ifdef TIME_DEBUG
        LOG_S(INFO) << "Timer 3";
#endif

        neoBuildInfoMat_new(if_has_exframe, mCurrentFrame, mLastFrame,true, infoMat, mpwithScore_lastKey, neodraw_inframe
                , nmatches);
//        neoBuildInfoMat(mCurrentFrame, false, infoScore, neodraw_inframe);
//        LOG_S(INFO) << "InfoMat Frame" << mCurrentFrame.mnId << ", Score:" << infoScore << ", nmatches:" << nmatches;
#ifdef TIME_DEBUG

        LOG_S(INFO) << "Timer 4";
        #endif

        infoScore = logDet(infoMat);
        drawPointsWrap(neodraw_inframe, nmatches, infoScore);
#ifdef  TIME_DEBUG
        LOG_S(INFO) << "Timer 5";
#endif

////////以下代码被wrap函数代替。。。
//
//        cv::Mat img_out;
//        cv::Mat img_in;
////    mImGray.copyTo(img_in);
//        cv::cvtColor(mImGray, img_in, CV_GRAY2RGB);
//
////    cv::drawKeypoints(mImGray,
////                      mCurrentFrame.mvKeys,
////                      out_img,
////                      cv::Scalar(0,0,255),
////                      cv::DrawMatchesFlags::DEFAULT);
//
//        for(vector<neodraw>::iterator iter = neodraw_inframe.begin(); iter!= neodraw_inframe.end(); iter++){
//            double color = (-iter->score );
//            cv::Vec3b color_BRG;
//            convert_to_rainbow(color,color_BRG);
////        LOG_S(INFO) << "SCORE SINGLE" << color;
//            cv::circle(img_in, cv::Point(iter->position[0],iter->position[1]), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));
//
//            // 在最后一次迭代中， 加入上限、下限的颜色
//            if(iter == neodraw_inframe.begin()){
//                convert_to_rainbow(0, color_BRG);
//                cv::circle(img_in, cv::Point(10,10), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));
//
//                convert_to_rainbow(255, color_BRG);
//                cv::circle(img_in, cv::Point(20,10), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));
//            }
//
//        }
//
//
////    cv::imshow("key_in",img_in);
//        ostringstream file_name;
//        file_name << "/home/da/active/key_dir/frame" << mCurrentFrame.mnId << ".png";
//        flip(img_in,img_out,-1); //翻转图片
//        cv::imwrite(file_name.str(), img_out);


        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (mCurrentFrame.mvbOutlier[i]) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        return nmatchesMap >= 10;
    }


    bool Tracking::TrackReferenceKeyFrame() {
        // Compute Bag of Words vector
        mCurrentFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, true);
        vector<MapPoint *> vpMapPointMatches;

        int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

        if (nmatches < 15)
            return false;

        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.mTcw);

        vector<neodraw> neodraw_inframe;
        double infoScore = 0;
        neoBuildInfoMat(mCurrentFrame, false, infoScore, neodraw_inframe);
//        LOG_S(INFO) << "InfoMat Frame" << mCurrentFrame.mnId << ", Score:" << infoScore << ", nmatches:" << nmatches;

        cv::Mat img_out;
        cv::Mat img_in;
//    mImGray.copyTo(img_in);
        cv::cvtColor(mImGray, img_in, CV_GRAY2BGR);

//    cv::drawKeypoints(mImGray,
//                      mCurrentFrame.mvKeys,
//                      out_img,
//                      cv::Scalar(0,0,255),
//                      cv::DrawMatchesFlags::DEFAULT);

        for(vector<neodraw>::iterator iter = neodraw_inframe.begin(); iter!= neodraw_inframe.end(); iter++){
            double color = (-iter->score );
            cv::Vec3b color_BRG;
            convert_to_rainbow(color,color_BRG);
//        LOG_S(INFO) << "SCORE SINGLE" << color;
            cv::circle(img_in, cv::Point(iter->position[0],iter->position[1]), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));

            // 在最后一次迭代中， 加入上限、下限的颜色
            if(iter == neodraw_inframe.begin()){
                convert_to_rainbow(0, color_BRG);
                cv::circle(img_in, cv::Point(10,10), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));

                convert_to_rainbow(255, color_BRG);
                cv::circle(img_in, cv::Point(20,10), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));
            }

        }


//    cv::imshow("key_in",img_in);
        ostringstream file_name;
        file_name << "/home/da/active/key_dir/frame" << mCurrentFrame.mnId << ".png";
        flip(img_in,img_out,-1); //翻转图片
        cv::imwrite(file_name.str(), img_out);


        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (mCurrentFrame.mvbOutlier[i]) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        return nmatchesMap >= 10;
    }



    void Tracking::UpdateLastFrame()
    {
        // Update pose according to reference keyframe
        KeyFrame* pRef = mLastFrame.mpReferenceKF;
        cv::Mat Tlr = mlRelativeFramePoses.back();

        mLastFrame.SetPose(Tlr*pRef->GetPose());

        if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
            return;

        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mLastFrame.N);
        for(int i=0; i<mLastFrame.N;i++)
        {
            float z = mLastFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(vDepthIdx.empty())
            return;

        sort(vDepthIdx.begin(),vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for(size_t j=0; j<vDepthIdx.size();j++)
        {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            MapPoint* pMP = mLastFrame.mvpMapPoints[i];
            if(!pMP)
                bCreateNew = true;
            else if(pMP->Observations()<1)
            {
                bCreateNew = true;
            }

            if(bCreateNew)
            {
                cv::Mat x3D = mLastFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

                mLastFrame.mvpMapPoints[i]=pNewMP;

                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            }
            else
            {
                nPoints++;
            }

            if(vDepthIdx[j].first>mThDepth && nPoints>100)
                break;
        }
    }

    bool Tracking::neoComputeLastFrameScore(bool if_has_exframe, const cv::Mat & lastimRGB,
//                                            const cv::Mat & lastimDepth,
                                            vector<MapPointWithScore> & lastMp_score,
                                            const cv::Mat & Tcw_exframe,
                                            const bool if_for_KF)
    {
        bool debug_for_score = false;
        //// This is for RGB-D...
        //// Stereo mode may need modification.
        if (!if_has_exframe){
            LOG_S(WARNING) << "HAS EXFRAME IS FALSE!!";
            return false;
        }
        cv::Mat img_gray, img_grad;
//        if(if_for_KF){
        if(1){
            img_gray = lastimRGB;
        } else{
            if(OIVIO_DIRECT_GRAY){
//                cv::imwrite("/home/da/test0.png",lastimRGB);
                cv::cvtColor(lastimRGB, img_gray, CV_RGB2GRAY);
            }else{
            cv::cvtColor(lastimRGB, img_gray, CV_RGB2GRAY);
            }
        }
        computeGradImg(img_gray, img_grad);
//        cv::convertScaleAbs(tmp_img, img_grad); // 转回uint8
        int fsize = lastMp_score.size();
        for(int i0 = 0; i0 < fsize; i0 ++ ) {
            if(lastMp_score[i0].target_pMp){
                if(debug_for_score)
                    LOG_S(INFO) << lastMp_score[i0].target_pMp;
                float score = -1;
                MapPoint *pmp_tmp = lastMp_score[i0].GetPMP();
                int u_tmp = (int) lastMp_score[i0].GetU();
                int v_tmp = (int) lastMp_score[i0].GetV();
                if(debug_for_score)
                    LOG_S(INFO) << "u" << lastMp_score[i0].GetU() << ",v" << lastMp_score[i0].GetV() << ", i0:" << i0;
                float entropy = localEntropy4uv(img_gray, u_tmp, v_tmp);

                if(debug_for_score)
                    LOG_S(INFO) << "Entropy:" << entropy;
                float grad_squ = img_grad.at<uchar>(v_tmp, u_tmp);
                if(debug_for_score)
                    LOG_S(INFO) << "GRAD:" << grad_squ
                                << ", u" << u_tmp << ",v" << v_tmp;

//                score = ( entropy * grad_squ )/ 40.f;
                score = uvScore_uni(entropy, grad_squ);
                if(debug_for_score)
                    LOG_S(INFO) << "SCORE: " << score;
//                score = entropy / 20.f + grad_squ / 40.f;
//            lastMp_score[i0].SetScore_x(score);
//            lastMp_score[i0].SetScore_y(score);
//            lastMp_score[i0].SetScore_z(0.005f);
                cv::Mat score_vec(4,1,CV_32F);
                score_vec.at<float>(0,0) = score + 1.f;
                score_vec.at<float>(1,0) = score + 1.f;
                score_vec.at<float>(2,0) = 0.05f;
                score_vec.at<float>(3,0) = 1.f;
                cv::Mat Twc;
                cv::invert(Tcw_exframe, Twc);
//                LOG_S(INFO) << "TWC" << Twc;
                score_vec = Twc * score_vec;
                if(debug_for_score)
                    LOG_S(INFO) << "SCORE_VEC:" << endl << score_vec;
                lastMp_score[i0].SetScore(score_vec.at<float>(0,0), score_vec.at<float>(1,0), score_vec.at<float>(2,0));
//                LOG_S(INFO) << "FLAG2";

            } else{

//                LOG_S(INFO) << "YES FLAG! No pmp.";
//                int flag = 0;

            }

        }


        return true;
    }

    bool Tracking::neoTrackWithMotionModel(bool if_has_exframe, const cv::Mat & lastimRGB, const cv::Mat & lastimDepth)
    {
        ORBmatcher matcher(0.9,true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        UpdateLastFrame();

        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

        vector<MapPointWithScore> mpwithScore_last; //可以手动指定1500个大小，应该够用了吧。。。。
        mpwithScore_last.resize(mCurrentFrame.mvpMapPoints.size(), MapPointWithScore(static_cast<MapPoint*>(NULL), -1, 0, 0));

        // Project points seen in previous frame
        int th;
        if(mSensor!=System::STEREO)
            th=15;
        else
            th=7;
        int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR,mpwithScore_last);

        // If few matches, uses a wider window search
        //
        if(nmatches<20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR, mpwithScore_last);
        }

        if(nmatches<20)
            return false;
#ifdef TIME_DEBUG
        LOG_S(INFO) << "Timer 1";
#endif

        int mpscore_size = mpwithScore_last.size();
//        LOG_S(INFO) << "mpscore size:" << mpscore_size;
        neoComputeLastFrameScore(if_has_exframe, lastimRGB, mpwithScore_last, mLastFrame.mTcw, false);

#ifdef TIME_DEBUG
        LOG_S(INFO) << "Timer 2";
#endif

//        LOG_S(INFO) << "mp 10's score:" << mpwithScore_last[10].score;
        vector<neodraw> neodraw_inframe;
        arma::mat infoMat;
#ifdef TIME_DEBUG
        LOG_S(INFO) << "Timer 3";
#endif
        neoBuildInfoMat_new( if_has_exframe, mCurrentFrame, mLastFrame,true, infoMat, mpwithScore_last, neodraw_inframe, nmatches);
        int match_before_discard = nmatches;
        float infoScore = logDet(infoMat);
#ifdef TIME_DEBUG

        LOG_S(INFO) << "Timer 4";
#endif

        drawPointsWrap(neodraw_inframe, nmatches, infoScore);
#ifdef  TIME_DEBUG
        LOG_S(INFO) << "Timer 5";
#endif

        // Optimize frame pose with all matches
//        Optimizer::neoPoseOptimization(&mCurrentFrame, neodraw_inframe); // neo de-slam
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }

//    LOG_S(INFO) << "Frame" << mCurrentFrame.mnId << ", before discard matches" << match_before_discard << ", after discard" << nmatches;

        if(mbOnlyTracking)
        {
            mbVO = nmatchesMap<10;
            return nmatches>20;
        }

        return nmatchesMap>=10;
    }
    bool Tracking::TrackWithMotionModel()
    {
        ORBmatcher matcher(0.9,true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        UpdateLastFrame();

        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

        vector<MapPointWithScore> mpwithScore_last;
//    fill(mpwithScore.begin(), mpwithScore.end(), static_cast<MapPointWithScore*>(NULL));
        // Project points seen in previous frame
        int th;
        if(mSensor!=System::STEREO)
            th=15;
        else
            th=7;
        int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR,mpwithScore_last);

        // If few matches, uses a wider window search
        //
        if(nmatches<20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR, mpwithScore_last);
        }

        if(nmatches<20)
            return false;

        int mpscore_size = mpwithScore_last.size();
//        LOG_S(INFO) << "mpscore size:" << mpscore_size;


        vector<neodraw> neodraw_inframe;
        double infoScore = 0;
        neoBuildInfoMat(mCurrentFrame,true, infoScore, neodraw_inframe);
        int match_before_discard = nmatches;
//    LOG_S(INFO) << "InfoMat Frame" << mCurrentFrame.mnId << ", Score:" << infoScore << ", nmatches:" << nmatches;

        cv::Mat img_out;
        cv::Mat img_in;
//    mImGray.copyTo(img_in);
        cv::cvtColor(mImGray, img_in, CV_GRAY2BGR);

//    cv::drawKeypoints(mImGray,
//                      mCurrentFrame.mvKeys,
//                      out_img,
//                      cv::Scalar(0,0,255),
//                      cv::DrawMatchesFlags::DEFAULT);

        for(vector<neodraw>::iterator iter = neodraw_inframe.begin(); iter!= neodraw_inframe.end(); iter++){
            double color = (-iter->score );
            cv::Vec3b color_BRG;
            convert_to_rainbow(color,color_BRG);
//        LOG_S(INFO) << "SCORE SINGLE" << color;
            cv::circle(img_in, cv::Point(iter->position[0],iter->position[1]), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));

            // 在最后一次迭代中， 加入上限、下限的颜色
            if(iter == neodraw_inframe.begin()){
                convert_to_rainbow(0, color_BRG);
                cv::circle(img_in, cv::Point(10,10), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));

                convert_to_rainbow(255, color_BRG);
                cv::circle(img_in, cv::Point(20,10), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));
            }

        }


//    cv::imshow("key_in",img_in);
        ostringstream file_name;
        file_name << "/home/da/active/key_dir/frame" << mCurrentFrame.mnId << ".png";
#ifdef NEO_FLIP_IMG
        flip(img_in,img_out,-1); //翻转图片
#else
        img_out = img_in.clone();
#endif
        ostringstream text;
        text << "Points:" << nmatches << ",score:" << infoScore;
        cv::Vec3b color_BRG;
        convert_to_rainbow(255, color_BRG);
        cv::putText(img_out, text.str(), cv::Point(50, 50), cv::FONT_HERSHEY_PLAIN,0.8, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]), 2);
        cv::imwrite(file_name.str(), img_out);

        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }

//    LOG_S(INFO) << "Frame" << mCurrentFrame.mnId << ", before discard matches" << match_before_discard << ", after discard" << nmatches;

        if(mbOnlyTracking)
        {
            mbVO = nmatchesMap<10;
            return nmatches>20;
        }

        return nmatchesMap>=10;
    }

    bool Tracking::TrackLocalMap()
    {
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.

        UpdateLocalMap();

        SearchLocalPoints();

        // Optimize Pose
        Optimizer::PoseOptimization(&mCurrentFrame);
        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if(!mbOnlyTracking)
                    {
                        if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                            mnMatchesInliers++;
                    }
                    else
                        mnMatchesInliers++;
                }
                else if(mSensor==System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
            return false;

        if(mnMatchesInliers<30)
            return false;
        else
            return true;
    }


    bool Tracking::NeedNewKeyFrame()
    {
        if(mbOnlyTracking)
            return false;

        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
            return false;

        const int nKFs = mpMap->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
            return false;

        // Tracked MapPoints in the reference keyframe
        int nMinObs = 3;
        if(nKFs<=2)
            nMinObs=2;
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        // Local Mapping accept keyframes?
        bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose= 0;
        if(mSensor!=System::MONOCULAR)
        {
            for(int i =0; i<mCurrentFrame.N; i++)
            {
                if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
                {
                    if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;
                }
            }
        }

        bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

        // Thresholds
        float thRefRatio = 0.75f;
        if(nKFs<2)
            thRefRatio = 0.4f;

        if(mSensor==System::MONOCULAR)
            thRefRatio = 0.9f;

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
        //Condition 1c: tracking is weak
        const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

        if((c1a||c1b||c1c)&&c2)
        {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise send a signal to interrupt BA
            if(bLocalMappingIdle)
            {
                return true;
            }
            else
            {
                mpLocalMapper->InterruptBA();
                if(mSensor!=System::MONOCULAR)
                {
                    if(mpLocalMapper->KeyframesInQueue()<3)
                        return true;
                    else
                        return false;
                }
                else
                    return false;
            }
        }
        else
            return false;
    }

    void Tracking::CreateNewKeyFrame()
    {
        if(!mpLocalMapper->SetNotStop(true))
            return;

        KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        if(mSensor!=System::MONOCULAR)
        {
            mCurrentFrame.UpdatePoseMatrices();

            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            vector<pair<float,int> > vDepthIdx;
            vDepthIdx.reserve(mCurrentFrame.N);
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {
                    vDepthIdx.push_back(make_pair(z,i));
                }
            }

            if(!vDepthIdx.empty())
            {
                sort(vDepthIdx.begin(),vDepthIdx.end());

                int nPoints = 0;
                for(size_t j=0; j<vDepthIdx.size();j++)
                {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                    if(!pMP)
                        bCreateNew = true;
                    else if(pMP->Observations()<1)
                    {
                        bCreateNew = true;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                    }

                    if(bCreateNew)
                    {
                        cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                        MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                        pNewMP->AddObservation(pKF,i);
                        pKF->AddMapPoint(pNewMP,i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpMap->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i]=pNewMP;
                        nPoints++;
                    }
                    else
                    {
                        nPoints++;
                    }

                    if(vDepthIdx[j].first>mThDepth && nPoints>100)
                        break;
                }
            }
        }

        mpLocalMapper->InsertKeyFrame(pKF);

        mpLocalMapper->SetNotStop(false);

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;
    }

    void Tracking::SearchLocalPoints()
    {
        // Do not search map points already matched
        for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
            {
                if(pMP->isBad())
                {
                    *vit = static_cast<MapPoint*>(NULL);
                }
                else
                {
                    pMP->IncreaseVisible();
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    pMP->mbTrackInView = false;
                }
            }
        }

        int nToMatch=0;

        // Project points in frame and check its visibility
        for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if(pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            if(mCurrentFrame.isInFrustum(pMP,0.5))
            {
                pMP->IncreaseVisible();
                nToMatch++;
            }
        }

        if(nToMatch>0)
        {
            ORBmatcher matcher(0.8);
            int th = 1;
            if(mSensor==System::RGBD)
                th=3;
            // If the camera has been relocalised recently, perform a coarser search
            if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
                th=5;
            matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
        }
    }

    void Tracking::UpdateLocalMap()
    {
        // This is for visualization
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    void Tracking::UpdateLocalPoints()
    {
        mvpLocalMapPoints.clear();

        for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
        {
            KeyFrame* pKF = *itKF;
            const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

            for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
            {
                MapPoint* pMP = *itMP;
                if(!pMP)
                    continue;
                if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                    continue;
                if(!pMP->isBad())
                {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
            }
        }
    }


    void Tracking::UpdateLocalKeyFrames()
    {
        // Each map point vote for the keyframes in which it has been observed
        map<KeyFrame*,int> keyframeCounter;
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                    for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }

        if(keyframeCounter.empty())
            return;

        int max=0;
        KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
        {
            KeyFrame* pKF = it->first;

            if(pKF->isBad())
                continue;

            if(it->second>max)
            {
                max=it->second;
                pKFmax=pKF;
            }

            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }


        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
        {
            // Limit the number of keyframes
            if(mvpLocalKeyFrames.size()>80)
                break;

            KeyFrame* pKF = *itKF;

            const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

            for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
            {
                KeyFrame* pNeighKF = *itNeighKF;
                if(!pNeighKF->isBad())
                {
                    if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            const set<KeyFrame*> spChilds = pKF->GetChilds();
            for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
            {
                KeyFrame* pChildKF = *sit;
                if(!pChildKF->isBad())
                {
                    if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            KeyFrame* pParent = pKF->GetParent();
            if(pParent)
            {
                if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }

        }

        if(pKFmax)
        {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }

    bool Tracking::Relocalization()
    {
        // Compute Bag of Words Vector
        mCurrentFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

        if(vpCandidateKFs.empty())
            return false;

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75,true);

        vector<PnPsolver*> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint*> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates=0;

        for(int i=0; i<nKFs; i++)
        {
            KeyFrame* pKF = vpCandidateKFs[i];
            if(pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
                if(nmatches<15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9,true);

        while(nCandidates>0 && !bMatch)
        {
            for(int i=0; i<nKFs; i++)
            {
                if(vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver* pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if(bNoMore)
                {
                    vbDiscarded[i]=true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if(!Tcw.empty())
                {
                    Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint*> sFound;

                    const int np = vbInliers.size();

                    for(int j=0; j<np; j++)
                    {
                        if(vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j]=NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if(nGood<10)
                        continue;

                    for(int io =0; io<mCurrentFrame.N; io++)
                        if(mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    if(nGood<50)
                    {
                        int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                        if(nadditional+nGood>=50)
                        {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if(nGood>30 && nGood<50)
                            {
                                sFound.clear();
                                for(int ip =0; ip<mCurrentFrame.N; ip++)
                                    if(mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                                // Final optimization
                                if(nGood+nadditional>=50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for(int io =0; io<mCurrentFrame.N; io++)
                                        if(mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io]=NULL;
                                }
                            }
                        }
                    }


                    // If the pose is supported by enough inliers stop ransacs and continue
                    if(nGood>=50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if(!bMatch)
        {
            return false;
        }
        else
        {
            mnLastRelocFrameId = mCurrentFrame.mnId;
            return true;
        }

    }

    void Tracking::Reset()
    {

        cout << "System Reseting" << endl;
        if(mpViewer)
        {
            mpViewer->RequestStop();
            while(!mpViewer->isStopped())
                usleep(3000);
        }

        // Reset Local Mapping
        cout << "Reseting Local Mapper...";
        mpLocalMapper->RequestReset();
        cout << " done" << endl;

        // Reset Loop Closing
        cout << "Reseting Loop Closing...";
        mpLoopClosing->RequestReset();
        cout << " done" << endl;

        // Clear BoW Database
        cout << "Reseting Database...";
        mpKeyFrameDB->clear();
        cout << " done" << endl;

        // Clear Map (this erase MapPoints and KeyFrames)
        mpMap->clear();

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        if(mpInitializer)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
        }

        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();

        if(mpViewer)
            mpViewer->Release();
    }

    void Tracking::ChangeCalibration(const string &strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if(k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        Frame::mbInitialComputations = true;
    }

    void Tracking::InformOnlyTracking(const bool &flag)
    {
        mbOnlyTracking = flag;
    }


    inline void Tracking::synthInfoMat(const Frame *F, const int &kptIdx, const MapPoint *pMP,
                                       const arma::mat &H_meas, const float &res_u, const float &res_v,
                                       const arma::mat &H_proj, arma::mat &H_rw_row,
                                       const arma::rowvec & sig_uv, const arma::rowvec & sig_p)
    {

        int measSz = H_meas.n_rows;
        arma::mat Sigma_r(measSz, measSz), W_r(measSz, measSz);
        Sigma_r.eye();
        Sigma_r = Sigma_r * sig_uv(0,0);


        arma::mat  Sigma_p ={
                {sig_p(0,0) , 0.f, 0.f},
                {0.f,sig_p(0,1) , 0.f },
                {0.f, 0.f, sig_p(0,2) }
        };
        Sigma_p = Sigma_p * H_proj.t();
        Sigma_p = H_proj * Sigma_p;

        Sigma_r = Sigma_p + Sigma_r;

        bool chol_state = arma::chol(W_r, Sigma_r,"lower");
        if(chol_state){
//            LOG_S(INFO) << "sIGMA_R:" << Sigma_r;
//            LOG_S(INFO) << "W_R:" << W_r;
            H_rw_row = arma::inv(W_r) * H_meas;
        }
        else{
            // do nothing
            std::cout << "chol failed!" << std::endl;
//                                std::cout << "oct level =" << kpUn.octave << "; invSigma2 = " << invSigma2 << std::endl;
        }

//        if (F != NULL && kptIdx >= 0 && kptIdx < F->mvKeysUn.size())
//        {
//            //
//            float Sigma2 = F->mvLevelSigma2[F->mvKeysUn[kptIdx].octave];
//            Sigma_r = Sigma_r * Sigma2;
//
////            std::cout << Sigma_r ;
////            std::cout << "sigma2:" << Sigma2 << std::endl;
//
//
//        }

//        // cholesky decomp of diagonal-block scaling matrix W
//        if (arma::chol(W_r, Sigma_r, "lower") == true)
//        {
//            // scale the meas. Jacobian with the scaling block W_r
//            H_rw = arma::inv(W_r) * H_meas;
////            std::cout << H_rw;
//        }
//        else{
//            // do nothing
//            std::cout << "chol failed!" << std::endl;
////                                std::cout << "oct level =" << kpUn.octave << "; invSigma2 = " << invSigma2 << std::endl;
//        }
//
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


    inline void Tracking::computerSigma(const Frame *F, const int &kptIdx, const MapPoint *pMP,
                                        const arma::mat &H_meas, const float &res_u, const float &res_v,
                                        const arma::mat &H_proj, arma::mat &H_rw,
                                        arma::rowvec & sig_uv, arma::rowvec & sig_p,
                                        const cv::Mat & score_3d_homo,
                                        const cv::Mat & img_grad_current)
    {

        sig_uv = {1.f, 1.f};
//        cv::Mat img_grad_current;
//        computeGradImg(mImGray, img_grad_current);
        float entropy_l = localEntropy4uv(mImGray, res_u, res_v);
        float grad_squ_l = img_grad_current.at<uchar>(int(res_v), int(res_u));
        float score_tmp = uvScore_uni(entropy_l, grad_squ_l);
        sig_uv = {1.f + score_tmp/150.f, 1.f + score_tmp/150.f};

        if (F != NULL && kptIdx >= 0 && kptIdx < F->mvKeysUn.size())
        {
            float level_factor = F->mvLevelSigma2[F->mvKeysUn[kptIdx].octave];
            sig_uv = sig_uv * level_factor;
        }
//        LOG_S(INFO) << "SIGUV:"<< sig_uv;

        sig_p = {1.f, 1.f, 1.f};

        cv::Mat score_proj(4,1, CV_32F);
        score_proj = F->mTcw * score_3d_homo;

        sig_p = {score_proj.at<float>(0,0), score_proj.at<float>(1,0), score_proj.at<float>(2,0)};


//        int measSz = H_meas.n_rows;
//        arma::mat Sigma_r(measSz, measSz), W_r(measSz, measSz);
//        Sigma_r.eye();
//
//        if (F != NULL && kptIdx >= 0 && kptIdx < F->mvKeysUn.size())
//        {
//            //
//            float Sigma2 = F->mvLevelSigma2[F->mvKeysUn[kptIdx].octave];
//            Sigma_r = Sigma_r * Sigma2;
//
////            std::cout << Sigma_r ;
////            std::cout << "sigma2:" << Sigma2 << std::endl;
//
//
//        }
//
//        // cholesky decomp of diagonal-block scaling matrix W
//        if (arma::chol(W_r, Sigma_r, "lower") == true)
//        {
//            // scale the meas. Jacobian with the scaling block W_r
//            H_rw = arma::inv(W_r) * H_meas;
////            std::cout << H_rw;
//        }
//        else{
//            // do nothing
//            std::cout << "chol failed!" << std::endl;
////                                std::cout << "oct level =" << kpUn.octave << "; invSigma2 = " << invSigma2 << std::endl;
//        }

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

    inline void Tracking::drawPointsWrap(vector<neodraw> & neodraw_inframe, int matches0, float  score0){
        cv::Mat img_out;
        cv::Mat img_in;
//    mImGray.copyTo(img_in);
        cv::cvtColor(mImGray, img_in, CV_GRAY2RGB);

//    cv::drawKeypoints(mImGray,
//                      mCurrentFrame.mvKeys,
//                      out_img,
//                      cv::Scalar(0,0,255),
//                      cv::DrawMatchesFlags::DEFAULT);

        for(vector<neodraw>::iterator iter = neodraw_inframe.begin(); iter!= neodraw_inframe.end(); iter++){
            double color = (-iter->score )  * 2;
            cv::Vec3b color_BRG;
            convert_to_rainbow(color,color_BRG);
//        LOG_S(INFO) << "SCORE SINGLE" << color;
            cv::circle(img_in, cv::Point(iter->position[0],iter->position[1]), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));

            // 在最后一次迭代中， 加入上限、下限的颜色
            if(iter == neodraw_inframe.begin()){
                convert_to_rainbow(0, color_BRG);
                cv::circle(img_in, cv::Point(10,10), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));

                convert_to_rainbow(255, color_BRG);
                cv::circle(img_in, cv::Point(20,10), 6, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]));
            }

        }


//    cv::imshow("key_in",img_in);
        ostringstream file_name;
        file_name << "/home/da/active/key_dir/frame" << mCurrentFrame.mnId << ".png";
#ifdef NEO_FLIP_IMG
        flip(img_in,img_out,-1); //翻转图片
#else
        img_out = img_in.clone();
#endif
        ostringstream text;
        text << "Points:" << matches0 << ",score:" << score0;
        cv::Vec3b color_BRG;
        convert_to_rainbow(255, color_BRG);
        cv::putText(img_out, text.str(), cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX,1.5, cv::Scalar(color_BRG[0],color_BRG[1],color_BRG[2]), 2);
        cv::imwrite(file_name.str(), img_out);


    }






} //namespace ORB_SLAM
