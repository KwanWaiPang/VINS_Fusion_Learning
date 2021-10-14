/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

// ################
// #include "../time_surface/TimeSurface.h"
// #include "../time_surface/TicToc.h"
#include "../arc_star/acd/arc_star_detector.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <std_msgs/Float32.h>
#include <glog/logging.h>
#include <thread>
#include "opencv2/core.hpp"

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <deque>
#include <mutex>
#include <Eigen/Eigen>
// ################

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

// ############################gwphku
using EventQueue = std::deque<dvs_msgs::Event>;//定义的event队列。deque就先理解为高级的vector

class FeatureTracker
{
public:
    FeatureTracker();
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    // ########################################################  gwphku  函数
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackEvent(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1, const dvs_msgs::EventArray::ConstPtr& event_msg);//const dvs_msgs::EventArray::ConstPtr& msg
  void goodevent_FeaturesToTrack(const dvs_msgs::EventArray::ConstPtr &event_msg, dvs_msgs::EventArray corner_msg,  vector<cv::Point2f>  n_pts, int need_num, int para, int MIN_DIST,  cv::Mat mask);
  void change_event2CVMat(const dvs_msgs::EventArray::ConstPtr &event_msg, dvs_msgs::EventArray cur_event_msg, cv::Mat cur_img);
  void init(int width, int height);

// ########################################################                                     
    void setMask();
    void readIntrinsicParameter(const vector<string> &calib_file);
    void showUndistortion(const string &name);
    void rejectWithF();
    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage();
    bool inBorder(const cv::Point2f &pt);

    int row, col;
    cv::Mat imTrack;
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> predict_pts;
    vector<cv::Point2f> predict_pts_debug;

// ########################################gwphku
    dvs_msgs::EventArray cur_event_msg,prev_event_msg;//注意一下是const dvs_msgs::EventArray::ConstPtr 还是const dvs_msgs::EventArray
    dvs_msgs::EventArray corner_msg; 

//下面是用于把event转换到cv::mat的
cv::Size sensor_size_;
 bool bSensorInitialized_;//对pEventQueueMat_的大小进行初始化

//#####################################################
    
    //成像平面的特征点
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;//上一帧的特征点，这一帧的特征点（左目），这一帧右目的特征点（双目的情况）
    //上一帧的点通过光流得到这一帧的点

    //归一化平面上的特征点。（或者理解为liftProjective上的点），//从2Dimage plane到3D投影空间
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
//特征点在成像平面上的速度
    vector<cv::Point2f> pts_velocity, right_pts_velocity;//（pixel/时间）
    vector<int> ids, ids_right;//单独用两个数组来存储特征点的id，与上述数组的索引相对应
    vector<int> track_cnt;//每个特征点被连续追踪到的次数
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    map<int, cv::Point2f> prevLeftPtsMap;
    vector<camodocal::CameraPtr> m_camera;//定义camera模型的指针
    double cur_time;
    double prev_time;
    bool stereo_cam;
    int n_id;
    bool hasPrediction;
};