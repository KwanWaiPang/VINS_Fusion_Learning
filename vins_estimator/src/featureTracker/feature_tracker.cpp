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

#include "feature_tracker.h"
#include "../arc_star/acd/arc_star_detector.h"
#include <cv_bridge/cv_bridge.h>
// #include "../time_surface/TimeSurface.h"
// #include "../time_surface/TicToc.h"

bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

//剔除外点的函数
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])//如果正确执行。相当于状态矩阵是1的情况，就要；如果是0就不要
            v[j++] = v[i];//给它赋值，最后得到v，就可以实现把不要的点去除掉
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
    stereo_cam = 0;
    n_id = 0;
    hasPrediction = false;//初始化的时候，将进行预测设置为false

    // #####################gwphku
    bool bSensorInitialized_=false;//用于生成TS或者可以理解为将event转为frame
    // if(pEventQueueMat_)
    // pEventQueueMat_->clear();
  sensor_size_ = cv::Size(0,0);
}

//设置mask
void FeatureTracker::setMask()//按照track_cnt的大小从高到低排序
{
//将当前的特征点按照被连续跟踪的次数从高到低进行排序
//设置mask，使得角点提取的时候分布均匀

    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));//mask被设置为更输入的图像一样大的Mat

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);//具体的set mask的操作主要在这
            //画了一个圆，圆里面有MIN_DIST，就是特征点之间的最小距离。就是在这个最小范围内的区域，不再提取特征
        }
    }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

// #####################################gwphku
// cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);//改写
//源码分析可以参考：https://blog.csdn.net/jaych/article/details/51203841
// 也可能需要参考ESVO看如何对event做tracking了
//time surface的数据形式为dvs_msgs::EventArray
//arc*检测出来的corner_msg是dvs_msgs::EventArray 。两者是等效的。
//关键看看esvo中如何做tracking

//esvo中发布的time_surface是sensor_msgs/Image，里面处理的所有timesurface都是sensor_msgs::ImageConstPtr，基本的pose计算也是。
// 所以需要用函数把dvs_msgs::EventArray 
// void FeatureTracker::goodevent_FeaturesToTrack( InputArray image, OutputArray corners,
//                                      int maxCorners, double qualityLevel, double minDistance,
//                                      InputArray mask = noArray(), int blockSize = 3,
//                                      bool useHarrisDetector = false, double k = 0.04 )
//                                      {

//                                      }

void FeatureTracker::change_event2CVMat(const dvs_msgs::EventArray::ConstPtr &event_msg, dvs_msgs::EventArray cur_event_msg,  cv::Mat cur_img)
{

      ///顺便生成cur_event_msg
    cur_event_msg.header = event_msg->header;
    cur_event_msg.width = event_msg->width;
    cur_event_msg.height = event_msg->height; 
    for (const auto& e : event_msg->events) {
      cur_event_msg.events.push_back(e);
    }

    //首先通过函数eventsCallback将event_msg放入pEventQueueMat_类中
    //pEventQueueMat_类是一个指针类，里面包含了：长宽、队列的长度（queueLen_）和一个EventQueue（using EventQueue = std::deque<dvs_msgs::Event>;）
    //pEventQueueMat_类可用于将event转换为Mat
    // esvo_time_surface::TimeSurface::eventsCallback(event_msg);//实现将事件数据放到pEventQueueMat_类中，不可以使用eventsCallback，由于其是私有成员函数


  if(!bSensorInitialized_)//看一下是否初始化了，如果没有就进行初始化pEventQueueMat_
   FeatureTracker::init(event_msg->width, event_msg->height);//顺便对event 序列 Mat指针进行初始化

   cv_bridge::CvImage cv_image;
  if (event_msg->events.size() > 0)//如果有event
    {
      cv_image.header.stamp = event_msg->events[event_msg->events.size()/2].ts;
    }

    cv_image.encoding = "bgr8";
    cv_image.image = cv::Mat(event_msg->height, event_msg->width, CV_8UC3);
    cv_image.image = cv::Scalar(0,0,0);//分别设置为0，三个通道

     for (int i = 0; i < event_msg->events.size(); ++i)
      {
        const int x = event_msg->events[i].x;
        const int y = event_msg->events[i].y;

        cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (
            event_msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      }

      cur_img=cv_image.image;  //赋值给cur_img
  //已经得到放入了pEventQueueMat_的event了
//   cv::Mat event_image_map;
//   event_image_map = cv::Mat::zeros(sensor_size_, CV_64F);
//   // Loop through all pixel coordinates
//     for(int y=0; y<sensor_size_.height; ++y){
//         for(int x=0; x<sensor_size_.width; ++x){
//             event_image_map.at<double>(y,x)=
//         }
//     }
}


void FeatureTracker::init(int width, int height)
{
  sensor_size_ = cv::Size(width, height);
  bSensorInitialized_ = true;//只进行初始化一次
//   pEventQueueMat_.reset(new EventQueueMat(width, height, max_event_queue_length_));
//   ROS_INFO("Sensor size: (%d x %d)", sensor_size_.width, sensor_size_.height);
}


void FeatureTracker::goodevent_FeaturesToTrack(const dvs_msgs::EventArray::ConstPtr &event_msg, dvs_msgs::EventArray corner_msg,  vector<cv::Point2f>  n_pts, int maxCorners, int para, int MIN_DIST,  cv::Mat mask)
{

    const int n_event = event_msg->events.size();//进行event的计数，然后输出
    // ROS_INFO("n_event:%d",n_event);//已经验证成功读入event

     if (n_event == 0) {
          cout << "event is empty " << endl;
          }

    //EventArray相对于Event的区别主要是：
    // Header header
    // uint32 height         # image height, that is, number of rows
    // uint32 width          # image width, that is, number of columns
    // # an array of events
    // Event[] events

    // # A DVS event
    // uint16 x
    // uint16 y
    // time ts
    // bool polarity

    // dvs_msgs::EventArray corner_msg;//形参传入corner_msg
    corner_msg.header = event_msg->header;
    corner_msg.width = event_msg->width;
    corner_msg.height = event_msg->height;
    // ROS_INFO("width:%d,height:%d", corner_msg.width,corner_msg.height);//长宽也没有问题

    // acd::ArcStarDetector detector = acd::ArcStarDetector((int) corner_msg.width, (int) corner_msg.height );//定义一个detector作为变量
  acd::ArcStarDetector detector = acd::ArcStarDetector();
  int ncorners = 0;

  for (const auto& e : event_msg->events) {

        //  ROS_INFO("event_t:%d,event_x:%d,event_y:%d,,event_p:%d",e.ts.toSec(),e.x, e.y,e.polarity);

      // Unroll event array and detect Corners
    if (detector.isCorner(e.ts.toSec(), e.x, e.y, e.polarity)) {

        ROS_INFO("here!!!!!!!!!");
      corner_msg.events.push_back(e);
      
      //把xy弄到n_pts中
      //查看goodFeaturesToTrack函数的定义在imgproc文件的featureselect.cpp（https://blog.csdn.net/xdfyoga1/article/details/44175637）
    //  corners.push_back(Point2f((float)x, (float)y));
        n_pts.push_back(cv::Point2f((float)e.x, (float)e.y));
        // ncorners++;//计算corners数目，若大于一定值，则退出
        // if( maxCorners > 0 && (int)ncorners == maxCorners )  
        //     break;     
    }
  }
  ROS_INFO("the size of the n_pts:%d", n_pts.size());   
//   const int n_corner = corner_msg.events.size();//定义一个不能被改变的int，
}


// ###################################################

// 对应的定义一个track event的函数来跟踪event！！！！
// #################################gwphku
//进入event detection and tracking
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackEvent(double _cur_time, const cv::Mat &_img, const dvs_msgs::EventArray::ConstPtr &event_msg, const cv::Mat &_img1)
{//此处进来必须是event的dvs_msgs::EventArray::ConstPtr，这样才方便做extraction
    TicToc t_r;//没怎么用，应该是时间？
    cur_time = _cur_time;
    cur_img = _img;//直接image frame输入。
    //或者把所谓的cur_img全部改为event的image
    FeatureTracker::change_event2CVMat(event_msg, cur_event_msg,  cur_img);//已经获取到image了

    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1;

    cur_pts.clear();//当前帧的特征点清0，是一个vector<cv::Point2f>，可以通过Mat赋值。检测到的角点转到cv::Mat再转到这个上

    if (prev_pts.size() > 0)//若上一帧的特征点大于0则实行LK光流跟踪
    {
        TicToc t_o;
        vector<uchar> status;//定义一个向量，用于剔除外点
        vector<float> err;
        if(hasPrediction)//若已经进行了预测，则执行。（一开始是false的，只有当执行了setPrediction，才是true）
        {
            cur_pts = predict_pts;//若一开始有预测的点的话，就先用预测的点
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);//这是通过img来进行跟踪的
            
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;//看看可以成功跟踪到多少个点
            }
            if (succ_num < 10)//如果少于10个点，那么继续
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
               //里面的“3”是图像金字塔相关的参数。
               //maxLevel ：基于0的最大金字塔等级数;如果设置为0，则不使用金字塔（单级），如果设置为1，则使用两个级别，
               //依此类推;如果将金字塔传递给输入，那么算法将使用与金字塔一样多的级别，但不超过maxLevel。
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);//通过光流来计算。
            //prev_img, cur_img, prev_pts,三个都是前面给定的
            //所以应该得到的是cur_pts，status就是匹配的效果
            //cur_pts输出二维点的矢量（具有单精度浮点坐标），包含第二图像中输入特征的计算新位置;
            // 当传递OPTFLOW_USE_INITIAL_FLOW标志时，向量cur_pts必须与输入中的大小相同。（但此处没有输入，前面根后面有）
            //status输出状态向量（无符号字符）;如果找到相应特征的流，则向量的每个元素设置为1，否则设置为0。
            //err ：输出错误的矢量; 向量的每个元素都设置为相应特征的错误，错误度量的类型可以在flags参数中设置; 如果未找到流，则未定义错误（使用status参数查找此类情况）。

        // reverse check
        if(FLOW_BACK)//这是读入的参数，是否需要进行二次的光流检测，一般都选了1
        {//perform forward and backward optical flow to improve feature tracking accuracy
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            //通过映射回去，再次得到输出状态向量reverse_status，然后起到double check的作用最终得到status
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);//外点剔除，剔除无用的特征点后，还是得到prev_pts
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)////每个特征点被连续追踪到的次数
        n++;//上面已经追踪到了，把被追踪的次数++

    if (1)//每一帧都会执行的角点检测与跟踪
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();//先把mask设置了
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect event feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());//离最大的特征点，还有多少个点
        // if (n_max_cnt > 0)//如果还不满足最大的特征点的要求，则进行ARC*特征点检测
        // {
        //     if(mask.empty())
        //         cout << "mask is empty ??? " << endl;
        //     if (mask.type() != CV_8UC1)
        //         cout << "mask type wrong " << endl;

        //     FeatureTracker::goodevent_FeaturesToTrack(event_msg, corner_msg,n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);//定义角点检测
            
        //     // cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);//通过这个来提取特征。
        //     // //Shi-Tomasi角点检测法。对于event camera的话，换为event camera的特征提取方法即可。刚开始调试的时候，应该将图片以及event的角点同时输出观测
        //     // //通过设置一个mask，来实现被提取角点的较均匀分布
        //     // //维护一个最大数量，光流一直跟踪，若更丢后，再补齐对应数量
        //     // //输出的角点cv::OutputArray corners, n_pts
        //     // //MAX_CNT - cur_pts.size()是最大的角点数目。就相当于目前的角点已有了，然后检测n_pts个，+上现有的等于设置的最大的
        //     // //MIN_DIST，最小距离，小于此距离的点忽略
        //     // //cv::InputArray mask = noArray(), // mask=0的点忽略

        //     //n_pts（是一个vector<cv::Point2f>且为输出的角点cv::OutputArray corners）已经获取了
        //         //  //接下来需要将cur_event_msg转换为cur_img（cv::Mat ）
        //         // FeatureTracker::change_event2CVMat(event_msg,  cur_event_msg, cur_img);
        //         // //  FeatureTracker::change_event2CVMat(const dvs_msgs::EventArray::ConstPtr &event_msg, dvs_msgs::EventArray cur_event_msg, cv::Mat cur_img);
        //         // //   n_pts;//是一个vector<cv::Point2f>且为输出的角点cv::OutputArray corners
        // }
        // else
        //     n_pts.clear();//若点是够的，那么就清空
        // ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

         FeatureTracker::goodevent_FeaturesToTrack(event_msg, corner_msg,n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);//定义角点检测

        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);//把上面检测的n_pts push到cur_pts中
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }
        n_pts.clear();
        //printf("feature cnt after add %d\n", (int)ids.size());
    }

//通过函数undistortedPts以及camera的模型。进行去畸变。对单独的图像进行去畸变
//对于event camera，可能就是camera[2],对于额外的camera需要怎么定义要仔细看清楚
   ROS_INFO("before cur_un_pts");
    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);//对当前检测到的角点进行去畸变的处理，得到当前帧在归一化平面上的特征点
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);//特征点在成像平面上的速度。
    // 里面的东西都是根cur_pts（成像平面上的特征点有关，不需要处理吧）

    if(SHOW_TRACK)//读入参数是否show跟踪，如果是的话，就运行下面函数，将特征点显示出来！
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

//执行完后，将当前帧的一些数据变为上一帧
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;//将进行预测设置为false

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;//创建一个特征帧，最终要返回的
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];//特征点的id
        double x, y ,z;
        x = cur_un_pts[i].x;//特征点在归一化平面的x
        y = cur_un_pts[i].y;
        z = 1;//归一化平面
        double p_u, p_v;
        p_u = cur_pts[i].x;//特征点在图像上的位置
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;//特征点在成像平面上的速度
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;//归一化平面上的点，成像平面上的点，成像平面上点的速度
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);//把特征点的信息都加进去
    }

    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;//将特征帧返回
}
// ################################# 

//最主要的函数trackImage，输入当前帧，如何获取新的tracking的image
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;//没怎么用，应该是时间？
    cur_time = _cur_time;
    cur_img = _img;
    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1;
    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts.clear();//当前帧的特征点清哦

    if (prev_pts.size() > 0)//若上一帧的特征点大于0则实行
    {
        TicToc t_o;
        vector<uchar> status;//定义一个向量，用于剔除外点
        vector<float> err;
        if(hasPrediction)//若已经进行了预测，则执行。（一开始是false的，只有当执行了setPrediction，才是true）
        {
            cur_pts = predict_pts;//若一开始有预测的点的话，就先用预测的点
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;//看看可以成功跟踪到多少个点
            }
            if (succ_num < 10)//如果少于10个点，那么继续
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
               //里面的“3”是图像金字塔相关的参数。
               //maxLevel ：基于0的最大金字塔等级数;如果设置为0，则不使用金字塔（单级），如果设置为1，则使用两个级别，
               //依此类推;如果将金字塔传递给输入，那么算法将使用与金字塔一样多的级别，但不超过maxLevel。
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);//通过光流来计算。
            //prev_img, cur_img, prev_pts,三个都是前面给定的
            //所以应该得到的是cur_pts，status就是匹配的效果
            //cur_pts输出二维点的矢量（具有单精度浮点坐标），包含第二图像中输入特征的计算新位置;
            // 当传递OPTFLOW_USE_INITIAL_FLOW标志时，向量cur_pts必须与输入中的大小相同。（但此处没有输入，前面根后面有）
            //status输出状态向量（无符号字符）;如果找到相应特征的流，则向量的每个元素设置为1，否则设置为0。
            //err ：输出错误的矢量; 向量的每个元素都设置为相应特征的错误，错误度量的类型可以在flags参数中设置; 如果未找到流，则未定义错误（使用status参数查找此类情况）。

        // reverse check
        if(FLOW_BACK)//这是读入的参数，是否需要进行二次的光流检测，一般都选了1
        {//perform forward and backward optical flow to improve feature tracking accuracy
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            //通过映射回去，再次得到输出状态向量reverse_status，然后起到double check的作用最终得到status
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);//外点剔除，剔除无用的特征点后，还是得到prev_pts
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)////每个特征点被连续追踪到的次数
        n++;//上面已经追踪到了，把被追踪的次数++

    if (1)//每一帧都会执行的角点检测与跟踪
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();//先把mask设置了
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);//通过这个来提取特征。
            //Shi-Tomasi角点检测法。对于event camera的话，换为event camera的特征提取方法即可。刚开始调试的时候，应该将图片以及event的角点同时输出观测
            //通过设置一个mask，来实现被提取角点的较均匀分布
            //维护一个最大数量，光流一直跟踪，若更丢后，再补齐对应数量
            //输出的角点cv::OutputArray corners, n_pts

            //MAX_CNT - cur_pts.size()是最大的角点数目。就相当于目前的角点已有了，然后检测n_pts个，+上现有的等于设置的最大的
            //MIN_DIST，最小距离，小于此距离的点忽略
            //cv::InputArray mask = noArray(), // mask=0的点忽略
        }
        else
            n_pts.clear();//这个是如果点不够的时候，额外再检测的
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);//把上面检测的n_pts push到cur_pts中
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }
        //printf("feature cnt after add %d\n", (int)ids.size());
    }

//通过函数undistortedPts以及camera的模型。进行去畸变。对单独的图像进行去畸变
//对于event camera，可能就是camera[2],对于额外的camera需要怎么定义要仔细看清楚
    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);//对当前检测到的角点进行去畸变的处理，得到当前帧在归一化平面上的特征点
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);//特征点在成像平面上的速度

    if(!_img1.empty() && stereo_cam)//若img1不是空的（那就以及是双目啦，且是双目）这是进行双目的操作
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);//采用光流法进行角点的跟踪。
            //直接采用opencv中的Lucas-Kanade稀疏光流法进行特征点的跟踪
            //需要保证点的数据形式都是一致的

            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if(SHOW_TRACK)//读入参数是否show跟踪，如果是的话，就运行下面函数，将特征点显示出来！
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

//执行完后，将当前帧的一些数据变为上一帧
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;//将进行预测设置为false

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;//创建一个特征帧，最终要返回的
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];//特征点的id
        double x, y ,z;
        x = cur_un_pts[i].x;//特征点在归一化平面的x
        y = cur_un_pts[i].y;
        z = 1;//归一化平面
        double p_u, p_v;
        p_u = cur_pts[i].x;//特征点在图像上的位置
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;//特征点在成像平面上的速度
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;//归一化平面上的点，成像平面上的点，成像平面上点的速度
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);//把特征点的信息都加进去
    }

    if (!_img1.empty() && stereo_cam)//若是双目的话，把右边的camera也加进去
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }

    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;//将特征帧返回
}

void FeatureTracker::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);//这个应该是从成像中的uv转换到3D的点tmp_p
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

//读取camera的内参进而实现特征的tracking
void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)//输入的是校正的camera的路径
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);//那么就关注一下m_camera怎么使用
    }
    if (calib_file.size() == 2)//如果camera的文件是两个的话，就把双目的参数stereo_cam设置为1
        stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}


void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{//通过VIO估计器来设置预测的点
    hasPrediction = true;//只有当设置预测才会变为true
    predict_pts.clear();
    predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);//从空间中的点转到平面中的uv
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts.push_back(prev_pts[i]);
    }
}

//删除外点的接口；而外点的认定在外部实现（重投影误差过大）。此处这个函数并没有使用。只用函数reduceVector来剔除外点
void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++)
    {
        itSet = removePtsIds.find(ids[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}