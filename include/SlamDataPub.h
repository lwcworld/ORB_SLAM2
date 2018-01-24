/**
* Description: This file is part of parkingEnvSensing.
* Date: 2017-06-20
* Final Edit: 2017-06-20
*/

#ifndef DATAPUB_H
#define DATAPUB_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "LocalMapping.h"
#include "LoopClosing.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <std_msgs/Int8.h>

//#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>  


// for mapping
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>

#include "MapPoint.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <time.h>


#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class LocalMapping;
class LoopClosing;

class SlamDataPub
{
public:
   // SlamDataPub(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath, Map *pMap, LocalMapping *pLocalMapper, LoopClosing *pLoopCLoser);
SlamDataPub(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath, Map *pMap);


    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    void SetCurrentCameraPose(const cv::Mat &Tcw);

    void SetLoopClosing(LoopClosing *pLoopClosing);

    void SetTracker(Tracking *pTracker);
    
private:

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;
    Map* mpMap;
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopCloser;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
    
    std::mutex mMutexCamera;
    cv::Mat mCameraPose;
    
    //ros::Publisher path_pub_;
    ros::Publisher CamPose_pub_;
    ros::Publisher VehiclePose_pub_;
    ros::Publisher CamPath_pub_;
    ros::Publisher VehiclePath_pub_;
    ros::Publisher AllPointCloud_pub_;
    ros::Publisher RefPointCloud_pub_;
    ros::Publisher pub_pts_and_pose_; 
    ros::Publisher pub_all_kf_and_pts_;
    ros::Publisher orbSlamStatus_pub_;
    ros::Subscriber sub_joy_;
    
    image_transport::Publisher DrawFrame_pub_;
    tf::TransformBroadcaster Vehicle2Ground_broadcaster_;
    
    //tf::TransformBroadcaster broadcaster;
    ros::NodeHandle nh;
    
    bool mbGetNewCamPose;
    
    void TrackingDataPub();   
    void GetCurrentROSCameraMatrix(geometry_msgs::PoseStamped &cam_pose);
    void GetCurrentROSVehicleMatrix(geometry_msgs::PoseStamped &vehicle_pose);
    void GetCurrentROSTrajectories(nav_msgs::Path &cam_path, nav_msgs::Path &vehicle_path);    
    
    void PointCloudPub();
    void GetCurrentROSAllPointCloud( sensor_msgs::PointCloud2 &all_point_cloud, sensor_msgs::PointCloud2 &ref_point_cloud);
    
    void DrawFramePub();

    void MapPup();
    
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    
    Eigen::Matrix3f mInitCam2Ground_R;
    Eigen::Vector3f mInitCam2Ground_t;
    Eigen::Matrix4f mTrans_cam2ground;	//calibration
    
    Eigen::Matrix3f mCam2Vehicle_R;	// camera is stationary to vehicle
    Eigen::Vector3f mCam2Vehicle_t;
    Eigen::Matrix4f mTrans_cam2vehicle;

    Eigen::Matrix4f mCam2GroundNow_T;   
    Eigen::Matrix4f mVehicle2GroundNow_T;  

    int all_pts_pub_gap;
    int pub_count; 
    bool pub_all_pts;
    sensor_msgs::Joy joy_msg;
    
};

}


#endif // DATAPUB_H
	

