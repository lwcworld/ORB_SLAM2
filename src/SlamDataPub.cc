/**
* Description: This file is part of parkingEnvSensing.
* Date: 2017-06-20
* Final Edit: 2017-06-20
*/

#include "SlamDataPub.h"

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 


#include <opencv2/core/eigen.hpp>

#include <mutex>

#define publisherRate 10

#define TFNAME "odom"

namespace ORB_SLAM2
{

//SlamDataPub::SlamDataPub(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Map *pMap, LocalMapping *pLocalMapper, LoopClosing *pLoopCLoser):
SlamDataPub::SlamDataPub(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Map *pMap): mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking), mpMap(pMap),
  //  mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking), mpMap(pMap), mpLocalMapper(pLocalMapper), mpLoopCloser(pLoopCLoser),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }
    
    // camera under ground      
    //smInitCam2Ground_R << 1,0,0,0,0,1,0,-1,0;  // camera coordinate represented in ground coordinate system
    mInitCam2Ground_R <<  0,0,1,
			  -1,0,0,
			  0,-1,0;  // camera coordinate represented in ground coordinate system
   
    mInitCam2Ground_t.setZero();     
    mTrans_cam2ground.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    mTrans_cam2ground.block<3,3>(0,0) = mInitCam2Ground_R;
    mTrans_cam2ground.block<3,1>(0,3) = mInitCam2Ground_t;  //< block_rows, block_cols >(pos_row, pos_col)

    // camera under vehicle, 
    mCam2Vehicle_R << 0,0,1,-1,0,0,0,-1,0;  // camera coordinate represented in vehicle coordinate system
    //mCam2Vehicle_R << 0,0,1,1,0,0,0,-1,0;  // camera coordinate represented in vehicle coordinate system
    mCam2Vehicle_t.setZero();
    mTrans_cam2vehicle.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    mTrans_cam2vehicle.block<3,3>(0,0) = mCam2Vehicle_R;
    mTrans_cam2vehicle.block<3,1>(0,3) = mCam2Vehicle_t;  //< block_rows, block_cols >(pos_row, pos_col)

	all_pts_pub_gap = 0;
	pub_count = 0;
	pub_all_pts = false;
	pub_count = 0;
    
}

void SlamDataPub::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopCloser=pLoopClosing;
}

void SlamDataPub::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void SlamDataPub::MapPup()
{
	int frame_id = 0;	
	
    while(1)
    {
		if (all_pts_pub_gap > 0 && pub_count >= all_pts_pub_gap) {
			pub_all_pts = true;
			pub_count = 0;
		}
	if(mpTracker->mCurrentFrame.is_keyframe)
		//cout << "Is KeyFrame:" << mpTracker->mCurrentFrame.is_keyframe << "pubCnt " << pub_count << endl;
	if (pub_all_pts || mpLoopCloser->loop_detected || mpTracker->loop_detected) {
		pub_all_pts = mpTracker->loop_detected = mpLoopCloser->loop_detected = false;
		cout << "Pub All pts" << endl;
		geometry_msgs::PoseArray kf_pt_array;
		vector<ORB_SLAM2::KeyFrame*> key_frames = mpMap->GetAllKeyFrames();
		//! placeholder for number of keyframes
		kf_pt_array.poses.push_back(geometry_msgs::Pose());
		sort(key_frames.begin(), key_frames.end(), ORB_SLAM2::KeyFrame::lId);
		unsigned int n_kf = 0;
		unsigned int n_pts_id = 0;
		for (auto key_frame : key_frames) {
			// pKF->SetPose(pKF->GetPose()*Two);

			if (!key_frame || key_frame->isBad()) {
				continue;
			}

			cv::Mat R = key_frame->GetRotation().t();
			vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
			cv::Mat twc = key_frame->GetCameraCenter();
			geometry_msgs::Pose kf_pose;

			kf_pose.position.x = twc.at<float>(0);
			kf_pose.position.y = twc.at<float>(1);
			kf_pose.position.z = twc.at<float>(2);
			kf_pose.orientation.x = q[0];
			kf_pose.orientation.y = q[1];
			kf_pose.orientation.z = q[2];
			kf_pose.orientation.w = q[3];
			kf_pt_array.poses.push_back(kf_pose);

			n_pts_id = kf_pt_array.poses.size();
			//! placeholder for number of points
			kf_pt_array.poses.push_back(geometry_msgs::Pose());
			std::set<ORB_SLAM2::MapPoint*> map_points = key_frame->GetMapPoints();
			unsigned int n_pts = 0;
			for (auto map_pt : map_points) {
				if (!map_pt || map_pt->isBad()) {
					//printf("Point %d is bad\n", pt_id);
					continue;
				}
				cv::Mat pt_pose = map_pt->GetWorldPos();
				if (pt_pose.empty()) {
					//printf("World position for point %d is empty\n", pt_id);
					continue;
				}
				geometry_msgs::Pose curr_pt;
				//printf("wp size: %d, %d\n", wp.rows, wp.cols);
				//pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
				curr_pt.position.x = pt_pose.at<float>(0);
				curr_pt.position.y = pt_pose.at<float>(1);
				curr_pt.position.z = pt_pose.at<float>(2);
				kf_pt_array.poses.push_back(curr_pt);
				++n_pts;
			}
			kf_pt_array.poses[n_pts_id].position.x = (double)n_pts;
			kf_pt_array.poses[n_pts_id].position.y = (double)n_pts;
			kf_pt_array.poses[n_pts_id].position.z = (double)n_pts;
			++n_kf;
		}
		kf_pt_array.poses[0].position.x = (double)n_kf;
		kf_pt_array.poses[0].position.y = (double)n_kf;
		kf_pt_array.poses[0].position.z = (double)n_kf;
		kf_pt_array.header.frame_id = "1";
		kf_pt_array.header.seq = frame_id + 1;
		printf("Publishing data for %u keyfranmes\n", n_kf);
		pub_all_kf_and_pts_.publish(kf_pt_array);
	}
	else if (mpTracker->mCurrentFrame.is_keyframe) {
		++pub_count;
		mpTracker->mCurrentFrame.is_keyframe = false;
		ORB_SLAM2::KeyFrame* pKF = mpTracker->mCurrentFrame.mpReferenceKF;

		cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

		// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
		//while (pKF->isBad())
		//{
		//	Trw = Trw*pKF->mTcp;
		//	pKF = pKF->GetParent();
		//}

		vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		Trw = Trw*pKF->GetPose()*Two;
		cv::Mat lit = mpTracker->mlRelativeFramePoses.back();
		cv::Mat Tcw = lit*Trw;
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
		//geometry_msgs::Pose camera_pose;
		//std::vector<ORB_SLAM2::MapPoint*> map_points = mpMap->GetAllMapPoints();
		std::vector<ORB_SLAM2::MapPoint*> map_points = mpSystem->GetTrackedMapPoints();
		int n_map_pts = map_points.size();

		//printf("n_map_pts: %d\n", n_map_pts);

		//pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		geometry_msgs::PoseArray pt_array;
		//pt_array.poses.resize(n_map_pts + 1);

		geometry_msgs::Pose camera_pose;

		camera_pose.position.x = twc.at<float>(0);
		camera_pose.position.y = twc.at<float>(1);
		camera_pose.position.z = twc.at<float>(2);

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];

		pt_array.poses.push_back(camera_pose);

		//printf("Done getting camera pose\n");

		for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){

			if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
				//printf("Point %d is bad\n", pt_id);
				continue;
			}
			cv::Mat wp = map_points[pt_id - 1]->GetWorldPos();

			if (wp.empty()) {
				//printf("World position for point %d is empty\n", pt_id);
				continue;
			}
			geometry_msgs::Pose curr_pt;
			//printf("wp size: %d, %d\n", wp.rows, wp.cols);
			//pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
			curr_pt.position.x = wp.at<float>(0);
			curr_pt.position.y = wp.at<float>(1);
			curr_pt.position.z = wp.at<float>(2);
			pt_array.poses.push_back(curr_pt);
			//printf("Done getting map point %d\n", pt_id);
		}
		/*sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*pcl_cloud, ros_cloud);
		ros_cloud.header.frame_id = "1";
		ros_cloud.header.seq = ni;

		//printf("valid map pts: %lu\n", pt_array.poses.size()-1);

		printf("ros_cloud size: %d x %d\n", ros_cloud.height, ros_cloud.width);
		spub_cloud.publish(ros_cloud);*/
		pt_array.header.frame_id = "1";
		pt_array.header.seq = frame_id + 1;
		pub_pts_and_pose_.publish(pt_array);
		//pub_kf.publish(camera_pose);
	}
	if(CheckFinish())
	      break;  
	  usleep(mT*1000/2*publisherRate); 
    }
    
}

void SlamDataPub::TrackingDataPub()
{
    geometry_msgs::PoseStamped camPose2Ground;  
    geometry_msgs::PoseStamped vehiclePose2Ground;  
    nav_msgs::Path cameraPath, vehiclePath;
    while(1)
    { 
	  if(mbGetNewCamPose)
	  {
	      GetCurrentROSCameraMatrix(camPose2Ground);
	      GetCurrentROSVehicleMatrix(vehiclePose2Ground);
	      GetCurrentROSTrajectories(cameraPath, vehiclePath);
	      CamPose_pub_.publish(camPose2Ground);  
	      VehiclePose_pub_.publish(vehiclePose2Ground);
	      CamPath_pub_.publish(cameraPath);   // KeyFrames
	      VehiclePath_pub_.publish(vehiclePath);
	      
	      float tf_q_x = vehiclePose2Ground.pose.orientation.x;
	      float tf_q_y = vehiclePose2Ground.pose.orientation.y;
	      float tf_q_z = vehiclePose2Ground.pose.orientation.z;
	      float tf_q_w = vehiclePose2Ground.pose.orientation.w;
	      float tf_x = vehiclePose2Ground.pose.position.x;
	      float tf_y = vehiclePose2Ground.pose.position.y;
	      float tf_z = vehiclePose2Ground.pose.position.z;
	      
	      Vehicle2Ground_broadcaster_.sendTransform(
		  tf::StampedTransform(
		  tf::Transform(tf::Quaternion(tf_q_x,tf_q_y,tf_q_z,tf_q_w), tf::Vector3(tf_x, tf_y, tf_z)),
		  ros::Time::now(),TFNAME, "vehicle"));   
	    
	  }
	  if(CheckFinish())
	      break;  
	   usleep(1*1000); 
    }
   
}

void SlamDataPub::PointCloudPub()
{
    sensor_msgs::PointCloud2 allMapPoints;
    sensor_msgs::PointCloud2 referenceMapPoints;
    while(1)
    {
// 	  if(mbGetNewCamPose)
// 	  {
	      GetCurrentROSAllPointCloud(allMapPoints, referenceMapPoints);
	      AllPointCloud_pub_.publish(allMapPoints);
	      RefPointCloud_pub_.publish(referenceMapPoints);
// 	  }
	  if(CheckFinish())
	      break;  
	  usleep(mT*1000/2*publisherRate); 
    }
    
}

void SlamDataPub::DrawFramePub()
{
    cv_bridge::CvImage cvi;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.header.stamp = ros::Time::now();
    while(1)
    {
// 	if(mbGetNewCamPose)
// 	{  
	cv::Mat img = mpFrameDrawer->DrawFrame();
	//cv::imshow("Current Frame",img);
	//cv::waitKey(mT/2);
	cvi.image = img;
	sensor_msgs::Image im;
	cvi.toImageMsg(im);
	DrawFrame_pub_.publish(im);
//	}
	if(CheckFinish())
	    break;  
	usleep(mT*1000*publisherRate); 
    }
 
}

void SlamDataPub::Run()
{
    mbFinished = false;
    mbStopped = false;

    CamPose_pub_ = nh.advertise<geometry_msgs::PoseStamped >("orb/camera_pose",1);
    VehiclePose_pub_ = nh.advertise<geometry_msgs::PoseStamped >("orb/vehicle_pose",1);
    CamPath_pub_ = nh.advertise<nav_msgs::Path>("orb/camera_path",1);
    VehiclePath_pub_ = nh.advertise<nav_msgs::Path>("orb/vehicle_path",1);
    AllPointCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("orb/point_cloud_all",1);
    RefPointCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("orb/point_cloud_ref",1);
    pub_pts_and_pose_ = nh.advertise<geometry_msgs::PoseArray>("orb/pts_and_pose", 1000);
    pub_all_kf_and_pts_ = nh.advertise<geometry_msgs::PoseArray>("orb/all_kf_and_pts", 1000);
    
    image_transport::ImageTransport it_(nh);
    DrawFrame_pub_ = it_.advertise("/orb/frame_now", 1);
       
    thread threadCamPosePub(&SlamDataPub::TrackingDataPub,this);   
    thread threadPointCloudPub(&SlamDataPub::PointCloudPub,this);  
    thread threadDrawFramePub(&SlamDataPub::DrawFramePub,this); 
	thread threadMapUp(&SlamDataPub::MapPup,this); 
    
    threadCamPosePub.join(); 
    threadPointCloudPub.join();

    SetFinish();
}

void SlamDataPub::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool SlamDataPub::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void SlamDataPub::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool SlamDataPub::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void SlamDataPub::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool SlamDataPub::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool SlamDataPub::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void SlamDataPub::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void SlamDataPub::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbGetNewCamPose = true;
}

void SlamDataPub::GetCurrentROSCameraMatrix(geometry_msgs::PoseStamped &cam_pose)
{
      if(!mCameraPose.empty())
      {
	  Eigen::Matrix4f cam_pose2firstcam;
 	  Eigen::Matrix4f cam_pose2ground;
 	  {
	      unique_lock<mutex> lock(mMutexCamera);
	      cv2eigen(mCameraPose.inv(),cam_pose2firstcam);
	      cam_pose2ground = mTrans_cam2ground * cam_pose2firstcam;
	      {
		  mCam2GroundNow_T = cam_pose2ground;
	      }
	  }

 	  cam_pose.pose.position.x = cam_pose2ground(0,3);
	  cam_pose.pose.position.y  = cam_pose2ground(1,3);
	  cam_pose.pose.position.z  = cam_pose2ground(2,3);
	   
	  Eigen::Matrix3f Rwc = cam_pose2ground.block<3,3>(0,0);
	  Eigen::Quaternionf q(Rwc);
	  cam_pose.pose.orientation.x = q.x();
	  cam_pose.pose.orientation.y = q.y();
	  cam_pose.pose.orientation.z = q.z();
	  cam_pose.pose.orientation.w = q.w();
	  
	  cam_pose.header.frame_id = "2ground";
	  cam_pose.header.stamp = ros::Time::now();  
	  
	  mbGetNewCamPose = false;
      }
      
}

void SlamDataPub::GetCurrentROSVehicleMatrix(geometry_msgs::PoseStamped &vehicle_pose)
{
      if(!mCameraPose.empty())
      {
	Eigen::Matrix4f vehicle_pose2ground;
	{
	    vehicle_pose2ground = mCam2GroundNow_T * mTrans_cam2vehicle.inverse();
	}
	{
	    mVehicle2GroundNow_T = vehicle_pose2ground;
	}
	vehicle_pose.pose.position.x = vehicle_pose2ground(0,3);
	vehicle_pose.pose.position.y  = vehicle_pose2ground(1,3);
	vehicle_pose.pose.position.z  = vehicle_pose2ground(2,3);
	  
	Eigen::Matrix3f Rwc = vehicle_pose2ground.block<3,3>(0,0);
	Eigen::Quaternionf q(Rwc);
	vehicle_pose.pose.orientation.x = q.x();
	vehicle_pose.pose.orientation.y = q.y();
	vehicle_pose.pose.orientation.z = q.z();
	vehicle_pose.pose.orientation.w = q.w();
	
	vehicle_pose.header.frame_id = TFNAME;
	vehicle_pose.header.stamp = ros::Time::now();  	
      }
}

void SlamDataPub::GetCurrentROSTrajectories(nav_msgs::Path &cam_path, nav_msgs::Path &vehicle_path)
{
      if(!mCameraPose.empty())
      {  
	  nav_msgs::Path cam_path_temp;
	  nav_msgs::Path vehicle_path_temp;
	  
	  geometry_msgs::PoseStamped cam_pose;
	   geometry_msgs::PoseStamped vehicle_pose;
	  
	  vector<cv::Mat> currentTrajectory;	
	  mpSystem->GetCurrentTrajectory(currentTrajectory);
	  
	  Eigen::Matrix4f cam_pose_temp;
	  
	  for(auto mt:currentTrajectory) // no need to inverse
	  {
	      cv2eigen(mt,cam_pose_temp);
	      
	      Eigen::Matrix4f cam_pose2ground = mTrans_cam2ground * cam_pose_temp;
	      Eigen::Matrix4f vehicle_pose2ground = cam_pose2ground * mTrans_cam2vehicle.inverse();
	      
	      cam_pose.pose.position.x = cam_pose2ground(0,3);
	      cam_pose.pose.position.y = cam_pose2ground(1,3);
	      cam_pose.pose.position.z = cam_pose2ground(2,3);
	      Eigen::Matrix3f Rwc = cam_pose2ground.block<3,3>(0,0);
	      Eigen::Quaternionf q(Rwc);	      
	      cam_pose.pose.orientation.x = q.x();
	      cam_pose.pose.orientation.y = q.y();
	      cam_pose.pose.orientation.z = q.z();
	      cam_pose.pose.orientation.w = q.w();
	      
	      vehicle_pose.pose.position.x = vehicle_pose2ground(0,3);
	      vehicle_pose.pose.position.y = vehicle_pose2ground(1,3);
	      vehicle_pose.pose.position.z = vehicle_pose2ground(2,3);
	      Eigen::Matrix3f Rwc2 = vehicle_pose2ground.block<3,3>(0,0);
	      Eigen::Quaternionf q2(Rwc2);	      
	      vehicle_pose.pose.orientation.x = q2.x();
	      vehicle_pose.pose.orientation.y = q2.y();
	      vehicle_pose.pose.orientation.z = q2.z();
	      vehicle_pose.pose.orientation.w = q2.w();
	      
	      vehicle_path_temp.poses.push_back(vehicle_pose);
	      cam_path_temp.poses.push_back(cam_pose);	     
	  }
 	  cam_path_temp.header.frame_id = TFNAME;
 	  cam_path_temp.header.stamp = ros::Time::now();   
	  vehicle_path_temp.header.frame_id = TFNAME;
	  vehicle_path_temp.header.stamp = ros::Time::now(); 
	  
	  cam_path = cam_path_temp;
	  vehicle_path = vehicle_path_temp;
      }
}

void SlamDataPub::GetCurrentROSAllPointCloud( sensor_msgs::PointCloud2 &all_point_cloud, sensor_msgs::PointCloud2 &ref_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_all( new pcl::PointCloud<pcl::PointXYZRGBA> );  
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ref( new pcl::PointCloud<pcl::PointXYZRGBA> );     
    
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();
    
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;
	
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        pcl::PointXYZRGBA p1;
	Eigen::Vector4f p1_temp, p1_temp_t;
	p1_temp(0) = pos.at<float>(0);
	p1_temp(1) = pos.at<float>(1);
	p1_temp(2) = pos.at<float>(2);
	p1_temp(3) = 1; 
	p1_temp_t = mTrans_cam2ground * p1_temp;	
	p1.x = p1_temp_t(0);
	p1.y = p1_temp_t(1);
	p1.z = p1_temp_t(2);
	p1.b = 255;
	p1.g = 255;
	p1.r = 255;
	p1.a = 255;
	cloud_all->points.push_back( p1 );
    }
    pcl::PCLPointCloud2 pcl_pc1;
    pcl::toPCLPointCloud2(*cloud_all, pcl_pc1);    // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc1, all_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    all_point_cloud.header.frame_id = TFNAME;  
    all_point_cloud.header.stamp = ros::Time::now();   
  
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        pcl::PointXYZRGBA p2;
	Eigen::Vector4f p2_temp, p2_temp_t;
	p2_temp(0) = pos.at<float>(0);
	p2_temp(1) = pos.at<float>(1);
	p2_temp(2) = pos.at<float>(2);
	p2_temp(3) = 1;
	p2_temp_t = mTrans_cam2ground * p2_temp;	
	p2.x = p2_temp_t(0);
	p2.y = p2_temp_t(1);
	p2.z = p2_temp_t(2);
	p2.b = 0;
	p2.g = 0;
	p2.r = 255;
	p2.a = 255;
	cloud_ref->points.push_back( p2 );
    }
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud_ref, pcl_pc2); // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc2, ref_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    ref_point_cloud.header.frame_id = TFNAME;
    ref_point_cloud.header.stamp = ros::Time::now();   

}

}


