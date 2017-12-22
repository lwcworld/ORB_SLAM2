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


#include <Eigen/Dense>

#include <math.h>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include"../../../include/System.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

#include "std_msgs/Float64.h"

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)


using namespace std;
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
	


    ORB_SLAM2::System* mpSLAM;

    double roll, yaw,pitch;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
	//ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw/compressed", 1, &imageCallback);
    ros::Publisher robotYaw = nodeHandler.advertise<std_msgs::Float64>("/robot/yaw", 100);
    ros::Subscriber sub = nodeHandler.subscribe("/camera/camera_eigen", 1, &ImageGrabber::GrabImage,&igb);
    ros::Rate loop_rate(50);
    //ros::spin();
    while (ros::ok())
	{
        robotYaw.publish(igb.yaw);
		ros::spinOnce();
        loop_rate.sleep();
	}

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
	cv::Mat image;
    try
    {
		//cv_ptr = cv_bridge::toCvShare(cv::imdecode(cv::Mat(msg->data),1));//convert compressed image data to cv::Mat
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
	//cv::Mat pose = mpSLAM->TrackMonocular(image,cv_ptr->header.stamp.toSec());

	if (pose.empty()){
        return;
	}


    /* global left handed coordinate system */
	static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
	static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
	// matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
	static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
			-1, 1,-1, 1,
			-1,-1, 1, 1,
			1, 1, 1, 1);

	//prev_pose * T = pose
	cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
	world_lh = world_lh * translation;
	pose_prev = pose.clone();

	tf::Matrix3x3 tf3d;
	tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
			pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
			pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

	tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

	//rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
	const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
			0, 0, 1,
			1, 0, 0);

	static tf::TransformBroadcaster br;

	tf::Matrix3x3 globalRotation_rh = tf3d;
	tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;

	tf::Quaternion tfqt;
	globalRotation_rh.getRotation(tfqt);

	double aux = tfqt[0];
	tfqt[0]=-tfqt[2];
	tfqt[2]=tfqt[1];
	tfqt[1]=aux;
    globalRotation_rh.getEulerYPR(roll,yaw,pitch,1);
    roll = radiansToDegrees(roll);
    yaw = radiansToDegrees(yaw);
    pitch = radiansToDegrees(pitch); 
    ROS_INFO("Roll: %.2f\tYaw: %.2f\tPitch: %.2f\t", roll, yaw, pitch);

	tf::Transform transform;
	transform.setOrigin(globalTranslation_rh);
	transform.setRotation(tfqt);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_pose"));
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
	cv::Mat image;
  try
  {
    cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
    cv::imshow("view", image);
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to image!");
  }
}


