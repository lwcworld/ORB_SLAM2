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

#include"../../../include/MapPoint.h"

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)


using namespace std;
using namespace tf;
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    tf::Quaternion hamiltonProduct(tf::Quaternion a, tf::Quaternion b);
	//void GrabImage(const sensor_msgs::ImageConstPtr& msg, ros::Publisher pos_pub, ros::Publisher cloud_pub);


    ORB_SLAM2::System* mpSLAM;

    double roll, yaw,pitch;

    ros::Publisher cloud_pub;
    ros::Publisher pos_pub;
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
    igb.cloud_pub = nodeHandler.advertise<sensor_msgs::PointCloud>("/slam/pointcloud", 1);
    igb.pos_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/slam/pos", 1);
    //ros::Subscriber sub = nodeHandler.subscribe("/camera/camera_eigen", 1, boost::bind(&ImageGrabber::GrabImage,&igb,_1,_2,_3,pos_pub,cloud_pub));
    //ros::Subscriber sub = nodeHandler.subscribe("/camera/camera_eigen", 1, boost::bind(&ImageGrabber::GrabImage,&igb,i));
    
    ros::Subscriber sub = nodeHandler.subscribe("/camera/camera_eigen", 1,&ImageGrabber::GrabImage,&igb);
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

tf::Quaternion ImageGrabber::hamiltonProduct(tf::Quaternion a, tf::Quaternion b) {

	tf::Quaternion c;

		c[0] = (a[0]*b[0]) - (a[1]*b[1]) - (a[2]*b[2]) - (a[3]*b[3]);
		c[1] = (a[0]*b[1]) + (a[1]*b[0]) + (a[2]*b[3]) - (a[3]*b[2]);
		c[2] = (a[0]*b[2]) - (a[1]*b[3]) + (a[2]*b[0]) + (a[3]*b[1]);
		c[3] = (a[0]*b[3]) + (a[1]*b[2]) - (a[2]*b[1]) + (a[3]*b[0]);

	return c;
}

//void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg, ros::Publisher pos_pub, ros::Publisher cloud_pub)
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

	if (pose.empty()){
        return;
	}
    cout << "Test1" << endl;

    //Quaternion
	tf::Matrix3x3 tf3d;
	tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
			pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
			pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);
	double aux = tfqt[0];
		tfqt[0]=-tfqt[2];
		tfqt[2]=tfqt[1];
		tfqt[1]=aux;

	//Translation for camera
	tf::Vector3 origin;
	origin.setValue(pose.at<float>(0,3),pose.at<float>(1,3),pose.at<float>(2,3));
	//rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
	const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
											0, 0, 1,
											-1, 0, 0);

	tf::Vector3 translationForCamera = origin * rotation270degXZ;

	//Hamilton (Translation for world)
	tf::Quaternion quaternionForHamilton(tfqt[3], tfqt[0], tfqt[1], tfqt[2]);
	tf::Quaternion secondQuaternionForHamilton(tfqt[3], -tfqt[0], -tfqt[1], -tfqt[2]);
	tf::Quaternion translationHamilton(0, translationForCamera[0], translationForCamera[1], translationForCamera[2]);
    cout << "Test2" << endl;

	tf::Quaternion translationStepQuat;
	translationStepQuat = hamiltonProduct(hamiltonProduct(quaternionForHamilton, translationHamilton), secondQuaternionForHamilton);

	tf::Vector3 translation(translationStepQuat[1], translationStepQuat[2], translationStepQuat[3]);

	//Scaling
	/*if(m_numScales > 0) {
		translation = m_scale * translation;
	}
	//Set world
	m_currentQ = tfqt;
	m_currentT = translation;
	translation = translation - m_worldT;
	tfqt = tfqt * m_worldQ.inverse();*/

	//Creates transform and populates it with translation and quaternion
	tf::Transform transformCurrent;
	transformCurrent.setOrigin(translation);
	transformCurrent.setRotation(tfqt);
	//Publishes transform
	static tf::TransformBroadcaster br;
    cout << "Test3" << endl;
	br.sendTransform(tf::StampedTransform(transformCurrent, ros::Time::now(), "world", "camera_pose"));


    
    // transform into right handed camera frame
    tf::Matrix3x3 rh_cameraPose(  - pose.at<float>(0,0),   pose.at<float>(0,1),   pose.at<float>(0,2),
                                  - pose.at<float>(1,0),   pose.at<float>(1,1),   pose.at<float>(1,2),
                                    pose.at<float>(2,0), - pose.at<float>(2,1), - pose.at<float>(2,2));

    tf::Vector3 rh_cameraTranslation( pose.at<float>(0,3),pose.at<float>(1,3), - pose.at<float>(2,3) );

    //rotate 270deg about z and 270deg about x
    tf::Matrix3x3 rotation270degZX( 0, 0, 1,
                                   -1, 0, 0,
                                    0,-1, 0);

    //publish right handed, x forward, y right, z down (NED)

    tf::Quaternion q;
    rh_cameraPose.getRotation(q);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "world";
    p.pose.position.x = rh_cameraTranslation[0];
    p.pose.position.y = rh_cameraTranslation[1];
    p.pose.position.z = rh_cameraTranslation[2];
    p.pose.orientation.x = q[0];
    p.pose.orientation.y = q[1];
    p.pose.orientation.z = q[2];
    p.pose.orientation.w = q[3];

    pos_pub.publish(p);

    // POINT CLOUD
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = "world";
    std::vector<geometry_msgs::Point32> geo_points;
    std::vector<ORB_SLAM2::MapPoint*> points = mpSLAM->GetTrackedMapPoints();
    if (points.empty()){
        return;
	}
    //cout << points.size() << endl;
    for (std::vector<int>::size_type i = 0; i != points.size(); i++) {
	    if (points[i]) {
		    cv::Mat coords = points[i]->GetWorldPos();
		    geometry_msgs::Point32 pt;
		    pt.x = coords.at<float>(0);
		    pt.y = coords.at<float>(2);
		    pt.z = -coords.at<float>(1);
		    geo_points.push_back(pt);
	    } else {
	    }
    }
    //cout << geo_points.size() << endl;
    cloud.points = geo_points;
    cloud_pub.publish(cloud);
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


