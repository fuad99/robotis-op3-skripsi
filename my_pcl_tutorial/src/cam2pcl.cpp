#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/point_cloud.h>
#include <string>

using namespace cv;
using namespace std;

float height, pitch, roll, yaw;
Eigen::Matrix3d R;
Eigen::Vector3d t;
Eigen::Vector3d Cg;


Eigen::Matrix3d K;

ros::Publisher pub2;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	
  try
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    
    cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2HSV);
    //cv::blur( cv_ptr->image, cv_ptr->image, Size(5,5));
    // cv::inRange(cv_ptr->image,Scalar(0, 102, 106),Scalar(32, 255, 255),cv_ptr->image);
    cv::inRange(cv_ptr->image,Scalar(21, 143, 63),Scalar(78, 255, 195),cv_ptr->image);
    //cv::Canny( cv_ptr->image, cv_ptr->image, 5, 80, 3);
    
    pcl::PointCloud<pcl::PointXYZ> combined;
    sensor_msgs::PointCloud2 output;
    
    for(int u=0; u < 640;u++){
		for(int v=0; v < 480;v++){
			if (cv_ptr->image.at<unsigned char>(v,u) >= 255){
				// detected edges at position u,v
				//cv_ptr->image.at<unsigned char>(y,x) = 0;
				
				Eigen::Vector3d Q(u,v,1);
				Eigen::Vector3d Qc = K.inverse()   * Q;
				Eigen::Vector3d Qg = R.transpose() * Qc - R.transpose() * t;

				float lambda = -Cg(2) / (Qg(2) - Cg(2) );
				float Xg = Cg(0) + lambda*( Qg(0) - Cg(0) );	
				float Yg = Cg(1) + lambda*( Qg(1) - Cg(1) );	
				
				pcl::PointXYZ toPush;
				toPush.x = -Xg; toPush.y = Yg; toPush.z = 0.0;
				combined.points.push_back(toPush);
			} 
		}
	}
	
	
 	
 	
	ros::Time time_st = ros::Time::now ();
 	combined.header.frame_id = "body_link";
 	combined.header.stamp = time_st.toNSec()/1e3; 	
 	pub2.publish(combined.makeShared());
 	
    
    cv::imshow("view", cv_ptr->image);
    cv::waitKey(2);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam2pcl");
  //~ ros::NodeHandle nh("~");
  ros::NodeHandle nh;
  
  string heightS = "0.34";
  string pitchS  = "68.0";
  
  //~ nh.param<string>("height", heightS, "0.34");
  //~ ros::param::param<string>("~pitch", pitchS, "45.0");
  
  cv::namedWindow("view");
  cv::waitKey(2);
  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints("compressed");
  image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback,hints);
  

  
  pub2 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/cloud_pcl", 100);
  
  roll   = -62 * M_PI / 180.0;
  pitch  =  0;
  yaw    =  0; //-M_PI / 2;
  height = -0.63; //33 
  
  
  
  // Calculation of 
  Eigen::AngleAxisd rollAngle(roll,Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch,Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw,Eigen::Vector3d::UnitZ());
  
  Eigen::Quaternion<double> q = rollAngle *  pitchAngle * yawAngle;
  
  R = q.matrix();
  t(0) = 0; 
  t(1) = -0.63; 
  t(2) = height; 
  
  cout << R << endl;
  cout << t << endl;
  
  Cg = -R.transpose() * t;
  
  cout << Cg << endl;
  
  K << 612.140384, 0.0, 299.051868, 0.0, 609.428178, 231.070631, 0.0, 0.0, 1.0;
  
  cout << K << endl;
  
  
  //~ cout << heightS.c_str() << endl;
  //~ cout << pitchS.c_str() << endl;
  
  ros::spin();
  cv::waitKey(3);
  cv::destroyWindow("view");
}
