#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
//#include <camera_info_manager.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <camera_calibration_parsers/parse.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


#define IMG_TERMICA_TOPIC "termica/thermal/image_raw"
#define IMG_TERMICA_COLOR_TOPIC "termica/thermal/cor_termica/image_raw"



class tempCor
{
	protected: 
		ros::NodeHandle nh_;
		image_transport::Subscriber image_sub_;
		image_transport::ImageTransport it_;
		image_transport::Publisher pub_;
		
	public:
		tempCor(ros::NodeHandle nh_) : it_(nh_)
		{
			image_sub_ = it_.subscribe(IMG_TERMICA_TOPIC, 1, &tempCor::imageCallback, this);
			pub_ = it_.advertise(IMG_TERMICA_COLOR_TOPIC, 1);
		}
		
		void imageCallback(const sensor_msgs::ImageConstPtr &msg)
		{
			cv::Mat imagem_termica;
			cv::Mat imagem_termica_cor;
			cv::Mat img_g;

			imagem_termica = cv_bridge::toCvShare(msg, "bgr8")->image;
			cv::cvtColor(imagem_termica, img_g, CV_BGR2GRAY);	
			cv::applyColorMap(img_g, imagem_termica_cor, cv::COLORMAP_JET);

			cv_bridge::CvImage img_bridge;
			sensor_msgs::Image img_msg;
			std_msgs::Header header; 
			//header.seq = counter; // user defined counter
			header.stamp = ros::Time::now(); 
			img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, imagem_termica_cor);
			img_bridge.toImageMsg(img_msg);
			pub_.publish(img_msg); 
		}
};




// Funcao main
int main(int argc, char** argv)
{
        ros::init(argc, argv, "termica_cor");
        ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	tempCor tC(nh);


        while (ros::ok())
	{
        	ros::spinOnce();
		loop_rate.sleep();
		// ...........................................................................................
        }


}


