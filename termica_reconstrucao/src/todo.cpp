#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
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

//// Descricao -- Reconstrucao termica online ////////////////////////////////////////////////////////////////
// o.   Ler arquivo contendo as informacoes de calibracao da camera termica [x]
// i.   Se inscreve nos topicos 'termica/thermal/image_raw', '../point_cloud', e '.../right/camera_info' [xxx]
// ii.  Varrer pontos da point cloud e projetar esses nas imagens termicas (p_2d = K(R|T)p_3d) [x]
// iii. Associar a intensidade do ponto da imagem termica projetado a intensidade do ponto tridimensional [x]
// iv.  Publicar point cloud termica [x]
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Definicoes
#define TOPICO_IMAGEM "stereo/right/image_raw"
#define CAMERA_INFO "stereo/right/camera_info"
#define TERMICA_YAML_DIR "file:///home/felipe/catkin_ws/src/RustBot/rustbot_calibration/calibration/teste2.yaml"
#define TOPICO_POINT_CLOUD "/stereo/points2"
typedef pcl::PointXYZRGB PointT;


// Variaveis globais
cv::Mat imagem_termica;
image_geometry::PinholeCameraModel cam_termica_modelo;
camera_info_manager::CameraInfoManager cam_termica();
pcl::PointCloud<PointT>::Ptr cloudTermica;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Callback imagem termica -- Le imagem termica e passa para variavel global;
void imagemTermicaCallback(const sensor_msgs::ImageConstPtr& msg)
{
  imagem_termica = cv_bridge::toCvShare(msg, "bgr8")->image;
  //ROS_INFO("Imagem termica lida! \n");
}

// Callback informacoes da camera -- Atualiza informacoes da posicao relativa da camera
void cameraInfoCallback(const sensor_msgs::CameraInfoPtr& camera_info)
{
	image_geometry::PinholeCameraModel cam_modelo;
	cam_modelo.fromCameraInfo(camera_info); 
	// Atualizar R e t da camera termica atraves do R e t da camera visual
	// ...
    //    ROS_INFO("Info da camera visual lida! \n");
}

// Callback point cloud -- Recebe point cloud e passa para variavel global
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
        ROS_INFO("p1\n");
        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_termica", 1);
        camera_info_manager::CameraInfoManager cam_termica(nh,"camera_termica",TERMICA_YAML_DIR);
        ROS_INFO("p2\n");
        cam_termica_modelo.fromCameraInfo(cam_termica.getCameraInfo());
	pcl::PointCloud<PointT>::Ptr cloud;	
	cloud = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
	cloudTermica = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
	pcl::fromROSMsg (*msg, *cloud);  // MSG->PCL
	sensor_msgs::PointCloud2 msg_out;
        ROS_INFO("p3\n");

	// Zerar intensidades da cloud termica
	cloudTermica = cloud;
  for(int j = 0; j < cloudTermica->points.size(); j++)
	{
    cloudTermica->points[j].r = 1;
    cloudTermica->points[j].g = 1;
    cloudTermica->points[j].b = 1;
	}

        ROS_INFO("p4\n");
	// Varrer point cloud e projetar pontos
        ROS_INFO_STREAM("point cloud size: " << cloud->points.size() << "\n");
  for(int i = 0; i < cloud->points.size(); i++)
	{
		// Ponto i da cloud
		cv::Point3d ponto3D;
		ponto3D.x = cloud->points[i].x;
		ponto3D.y = cloud->points[i].y;
		ponto3D.z = cloud->points[i].z;

                //ROS_INFO("p41\n");
		// Projetando
                cv::Point2d pontoProjetado; //project3dToPixel
                pontoProjetado = cam_termica_modelo.project3dToPixel(ponto3D);
		
                //ROS_INFO("p42\n");
		// Associando intensidade projetada ao ponto na cloud termica
                int intensidadeTermica;
                //intensidadeTermica = imagem_termica.at<int>(pontoProjetado.y, pontoProjetado.x);
                //ROS_INFO_STREAM("size: " << imagem_termica.size() << "\n");
                intensidadeTermica = imagem_termica.at<int>(10, 10);


                //ROS_INFO("p43\n");
                cloudTermica->points[i].r = intensidadeTermica;
		cloudTermica->points[i].g = intensidadeTermica;
		cloudTermica->points[i].b = intensidadeTermica;
	}	
        ROS_INFO("p5\n");
	// Limpando pontos que nao possuem projecao na imagem termica
        //std::vector<int> indicesInvalidos;
        //pcl::removeNaNFromPointCloud(*cloudTermica, *cloudTermica, indicesInvalidos);

        ROS_INFO("p6\n");
	// Publicando point cloud termica
        pcl::toROSMsg(*cloudTermica.get(), msg_out);
	msg_out.header.stamp = msg->header.stamp;
        pub.publish(msg);

        ROS_INFO("p7\n");
	// Resetando
	cloud.reset();
	cloudTermica.reset();

        ROS_INFO("Point cloud lida! \n");
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Funcao main
int main(int argc, char** argv)
{
	ros::init(argc, argv, "reconstrucao_termica");
        ros::NodeHandle nh;
        ROS_INFO("Inicio -- Reconstrucao Termica! \n");
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub_img_termica = it.subscribe(TOPICO_IMAGEM, 1, imagemTermicaCallback);
        ros::Subscriber sub_camera_info = nh.subscribe(CAMERA_INFO, 1, cameraInfoCallback);
        ros::Subscriber sub_cloud = nh.subscribe (TOPICO_POINT_CLOUD, 1, pointCloudCallback);
        //cam_termica_modelo.fromCameraInfo(cam_termica.getCameraInfo());
        //camera_info_manager::CameraInfoManager cam_tmp(nh,"camera_termica",TERMICA_YAML_DIR);
        //cam_termica = cam_tmp;

        while (ros::ok())
	{
            ros::spinOnce();
		// ...........................................................................................
  	}


}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

