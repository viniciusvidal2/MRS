#define PCL_NO_PRECOMPILE
#include "ros/ros.h"
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <camera_calibration_parsers/parse.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace pcl;
using namespace std;
using namespace message_filters;
using namespace Eigen;

typedef pcl::PointXYZRGB PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicy;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis globais ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<PointT>::Ptr cloud;
pcl::PointCloud<PointT>::Ptr cloud_termica_instantanea;
image_geometry::PinholeCameraModel model;

ros::Publisher pc_termica_pub;
ros::Publisher odom_pub;

boost::array<double, 12> P_correta{1400, 0, 330, -50, 0, 1400, 270, -8.16, 0, 0, 1, 0};

cv::Size image_size;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Inicia variavel global de modelo da camera ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void inicia_modelo_camera(sensor_msgs::CameraInfo ci){
  ci.P = P_correta;
  model.fromCameraInfo(ci);
  image_size = model.fullResolution(); // Armazena ja o tamanho da imagem pra nao ter erro ali nos testes
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback para receber mensagens e calcular projecao ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void projecao_callback(const sensor_msgs::ImageConstPtr imagem_input,
                       const sensor_msgs::PointCloud2ConstPtr cloud_input,
                       const nav_msgs::OdometryConstPtr odom_input){
  ROS_INFO("Processo rodando normalmente");

  // Coisas de imagem
  cv::Mat img_g;
  cv::Mat imagem_termica_cor;
  cv::Mat imgCv_ = cv_bridge::toCvShare(imagem_input, "bgr8")->image;
  cv::cvtColor(imgCv_, img_g, CV_BGR2GRAY);
  cv::applyColorMap(img_g, imagem_termica_cor, cv::COLORMAP_JET);
//  cv::applyColorMap(img_g, imgCv_, cv::COLORMAP_JET);
  imgCv_ = imagem_termica_cor;

  // Nuvens de pontos
  pcl::fromROSMsg(*cloud_input, *cloud);
  std::vector<int> indicesNAN;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);

  ////// Aqui o processo de projecao propriamente dito com filtragem
  if(cloud->size() > 0 && cloud->size() < 1000000)
  {
    // Pontos que serao validos se projetados corretamente
    PointT point_termica;

    // Projetando os pontos para a imagem
    for(int i = 0; i < cloud->size(); i++)
    {
      cv::Point3d ponto3D;
      cv::Point2d pontoProjetado;
      ponto3D.x = cloud->points[i].x;
      ponto3D.y = cloud->points[i].y;
      ponto3D.z = cloud->points[i].z;

      pontoProjetado = model.project3dToPixel(ponto3D);

      // Verificar se a projecao esta dentro da imagem
      if(pontoProjetado.x > 0  && pontoProjetado.x < image_size.width && pontoProjetado.y > 0 && pontoProjetado.y < image_size.height)
      {
        int b = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[0];
        int g = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[1];
        int r = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[2];

        point_termica.x = ponto3D.x;
        point_termica.y = ponto3D.y;
        point_termica.z = ponto3D.z;
        point_termica.r = r;
        point_termica.g = g;
        point_termica.b = b;

        cloud_termica_instantanea->push_back(point_termica);
      }
    } // fim do for

    cout << "\nTamanho da nuvem termica: " << cloud_termica_instantanea->size() << endl;

    // Empacota e publica novamente a nuvem termica e a odometria correspondente
    sensor_msgs::PointCloud2 termica_out;
    pcl::toROSMsg (*cloud_termica_instantanea, termica_out);
    nav_msgs::Odometry odom_out;
    odom_out = *odom_input;

    termica_out.header.frame_id = cloud_input->header.frame_id;
    termica_out.header.stamp    = ros::Time::now();
    odom_out.header.frame_id = odom_input->header.frame_id;
    odom_out.header.stamp    = ros::Time::now();
    pc_termica_pub.publish(termica_out);
    odom_pub.publish(odom_out);

    // Liberando as nuvens, importante demais
    cloud_termica_instantanea->clear(); cloud->clear();
  } // fim do if

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "projeta_termica");
  ros::NodeHandle nh;
  ros::NodeHandle n_("~");

  // Modelo de camera
  std::string termicaCalibrationYAML;
  n_.getParam("termica_calibration_yaml", termicaCalibrationYAML);
  termicaCalibrationYAML = termicaCalibrationYAML + std::string(".yaml");
  camera_info_manager::CameraInfoManager cam_info(n_, "termica", termicaCalibrationYAML);
  inicia_modelo_camera(cam_info.getCameraInfo());

  // Inicia nuvens de pontos alocando memoria
  cloud                     = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  cloud_termica_instantanea = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

  // Publicadores
  pc_termica_pub = nh.advertise<sensor_msgs::PointCloud2> ("/termica/cloud_inst", 1000);
  odom_pub       = nh.advertise<nav_msgs::Odometry      > ("/termica/odometry"  , 1000);

  // Iniciando o subscriber sincronizado
  message_filters::Subscriber<sensor_msgs::Image>       image_sub(nh, "/overlap/image"        , 200);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/overlap/termica_cloud", 200);
  message_filters::Subscriber<nav_msgs::Odometry>       odom_sub (nh, "/overlap/odometry"     , 200);

  Synchronizer<syncPolicy> sync(syncPolicy(200), image_sub, cloud_sub, odom_sub);
  sync.registerCallback( boost::bind(&projecao_callback, _1, _2, _3) );

  ros::spin();
}
