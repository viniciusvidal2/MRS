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
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> syncPolicy;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Estrutura do ponto com visual e termica ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct PointC
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_RGB;
  float l;
  float o;
  float p;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointC,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, l, l)
                                   (float, o, o)
                                   (float, p, p)
                                   )

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis globais ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//pcl::PointCloud<PointT>::Ptr cloud_;
pcl::PointCloud<PointT>::Ptr cloud_visual_instantanea;
pcl::PointCloud<PointT>::Ptr cloud_termica_instantanea;
image_geometry::PinholeCameraModel camTermicaModelo_;

ros::Publisher pc_termica_pub;
ros::Publisher pc_completa_pub;

int nPontos;

boost::array<double, 12> P_correta{1400, 0, 330, -50, 0, 1400, 270, -8.16, 0, 0, 1, 0};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback para receber mensagens e calcular projecao ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void projecao_callback(const sensor_msgs::ImageConstPtr imagem_input, sensor_msgs::CameraInfoConstPtr camera_input, sensor_msgs::PointCloud2ConstPtr cloud_input){
  ROS_INFO("Processo rodando normalmente");

  // Coisas de imagem
  cv::Mat img_g;
  cv::Mat imagem_termica_cor;
  cv::Mat imgCv_ = cv_bridge::toCvShare(imagem_input, "bgr8")->image;
  cv::cvtColor(imgCv_, img_g, CV_BGR2GRAY);
  cv::applyColorMap(img_g, imagem_termica_cor, cv::COLORMAP_JET);
  imgCv_ = imagem_termica_cor;

  // Coloca informacoes da camera no modelo
  sensor_msgs::CameraInfo temp = *camera_input; //->R[0] = 0;
  temp.P = P_correta;
  camTermicaModelo_.fromCameraInfo(temp);

  // Nuvens de pontos
  std::string id = cloud_input->header.frame_id;
  pcl::fromROSMsg(*cloud_input, *cloud_visual_instantanea);
  std::vector<int> indicesNAN;
  pcl::removeNaNFromPointCloud(*cloud_visual_instantanea, *cloud_visual_instantanea, indicesNAN);
  nPontos = int(cloud_visual_instantanea->size());

  ////// Aqui o processo de projecao propriamente dito com filtragem
  if(nPontos > 0 && nPontos < 1000000)
  {
//    cloud_termica_instantanea = cloud_;
//    cloud_visual_instantanea->resize(nPontos);

    sensor_msgs::PointCloud2 termica_out;
    sensor_msgs::PointCloud2 visual_out;

    // Pontos que serao validos se projetados corretamente
//    PointC point_completa;
    PointT point_termica;

    // Projetando os pontos para a imagem
    for(int i = 0; i < nPontos; i++)
    {
      cv::Point3d ponto3D;
      cv::Point2d pontoProjetado;
      ponto3D.x = cloud_visual_instantanea->points[i].x;
      ponto3D.y = cloud_visual_instantanea->points[i].y;
      ponto3D.z = cloud_visual_instantanea->points[i].z;

//      cloud_visual_instantanea->points[i].x = cloud_termica_instantanea->points[i].x;
//      cloud_visual_instantanea->points[i].y = cloud_termica_instantanea->points[i].y;
//      cloud_visual_instantanea->points[i].z = cloud_termica_instantanea->points[i].z;
//      cloud_visual_instantanea->points[i].b = cloud_termica_instantanea->points[i].b;
//      cloud_visual_instantanea->points[i].g = cloud_termica_instantanea->points[i].g;
//      cloud_visual_instantanea->points[i].r = cloud_termica_instantanea->points[i].r;

      pontoProjetado = camTermicaModelo_.project3dToPixel(ponto3D);

      // Verificar se a projecao esta dentro da imagem
      if(pontoProjetado.x > 0  && pontoProjetado.x < imgCv_.cols && pontoProjetado.y > 0 && pontoProjetado.y < imgCv_.rows)
      {
        int b = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[0];
        int g = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[1];
        int r = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[2];

//        point_completa.x = ponto3D.x;
//        point_completa.y = ponto3D.y;
//        point_completa.z = ponto3D.z;
//        point_completa.r = cloud_->points[i].r;
//        point_completa.g = cloud_->points[i].g;
//        point_completa.b = cloud_->points[i].b;
//        point_completa.l = r;
//        point_completa.o = g;
//        point_completa.p = b;

        point_termica.x = ponto3D.x;
        point_termica.y = ponto3D.y;
        point_termica.z = ponto3D.z;
        point_termica.r = r;
        point_termica.g = g;
        point_termica.b = b;

        cloud_termica_instantanea->push_back(point_termica);
//        cloud_visual_instantanea->push_back(point_completa);



//        cloud_termica_instantanea->points[i].b = b;
//        cloud_termica_instantanea->points[i].g = g;
//        cloud_termica_instantanea->points[i].r = r;

//        cloud_visual_instantanea->points[i].l = r;
//        cloud_visual_instantanea->points[i].o = g;
//        cloud_visual_instantanea->points[i].p = b;
//      }
//      else
//      {
//        cloud_termica_instantanea->points[i].b = nan("");
//        cloud_termica_instantanea->points[i].g = nan("");
//        cloud_termica_instantanea->points[i].r = nan("");
//        cloud_visual_instantanea->points[i].l = nan("");
//        cloud_visual_instantanea->points[i].o = nan("");
//        cloud_visual_instantanea->points[i].p = nan("");
      }
    } // fim do for
  } // fim do if

  // Filtro final
//  std::vector<int> indicesNAN2;
//  pcl::removeNaNFromPointCloud(*cloud_termica_instantanea, *cloud_termica_instantanea, indicesNAN2);
//  pcl::removeNaNFromPointCloud(*cloud_visual_instantanea, *cloud_visual_instantanea, indicesNAN2);

  // Empacota e publica novamente
  sensor_msgs::PointCloud2 termica_out;
  sensor_msgs::PointCloud2 visual_out;
  pcl::toROSMsg (*cloud_termica_instantanea, termica_out);
  pcl::toROSMsg (*cloud_visual_instantanea, visual_out);

  termica_out.header.frame_id = id;
  termica_out.header.stamp = cloud_input->header.stamp;
  pc_termica_pub.publish(termica_out);
  visual_out.header.frame_id = id;
  visual_out.header.stamp = cloud_input->header.stamp;
  pc_completa_pub.publish(visual_out);

  // Liberando as nuvens, importante demais
  cloud_termica_instantanea->clear();
  cloud_visual_instantanea->clear();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "reconstrucao");
  ros::NodeHandle nh;

  // Modelo de camera
  std::string termicaCalibrationYAML;
  nh.getParam("termica_calibration_yaml", termicaCalibrationYAML);
  termicaCalibrationYAML = termicaCalibrationYAML + std::string(".yaml");

  // Inicia nuvens de pontos alocando memoria
//  cloud_            = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  cloud_visual_instantanea     = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  cloud_termica_instantanea = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

  // Publicadores
  std::string topico_out;
  nh.getParam("topico_out", topico_out);

  pc_termica_pub  = nh.advertise<sensor_msgs::PointCloud2> ("/termica/termica_pc", 1000);
  pc_completa_pub = nh.advertise<sensor_msgs::PointCloud2> ("/visual_pc"       , 1000);

  // Iniciando o subscriber sincronizado
  message_filters::Subscriber<sensor_msgs::Image>       image_sub(  nh, "termica/thermal/image_raw"  , 200);
  message_filters::Subscriber<sensor_msgs::CameraInfo>  camera_sub( nh, "termica/thermal/camera_info", 200);
//  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(  nh, "/loop_closure_cloud"        , 200);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(  nh, "stereo/points2"             , 200);

  Synchronizer<syncPolicy> sync(syncPolicy(200), image_sub, camera_sub, cloud_sub);
  sync.registerCallback( boost::bind(&projecao_callback, _1, _2, _3) );

  ros::spin();
}