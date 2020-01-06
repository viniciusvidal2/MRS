//Includes
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <camera_info_manager/camera_info_manager.h>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rosbag/bag.h>

/// Namespaces
using namespace pcl;
using namespace std;
using namespace message_filters;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace Eigen;
using namespace cv;

/// Definitions
typedef PointXYZRGB PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicy;

/// Variaveis globais
Eigen::Quaternion<double> q_anterior, q_anterior_inverso, q_atual, q_relativo;
Eigen::Vector3d t_anterior, t_atual, t_relativo;

bool primeira_vez = true;

float thresh_dist = 7.0, thresh_ang = 10.0*M_PI/180.0;

rosbag::Bag bag;

/// Transformacao relativa overlap
void calcula_transformacao_relativa(){
  q_anterior_inverso = q_anterior.inverse();
  q_relativo = q_anterior_inverso*q_atual;
  t_relativo = q_anterior_inverso*(t_atual - t_anterior);
}

/// Callback
void salvarDadosCallback(const sensor_msgs::ImageConstPtr& msg_ima,
                         const sensor_msgs::ImageConstPtr& msg_bit,
                         const sensor_msgs::PointCloud2ConstPtr& msg_ptc,
                         const nav_msgs::OdometryConstPtr& msg_odo
                        ){
    if(!primeira_vez){
      // Pega rotacao e translacao atual
      q_atual.x() = (double)msg_odo->pose.pose.orientation.x;
      q_atual.y() = (double)msg_odo->pose.pose.orientation.y;
      q_atual.z() = (double)msg_odo->pose.pose.orientation.z;
      q_atual.w() = (double)msg_odo->pose.pose.orientation.w;
      t_atual = {msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z};

      // Calcular transformacao relativa entre os instantes
      calcula_transformacao_relativa();

      // Aqui ja checa o que foi deslocado, se passou do limite renova a pose da camera referencia
      if(t_relativo.norm() > thresh_dist || q_atual.angularDistance(q_anterior) > thresh_ang){
        ROS_INFO("Estamos gravando. Distancia: %.2f   Angulo: %.2f", t_relativo.norm(), q_atual.angularDistance(q_anterior));
        // Armazena para a proxima iteracao a odometria de referencia
        q_anterior = q_atual;
        t_anterior = t_atual;
        // Salvar no bag atual
        ros::Time t = ros::Time::now();
        bag.write("/dados_sync/image_scaled"         , t, *msg_ima);
        bag.write("/dados_sync/image_8bits"          , t, *msg_bit);
        bag.write("/zed/point_cloud/cloud_registered", t, *msg_ptc);
        bag.write("/zed/odom"                        , t, *msg_odo);
      }

    } else {

      ROS_INFO("Primeira aquisicao ....");

      // Segura a primeira odometria para referencia
      q_anterior.x() = (double)msg_odo->pose.pose.orientation.x;
      q_anterior.y() = (double)msg_odo->pose.pose.orientation.y;
      q_anterior.z() = (double)msg_odo->pose.pose.orientation.z;
      q_anterior.w() = (double)msg_odo->pose.pose.orientation.w;
      t_anterior = {msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z};

      q_atual = q_anterior;
      t_atual = t_anterior;

      // Salvar no bag a primeira mensagem
      ros::Time t = ros::Time::now();
      bag.write("/dados_sync/image_scaled"         , t, *msg_ima);
      bag.write("/dados_sync/image_8bits"          , t, *msg_bit);
      bag.write("/zed/point_cloud/cloud_registered", t, *msg_ptc);
      bag.write("/zed/odom"                        , t, *msg_odo);

      primeira_vez = false;

    } // fim do if


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dadosComZed");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    std::string nome_bag;
    n.getParam("nome_bag", nome_bag);

    ROS_INFO("Comecamos aquisicao para bag %s.....", nome_bag.c_str());

    // Inicia gravador de bag
    bag.open(nome_bag.c_str(), rosbag::bagmode::Write);

    // Subscriber para a nuvem instantanea e odometria
    message_filters::Subscriber<sensor_msgs::Image>       subima(nh, "/dados_sync/image_scaled"         , 100);
    message_filters::Subscriber<sensor_msgs::Image>       subbit(nh, "/dados_sync/image_8bits"          , 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subptc(nh, "/zed/point_cloud/cloud_registered", 100);
    message_filters::Subscriber<Odometry>                 subodo(nh, "/zed/odom"                        , 100);

    // Sincroniza as leituras dos topicos
    Synchronizer<syncPolicy> sync(syncPolicy(100), subima, subbit, subptc, subodo);
    sync.registerCallback(boost::bind(&salvarDadosCallback, _1, _2, _3, _4));

    ros::Rate r(2);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    // Se matar o no, fecha a bag tambem
    bag.close();

    return 0;
}
