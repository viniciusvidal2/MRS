//Includes
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
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
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, Odometry> syncPolicy;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis globais
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mat imagem_anterior;
Eigen::Quaternion<double> q_anterior, q_anterior_inverso, q_atual, q_relativo;
Eigen::Vector3d t_anterior, t_atual, t_relativo;
PointCloud<PointT>::Ptr cloud, cloud_tf, cloud_filt;
image_geometry::PinholeCameraModel model;

bool primeira_vez = true;

// Publicador
ros::Publisher cloud_pub;
ros::Publisher image_pub;
ros::Publisher odom_pub;

// Itens vindos do launch
std::string camera_calibration_yaml;
std::string cloud_topic_in, cloud_topic_out, image_topic_in, image_topic_out, odom_topic_in, odom_topic_out;
std::string camera_type;
double overlap_rate;

// Overlap de 0 a 1 de quanto ficaria diferente
float overlap_0_a_1;

// Contagem de pontos projetados no frame de referencia
int n_pontos_fora_da_imagem_referencia = 0;

// Tamanho da imagem de entrada, nao precisa da imagem em si aqui
cv::Size image_size;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Iniciar o modelo da camera
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void inicia_modelo_camera(sensor_msgs::CameraInfo ci){
  model.fromCameraInfo(ci);
  image_size = model.fullResolution(); // Armazena ja o tamanho da imagem pra nao ter erro ali nos testes
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Retornar transformacao relativa entre os instantes
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calcula_transformacao_relativa(){
  q_anterior_inverso = q_anterior.inverse();
  q_relativo = q_anterior_inverso*q_atual;
  t_relativo = q_anterior_inverso*(t_atual - t_anterior);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback para evitar repeticao da nuvem por projecao na imagem anterior
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop_closure_callback(const sensor_msgs::ImageConstPtr& msg_ima,
                           const sensor_msgs::PointCloud2ConstPtr& msg_ptc,
                           const OdometryConstPtr& msg_odo){
  // Ler nuvem de pontos
  fromROSMsg(*msg_ptc, *cloud);
  cloud_filt->header.frame_id = cloud->header.frame_id;

  // Remover NaN
  vector<int> indicesNAN;
  removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);

  // Preparando mensagem da nuvem de saida
  sensor_msgs::PointCloud2 msg_ptc_out;
  msg_ptc_out.header.frame_id = msg_ptc->header.frame_id;

  // Preparar saida de odometria
  Odometry msg_odom_out;
  msg_odom_out.header.frame_id = msg_odo->header.frame_id;

  // Preparar saida de imagem
  sensor_msgs::Image msg_image_out;
  msg_image_out.header.frame_id = msg_ima->header.frame_id;

  // Vamos publicar a nuvem de saida? a principio nao, convencer do contrario com o overlap
  bool podemos_publicar = false;

  /// LOOP de transformacao e PROJECAO sobre a nuvem
  if(!primeira_vez){
    // Pega rotacao e translacao atual
    q_atual.x() = (double)msg_odo->pose.pose.orientation.x;
    q_atual.y() = (double)msg_odo->pose.pose.orientation.y;
    q_atual.z() = (double)msg_odo->pose.pose.orientation.z;
    q_atual.w() = (double)msg_odo->pose.pose.orientation.w;
    t_atual = {msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z};

    // Calcular transformacao relativa entre os instantes
    calcula_transformacao_relativa();

    // Transformar camera para o ponto onde estava na iteracao anterior, e a nuvem para a iteracao de agora
    transformPointCloud<PointT>(*cloud, *cloud_tf, t_relativo, q_relativo);

    // Projeta cada ponto da nuvem na imagem e se cair fora adiciona na nuvem filtrada - OU REMOVE DA NUVEM
    cv::Point3d ponto3D;
    cv::Point2d pontoProjetado;
    n_pontos_fora_da_imagem_referencia = 0;

    if (overlap_0_a_1 < 0.4){
      (*cloud_filt) = (*cloud);
    } else {
      for(int i=0; i < cloud_tf->size(); i++){
        ponto3D.x = cloud_tf->points[i].x;
        ponto3D.y = cloud_tf->points[i].y;
        ponto3D.z = cloud_tf->points[i].z;

        pontoProjetado = model.project3dToPixel(ponto3D);
        // Se cair fora da imagem pode-se considerar ponto novo e armazena
        if(!(pontoProjetado.x > 0 && pontoProjetado.x < image_size.width && pontoProjetado.y > 0 && pontoProjetado.y < image_size.height)){
          n_pontos_fora_da_imagem_referencia++;
          if(overlap_0_a_1 > 0.4)
            cloud_filt->push_back(cloud->points[i]); // Guardar o ponto sem transformacao relativa
        }
      } // Fim do for
    }

    cout << "Estamos com " << cloud_filt->size() << " nao projetados dentre " << cloud_tf->size() << " totais." << endl;

    // Testar se reduzimos o suficiente o overlap desejado, se sim podemos atualizar a referencia e seguir com mensagens publicadas
    if( float(cloud->size() - n_pontos_fora_da_imagem_referencia) / float(cloud->size())  <  overlap_0_a_1 ){

      cout << "\nQuantos pontos fora da imagem: " << n_pontos_fora_da_imagem_referencia << "\tTotal: " << cloud->size() << endl;
      cout << "Taxa de relacao: " << float(cloud->size() - n_pontos_fora_da_imagem_referencia) / float(cloud->size())  << endl;

      podemos_publicar = true; // Podemos publicar qual seja a nuvem filtrada e as imagens
      msg_odom_out  = *msg_odo;
      msg_image_out = *msg_ima; // Separadas as mensagens de imagem e odometria pra passar em frente sincronizado, camera termica somente
      // Armazena para a proxima iteracao a odometria de referencia
      q_anterior = q_atual;
      t_anterior = t_atual;

    } // Fim do if de overlap

  } else {

    cout << "Primeira iteracao" << endl;

    // A odometria tambem e igual nesse caso
    msg_odom_out = *msg_odo;

    // A imagem tambem e igual passa direto
    msg_image_out = *msg_ima;

    // Nuvem segue como veio
    (*cloud_filt) = (*cloud);

    // Segura a primeira odometria para referencia
    q_anterior.x() = (double)msg_odo->pose.pose.orientation.x;
    q_anterior.y() = (double)msg_odo->pose.pose.orientation.y;
    q_anterior.z() = (double)msg_odo->pose.pose.orientation.z;
    q_anterior.w() = (double)msg_odo->pose.pose.orientation.w;
    t_anterior = {msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z};

    primeira_vez = false;
    podemos_publicar = true;

    ros::Rate rate(1);
    rate.sleep(); // Para dar tempo do no acumulador comecar
  } // fim do if

  // Publica tudo aqui a depender da camera e do resultado da avaliacao do overlap
  if(podemos_publicar){

    ros::Time t = ros::Time::now();
    msg_ptc_out.header.stamp   = t;
    msg_odom_out.header.stamp  = t;
    msg_image_out.header.stamp = t;

    toROSMsg(*cloud, msg_ptc_out);
//    toROSMsg(*cloud_filt, msg_ptc_out);
    cloud_pub.publish(msg_ptc_out);

    odom_pub.publish(msg_odom_out); // Odometria a depender da iteracao

    if(camera_type == "termica"){
      image_pub.publish(msg_image_out);    // Imagem passa direto por aqui, so sincronizada
    }

  }

  // Libera as nuvens
  cloud->clear(); cloud_filt->clear(); cloud_tf->clear();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main ///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv){
  ros::init(argc, argv, "projecao_overlap");
  ros::NodeHandle nh;
  ros::NodeHandle n_("~"); // Frescura da camera

  ROS_INFO("Comecamos a calcular o overlap.");

  // Leitura de todos os parametros e topicos vindos do arquivo launch
  n_.getParam("camera_calibration_yaml", camera_calibration_yaml);
  camera_calibration_yaml = camera_calibration_yaml + std::string(".yaml");
  n_.getParam("cloud_topic_in" , cloud_topic_in );
  n_.getParam("cloud_topic_out", cloud_topic_out);
  n_.getParam("image_topic_in" , image_topic_in );
  n_.getParam("image_topic_out", image_topic_out);
  n_.getParam("odom_topic_in"  , odom_topic_in  );
  n_.getParam("odom_topic_out" , odom_topic_out );
  n_.getParam("camera_type"    , camera_type    );
  n_.getParam("overlap_rate"   , overlap_rate   );

  // Porcentagem que a nuvem vai ter de diferenca para a referencia
  overlap_0_a_1 = overlap_rate/100.0;

  // Aloca as nuvens
  cloud      = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  cloud_tf   = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  cloud_filt = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

  // Inicia o publicador
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> (cloud_topic_out, 100);
  image_pub = nh.advertise<sensor_msgs::Image      > (image_topic_out, 100);
  odom_pub  = nh.advertise<Odometry                > (odom_topic_out , 100);

  // Inicia o modelo da camera com arquivo de calibracao
  camera_info_manager::CameraInfoManager cam_info(n_, "left_optical", camera_calibration_yaml);
  inicia_modelo_camera(cam_info.getCameraInfo());

  // Subscriber para a nuvem instantanea e odometria
  message_filters::Subscriber<sensor_msgs::Image>       subima(nh, image_topic_in, 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> subptc(nh, cloud_topic_in, 100);
  message_filters::Subscriber<Odometry>                 subodo(nh, odom_topic_in , 100);

  // Sincroniza as leituras dos topicos
  Synchronizer<syncPolicy> sync(syncPolicy(100), subima, subptc, subodo);
  sync.registerCallback(boost::bind(&loop_closure_callback, _1, _2, _3));

  ros::spin();
}
