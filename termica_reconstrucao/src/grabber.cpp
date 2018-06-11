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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <fstream>
#include <iostream>

// Se inscreve no topico /termica/thermal/image_raw e salva as imagens em uma pasta
// Se inscreve no topico stereo/right/camera_info
// Publica o topico /termica/thermal/camera_info a partir das configuracoes iniciais da camera termica dadas a partir de um arquivo
//.yaml e as matrizes R_relativa e T_relativa


#define IMG_SAVE_DIR "/home/felipe/catkin_ws/src/termica_reconstrucao/Dados/imagensTermicas"
#define IMG_TERMICA_TOPIC "stereo/right/image_raw"
#define CAMERA_INFO_VISUAL "stereo/right/camera_info"
#define TERMICA_YAML_DIR "file:////home/felipe/catkin_ws/src/termica_reconstrucao/Dados/calibracao/right.yaml"
#define CAMERA_POSE_OUTPUT_FILE "/home/felipe/catkin_ws/src/termica_reconstrucao/Dados/termica_pose/termica_pose.txt"

// Variaveis globais
int iterador = 0;
sensor_msgs::CameraInfo termica_info;
Eigen::MatrixXd Rrelativa(3,3);
Eigen::MatrixXd Crelativa(3,1);
ros::Publisher pub;
std::ofstream file(CAMERA_POSE_OUTPUT_FILE);


// Callback imagem termica -- Le imagem termica e salva em uma pasta -- falta
void imagemTermicaCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::string nome_imagem;
    nome_imagem = std::string(IMG_SAVE_DIR) + "/imagem" + std::to_string(iterador) + ".png";   // <<  ".png";

    cv::Mat imagem_termica = cv_bridge::toCvShare(msg, "bgr8")->image;
    //ROS_INFO_STREAM("a: " << nome_imagem);

     cv::imwrite( nome_imagem,  imagem_termica );
     iterador++;
}

// Callback informacoes da camera -- Le camera_info da imagem visual e publica camera_info da imagem termica
void cameraInfoCallback(const sensor_msgs::CameraInfoPtr& camera_info)
{
    //ROS_INFO_STREAM("Camera info \n");
    ros::NodeHandle nh;

    Eigen::MatrixXd Rvisual(3,3);
    Eigen::MatrixXd Tvisual(3,1);
    Eigen::MatrixXd Cvisual(3,1);

    Eigen::MatrixXd Rtermica(3,3);
    Eigen::MatrixXd Ttermica(3,1);
    Eigen::MatrixXd Ctermica(3,1);


    // Inicializando matriz de rotacao
    Rvisual(0,0) = camera_info->R[0];
    Rvisual(0,1) = camera_info->R[1];
    Rvisual(0,2) = camera_info->R[2];
    Rvisual(1,0) = camera_info->R[3];
    Rvisual(1,1) = camera_info->R[4];
    Rvisual(1,2) = camera_info->R[5];
    Rvisual(2,0) = camera_info->R[6];
    Rvisual(2,1) = camera_info->R[7];
    Rvisual(2,2) = camera_info->R[8];


    // Inicializando matriz de translacao
    Tvisual(0,0) = camera_info->P[3];
    Tvisual(1,0) = camera_info->P[7];
    Tvisual(2,0) = camera_info->P[11];
    Cvisual = -Rvisual.inverse()*Tvisual;

    // Calculando a Rotacao e Translacao da camera termica a partir da camera visual
    Rtermica = Rvisual*Rrelativa;
    Ctermica = Cvisual + Crelativa;
    Ttermica = -Rtermica*Ctermica;

    // Modificando a camera_info da camera termica
    termica_info.R[0] = Rtermica(0,0);
    termica_info.R[1] = Rtermica(0,1);
    termica_info.R[2] = Rtermica(0,2);
    termica_info.R[3] = Rtermica(1,0);
    termica_info.R[4] = Rtermica(1,1);
    termica_info.R[5] = Rtermica(1,2);
    termica_info.R[6] = Rtermica(2,0);
    termica_info.R[7] = Rtermica(2,1);
    termica_info.R[8] = Rtermica(2,2);
    termica_info.P[3] = Ttermica(0,0);
    termica_info.P[7] = Ttermica(1,0);
    termica_info.P[11] =Ttermica(2,0);

    file << Rtermica(0,0) << " " << Rtermica(0,1) << " " << Rtermica(0,2) << " "<< Rtermica(1,0) << " "<< Rtermica(1,1) << " "<< Rtermica(1,2) << " "<< Rtermica(2,0) << " "<< Rtermica(2,1) << " "<< Rtermica(2,2) << " " << Ttermica(0,0) << " " << Ttermica(1,0) << " " << Ttermica(2,0) <<"\n";
    pub.publish(termica_info);

}

// Inicializa relativa
void inicializaRelativo()
{
   Rrelativa <<   1, 0, 0,
                  0, 1, 0,
                  0, 0, 1;

    Crelativa <<  0,
                  0,
                  0;
}

// Funcao main
int main(int argc, char** argv)
{
        ros::init(argc, argv, "grabber_informacoes");
        ros::NodeHandle nh;
        ROS_INFO("Inicio -- Grabber de informacoes! \n");
        pub = nh.advertise<sensor_msgs::CameraInfo>("termica_info", 1000);
        inicializaRelativo();
        camera_info_manager::CameraInfoManager cam_termica(nh,"camera_termica",TERMICA_YAML_DIR);
        termica_info = cam_termica.getCameraInfo();
	image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub_img_termica = it.subscribe(IMG_TERMICA_TOPIC, 1, imagemTermicaCallback);
        ros::Subscriber sub_camera_info = nh.subscribe(CAMERA_INFO_VISUAL, 1, cameraInfoCallback);

        //std::ofstream file("/home/felipe/catkin_ws/src/termica_reconstrucao/Dados/termica_pose/termica_pose.txt");
        //file.open();
        //file << "nada \n";
        //file.close();

        while (ros::ok())
	{
            ros::spinOnce();
		// ...........................................................................................
        }

        file.close();
}

