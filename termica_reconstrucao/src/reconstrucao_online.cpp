/////////////////////////////////////////////////////////////////////
//////////////// Reconstrucao Termica 3D////////////////////////////
////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////




// Bibliotecas
#include "ros/ros.h"
#include <sensor_msgs/CameraInfo.h>
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




// Definicoes
typedef pcl::PointXYZRGB PointT;




// Classe reconstrucaoTermica
class reconstrucaoTermica
{
	protected:
		ros::NodeHandle nh_;
                image_transport::Subscriber image_sub_;
                //image_transport::CameraSubscriber image_sub_;
                image_transport::ImageTransport it_;
		std::string point_cloud_PLY_;
                ros::Publisher pc_pub_;
                ros::Publisher pc_visual_pub_;


	public:
                ros::Subscriber pc_sub_;
		pcl::PointCloud<PointT> cloud_;
		sensor_msgs::ImageConstPtr imgMsg_;
                cv::Mat imgCv_;
		pcl::PointCloud<PointT> cloudTransformada_;
                image_geometry::PinholeCameraModel camTermicaModelo_;
                camera_info_manager::CameraInfoManager camInfo_;
                int nPontos_;



		// Construtor reconstrucaoTermica
                reconstrucaoTermica(ros::NodeHandle nh_, std::string termicaCalibrationYAML, std::string topico_imagem, std::string topico_pc, std::string topico_out)
                : it_(nh_), camInfo_(nh_,"termica",termicaCalibrationYAML)
		{
                        image_sub_ = it_.subscribe(topico_imagem, 1, &reconstrucaoTermica::imageCallback, this);
                        //image_sub_ = it_.subscribeCamera(topico_imagem, 1, &reconstrucaoTermica::imageCallback, this);
                        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (topico_out, 1000);
                        pc_visual_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("termica/visual_pc", 1000);
                        pc_sub_= nh_.subscribe(topico_pc, 1, &reconstrucaoTermica::pointCloudCallback, this);
                        camTermicaModelo_.fromCameraInfo(camInfo_.getCameraInfo());
                } //fim reconstrucaoTermica()
	


                // Callback point cloud
                void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
                {
                    pcl::fromROSMsg(*msg, cloud_);
                    nPontos_ = int(cloud_.points.size());
                } // fim pointCloudCallback()


                // Callback da imagem termica - Recebe imagem termica
                void imageCallback(const sensor_msgs::ImageConstPtr& msg)
                {
//                        camTermicaModelo_.fromCameraInfo(info_msg);
                        cv::Mat img_g;
                        cv::Mat imagem_termica_cor;
                        imgMsg_ = msg;
                        imgCv_ = cv_bridge::toCvShare(msg, "bgr8")->image;
                        cv::cvtColor(imgCv_, img_g, CV_BGR2GRAY);
                        cv::applyColorMap(img_g, imagem_termica_cor, cv::COLORMAP_JET);
                        imgCv_ = imagem_termica_cor;
                } // fim imageCallback()


                // Callback da imagem termica - Recebe imagem termica
               /* void imageCallback(const sensor_msgs::ImageConstPtr& msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg)
		{
//                        camTermicaModelo_.fromCameraInfo(info_msg);
                        cv::Mat img_g;
                        cv::Mat imagem_termica_cor;
			imgMsg_ = msg;			
                        imgCv_ = cv_bridge::toCvShare(msg, "bgr8")->image;
                        cv::cvtColor(imgCv_, img_g, CV_BGR2GRAY);
                        cv::applyColorMap(img_g, imagem_termica_cor, cv::COLORMAP_JET);
                        imgCv_ = imagem_termica_cor;
                } // fim imageCallback()*/



		// Reconstrucao Termica	
		void reconstrucao()
                {
                    cloudTransformada_ = cloud_;
                    sensor_msgs::PointCloud2 msg_out;
                    sensor_msgs::PointCloud2 visual_out;

                // Projetando os pontos para a imagem
                    float erroTotal = 0;
                        for(int i = 0; i < nPontos_; i++)
                        {
                            cv::Point3d ponto3D;
                            cv::Point2d pontoProjetado;
                            ponto3D.x = cloudTransformada_.points[i].x;
                            ponto3D.y = cloudTransformada_.points[i].y;
                            ponto3D.z = cloudTransformada_.points[i].z;
                            pontoProjetado = camTermicaModelo_.project3dToPixel(ponto3D);
                            //std::cout << pontoProjetado << "\n";

                            // Verificar se a projecao esta dentro da imagem
                            if(pontoProjetado.x > 0  && pontoProjetado.x < imgCv_.cols && pontoProjetado.y > 0 && pontoProjetado.y < imgCv_.rows)
                            {
                                int b = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[0];
                                int g = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[1];
                                int r = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[2];

                                erroTotal += std::abs(b-cloudTransformada_.points[i].b) + std::abs(g-cloudTransformada_.points[i].g) + std::abs(r-cloudTransformada_.points[i].r);

                                cloudTransformada_.points[i].b = b;
                                cloudTransformada_.points[i].g = g;
                                cloudTransformada_.points[i].r = r;
                            }
                            else
                            {
                                /*cloudTransformada_.points[i].b = 0;
                                cloudTransformada_.points[i].g = 0;
                                cloudTransformada_.points[i].r = 0;*/
                            }
                        }

                        //ROS_INFO_STREAM("Erro total: " + std::to_string(erroTotal) + "\n");
                        pcl::toROSMsg (cloudTransformada_, msg_out);
                        pcl::toROSMsg (cloud_, visual_out);

                        pc_pub_.publish(msg_out);
                        pc_visual_pub_.publish(visual_out);


                        // Limpar pontos que nao tenham reprojecao (TODO)
                        // ...............................................

                } // fim reconstrucao()



                // Recebe imagem e point cloud, em seguida, processa info
                void spinTermica()
                {
                    ros::spinOnce();
                    reconstrucao();
                } // fim spinTermica()

}; // fim classe reconstrucaoTermica





// Funcao main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "reconstrucao_tridimensional_termica");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");
    ROS_INFO_STREAM("Reconstrucao Tridimensional Termica Iniciada.");
    std::string topico_imagem;
    std::string termicaCalibrationYAML;
    std::string topico_pc;
    std::string topico_out;

    n_.getParam("topico_imagem", topico_imagem);
    n_.getParam("topico_out", topico_out);
    n_.getParam("topico_pc", topico_pc);
    n_.getParam("termica_calibration_yaml", termicaCalibrationYAML);
    termicaCalibrationYAML = termicaCalibrationYAML + std::string(".yaml");

    reconstrucaoTermica rT(nh, termicaCalibrationYAML, topico_imagem, topico_pc, topico_out);

    ros::Rate loop_rate(10);
    while ( ros::ok() )
    {
        rT.spinTermica();
        loop_rate.sleep();
    }

} // fim main()


