/////////////////////////////////////////////////////////////////////
//////////////// Reconstrucao Termica 3D////////////////////////////
////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////




// Bibliotecas
#define PCL_NO_PRECOMPILE
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
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>


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

/*
struct PointC
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float r;
    float g;
    float b;
    float l;
    float o;
    float p;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
    } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

    POINT_CLOUD_REGISTER_POINT_STRUCT (PointT,           // here we assume a XYZ + "test" (as fields)
                                     (float, x, x)
                                     (float, y, y)
                                     (float, z, z)
                                     (float, r, r)
                                     (float, g, g)
                                     (float, b, b)
                                     (float, l, l)
                                     (float, o, o)
                                     (float, p, p)
)*/


// Definicoes
//typedef pcl::PointCloud<MyPointType> PointT; //PointXYZRGBA
//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointXYZRGBA PointC;

typedef pcl::PointXYZRGB PointT;



// Classe reconstrucaoTermica
class reconstrucaoTermica
{
	protected:
		ros::NodeHandle nh_;
                //image_transport::Subscriber image_sub_;
                image_transport::CameraSubscriber image_sub_;
                image_transport::ImageTransport it_;
		std::string point_cloud_PLY_;
                ros::Publisher pc_pub_;
                ros::Publisher pc_visual_pub_;
                ros::Publisher pc_completa_pub_;


	public:
                ros::Subscriber pc_sub_;
		pcl::PointCloud<PointT> cloud_;
                pcl::PointCloud<PointC> cloudCompleta_;
		sensor_msgs::ImageConstPtr imgMsg_;
                cv::Mat imgCv_;
                cv::Mat imgGray_;
		pcl::PointCloud<PointT> cloudTransformada_;
                image_geometry::PinholeCameraModel camTermicaModelo_;
                camera_info_manager::CameraInfoManager camInfo_;
                sensor_msgs::PointCloud2 completa_out;
                std::string id_;
                int nPontos_;



		// Construtor reconstrucaoTermica
                reconstrucaoTermica(ros::NodeHandle nh_, std::string termicaCalibrationYAML, std::string topico_imagem, std::string topico_pc, std::string topico_out)
                : it_(nh_), camInfo_(nh_,"termica",termicaCalibrationYAML)
		{
                        //image_sub_ = it_.subscribe(topico_imagem, 1, &reconstrucaoTermica::imageCallback, this);
                        image_sub_ = it_.subscribeCamera(topico_imagem, 1, &reconstrucaoTermica::imageCallback, this);
                        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (topico_out, 1000);
                        pc_visual_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("termica/visual_pc", 1000);
                        pc_completa_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/completa_pc", 1000);
                        pc_sub_= nh_.subscribe(topico_pc, 1, &reconstrucaoTermica::pointCloudCallback, this);
                        camTermicaModelo_.fromCameraInfo(camInfo_.getCameraInfo());
                } //fim reconstrucaoTermica()
	


                // Callback point cloud
                void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
                {
                    id_ = msg->header.frame_id;
                    pcl::fromROSMsg(*msg, cloud_);
                    std::vector<int> indicesNAN;
                    pcl::removeNaNFromPointCloud(cloud_, cloud_, indicesNAN);
                    nPontos_ = int(cloud_.points.size());

                } // fim pointCloudCallback()


                // Callback da imagem termica - Recebe imagem termica
                /*void imageCallback(const sensor_msgs::ImageConstPtr& msg)
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


                // Callback da imagem termica - Recebe imagem termica
               void imageCallback(const sensor_msgs::ImageConstPtr& msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg)
		{
                        sensor_msgs::CameraInfo a = *info_msg; //->R[0] = 0;
                        /*a.R[0] = 1;
                        a.R[1] = 0;
                        a.R[2] = 0;
                        a.R[3] = 0;
                        a.R[4] = 1;
                        a.R[5] = 0;
                        a.R[6] = 0;
                        a.R[7] = 0;
                        a.R[8] = 1;

                        a.D[0] = -0.52910;
                        a.D[1] = 2.240600;
                        a.D[2] = 0;
                        a.D[3] = 0;
                        a.D[4] = 0;

                        a.K[0] = 800;
                        a.K[1] = 0;
                        a.K[2] = -15;
                        a.K[3] = 0;
                        a.K[4] = 800;
                        a.K[5] = 40;
                        a.K[6] = 0;
                        a.K[7] = 0;
                        a.K[8] = 1;*/

                        a.P[0] = 1400;   // fx (800)
                        a.P[1] = 0;     // 0  (0)
                        a.P[2] = 70;   // cx (-15)
                        a.P[3] = -50;   // Tx (-35)
                        a.P[4] = 0;     // 0  (0)
                        a.P[5] = 1400;   // fy (800)
                        a.P[6] = 270;    // cy (40)
                        a.P[7] = -8.16; // Ty (-8.16)
                        a.P[8] = 0;     // 0  (0)
                        a.P[9] = 0;     // 0  (0)
                        a.P[10] = 1;    // 1  (1)
                        a.P[11] = 0;    // 0  (0)



                        camTermicaModelo_.fromCameraInfo(a);
                        cv::Mat img_g;
                        cv::Mat imagem_termica_cor;
			imgMsg_ = msg;			
                        imgCv_ = cv_bridge::toCvShare(msg, "bgr8")->image;
                        cv::cvtColor(imgCv_, img_g, CV_BGR2GRAY);
                        imgGray_ = img_g;
                        cv::applyColorMap(img_g, imagem_termica_cor, cv::COLORMAP_JET);
                        imgCv_ = imagem_termica_cor;
                } // fim imageCallback()



		// Reconstrucao Termica	
		void reconstrucao()
                {
                    cloudTransformada_ = cloud_;

                    if(nPontos_ > 0)
                    {
                        //std::cout << nPontos_;
                        cloudCompleta_.points.resize (nPontos_);
                    }
                    sensor_msgs::PointCloud2 msg_out;
                    sensor_msgs::PointCloud2 visual_out;

                // Projetando os pontos para a imagem
                    //float erroTotal = 0;
                    //std::cout << "size: " << nPontos_ << "\n";
                        for(int i = 0; i < nPontos_; i++)
                        {
                            cv::Point3d ponto3D;
                            cv::Point2d pontoProjetado;
                            ponto3D.x = cloudTransformada_.points[i].x;
                            ponto3D.y = cloudTransformada_.points[i].y;
                            ponto3D.z = cloudTransformada_.points[i].z;
                            //
                            /*
                            std::cout << "x: " << cloud_.points[i].x << "\n";
                            std::cout << "y: " << cloud_.points[i].y << "\n";
                            std::cout << "z: " << cloud_.points[i].z << "\n";
                            std::cout << "r: " << cloud_.points[i].r << "\n";
                            std::cout << "g: " << cloud_.points[i].g << "\n";
                            std::cout << "b: " << cloud_.points[i].b << "\n";
                            std::cout << "a: " << cloud_.points[i].a << "\n";*/
                            ///*
                            cloudCompleta_.points[i].x = cloudTransformada_.points[i].x;
                            cloudCompleta_.points[i].y = cloudTransformada_.points[i].y;
                            cloudCompleta_.points[i].z = cloudTransformada_.points[i].z;
                            cloudCompleta_.points[i].b = cloudTransformada_.points[i].b;
                            cloudCompleta_.points[i].g = cloudTransformada_.points[i].g;
                            cloudCompleta_.points[i].r = cloudTransformada_.points[i].r;//*/

                            pontoProjetado = camTermicaModelo_.project3dToPixel(ponto3D);
                            //std::cout << pontoProjetado << "\n";

                            // Verificar se a projecao esta dentro da imagem
                            if(pontoProjetado.x > 0  && pontoProjetado.x < imgCv_.cols && pontoProjetado.y > 0 && pontoProjetado.y < imgCv_.rows)
                            {
                                int b = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[0];
                                int g = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[1];
                                int r = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[2];
                                int gray = imgGray_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[0];

                                //erroTotal += std::abs(b-cloudTransformada_.points[i].b) + std::abs(g-cloudTransformada_.points[i].g) + std::abs(r-cloudTransformada_.points[i].r);

                                cloudTransformada_.points[i].b = b;
                                cloudTransformada_.points[i].g = g;
                                cloudTransformada_.points[i].r = r;

                                //cloudCompleta_.points[i].a = gray;
                                cloudCompleta_.points[i].l = r;
                                cloudCompleta_.points[i].o = g;
                                cloudCompleta_.points[i].p = b;
                            }
                            else
                            {
                                /*cloudTransformada_.points[i].b = 40;
                                cloudTransformada_.points[i].g = 40;
                                cloudTransformada_.points[i].r = 40;*/
                                //cloudCompleta_.points[i].a = -1;
                                cloudCompleta_.points[i].l = nan("");;
                                cloudCompleta_.points[i].o = nan("");;
                                cloudCompleta_.points[i].p = nan("");;
                            }


                        }

                        //ROS_INFO_STREAM("Erro total: " + std::to_string(erroTotal) + "\n");
                        pcl::toROSMsg (cloudTransformada_, msg_out);
                        pcl::toROSMsg (cloud_, visual_out);
                        pcl::toROSMsg (cloudCompleta_, completa_out);
                        //std::cout << "wid: " << cloudTransformada_.width << ", hei: " <<cloudTransformada_.height << "\n";


                        pc_pub_.publish(msg_out);
                        pc_visual_pub_.publish(visual_out);
                        completa_out.header.frame_id = id_;
                        completa_out.header.stamp = ros::Time::now();
                        pc_completa_pub_.publish(completa_out);


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


