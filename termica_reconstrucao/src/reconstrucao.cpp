/////////////////////////////////////////////////////////////////////
//////////////// Reconstrucao Termica //////////////////////////////
////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

// Bibliotecas
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <camera_info_manager/camera_info_manager.h>
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
#include<pcl/io/ply_io.h>
//#include <filesystem>
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include<stdio.h>
#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

// Definicoes
typedef pcl::PointXYZRGB PointT;

// Classe reconstrucaoTermica
class reconstrucaoTermica
{
	protected:
		ros::NodeHandle nh_;
                image_transport::Subscriber image_sub_;
                image_transport::ImageTransport it_;
		std::string point_cloud_PLY_;
                tf::TransformListener tf_listener_;

	public:
		pcl::PointCloud<PointT> cloud_;
		sensor_msgs::ImageConstPtr imgMsg_;
                cv::Mat imgCv_;
		pcl::PointCloud<PointT> cloudTransformada_;
		tf::StampedTransform transform_;
                image_geometry::PinholeCameraModel camTermicaModelo_;
                camera_info_manager::CameraInfoManager camInfo_;
                int nImagens_;
                int nPontos_;

                float* intensidadeMinima;
                float* intensidadeMaxima;
                float* intensidadeMedia;

		// Construtor reconstrucaoTermica
                reconstrucaoTermica(ros::NodeHandle nh_, std::string pointCloudPLY, std::string termicaCalibrationYAML, std::string topico_imagem)
                : it_(nh_), camInfo_(nh_,"left_optical",termicaCalibrationYAML)
		{
                        //ROS_INFO_STREAM("1.\n");
                        image_sub_ = it_.subscribe(topico_imagem, 1, &reconstrucaoTermica::imageCallback, this);
                        //ROS_INFO_STREAM("1.1.\n");
                        //camTermicaModelo_(nh_,"camera_termica",termicaCalibrationYAML);
			point_cloud_PLY_ = pointCloudPLY;
                        //ROS_INFO_STREAM("1.2.\n");
			lerPointCloud();
                        //ROS_INFO_STREAM("1.3.\n");
                        camTermicaModelo_.fromCameraInfo(camInfo_.getCameraInfo());
                        //ROS_INFO_STREAM("2.\n");
                        nPontos_ = int(cloud_.points.size());
                        nImagens_ = 0;
                        intensidadeMaxima = new float[nPontos_];
                        intensidadeMinima = new float[nPontos_];
                        intensidadeMedia  = new float[nPontos_];

                        ROS_INFO_STREAM("nPontos = " << nPontos_ << "\n");
                        // Inicializar vetores
                        for(int i = 0; i < nPontos_; i++)
                        {
                            intensidadeMaxima[i] = -1;
                            intensidadeMinima[i] = 1000;
                            intensidadeMedia[i] = 0;
                        }
                } //fim reconstrucaoTermica()
	

		// Callback da imagem termica - Recebe imagem termica e a transformada /world -> /left_optical
		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
                        //ROS_INFO_STREAM("Nova imagem. \n");
			imgMsg_ = msg;			
                        nImagens_++;
                        imgCv_ = cv_bridge::toCvShare(msg, "bgr8")->image;
                        /* pegando transformada da point cloud de /map para /left_optical
    			try{
                                tf_listener_.lookupTransform(ros::names::remap("/left_optical"), ros::names::remap("/map"), ros::Time(0), transform_);
    			   }

    			catch (tf::TransformException ex){
      				ROS_ERROR("%s",ex.what());
                                ros::Duration(5.0).sleep();
                         }*/
                } // fim imageCallback()


		// Leitor da point cloud de um arquivo .ply
                void lerPointCloud()
		{
                        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());;
                        pcl::PointCloud<PointT> cloud;

                    //pcl::PLYReader Reader;
                        //ROS_INFO_STREAM("teste1.\n");
                        //Reader.read(point_cloud_PLY_, cloud);
                        //pcl::io::loadPLYFile(std::string("/home/felipe/catkin_ws/src/termica_reconstrucao/Dados/ply_input/nuvem.ply"), *cloud);
                        pcl::io::loadPCDFile<PointT>("/home/felipe/catkin_ws/src/termica_reconstrucao/Dados/ply_input/completo.pcd", cloud);
                        //ROS_INFO_STREAM("teste2.\n");
                        //pcl::copyPointCloud(*cloud, cloud_);
                        cloud_ = cloud;
                } // fim lerPointCloud()


		// Reconstrucao Termica	
		void reconstrucao()
                {

		// Transformando os pontos das coordenadas globais(/world) para as coordenadas da camera esquerda(/left_optical)
                        //pcl_ros::transformPointCloud(cloud_, cloudTransformada_, transform_); // Cloud ja esta referenciada no left_optical
                cloudTransformada_ = cloud_;
                int pp=0;

                // Projetando os pontos para a imagem
                        for(int i = 0; i < nPontos_; i++)
                        {
                            cv::Point3d ponto3D;
                            cv::Point2d pontoProjetado;
                            float intensidadeTermica;
                            ponto3D.x = cloudTransformada_.points[i].x;
                            ponto3D.y = cloudTransformada_.points[i].y;
                            ponto3D.z = cloudTransformada_.points[i].z;
                            pontoProjetado = camTermicaModelo_.project3dToPixel(ponto3D);
                            //ROS_INFO_STREAM("i: " << i << "\n" );

                            /*ROS_INFO_STREAM("Matriz: " << camTermicaModelo_.fullProjectionMatrix() << "\n");
                            ROS_INFO_STREAM("ponto3d:" << ponto3D << "\n");
                            ROS_INFO_STREAM("projetado:" << pontoProjetado << "\n");
                            ROS_INFO_STREAM("-------------------\n");*/
                            // Verificar se a projecao esta dentro da imagem
                            if(std::abs(pontoProjetado.x) < imgCv_.cols && std::abs(pontoProjetado.y) < imgCv_.rows)
                            {
                                pp++;
                                //ROS_INFO_STREAM("teste1\n");
                                ROS_INFO_STREAM("x: " << int(pontoProjetado.y) << " e y: " << int(pontoProjetado.x) << "\n" );
                                intensidadeTermica = imgCv_.at<int>(int(pontoProjetado.y), int(pontoProjetado.x));
                                //ROS_INFO_STREAM("teste2\n");

                                if(intensidadeTermica > intensidadeMaxima[i])
                                    intensidadeMaxima[i] = intensidadeTermica;

                                if(intensidadeTermica <  intensidadeMinima[i])
                                    intensidadeMinima[i]= intensidadeTermica;

                                intensidadeMedia[i]+=intensidadeTermica;
                            }
                        }
                        ROS_INFO_STREAM("p: " << pp << "\n");
                       // ROS_INFO_STREAM("Fim reconstrucao.\n");
                } // fim reconstrucao()


                // Apos capturar as intensidades para todas as imagens termicas, definir qual sera a intensidade final
                void definirIntensidades(int metodo)
                {
                    //ROS_INFO_STREAM("Definindo intensidades de temperatura \n");
                    std::cout << "Definindo as intensidades \n";
                    for(int i = 0; i < nPontos_; i++)
                    {
                        switch (metodo) {
                        case 0: // maximo
                            cloudTransformada_.points[i].r = intensidadeMaxima[i];
                            cloudTransformada_.points[i].g = intensidadeMaxima[i];
                            cloudTransformada_.points[i].b = intensidadeMaxima[i];
                            break;

                        case 1: // minimo
                            if(intensidadeMinima[i] != 1000)
                            {
                                cloudTransformada_.points[i].r = intensidadeMinima[i];
                                cloudTransformada_.points[i].g = intensidadeMinima[i];
                                cloudTransformada_.points[i].b = intensidadeMinima[i];
                            }
                            else{
                                cloudTransformada_.points[i].r = -1;
                                cloudTransformada_.points[i].g = -1;
                                cloudTransformada_.points[i].b = -1;
                            }

                            break;

                        case 2: // media
                            cloudTransformada_.points[i].r = intensidadeMedia[i]/float(nImagens_);
                            cloudTransformada_.points[i].g = intensidadeMedia[i]/float(nImagens_);
                            cloudTransformada_.points[i].b = intensidadeMedia[i]/float(nImagens_);
                            break;
                        default:
                            cloudTransformada_.points[i].r = intensidadeMedia[i]/float(nImagens_);
                            cloudTransformada_.points[i].g = intensidadeMedia[i]/float(nImagens_);
                            cloudTransformada_.points[i].b = intensidadeMedia[i]/float(nImagens_);
                            break;
                        }


                    }

                    // Limpar pontos que tenham intensidade = -1 (TODO)


                    // Salvando point cloud num arquivo .pcd
                    time_t t = time(0);
                    struct tm * now = localtime( & t );
                    std::string year, month, day, hour, minutes;
                    year    = boost::lexical_cast<std::string>(now->tm_year + 1900);
                    month   = boost::lexical_cast<std::string>(now->tm_mon );
                    day     = boost::lexical_cast<std::string>(now->tm_mday);
                    hour    = boost::lexical_cast<std::string>(now->tm_hour);
                    minutes = boost::lexical_cast<std::string>(now->tm_min );
                    std::string date = "_" + year + "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m";
                    std::string filename = "/home/felipe/Desktop/out_"+date;
                    std::string filename2 = "/home/felipe/Desktop/filePCD_out_"+date;
                    //pcl::io::savePLYFileASCII(filename, *output_cloud_ptr);
                    pcl::io::savePCDFileASCII(filename2, cloudTransformada_);


                } // fim definirIntensidades()


                // Recebe imagem e processa
                void spinTermica()
                {
                    ros::spinOnce();
                    reconstrucao();
                } // fim spinTermica()

}; // fim classe reconstrucaoTermica




/*
int getch() // capturar caracter no terminal
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON);
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);

  int c = getchar();

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
  return c;
} // fim getch()*/









int main(int argc, char **argv) // -- Funcao main
{
    ros::init(argc, argv, "reconstrucao_tridimensional_termica");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");
    ROS_INFO_STREAM("Reconstrucao Tridimensional Termica Iniciada.");
    // Capturando parametros
    std::string topico_imagem = "/stereo/left/image_raw";
    std::string termicaCalibrationYAML = "file:///home/felipe/catkin_ws/src/termica_reconstrucao/Dados/calibracao/termica.yaml";
    std::string pointCloudPLY = "/home/felipe/catkin_ws/src/termica_reconstrucao/Dados/ply_input/nuvem.ply";

    /*n_.getParam("topico_imagem", topico_imagem);
    n_.getParam("termica_calibration_yaml", termicaCalibrationYAML);
    n_.getParam("point_cloud_ply", pointCloudPLY);
    termicaCalibrationYAML = termicaCalibrationYAML + std::string(".yaml");
    pointCloudPLY = pointCloudPLY + std::string(".ply");*/

    reconstrucaoTermica rT(nh, pointCloudPLY, termicaCalibrationYAML, topico_imagem);
    int c;

    ros::Rate loop_rate(10);
    ROS_INFO_STREAM("Pressione s para finalizar a reconstrucao termica.\n");
    while ( ros::ok() )
    {
        rT.spinTermica();
        loop_rate.sleep();

        /*c = getch();
        if(c == 's')
        {
            rT.definirIntensidades(0); // media das intensidades
            break;
        }*/

    }
    rT.definirIntensidades(0);


} // fim main()


