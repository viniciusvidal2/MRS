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

//// Descricao -- Reconstrucao termica online ////////////////////////////////////////////////////////////////
// 00. Le arquivos de calibracao da camera visual e camera termica (.yaml);
// 01. Le arquivos contendo a point cloud acumulada RGB (.ply);
// 02. Le arquivos contendo as rotacoes e translacoes da camera termica para cada frame (.txt);
// 03. Varrer point cloud e projetar esses pontos nas imagens termicas. Acumular todas as intensidades
//projetadas e aplicar diferentes tecnicas (minimo das intensidades, maximo das intensidades, media das
//intensidades, etc;
// 04. Gravar point cloud termica em um arquivo (.ply).
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Definicoes
#define TERMICA_YAML_DIR "file:///home/felipe/catkin_ws/src/termica_reconstrucao/Dados/calibracao/termica.yaml"
#define VISUAL_YAML_DIR  "file:///home/felipe/catkin_ws/src/termica_reconstrucao/Dados/calibracao/right.yaml"
#define TERMICA_POSE_DIR "/home/felipe/catkin_ws/src/termica_reconstrucao/Dados/termica_pose/termica_pose.txt"
#define CAMERA_TERMICA_IMG_DIR "/home/felipe/catkin_ws/src/termica_reconstrucao/Dados/imagensTermicas"
#define PLY_OUTPUT_FILE_DIR "/home/felipe/catkin_ws/src/termica_reconstrucao/Dados/ply_output"
#define PLY_INPUT_FILE_DIR "/home/felipe/catkin_ws/src/termica_reconstrucao/Dados/ply_input/bunny.ply"
typedef pcl::PointXYZRGB PointT;


// Variaveis globais
image_geometry::PinholeCameraModel cam_termica_modelo;
//camera_info_manager::CameraInfoManager cam_termica;
sensor_msgs::CameraInfo termica_info;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Contar numero de arquivos em um diretorio
int count(std::string directory, std::string ext)
{
        namespace fs = boost::filesystem;
        fs::path Path(directory);
        int Nb_ext = 0;
        fs::directory_iterator end_iter;

        for (fs::directory_iterator iter(Path); iter != end_iter; ++iter)
                if (iter->path().extension() == ext)
                        ++Nb_ext;

        return Nb_ext;
}

// Tecnica aplicada as intensidades projetadas
int intensidadeTermicaFunc(int intensidadesProjetadas[], int tecnicaEscolhida)
{

    // Dado o vetor de intensidades projetadas, escolher a tecnica utilizada para gerar a inteisdade
    // termica final.
    int intensidadeFinal=100;
    int nIntensidadesProjetadas = int ( sizeof(intensidadesProjetadas)/sizeof(intensidadesProjetadas[0])  );

    // Verificar se o vetor esta vazio
    if (nIntensidadesProjetadas == 0)
        return -1;

    int media=0;
    int acumulador = 0;
    int maximo = -100;
    int minimo = 500;

    for(int iterator = 0; iterator < nIntensidadesProjetadas; iterator++)
    {
         acumulador = acumulador + intensidadesProjetadas[iterator];

         if(intensidadesProjetadas[iterator] > maximo)
             maximo = intensidadesProjetadas[iterator];

         if(intensidadesProjetadas[iterator] < minimo)
             minimo = intensidadesProjetadas[iterator];
    }
    media = int(acumulador/nIntensidadesProjetadas);


    switch(tecnicaEscolhida){
        case 0:
            return media;
            break;

        case 1:
            return maximo;
            break;

        case 2:
            return minimo;
            break;
    }

    return 1;
}

void atualizarInfoCameraTermica(int nImagemTermica)
{
    // Le arquivo .txt contendo a pose da camera termica

    //ROS_INFO_STREAM("atualizar\n");
    std::ifstream file_input ("/home/felipe/catkin_ws/src/termica_reconstrucao/Dados/termica_pose/termica_pose.txt");
    float aux;
    std::string line;
    int nLinha = 0;
    std::getline(file_input, line);

    while( nLinha!= nImagemTermica && std::getline(file_input, line))
    {
        ++nLinha;
    }

    std::stringstream ss(line);
    ss >> termica_info.R[0] >> termica_info.R[1] >> termica_info.R[2] >> termica_info.R[3] >> termica_info.R[4]>> termica_info.R[5]>> termica_info.R[6]>> termica_info.R[7]>> termica_info.R[8]>> termica_info.P[3]>> termica_info.P[7]>> termica_info.P[11];

}


pcl::PointCloud<PointT> gerarPointCloudTermica(ros::NodeHandle nh, pcl::PointCloud<PointT> cloudVisual)
{
    ROS_INFO_STREAM("Inicio: reconstrucao tridimensional termica" << "\n");

    pcl::PointCloud<PointT> cloudTermica;
    cloudTermica = cloudVisual;

    // Zerando as cores da point cloud termica
    for(int j = 0; j < cloudTermica.points.size(); j++)
    {
        cloudTermica.points[j].r = -1;
        cloudTermica.points[j].g = -1;
        cloudTermica.points[j].b = -1;
    }

    ROS_INFO_STREAM("Reprojetando pontos 3D nas imagens termica");
    int iteradorImagens = 0;
    int iteradorPontos = 0;
    int nImagensTermicas = count(CAMERA_TERMICA_IMG_DIR,".jpg");
    int vetorIntensidades[nImagensTermicas];
    int nPontos = int(cloudVisual.points.size());
    cv::Point3d ponto3D;
    cv::Point2d pontoProjetado;
    int intensidadeTermica;
    cv::Mat imagemTermica;

    for(iteradorPontos = 0; iteradorPontos < nPontos; iteradorPontos++)
    {
        for(iteradorImagens = 0; iteradorImagens < nImagensTermicas; iteradorImagens++)
        {
            // Lendo imagem termica
            imagemTermica = cv::imread(CAMERA_TERMICA_IMG_DIR, 0);

            // Construindo ponto 3D
            ponto3D.x = cloudVisual.points[iteradorPontos].x;
            ponto3D.y = cloudVisual.points[iteradorPontos].y;
            ponto3D.z = cloudVisual.points[iteradorPontos].z;

            // Projetando ponto na imagem
            atualizarInfoCameraTermica(iteradorImagens);
            pontoProjetado = cam_termica_modelo.project3dToPixel(ponto3D);

            // Se a projecao estiver dentro da image, pegar intensidade projetada
            if(pontoProjetado.x < imagemTermica.cols && pontoProjetado.y < imagemTermica.rows)
            {
                intensidadeTermica = imagemTermica.at<int>(int(pontoProjetado.y), int(pontoProjetado.x));
                vetorIntensidades[iteradorImagens] = intensidadeTermica;
            }
            else
            {
                intensidadeTermica = -1;
            }
        }

        // Associar intensidade termica ao ponto tridimensional aplicando alguma tecnica
        int intensidadeFinal;
        intensidadeFinal = intensidadeTermicaFunc(vetorIntensidades, 0);
        cloudTermica.points[iteradorPontos].r = intensidadeFinal;
        cloudTermica.points[iteradorPontos].g = intensidadeFinal;
        cloudTermica.points[iteradorPontos].b = intensidadeFinal;
    }

    return cloudTermica;
}


void gravarPointCloud(pcl::PointCloud<PointT> cloud)
{
    // Gravar point cloud em um arquivo .ply
    pcl::io::savePLYFileASCII(PLY_OUTPUT_FILE_DIR, cloud);
}


pcl::PointCloud<PointT> lerPointCloud()
{
    // Ler point cloud a partir de um arquivo .ply
    pcl::PointCloud<PointT> cloud;
    pcl::PLYReader Reader;
    Reader.read(PLY_INPUT_FILE_DIR, cloud);
    return cloud;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Funcao main
int main(int argc, char** argv)
{
        ros::init(argc, argv, "reconstrucao_termica_offline");
        ros::NodeHandle nh;
        ROS_INFO("Inicio -- Reconstrucao Termica! \n");

        // Lendo informacoes da camera termica [v]
        ROS_INFO_STREAM("Lendo informacoes da camera termica.\n");
        std::string cameraTermicaNum = std::string(TERMICA_YAML_DIR) + "termica.yaml";
        camera_info_manager::CameraInfoManager ct(nh,"camera_termica",cameraTermicaNum);
        termica_info = ct.getCameraInfo();
        cam_termica_modelo.fromCameraInfo(ct.getCameraInfo());

        //atualizarInfoCameraTermica(0);
        // Ler point cloud visual [v]
        pcl::PointCloud<PointT> cloudVisual = lerPointCloud();

        // Gerar point cloud termica [v]
        //pcl::PointCloud<PointT> cloudTermica = gerarPointCloudTermica(nh, cloudVisual);

        // Gravar point cloud termica em um arquivo [v]
        //gravarPointCloud(cloudTermica);

        while (ros::ok())
	{
            ros::spinOnce();
                // ...........................................................................................
  	}


}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////
