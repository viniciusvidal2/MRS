#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

using namespace pcl;
using namespace std;

typedef PointXYZRGB PointT;

PointCloud<PointT>::Ptr portal, portal_acumulado;
ros::Publisher pub;

bool primeira_vez;
Eigen::Vector3d t_prev;

void escuta_odometria(const nav_msgs::OdometryConstPtr &msg_odom)
{
    if(!primeira_vez){

        // Pega odometria
        Eigen::Quaternion<double> q;
        q.x() = (double)msg_odom->pose.pose.orientation.x;
        q.y() = (double)msg_odom->pose.pose.orientation.y;
        q.z() = (double)msg_odom->pose.pose.orientation.z;
        q.w() = (double)msg_odom->pose.pose.orientation.w;
        Eigen::Vector3d t_atual(msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z);

        // Vetor z como esta segundo quaternion (para frente no frame da da camera)
        Eigen::Vector3d vetor_z(0, 0, 1);
        vetor_z = q.matrix()*vetor_z;

        // Deslocamento relativo para a iteracao passada
        Eigen::Vector3d deslocamento = t_atual - t_prev;

        // Rotacao relativa para casar com sentido do deslocamento
        Eigen::Quaternion<double> q_rel;
        q_rel.setFromTwoVectors(vetor_z, deslocamento);

        // Transforma a nuvem para o frame desejado
        PointCloud<PointT>::Ptr portal_trans (new PointCloud<PointT>());
        portal_trans->header.frame_id = "odom";
        transformPointCloud<PointT>(*portal, *portal_trans, t_atual, q*q_rel);

        // Acumula depois de filtrar um pouco para reduzir tamanho talvez
        *portal_acumulado += *portal_trans;

        // Publica a nuvem
        sensor_msgs::PointCloud2 portal_msg;
        portal_msg.header.frame_id = "odom";
        toROSMsg(*portal_acumulado, portal_msg);
        pub.publish(portal_msg);

        portal_trans->clear();
        t_prev = t_atual;

    } else {

        // Armazena posicao anterior para saber orientacao no futuro
        t_prev << msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z;

    }


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "regiao_limite");
    ros::NodeHandle nh;

    // Iniciar nuvem do portal
    portal           = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    portal_acumulado = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    portal->header.frame_id = "odom";
    portal_acumulado->header.frame_id = "odom";
    // Iniciar publicador
    pub = nh.advertise<sensor_msgs::PointCloud2>("/portal", 10);
    /// Criar a nuvem de pontos constante do portal
    float comp_altura = 2, comp_largura = 1, comp_profundidade = 2, amostras = 10;
    PointT ponto;
    ponto.r = 250; ponto.g = 0; ponto.b = 0;
    float coord_x_1 = -comp_largura/2, coord_x_2 = comp_largura/2;
    float coord_y_barra = -comp_altura;

    for(int z=0; z<1; z++){
        float coord_z = 0;// - comp_profundidade/2 + z*comp_profundidade/amostras;
         ponto.z = coord_z;
        // Laterais
        for(int y=0; y<amostras; y++){
            float coord_y = 0 - y*comp_altura/amostras;
            ponto.y = coord_y;
            ponto.x = coord_x_1;
            portal->push_back(ponto);
            ponto.x = coord_x_2;
            portal->push_back(ponto);
        }
        // Trave superior
        ponto.y = coord_y_barra;
        for(int x=0; x<amostras; x++){
            float coord_x = 0 - comp_largura/2 + x*comp_largura/amostras;
            ponto.x = coord_x;
            portal->push_back(ponto);
        }
    }

    ros::Subscriber sub = nh.subscribe("/stereo_odometer/odometry", 10, escuta_odometria);

    ros::Rate r(1);
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
