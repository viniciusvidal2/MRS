#include "../include/monitor_mrs/GigeImageReader.hpp"
#include <QTime>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <sstream>
#include <string>
#include <QStringListModel>
#include <QFuture>
#include <QtConcurrentRun>

namespace monitor_mrs {

using namespace std;

GigeImageReader::GigeImageReader(int argc, char **argv, QMutex *nmutex):init_argc(argc),
  init_argv(argv),mutex(nmutex)
{

  QFuture<void> future = QtConcurrent::run(this, &GigeImageReader::init);

}

GigeImageReader::~GigeImageReader()
{
  if(ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }
wait();
}

void GigeImageReader::init(){

  cout << "iniciando no gige_reader" << endl;

  ros::init(init_argc,init_argv,"Gige_reader");

  if ( ! ros::master::check() )  {
    cout << "check ros master not good" << endl;
    return;
  }

  ros::start();

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_= image_transport::ImageTransport(nh_);

  //image_sub_ = it_.subscribe("/stereo/left/image_color", 1, &GigeImageReader::imageCb, this);
  image_sub_ = it_.subscribe("/stereo/left/image_raw", 1, &GigeImageReader::imageCb, this);
  offset_pub      = nh_.advertise<std_msgs::Int8>("offset_pub", 100);
  offset_tilt_pub = nh_.advertise<std_msgs::Int8>("offset_tilt_pub", 100);
  offset = 0; // a ser publicado para alterar pan do motor
  offset_tilt = 0; // a ser publicado para alterar tilt do motor
  sub_estamosdentro = nh_.subscribe("/estamos_dentro", 10, &GigeImageReader::estamosdentroCb, this);

  estado_anterior_gravar = 0.0f;

  ros::spin();

}

void GigeImageReader::imageCb(const sensor_msgs::ImageConstPtr &msg)
{

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  mutex->lock();
  qint64 timestamp = QDateTime().currentMSecsSinceEpoch();
  send_mat_image(cv_ptr->image,timestamp);
  // Enviar com uma certa frequencia aqui o offset para deslocar o motor
  msg_off.data = offset;
  msg_off_tilt.data = offset_tilt;
  offset_pub.publish(msg_off);
  offset_tilt_pub.publish(msg_off_tilt);
  mutex->unlock();

  // Update GUI Window
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //cv::waitKey(3);
}

void GigeImageReader::estamosdentroCb(const std_msgs::Float32 &msg){
  // A mensagem fala se estamos dentro ou nao, mas chamaremos somente na transicao entre a entrada e a saida.
  // Portanto, pegaremos o chaveamento armazenando o estado anterior.
  if(stereo_funcionando){ // se clicou no stereo para processar la na janela principal
    if(msg.data - estado_anterior_gravar == 1){ // Temos que gravar o bag, saiu de 0 para 1
      // Entrar na pasta que queremos e comecar o bag no tempo e coordenadas aqui certas

    } else if(msg.data - estado_anterior_gravar == -1) { // nao vamos gravar, parar a gravacao
      // Finalizar o processo
    }
    estado_anterior_gravar = msg.data;
  }
}

void GigeImageReader::set_nomeDaPasta(std::string nome){
  pasta = nome;
}

void GigeImageReader::vamos_gravar(bool decisao){
  stereo_funcionando = decisao;
}

void GigeImageReader::setOffset(int offp, int offt)
{
  offset = offp;
  offset_tilt = offt;
}

}
