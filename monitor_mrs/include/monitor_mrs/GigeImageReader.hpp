#ifndef QTIMAGENODE_H
#define QTIMAGENODE_H

#endif // QTIMAGENODE_H

#include <string>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <qobject.h>
#include <QThread>
#include <QMutex>

#include "mavros_msgs/GlobalPositionTarget.h"
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamSetRequest.h>
#include <mavros_msgs/ParamSetResponse.h>
#include <mavros_msgs/ParamValue.h>

#include <sys/syscall.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <cstdlib>
#include <csignal>
#include <ctime>
#include <math.h>


namespace monitor_mrs {

static const std::string OPENCV_WINDOW = "Image window";

class GigeImageReader:public QThread {
  Q_OBJECT
public:

  image_transport::Subscriber image_sub_;

  GigeImageReader(int argc, char** argv, QMutex*);
  virtual ~GigeImageReader();
  void init();
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void estamosdentroCb(const std_msgs::Int8& msg);
  QMutex* mutex;

  void setOffset(int offp, int offt);
  void set_esquema(int esq);
  void set_nomeDaPasta(std::string nome);
  void vamos_gravar(bool decisao);
  void ler_gps(const sensor_msgs::NavSatFixConstPtr& msg);
  int  getProcIdByName(std::string procName);
  bool set_raio(float raio);

  enum tipo_imagem {visual, termica};
  void set_imagem(int tipo); // Para definir se imagem termica ou visual no visor online

Q_SIGNALS:
  void send_mat_image(cv::Mat img,qint64 timestamp);
  void loggingUpdated();
  void rosShutdown();

private:
  int init_argc;
  char** init_argv;

  ros::Publisher offset_pub;
  ros::Publisher offset_tilt_pub;
  ros::Publisher esquema_pub;
  int offset; // Para guardar os offsets a serem publicados
  int offset_tilt;
  std_msgs::Int8 msg_off;
  std_msgs::Int8 msg_off_tilt;
  std_msgs::Int8 msg_esq;


  tipo_imagem tt;
  bool toggle_imagem;

  ros::Subscriber sub_estamosdentro;
  ros::Subscriber sub_gps;
  int estado_anterior_gravar; // Se estamos no raio ok, liga e comeca a gravar um bag
  bool stereo_funcionando; // mensagem se estamos com o stereo ligado
  std::string pasta; // pasta atual para gravar bags automaticos

  double lat;
  double lon; // Vindo da placa

  std::string nome_bag; // para gravar automaticamente

  ros::ServiceClient raio_client; // Para alterar o raio de acao do robo pela GUI
};

}
