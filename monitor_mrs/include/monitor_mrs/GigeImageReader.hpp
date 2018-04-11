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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <qobject.h>
#include <QThread>
#include <QMutex>


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
  void estamosdentroCb(const std_msgs::Float32& msg);
  QMutex* mutex;

  void setOffset(int offp, int offt);
  void set_nomeDaPasta(std::string nome);
  void vamos_gravar(bool decisao);

Q_SIGNALS:
  void send_mat_image(cv::Mat img,qint64 timestamp);
  void loggingUpdated();
  void rosShutdown();

private:
  int init_argc;
  char** init_argv;

  ros::Publisher offset_pub;
  ros::Publisher offset_tilt_pub;
  int offset; // Para guardar os offsets a serem publicados
  int offset_tilt;
  std_msgs::Int8 msg_off;
  std_msgs::Int8 msg_off_tilt;

  ros::Subscriber sub_estamosdentro;
  int estado_anterior_gravar; // Se estamos no raio ok, liga e comeca a gravar um bag
  bool stereo_funcionando; // mensagem se estamos com o stereo ligado
  std::string pasta; // pasta atual para gravar bags automaticos
};

}
