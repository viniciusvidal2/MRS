#ifndef QTIMAGENODE_H
#define QTIMAGENODE_H

#endif // QTIMAGENODE_H

#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <qobject.h>
#include <QThread>
#include <QMutex>


namespace guiROS {

static const std::string OPENCV_WINDOW = "Image window";

class GigeImageReader:public QThread {
  Q_OBJECT
public:

  image_transport::Subscriber image_sub_;
 // image_transport::Publisher image_pub_;



  GigeImageReader(int argc, char** argv, QMutex*);
  virtual ~GigeImageReader();
  void init();
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  QMutex* mutex;

Q_SIGNALS:
  void send_mat_image(cv::Mat img,qint64 timestamp);
  void loggingUpdated();
  void rosShutdown();

private:
  int init_argc;
  char** init_argv;



};

}
