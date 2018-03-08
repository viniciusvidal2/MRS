#include "../include/monitor_mrs/GigeImageReader.hpp"
#include <QTime>
#include <iostream>
#include <std_msgs/String.h>
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


  ros::spin();

}

void GigeImageReader::imageCb(const sensor_msgs::ImageConstPtr &msg)
{

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //mutex->lock();
  mutex->lock();
  qint64 timestamp = QDateTime().currentMSecsSinceEpoch();
  send_mat_image(cv_ptr->image,timestamp);
  mutex->unlock();

  // Update GUI Window
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //cv::waitKey(3);
}


}
