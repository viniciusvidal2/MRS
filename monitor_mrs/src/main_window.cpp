/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/monitor_mrs/main_window.hpp"
#include "opencv2/videoio.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace monitor_mrs {

using namespace Qt;

/*****************************************************************************
**
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent),qnode(argc,argv),gige_ir(argc,argv,&mutex)
{
  //

  qRegisterMetaType<cv::Mat>("cv::Mat");
  connect(&gige_ir,SIGNAL(send_mat_image(cv::Mat,qint64)),this,SLOT(receive_mat_image(cv::Mat,qint64)));
  ui.setupUi(this);
  timer = new QTimer(this);

  ui.radioButton_automatico->setChecked(true);
}

MainWindow::~MainWindow() {}

/*****************************************************************************
**
*****************************************************************************/
void MainWindow::receive_mat_image(Mat img, qint64 timestamp)
{

  mutex.lock();
   qt_image = QImage((const unsigned char*) (img.data), img.cols, img.rows, QImage::Format_RGB888);
   ui.label_4->setPixmap(QPixmap::fromImage(qt_image));
   ui.label_4->resize(ui.label_4->pixmap()->size());
  mutex.unlock();

}

void MainWindow::update_window(){
  cap >> frame;

  cvtColor(frame,frame,CV_BGR2RGB);
  qt_image = QImage((const unsigned char*) (frame.data), frame.cols,frame.rows,QImage::Format_RGB888);

  //cvtColor(frame,frame,cv::COLOR_RGB2GRAY);
  //qt_image = QImage((const unsigned char*) (frame.data), frame.cols,frame.rows,QImage::Format_Indexed8);

  ui.label_4->setPixmap(QPixmap::fromImage(qt_image));
  ui.label_4->resize(ui.label_4->pixmap()->size());
}


void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

}  // namespace monitor_mrs


void monitor_mrs::MainWindow::on_pushButton_rviz_clicked()
{
  system("gnome-terminal -x sh -c 'rosrun rviz rviz'");
//  QEventLoop::processEvents();
}

