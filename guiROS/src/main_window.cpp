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
#include "../include/guiROS/main_window.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace guiROS {

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

void MainWindow::on_pushButton_open_webcam_clicked(){
  cap.open(0);
  if(!cap.isOpened()){
    cout << "camera nao esta aberta" << endl;
  }
  else
    cout <<"camera esta aberta"<< endl;

  QObject::connect(timer, SIGNAL(timeout()),this, SLOT(update_window()));
  timer->start(20);
}

void MainWindow::on_pushButton_close_webcam_clicked(){
  disconnect(timer, SIGNAL(timeout()),this,SLOT());
  cap.release();
  Mat image = Mat::zeros(frame.size(),CV_8UC3);
  qt_image = QImage((const unsigned char*) (image.data), image.cols, image.rows, QImage::Format_RGB888);
  ui.label_4->setPixmap(QPixmap::fromImage(qt_image));
  ui.label_4->resize(ui.label_4->pixmap()->size());
  cout << "camera esta fechada" <<endl;
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

}  // namespace guiROS

