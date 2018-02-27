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
}

void monitor_mrs::MainWindow::on_pushButton_motores_clicked()
{
  if(ui.radioButton_automatico->isChecked()){ // Aqui estamos com a pixhawk
    system("gnome-terminal -x sh -c 'roslaunch automatico_mrs lancar_gimbal.launch'");

  }else if(ui.radioButton_manual->isChecked()){ // Aqui estamos com o joy
    system("gnome-terminal -x sh -c 'roslaunch automatico_mrs lancar_gimbal.launch automatico:=false'");
  }
}

void monitor_mrs::MainWindow::on_pushButton_qground_clicked()
{
  system("gnome-terminal -x sh -c 'cd ~/Desktop && ./QGroundControl.AppImage'");
}

void monitor_mrs::MainWindow::on_pushButton_resetaPX4_clicked()
{
  system("gnome-terminal -x sh -c 'echo violao05 | sudo -S modprobe -r cdc_acm && echo violao05 | sudo -S modprobe cdc_acm");
}

void monitor_mrs::MainWindow::on_pushButton_iniciaStereo_clicked()
{
  system("roslaunch rustbot_bringup all.launch do_accumulation:=false do_gps:=true do_fusion:=false do_slam:=false do_stereo:=true");
}

void monitor_mrs::MainWindow::on_pushButton_salvaBag_clicked()
{
  std::string nome = ui.lineEdit_nomeBag->text().toStdString();
  std::string comando_full = "roslaunch rustbot_bringup record_raw.launch only_raw_data:=true bag:=";
  if(nome.length() == 0){
    system("roslaunch rustbot_bringup record_raw.launch only_raw_data:=true");
  } else {
    system((comando_full+=nome).c_str());
  }
}
