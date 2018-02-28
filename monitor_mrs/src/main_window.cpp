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

  ui.radioButton_automatico->setChecked(true); // Comecar sempre com o controle automatico selecionado
  ui.radioButton_manual->setChecked(false);

  controle_gravacao = false; // Se false, nao estamos gravando, pode gravar
  ui.pushButton_salvaBag->setAutoFillBackground(true);
  ui.pushButton_salvaBag->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)"); // Assim esta quando pode gravars
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

int MainWindow::getProcIdByName(string procName)
{
    int pid = -1;

    // Open the /proc directory
    DIR *dp = opendir("/proc");
    if (dp != NULL)
    {
        // Enumerate all entries in directory until process found
        struct dirent *dirp;
        while (pid < 0 && (dirp = readdir(dp)))
        {
            // Skip non-numeric entries
            int id = atoi(dirp->d_name);
            if (id > 0)
            {
                // Read contents of virtual /proc/{pid}/cmdline file
                string cmdPath = string("/proc/") + dirp->d_name + "/cmdline";
                ifstream cmdFile(cmdPath.c_str());
                string cmdLine;
                getline(cmdFile, cmdLine);
                if (!cmdLine.empty())
                {
                    // Keep first cmdline item which contains the program path
                    size_t pos = cmdLine.find('\0');
                    if (pos != string::npos)
                        cmdLine = cmdLine.substr(0, pos);
                    // Keep program name only, removing the path
                    pos = cmdLine.rfind('/');
                    if (pos != string::npos)
                        cmdLine = cmdLine.substr(pos + 1);
                    // Compare against requested process name
                    if (procName == cmdLine)
                        pid = id;
                }
            }
        }
    }

    closedir(dp);

    return pid;
}

}  // namespace monitor_mrs

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
  system("gnome-terminal -x sh -c 'roslaunch rustbot_bringup all.launch do_accumulation:=false do_gps:=true do_fusion:=false do_slam:=false do_stereo:=true'");
}

void monitor_mrs::MainWindow::on_pushButton_salvaBag_clicked()
{
  if(!controle_gravacao){ // Nao estamos gravando, pode gravar
    // Botao fica vermelho, mostrando que vamos ficar gravando
    ui.pushButton_salvaBag->setAutoFillBackground(true);
    ui.pushButton_salvaBag->setStyleSheet("background-color: rgb(200, 0, 20); color: rgb(0, 0, 0)"); // Assim esta quando pode gravar
    // ajusta flag para estamos gravando
    controle_gravacao = true;
    // COmeca gravacao segundo nome do bag
    std::string nome = ui.lineEdit_nomeBag->text().toStdString();
    std::string comando_full = "gnome-terminal -x sh -c 'roslaunch rustbot_bringup record_raw.launch only_raw_data:=true bag:=";
    if(nome.length() == 0){
      system("gnome-terminal -x sh -c 'roslaunch rustbot_bringup record_raw.launch only_raw_data:=true'");
    } else {
      system((comando_full+=(nome+"'")).c_str());
    }
  } else if(controle_gravacao){ // Estamos gravando
    // Traz aparencia novamente para poder gravar
    ui.pushButton_salvaBag->setAutoFillBackground(true);
    ui.pushButton_salvaBag->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)"); // Assim esta quando pode gravars
    // ajusta flag de novo para gravar
    controle_gravacao = false;
    // Envia sinal para parar bag (SIGKILL)

    int pid = getProcIdByName("rosbag_record");
    ui.lineEdit_nomeBag->setText(QString::number(pid));
  }
}

void monitor_mrs::MainWindow::on_pushButton_nuvemInstantanea_clicked()
{
  system("gnome-terminal -x sh -c 'rosrun rviz rviz -f left_optical -d ~/mrs_ws/src/MRS/monitor_mrs/resources/salvacao_do_mundo.rviz'");
}