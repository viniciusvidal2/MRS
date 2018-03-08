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
#include <string>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace monitor_mrs {

using namespace Qt;
using namespace std;

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

  // Ajustando design
  ui.radioButton_automatico->setChecked(true); // Comecar sempre com o controle automatico selecionado
  ui.radioButton_manual->setChecked(false);

  controle_gravacao = false; // Se false, nao estamos gravando, pode gravar
  controle_stereo = false; // Se false, nao estamos fazendo stereo
  ui.pushButton_salvaBag->setAutoFillBackground(true);
  ui.pushButton_salvaBag->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)"); // Assim esta quando pode gravar
  ui.pushButton_reiniciarTudo->setStyleSheet("background-color: rgb(200, 0, 20); color: rgb(0, 0, 0)");
  ui.pushButton_nuvemInstantanea->setStyleSheet("background-color: rgb(200, 200, 200); color: rgb(0, 0, 0)");

  ui.listWidget->addItem(QString::fromStdString("Iniciando o programa MRS monitor!"));

  ui.horizontalSlider_offset->setValue(49); // Posicionar no centro, que e 49 aproximadamente
  offset = 49; // Centro do range

  // Por que esse no nao funciona?
  qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
**
*****************************************************************************/
void MainWindow::receive_mat_image(Mat img, qint64 timestamp)
{

  mutex.lock();
   qt_image = QImage((const unsigned char*) (img.data), img.cols, img.rows, QImage::Format_RGB888);
   ui.imagem_tab1->setPixmap(QPixmap::fromImage(qt_image));
   ui.imagem_tab1->resize(ui.imagem_tab1->pixmap()->size());
  mutex.unlock();

}

void MainWindow::update_window(){
  cap >> frame;

  cvtColor(frame,frame,CV_BGR2RGB);
  qt_image = QImage((const unsigned char*) (frame.data), frame.cols,frame.rows,QImage::Format_RGB888);

  //cvtColor(frame,frame,cv::COLOR_RGB2GRAY);
  //qt_image = QImage((const unsigned char*) (frame.data), frame.cols,frame.rows,QImage::Format_Indexed8);

  ui.imagem_tab1->setPixmap(QPixmap::fromImage(qt_image));
  ui.imagem_tab1->resize(ui.imagem_tab1->pixmap()->size());
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
    ui.listWidget->addItem(QString::fromStdString("Motores ligados, controle automatico."));
  }else if(ui.radioButton_manual->isChecked()){ // Aqui estamos com o joy
    system("gnome-terminal -x sh -c 'roslaunch automatico_mrs lancar_gimbal.launch automatico:=false'");
    ui.listWidget->addItem(QString::fromStdString("Motores ligados, controle por Joystick."));
  }
}

void monitor_mrs::MainWindow::on_pushButton_qground_clicked()
{
  system("gnome-terminal -x sh -c 'cd ~/Desktop && ./QGroundControl.AppImage'");
  ui.listWidget->addItem(QString::fromStdString("Abrindo QGroundControl, aguarde... Se nao houver resposta, clique novamente."));
}

void monitor_mrs::MainWindow::on_pushButton_resetaPX4_clicked()
{
  system("gnome-terminal -x sh -c 'echo ufjf | sudo -S modprobe -r cdc_acm && echo ufjf | sudo -S modprobe cdc_acm'");
  ui.listWidget->addItem(QString::fromStdString("Drivers foram reiniciados, pode dar seguimento ao procedimento"));
}

void monitor_mrs::MainWindow::on_pushButton_iniciaStereo_clicked()
{
  if(!controle_stereo){ // Nao estamos fazendo stereo, clicou para comecar
    ui.pushButton_iniciaStereo->setAutoFillBackground(true);
    ui.pushButton_iniciaStereo->setStyleSheet("background-color: rgb(230, 0, 20); color: rgb(0, 0, 0)"); // Assim esta deve parar stereo
    ui.pushButton_iniciaStereo->setText("Parar processo Stereo");

    system("gnome-terminal -x sh -c 'roslaunch rustbot_bringup all.launch do_accumulation:=false do_gps:=true do_fusion:=false do_slam:=false do_stereo:=true online_stereo:=false'");
    ui.listWidget->addItem(QString::fromStdString("Processamento Stereo lancado, sistema rodando..."));
    ui.listWidget->addItem(QString::fromStdString("Assim que possivel, inicie a coleta e armazenagem de dados."));

    controle_stereo = true;
  } else {
    ui.pushButton_iniciaStereo->setAutoFillBackground(true);
    ui.pushButton_iniciaStereo->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)"); // Assim esta para comecar a gravar
    ui.pushButton_iniciaStereo->setText("Iniciar processo Stereo");

    int pid = getProcIdByName("stereo_image_proc");
    if(pid != -1)
      kill(pid, SIGINT);
    ui.listWidget->addItem(QString::fromStdString("Processamento Stereo interrompido..."));

    controle_stereo = false;
  }
}

void monitor_mrs::MainWindow::on_pushButton_salvaBag_clicked()
{
  // Ver o tempo para diferenciar bags gravadas automaticamente
  time_t t = time(0);
  struct tm * now = localtime( & t );
  string year, month, day, hour, minutes;
  year    = boost::lexical_cast<std::string>(now->tm_year + 1900);
  month   = boost::lexical_cast<std::string>(now->tm_mon );
  day     = boost::lexical_cast<std::string>(now->tm_mday);
  hour    = boost::lexical_cast<std::string>(now->tm_hour);
  minutes = boost::lexical_cast<std::string>(now->tm_min );
  string date = "_" + year + "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m";

  if(!controle_gravacao){ // Nao estamos gravando, pode gravar
    // Botao fica vermelho, mostrando que vamos ficar gravando
    ui.pushButton_salvaBag->setAutoFillBackground(true);
    ui.pushButton_salvaBag->setStyleSheet("background-color: rgb(230, 0, 20); color: rgb(0, 0, 0)"); // Assim esta quando pode gravar
    ui.pushButton_salvaBag->setText("Parar gravacao");
    // ajusta flag para estamos gravando
    controle_gravacao = true;
    // COmeca gravacao segundo nome do bag
    nome = ui.lineEdit_nomeBag->text().toStdString();
    std::string comando_full = "gnome-terminal -x sh -c 'roslaunch rustbot_bringup record_raw.launch only_raw_data:=true bag:=";
    if(nome.length() == 0){
      nome = "mrs_"+date+".bag";
      system((comando_full+=(nome+"'")).c_str());
    } else {
      nome += (date+".bag");
      system((comando_full+=(nome+"'")).c_str());
    }
    // Anunciar ao usuario
    ui.listWidget->addItem(QString::fromStdString("Iniciando gravacao do arquivo na area de trabalho..."));
    ui.listWidget->addItem(QString::fromStdString("ATENCAO: APOS 20 MINUTOS REINICIE o programa por seguranca."));
  } else if(controle_gravacao){ // Estamos gravando
    // Traz aparencia novamente para poder gravar
    ui.pushButton_salvaBag->setAutoFillBackground(true);
    ui.pushButton_salvaBag->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)"); // Assim esta quando pode gravars
    ui.pushButton_salvaBag->setText("Gravar dados");
    // ajusta flag de novo para gravar
    controle_gravacao = false;
    // Envia sinal para parar bag (SIGINT)
    int pid = getProcIdByName("record");
    if(pid!=-1)
      kill(pid, SIGINT);
    // Anunciar ao usuario
    ui.listWidget->addItem(QString::fromStdString("Arquivo gravado para pos processamento. Conferir na area de trabalho por "+nome));
  }
}

void monitor_mrs::MainWindow::on_pushButton_nuvemInstantanea_clicked()
{
  system("gnome-terminal -x sh -c 'rosrun rviz rviz -f left_optical -d src/MRS/monitor_mrs/resources/salvacao_do_mundo.rviz'"); // Ja estamos por default no diretorio do ws
  ui.listWidget->addItem(QString::fromStdString("Abrindo visualizador para reconstrucao instantanea..."));
}

void monitor_mrs::MainWindow::on_pushButton_reiniciarTudo_clicked()
{
  system("gnome-terminal -x sh -c 'killall -9 roscore && killall -9 rosmaster && killall -9 rosout && killall -9 record'");
  ui.listWidget->addItem(QString::fromStdString("Todos os processos foram interrompidos. Feche todos os terminais que restarem e reinicie esse programa para funcionamento adequado."));
  system("gnome-terminal -x sh -c 'roscore'");
}

void monitor_mrs::MainWindow::on_pushButton_limpaTexto_clicked()
{
  ui.listWidget->clear();
}

void monitor_mrs::MainWindow::on_horizontalSlider_offset_sliderMoved()
{
  offset = ui.horizontalSlider_offset->value() - 49;
  gige_ir.setOffset(offset);
}
