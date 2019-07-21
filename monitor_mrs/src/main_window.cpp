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
#include <QPixmap>

#include <sys/stat.h>

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
  ui.radioButton_pontosdeinteresse->setChecked(true); // Comecar com a gravacao por trecho automatica
  ui.radioButton_caminhocompleto->setChecked(false);

  controle_gravacao = false; // Se false, nao estamos gravando, pode gravar
  controle_stereo = false; // Se false, nao estamos fazendo stereo
  ui.pushButton_salvaBag->setAutoFillBackground(true);
  ui.pushButton_salvaBag->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)"); // Assim esta quando pode gravar
  ui.pushButton_reiniciarTudo->setStyleSheet("background-color: rgb(200, 0, 20); color: rgb(0, 0, 0)");
//  ui.pushButton_nuvemInstantanea->setStyleSheet("background-color: rgb(200, 200, 200); color: rgb(0, 0, 0)");
//  ui.pushButton_cameratermica->setStyleSheet("background-color: rgb(200, 200, 200); color: rgb(0, 0, 0)");
  ui.pushButton_iniciaStereo->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)"); // Assim esta para comecar a gravar
  ui.horizontalSlider_offset->hide();
  ui.horizontalSlider_offset->setRange(-120, 120); // Casa com os limites de angulos setados para o motor MX-106
  ui.horizontalSlider_offset->setValue(0);
  ui.verticalSlider_offset->hide();
  ui.verticalSlider_offset->setRange(-15, 15); // Casa com os limites de angulos setados para o motor MX-64
  ui.verticalSlider_offset->setValue(0);

  ui.pushButton_setaimagem->setAutoFillBackground(true);
  ui.pushButton_setaimagem->setStyleSheet("background-color: rgb(230, 230, 0); color: rgb(0, 0, 0)");
  ui.pushButton_setaimagem->setText("Imagem termica");

  ui.radioButton_caminhocompleto->setChecked(true);
  ui.radioButton_pontosdeinteresse->setChecked(false);

  ui.listWidget->addItem(QString::fromStdString("Iniciando o programa MRS monitor!"));

  QPixmap pix(":/images/mrs.jpg");
  ui.label_MRS->setPixmap(pix.scaled(120,120,Qt::KeepAspectRatio));
  ui.label_MRS2->setPixmap(pix.scaled(120,120,Qt::KeepAspectRatio));

  offset_pan = ui.horizontalSlider_offset->value();
  offset_tilt = ui.verticalSlider_offset->value();
  esquema_apontar_caminho = 1; // Default olhar pra frente

  // Por que esse no nao funciona?
  qnode.init();

  gige_ir.setOffset(0, 0);
  gige_ir.vamos_gravar(false);

  // Qual imagem vai comecar rodando?
  fonte_imagem = visual; // Imagem que estara aparecendo
  gige_ir.set_imagem(0); // 0 esta para visual, default
}

MainWindow::~MainWindow() {}

/*****************************************************************************
**
*****************************************************************************/
void MainWindow::receive_mat_image(Mat img, qint64 timestamp)
{

  mutex.lock();
   qt_image = QImage((const unsigned char*) (img.data), img.cols, img.rows, QImage::Format_RGB888);
   qt_image.scaled(400, 300);
   ui.imagem_tab1->setPixmap(QPixmap::fromImage(qt_image));
   ui.imagem_tab1->resize(ui.imagem_tab1->pixmap()->size());
   ui.imagem_tab1->setScaledContents(true);
   ui.imagem_tab1->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

   //Lucas, aba 2
   ui.imagem_tab2->setPixmap(QPixmap::fromImage(qt_image));
   ui.imagem_tab2->resize(ui.imagem_tab2->pixmap()->size());
   ui.imagem_tab2->setScaledContents(true);
   ui.imagem_tab2->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

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

  //Lucas, aba 2
  ui.imagem_tab2->setPixmap(QPixmap::fromImage(qt_image));
  ui.imagem_tab2->resize(ui.imagem_tab2->pixmap()->size());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  int pid = getProcIdByName("play");
  if(pid!=-1)
    kill(pid, SIGINT);
  if(ui.radioButton_automatico->isChecked()){ // Aqui para automatico
  pid = getProcIdByName("controle_automatico");
  if(pid!=-1)
    kill(pid, SIGINT);
  } else { // Aqui para manual
  pid = getProcIdByName("escutadeiro_node");
  if(pid!=-1)
    kill(pid, SIGINT);
  }
  sleep(2);
  system("gnome-terminal -x sh -c 'rosservice call /joint_command raw 2118 1900'"); // Posiciona o motor no minimo cuidadosamente ao desligar
  sleep(5);
  QMainWindow::closeEvent(event);
  system("gnome-terminal -x sh -c 'rosnode kill -a'");
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
//          ROS_INFO("PROC %s", cmdLine.c_str());
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
  if(ui.radioButton_automatico->isChecked()) { // Aqui estamos com a pixhawk
    system("gnome-terminal -x sh -c 'roslaunch automatico_mrs lancar_gimbal.launch'");
    sleep(1); // Para o no iniciar ok
    system("gnome-terminal -x sh -c 'rosservice call /joint_command raw 2118 2200'"); // Posiciona o robo olhando para frente
    ui.listWidget->addItem(QString::fromStdString("Motores ligados, controle automatico."));
  } else if(ui.radioButton_manual->isChecked()) { // Aqui estamos com o joy
    system("gnome-terminal -x sh -c 'roslaunch automatico_mrs lancar_gimbal.launch automatico:=false'");
    ui.listWidget->addItem(QString::fromStdString("Motores ligados, controle por Joystick."));
  }
  ui.groupBox_raio->setEnabled(true);
  ui.radioButton_automatico->hide();
  ui.radioButton_manual->hide();
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
  gige_ir.setOffset(0, 0); // Assim apontamos para frente forcado

  if(!controle_stereo){ // Nao estamos fazendo stereo, clicou para comecar
    ui.pushButton_iniciaStereo->setAutoFillBackground(true);
    ui.pushButton_iniciaStereo->setStyleSheet("background-color: rgb(230, 0, 20); color: rgb(0, 0, 0)"); // Assim esta deve parar stereo
    ui.pushButton_iniciaStereo->setText("Parar processo Stereo");

    system("gnome-terminal -x sh -c 'roslaunch rustbot_bringup all.launch do_accumulation:=false do_gps:=true do_fusion:=false do_slam:=true do_stereo:=true online_stereo:=true'");
    ui.listWidget->addItem(QString::fromStdString("Processamento Stereo lancado, sistema rodando..."));
    ui.listWidget->addItem(QString::fromStdString("Assim que possivel, inicie a coleta e armazenagem de dados."));
    if(ui.radioButton_automatico->isChecked()){
      ui.horizontalSlider_offset->show();
      ui.verticalSlider_offset->show();
    }
    string comando_termica_felipe = "gnome-terminal -x sh -c 'rosrun termica_reconstrucao imTermicaScaledOnline.py "+ui.lineEdit_temperaturacritica->text().toStdString()+"'";
    system(comando_termica_felipe.c_str());
    controle_stereo = true;

  } else {
    ui.pushButton_iniciaStereo->setAutoFillBackground(true);
    ui.pushButton_iniciaStereo->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)"); // Assim esta para comecar a gravar
    ui.pushButton_iniciaStereo->setText("Iniciar processo Stereo");

    int pid = getProcIdByName("stereo_image_proc");
    if(pid != -1)
      kill(pid, SIGINT);
    ui.listWidget->addItem(QString::fromStdString("Processamento Stereo interrompido..."));
    ui.horizontalSlider_offset->hide();
    ui.verticalSlider_offset->hide();

    controle_stereo = false;
    gige_ir.vamos_gravar(false);
  }
}

void monitor_mrs::MainWindow::on_pushButton_cameratermica_clicked()
{
  system("gnome-terminal -x sh -c 'rqt_image_view /dados_sync/image_8bits'");
}
void monitor_mrs::MainWindow::on_pushButton_reconstrucaoInstantaneaTermica_clicked()
{
  system("gnome-terminal -x sh -c 'rosrun rviz rviz -f left_optical -d $HOME/mrs_ws/src/MRS/monitor_mrs/resources/salvacao_do_mundo_termica_inst.rviz'"); // Ja estamos por default no diretorio do ws
  ui.listWidget->addItem(QString::fromStdString("Abrindo visualizador para reconstrucao termica instantanea..."));
}

void monitor_mrs::MainWindow::on_radioButton_pontosdeinteresse_clicked()
{
  esquema_apontar_caminho = 2;
  gige_ir.set_esquema(esquema_apontar_caminho); // Aqui vamos setar o controle como esta
}

void monitor_mrs::MainWindow::on_radioButton_caminhocompleto_clicked()
{
  esquema_apontar_caminho = 1;
  gige_ir.set_esquema(esquema_apontar_caminho); // Aqui vamos setar o controle como esta
}

void monitor_mrs::MainWindow::on_pushButton_salvaBag_clicked()
{
  string date;

  if(!controle_gravacao){ // Nao estamos gravando, pode gravar

    // Ver o tempo para diferenciar bags gravadas automaticamente
    time_t t = time(0);
    struct tm * now = localtime( & t );
    string year, month, day, hour, minutes;
    year    = boost::lexical_cast<std::string>(now->tm_year + 1900);
    month   = boost::lexical_cast<std::string>(now->tm_mon );
    day     = boost::lexical_cast<std::string>(now->tm_mday);
    hour    = boost::lexical_cast<std::string>(now->tm_hour);
    minutes = boost::lexical_cast<std::string>(now->tm_min );
    date = "_" + year + "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m";

    // Botao fica vermelho, mostrando que vamos ficar gravando
    ui.pushButton_salvaBag->setAutoFillBackground(true);
    ui.pushButton_salvaBag->setStyleSheet("background-color: rgb(230, 0, 20); color: rgb(0, 0, 0)"); // Assim esta quando pode gravar
    ui.pushButton_salvaBag->setText("Parar gravacao");
    // ajusta flag para estamos gravando
    controle_gravacao = true;
    gige_ir.set_flag_gravando_bag(controle_gravacao);
    // Comeca gravacao segundo nome do bag
    if(ui.radioButton_caminhocompleto->isChecked()){
    nome = ui.lineEdit_nomeBag->text().toStdString();
    if(nome.length() == 0){
      nome = "mrs_"+date;
    } else {
      nome += date;
    }
    string comando_fazer_pasta = "gnome-terminal -x sh -c 'cd ~/Desktop && mkdir -p "+nome+"/Quentes'";
    system(comando_fazer_pasta.c_str());
    std::string comando_full = "gnome-terminal -x sh -c 'roslaunch rustbot_bringup record_raw.launch only_raw_data:=true bag:="+nome+".bag";
    comando_full +=" folder:=$HOME/Desktop/"+nome+"'";
    system(comando_full.c_str());

    } else if(ui.radioButton_pontosdeinteresse->isChecked()) {
      gige_ir.set_nomeDaPasta(nome);
      // Criando a pasta na area de trabalho
      string comando_temp = "gnome-terminal -x sh -c 'cd ~/Desktop && mkdir -p "+nome+"/Quentes'";
      system(comando_temp.c_str());
      gige_ir.vamos_gravar(true);
    }
    // Anunciar ao usuario
    ui.listWidget->addItem(QString::fromStdString("Iniciando gravacao do arquivo na area de trabalho..."));
    ui.listWidget->addItem(QString::fromStdString("ATENCAO: APOS 20 MINUTOS REINICIE o programa por seguranca."));
  } else if(controle_gravacao) { // Estamos gravando e terminando agora
    // Traz aparencia novamente para poder gravar
    ui.pushButton_salvaBag->setAutoFillBackground(true);
    ui.pushButton_salvaBag->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)"); // Assim esta quando pode gravar
    ui.pushButton_salvaBag->setText("Gravar dados");
    // ajusta flag de novo para gravar futuramente - PARAMOS DE GRAVAR AGORA
    controle_gravacao = false;
    gige_ir.set_flag_gravando_bag(controle_gravacao);
    // Envia sinal para parar bag (SIGINT)
    int pid = getProcIdByName("record");
    if(pid!=-1)
      kill(pid, SIGINT);
    // Checar se o arquivo foi quente ou nao, dai mover para a outra pasta -> Aqui so para caminho completo,
    // pontos de interesse dentro da classe gige_ir
    if(ui.radioButton_caminhocompleto->isChecked()){
      if(gige_ir.get_flag_temperatura() == 1){
        std::string comando_muda_pasta = "gnome-terminal -x sh -c 'mv ~/Desktop/"+nome+"/"+nome+".bag ~/Desktop/"+nome+"/Quentes/"+nome+".bag'";
        sleep(1);
        system(comando_muda_pasta.c_str());
      }
    }
    // Anunciar ao usuario
    ui.listWidget->addItem(QString::fromStdString("Arquivo gravado para pos processamento. Conferir na area de trabalho"));
    if(ui.radioButton_pontosdeinteresse->isChecked())
      gige_ir.vamos_gravar(false);
  }
}

void monitor_mrs::MainWindow::on_pushButton_nuvemInstantanea_clicked()
{
  system("gnome-terminal -x sh -c 'rosrun rviz rviz -f left_optical -d $HOME/mrs_ws/src/MRS/monitor_mrs/resources/salvacao_do_mundo.rviz'"); // Ja estamos por default no diretorio do ws
  ui.listWidget->addItem(QString::fromStdString("Abrindo visualizador para reconstrucao visual instantanea..."));
}

void monitor_mrs::MainWindow::on_pushButton_reiniciarTudo_clicked()
{
  system("gnome-terminal -x sh -c 'rosservice call /joint_command raw 2118 1900'"); //Posiciona o motor no minimo cuidadosamente ao desligar
  sleep(5);
  int pid = getProcIdByName("play");
  if(pid!=-1)
    kill(pid, SIGINT);
  system("gnome-terminal -x sh -c 'rosnode kill -a'");
  ui.listWidget->addItem(QString::fromStdString("Todos os processos foram interrompidos. Feche todos os terminais que restarem e reinicie esse programa para funcionamento adequado."));
}

void monitor_mrs::MainWindow::on_pushButton_limpaTexto_clicked()
{
  ui.listWidget->clear();
}

void monitor_mrs::MainWindow::on_horizontalSlider_offset_sliderMoved()
{
  offset_pan = -ui.horizontalSlider_offset->value();
  gige_ir.setOffset(offset_pan, offset_tilt);
}

void monitor_mrs::MainWindow::on_verticalSlider_offset_sliderMoved()
{
  offset_tilt = ui.verticalSlider_offset->value(); // Aqui diferente por causa do nivel horizontal estar em 60% do range, invertendo tudo
  gige_ir.setOffset(offset_pan, offset_tilt);
}

void monitor_mrs::MainWindow::on_pushButton_enviaraio_clicked()
{
  bool ok(false);
  float raio = ui.lineEdit_raio->text().toDouble(&ok);
  if(!ok){
    raio = 30.0;
    ui.listWidget->addItem(QString::fromStdString("Digite um valor numerico, caso contrario o raio sera ajustado para 30 metros."));
  } else {
    bool envio = gige_ir.set_raio(raio);
    if(envio)
      ui.listWidget->addItem(QString::fromStdString("Raio ajustado com sucesso para "+ui.lineEdit_raio->text().toStdString()+"."));
    else
      ui.listWidget->addItem(QString::fromStdString("Raio nao pode ser ajustado, mantendo valor inicial. Se desejar reinicie o equipamento para alterar."));
  }

}

//Lucas, aba 2
void monitor_mrs::MainWindow::on_pushButtonSelectBag_clicked()
{
 filename = QFileDialog::getOpenFileName(this, "Abrir arquivo", "", "Bag Files(*.bag)");
 ui.labelBagName->setText(filename);
 ui.listWidget_2->addItem(QString::fromStdString("Bag pronta para ser inicializada. Clique em Iniciar Bag para prosseguir."));
 ui.pushButton_playBag->setEnabled(true);
 ui.horizontalLayout_tempoinicio->setEnabled(true);
}

void monitor_mrs::MainWindow::on_pushButton_playBag_clicked()
{
 QString local, arquivo; // Obtem o nome do arquivo e do local em que se encontra (para selecionar a pasta)
 int i;
 for(i=filename.length();i>0;i--)
 {
   if (filename[i]=='/')
   {
     local = filename.left(i);
     arquivo = filename.right(filename.length()-i-1);
     break;
   }
 }
 system("gnome-terminal -x sh -c 'roslaunch rustbot_bringup all.launch do_stereo:=true online_stereo:=false do_accumulation:=true'");
 system("gnome-terminal -x sh -c 'rosrun rustbot_accumulate_point_clouds save_cloud'");
 string comando_termica_felipe = "gnome-terminal -x sh -c 'rosrun termica_reconstrucao imTermicaScaled.py'";
 system(comando_termica_felipe.c_str());
 sleep(5);
 QString teste_comando = "gnome-terminal -x sh -c 'roslaunch rustbot_bringup playback.launch bag:=";
 teste_comando.append(arquivo.left(arquivo.size()-4));
 teste_comando.append(" local:=");
 teste_comando.append(local);
 teste_comando.append(" tempo:=");
 teste_comando.append(ui.lineEdit_tempoinicio->text());
 teste_comando.append("'");
 localstd = teste_comando.toStdString();
 system(localstd.c_str());
 ui.listWidget_2->addItem(QString::fromStdString("Bag inicializada."));

 // Para acompanhar o processo e salvar as nuvens ao final
 gige_ir.set_nome_bag_offline(filename.toStdString());

 ui.pushButton_paraBag->setEnabled(true);
 ui.pushButton_salvaNuvem->setEnabled(true);
 ui.pushButton_recAcumulada->setEnabled(true);
 ui.pushButton_recAcumuladaTermica->setEnabled(true);
}

void monitor_mrs::MainWindow::on_pushButton_clear2_clicked()
{
 ui.listWidget_2->clear();
}

void monitor_mrs::MainWindow::on_pushButton_paraBag_clicked()
{
 int pid = getProcIdByName("play");
 if(pid!=-1)
   kill(pid, SIGINT);
 ui.listWidget_2->addItem(QString::fromStdString("Bag parada com sucesso."));
 ui.pushButton_paraBag->setEnabled(false);
 ui.pushButton_salvaNuvem->setEnabled(false);
 ui.pushButton_recAcumulada->setEnabled(false);
 ui.pushButton_recAcumuladaTermica->setEnabled(false);

 gige_ir.set_salvar_nuvens(false); // Assim nao iniciamos outra bag salvando nuvens desesperadamente
}

void monitor_mrs::MainWindow::on_pushButton_salvaNuvem_clicked()
{
  gige_ir.set_salvar_nuvens(true);
// system("gnome-terminal -x sh -c 'rosrun rustbot_accumulate_point_clouds save_cloud'");
 ui.listWidget_2->addItem(QString::fromStdString("OBSERVE SE OS ARQUIVOS ESTAO SENDO SALVOS PELO TERMINAL CORRETO."));
}

void monitor_mrs::MainWindow::on_pushButton_recAcumulada_clicked()
{
 system("gnome-terminal -x sh -c 'rosrun rviz rviz -f odom -d $HOME/mrs_ws/src/MRS/monitor_mrs/resources/salvacao_do_mundo_2.rviz'");
}

void monitor_mrs::MainWindow::on_pushButton_recAcumuladaTermica_clicked()
{
  system("gnome-terminal -x sh -c 'rosrun rviz rviz -f odom -d $HOME/mrs_ws/src/MRS/monitor_mrs/resources/salvacao_do_mundo_termica.rviz'");
}

void monitor_mrs::MainWindow::on_pushButton_setaimagem_clicked()
{

  if(fonte_imagem == visual){ // Estamos na visual e ao clicar mudamos pra termica, tudo tem que indicar o caminho de volta pra visual
    ui.pushButton_setaimagem->setAutoFillBackground(true);
    ui.pushButton_setaimagem->setStyleSheet("background-color: rgb(0, 100, 230); color: rgb(250, 250, 250)");
    ui.pushButton_setaimagem->setText("Imagem visual");

    gige_ir.set_imagem(1); // Chama a imagem termica
    fonte_imagem = termica; // Agora nossa fonte e a termica

  } else if(fonte_imagem == termica) {
    ui.pushButton_setaimagem->setAutoFillBackground(true);
    ui.pushButton_setaimagem->setStyleSheet("background-color: rgb(230, 230, 0); color: rgb(0, 0, 0)");
    ui.pushButton_setaimagem->setText("Imagem termica");

    gige_ir.set_imagem(0); // Chama a imagem visual
    fonte_imagem = visual; // Agora nossa fonte e a visual

  }
}
