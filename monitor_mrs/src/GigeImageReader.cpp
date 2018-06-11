#include "../include/monitor_mrs/GigeImageReader.hpp"
#include <QTime>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <sstream>
#include <string>
#include <QStringListModel>
#include <QFuture>
#include <QtConcurrentRun>

#include <boost/lexical_cast.hpp>

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
  image_sub_ = it_.subscribe("/stereo/left/image_rect", 1, &GigeImageReader::imageCb, this);
  offset_pub      = nh_.advertise<std_msgs::Int8>("offset_pub", 100);
  offset_tilt_pub = nh_.advertise<std_msgs::Int8>("offset_tilt_pub", 100);
  esquema_pub     = nh_.advertise<std_msgs::Int8>("esquema_pub", 100);
  offset = 0; // a ser publicado para alterar pan do motor
  offset_tilt = 0; // a ser publicado para alterar tilt do motor
  sub_estamosdentro = nh_.subscribe("/estamos_dentro", 10, &GigeImageReader::estamosdentroCb, this);

  sub_gps = nh_.subscribe("/mavros/global_position/global", 10, &GigeImageReader::ler_gps, this);

  estado_anterior_gravar = 0.0f;

  raio_client = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");

  msg_esq.data = 1; // Inicia com o caminho todo

  ros::spin();

}

void GigeImageReader::imageCb(const sensor_msgs::ImageConstPtr &msg)
{

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  mutex->lock();
  qint64 timestamp = QDateTime().currentMSecsSinceEpoch();
  send_mat_image(cv_ptr->image,timestamp);
  // Enviar com uma certa frequencia aqui o offset para deslocar o motor
  msg_off.data = offset;
  msg_off_tilt.data = offset_tilt;
  offset_pub.publish(msg_off);
  offset_tilt_pub.publish(msg_off_tilt);
  esquema_pub.publish(msg_esq);
  mutex->unlock();

  // Update GUI Window
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //cv::waitKey(3);
}

void GigeImageReader::estamosdentroCb(const std_msgs::Int8 &msg){
  // A mensagem fala se estamos dentro ou nao, mas chamaremos somente na transicao entre a entrada e a saida.
  // Portanto, pegaremos o chaveamento armazenando o estado anterior.
  // Ver o tempo para diferenciar bags gravadas automaticamente
  time_t t = time(0);
  struct tm * now = localtime( & t );
  string year, month, day, hour, minutes;
  year    = boost::lexical_cast<std::string>(now->tm_year + 1900);
  month   = boost::lexical_cast<std::string>(now->tm_mon );
  day     = boost::lexical_cast<std::string>(now->tm_mday);
  hour    = boost::lexical_cast<std::string>(now->tm_hour);
  minutes = boost::lexical_cast<std::string>(now->tm_min );
//  ROS_INFO("estamos_dentro_cb = %d", stereo_funcionando);
  if(stereo_funcionando){ // se clicou no stereo para processar la na janela principal
    if(msg.data - estado_anterior_gravar == 1){ // Temos que gravar o bag, saiu de 0 para 1
      // Entrar na pasta que queremos e comecar o bag no tempo e coordenadas aqui certas

      if(lat != 0 && lon != 0) // Assim o gps ja esta funcionando ok
        nome_bag = "ponto_lat_"+boost::lexical_cast<string>(-lat)+"_lon_"+boost::lexical_cast<string>(-lon)+".bag";
      else
        nome_bag = "ponto_"+year+"_"+month+"_"+day+"_"+hour+"h_"+minutes+"m.bag";
      string comando_full = "gnome-terminal -x sh -c 'roslaunch rustbot_bringup record_raw.launch only_raw_data:=true folder:="+pasta+" bag:=";
//      ROS_INFO("%s", (comando_full+nome_bag+"'").c_str());
//      sleep((10));
      system((comando_full+nome_bag+"'").c_str());

    } else if(msg.data - estado_anterior_gravar == -1) { // nao vamos gravar, parar a gravacao
      // Finalizar o processo
      int pid = getProcIdByName("record");
      if(pid!=-1)
        kill(pid, SIGINT);
    }
    estado_anterior_gravar = msg.data;
  }
}

void GigeImageReader::ler_gps(const sensor_msgs::NavSatFixConstPtr &msg){
  lat = msg->latitude;
  lon = msg->longitude;
}

void GigeImageReader::set_nomeDaPasta(std::string nome){
  pasta = "/home/mrs/Desktop/"+nome;
}

void GigeImageReader::vamos_gravar(bool decisao){
  stereo_funcionando = decisao;
}

void GigeImageReader::setOffset(int offp, int offt)
{
  offset = offp;
  offset_tilt = offt;
}

int GigeImageReader::getProcIdByName(string procName)
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

bool GigeImageReader::set_raio(float raio){
  // Compondo a mensagem para enviar
  mavros_msgs::ParamSet raio_msg;
  mavros_msgs::ParamValue valor_enviar;

  raio_msg.request.param_id = "WP_RADIUS";
  valor_enviar.integer = 0;
  valor_enviar.real = raio;
  raio_msg.request.value   = valor_enviar;

  // Enviando
  raio_client.call(raio_msg);

  ROS_INFO("Retorno do parametro: %d", raio_msg.response.success);

  return raio_msg.response.success;
}

void GigeImageReader::set_esquema(int esq){
  msg_esq.data = esq;
}

}
