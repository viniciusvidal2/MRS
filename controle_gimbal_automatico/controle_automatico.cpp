#include "ros/ros.h"
#include "std_msgs/String.h"

#include <std_msgs/Int8.h>

#include <mavros/mavros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavlink/config.h>
#include <mavros/utils.h>
#include <mavros/mavros.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/StreamRate.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>

#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>

class PixhawkeMotor
{
private:
  // Dados vindos da placa
  float pitch_para_apontar, yaw_atual, yaw_para_apontar, estamos_dentro;
  // Ranges para alcance de pwm e angulo [DEGREES] de yaw e pitch
  int pwm_yaw_range[2]     = {0, 1023}; // [PWM]
  int pwm_pitch_range[2]   = {2015, 2343}; // [PWM]
  float ang_yaw_range[2]   = {0.0  , 300.0}; // [DEGREES]
  float ang_pitch_range[2] = {175.0, 205.0}; // [DEGREES]
  float ang_pitch_horizontal = 186.0; // [DEGREES]
  float ang_yaw_frente = 157.0; // [DEGREES]
  int pwm_pitch_horizontal = 2142; // [PWM]
  int pwm_yaw_frente = 537; // apontar sempre para frente do veiculo caso nao precise virar [PWM]
  float yaw_mid_range, pitch_mid_range; // [DEGREES]
  // Relacao pwm/ang[PWM/DEGREES] para os dois casos
  float pwm_ang_yaw, pwm_ang_pitch;
  // Diferenca entre angulos de yaw e pitch
  float delta_yaw, delta_pitch;
  // Posicoes para os motores de yaw e tilt
  float ang_pan, ang_tilt; // PAN esta para YAW e TILT para PITCH [DEGREES]
  int   pwm_pan, pwm_tilt; // [PWM]
  // ENtidades do ROS
  ros::NodeHandle nh_;
  ros::Subscriber subPix;
  ros::ServiceClient motor;
  // Mensagem destinada ao servico do motor
  dynamixel_workbench_msgs::JointCommand posicao;
  // Offset vindo da GUI, movimenta em PAN e TILT
  int offset = 0; // Vem na mensagem de -48 a 49
  float offset_ang = 0; // Convertido para angulos [DEGREES]
  int offset_tilt = 0; // Vem na mensagem de -59 a 39
  float offset_tilt_ang = 0; // Convertido para angulos [DEGREES]
  ros::Subscriber subOff;
  ros::Subscriber subOffTilt;
  // Publicando se estamos dentro ou nao, para controle de gravacao de bag
  ros::Publisher pub_estamosdentro;
  // Ler se e so apontar pra frente ou virar no ponto de interesse
  ros::Subscriber subesquema;
  int esquema = 1;
  // Subscribers para o estado do motor atual e publisher para enviar as duas informacoes de uma vez so sincronizadas
  ros::Subscriber subpandyn;
  ros::Subscriber subtiltdyn;
  ros::Publisher  pubdyn;
  // Mensagem sobre os dados dos motores a ser preenchida
  nav_msgs::Odometry dyn_msg;

public:
  PixhawkeMotor()
  {
    // Inicia a escuta do topico ja publicado de chegada da mensagem
    subPix = nh_.subscribe("/mavros/vfr_hud", 10, &PixhawkeMotor::escutarPixhawk, this);
    // Inicia a relacao de pwm/ang para cada motor
    pwm_ang_yaw   = (pwm_yaw_range[1] - pwm_yaw_range[0]) / (ang_yaw_range[1] - ang_yaw_range[0]);         // [PWM/DEGREES]
    pwm_ang_pitch = (pwm_pitch_range[0] - pwm_pitch_range[1]) / (ang_pitch_range[1] - ang_pitch_range[0]); // [PWM/DEGREES] sinal invertido nesse caso
    // Pontos centrais dos dois servos
    yaw_mid_range   = (ang_yaw_range[1]   + ang_yaw_range[0])  /2; // [DEGREES]
    pitch_mid_range = (ang_pitch_range[1] + ang_pitch_range[0])/2; // [DEGREES]
    // Servico que envia valores ao motor
    motor = nh_.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

    // Inicia subscriber para lidar com offset de PAN e TILT
    subOff     = nh_.subscribe("/offset_pub"     , 10, &PixhawkeMotor::escutarOffset     , this);
    subOffTilt = nh_.subscribe("/offset_tilt_pub", 10, &PixhawkeMotor::escutarOffset_tilt, this);
    // Inicia publisher de estamos dentro ou fora do raio de interesse
    pub_estamosdentro = nh_.advertise<std_msgs::Int8>("/estamos_dentro", 10);
    // Inicia o subscriber de ver se olha so para frente ou para os pontos de interesse
    subesquema = nh_.subscribe("/esquema_pub", 10, &PixhawkeMotor::escutarEsquema, this);
    // Inicia subscribers dos motores
    subpandyn  = nh_.subscribe("/multi_port/pan_state" , 100, &PixhawkeMotor::escutarPan , this);
    subtiltdyn = nh_.subscribe("/multi_port/tilt_state", 100, &PixhawkeMotor::escutarTilt, this);
    // Publisher dos motores sincronizados
    pubdyn     = nh_.advertise<nav_msgs::Odometry>("/dynamixel_sync", 100);

    // Chama o servico que altera a taxa de transferencia do mavros
    ros::ServiceClient srvRate = nh_.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    mavros_msgs::StreamRate rate;
    rate.request.stream_id = 0;
    rate.request.message_rate = 10; // X Hz das mensagens que vem
    rate.request.on_off = 1; // Nao sei
    if(srvRate.call(rate))
      ROS_INFO("Taxado mavros mudada para %d Hz", rate.request.message_rate);
    else
      ROS_INFO("Nao pode chamar o servico, taxa nao mudada.");

    dyn_msg.header.frame_id = "odom"; // Por desencargo
  }

  int ExecutarClasse(int argc, char **argv)
  {
    calcularAngulosMotores();
    enviarAngulosMotores();

//    dyn_msg.header.stamp = ros::Time::now(); // Colocar o tempo para sincronizar a frente
    pubdyn.publish(dyn_msg); // Consegue publicar a 6 Hz, por sorte
  }

private:
  void calcularAngulosMotores()
  {
    // Inserindo o offset vindo da GUI
    offset_ang = -300.0f*((float)offset + 48.0f)/97.0f + 300 - ang_yaw_frente; // Diferenca para o centro do range
    offset_tilt_ang = (ang_pitch_range[0]-ang_pitch_range[1])*((float)offset_tilt - 48.0f)/(-99.0f) + ang_pitch_range[1] - ang_pitch_horizontal; // Diferenca para o centro do range

    offset_tilt_ang = offset_tilt_ang*2; // Para condizer com os novos limites apos adicao do bumper

    if (esquema == 2) { // Pegar os pontos de interesse

      //    ROS_INFO("yaw ATUAL: %.2f\tyaw APONTAR: %.2f\tdelta yaw: %.2f", yaw_atual, yaw_para_apontar, delta_yaw);
      if(estamos_dentro == 0.0f){ // Se nao estamos dentro o offset vale, se estamos dentro so vale o automatico
        delta_yaw   = offset_ang;
        delta_pitch = offset_tilt_ang;
        //      ROS_INFO("OFFSET DE TILT: %.2f", offset_tilt_ang);

      } else { // Dentro do controle automatico
        //      delta_yaw = (int)(delta_yaw/60) * 60; // Aqui arredonda para multiplos de 60, creio eu
        // Analisando diferenca de yaw
        delta_yaw = wrap180(yaw_atual, yaw_para_apontar);
        // Analisando diferenca de pitch -> somente a mesma sobre o pwm para manter horizontal
        delta_pitch = pitch_para_apontar;
      }
      // Uma vez todos os angulos calculados, converter para valor de pwm para enviar aos motores
      pwm_pan  = pwm_yaw_frente + delta_yaw*pwm_ang_yaw;
      pwm_pan  = (pwm_pan > pwm_yaw_range[1]) ? pwm_yaw_range[1] : (pwm_pan < pwm_yaw_range[0] ? pwm_yaw_range[0] : pwm_pan); // Limitando em maximo e minimo aqui, mais seguro

      pwm_tilt = pwm_pitch_horizontal + delta_pitch*pwm_ang_pitch;
      pwm_tilt = (pwm_tilt > pwm_pitch_range[1]) ? pwm_pitch_range[1] : (pwm_tilt < pwm_pitch_range[0] ? pwm_pitch_range[0] : pwm_tilt); // Limitando em maximo e minimo aqui, mais seguro

    } else if (esquema == 1) { // Apontar so para frente, a nao ser que queira mexer na camera

      delta_yaw   = offset_ang;
      delta_pitch = offset_tilt_ang;

      pwm_pan  = pwm_yaw_frente + delta_yaw*pwm_ang_yaw;
      pwm_pan  = (pwm_pan > pwm_yaw_range[1]) ? pwm_yaw_range[1] : (pwm_pan < pwm_yaw_range[0] ? pwm_yaw_range[0] : pwm_pan); // Limitando em maximo e minimo aqui, mais seguro

      pwm_tilt = pwm_pitch_horizontal + delta_pitch*pwm_ang_pitch;
      pwm_tilt = (pwm_tilt > pwm_pitch_range[1]) ? pwm_pitch_range[1] : (pwm_tilt < pwm_pitch_range[0] ? pwm_pitch_range[0] : pwm_tilt); // Limitando em maximo e minimo aqui, mais seguro
    }
  }

  void enviarAngulosMotores()
  {
    posicao.request.pan_pos  = (int)pwm_pan;
    posicao.request.tilt_pos = (int)pwm_tilt;
    posicao.request.unit     = "raw";

    posicao.response.pan_pos = (int)pwm_pan;
    posicao.response.tilt_pos = (int)pwm_tilt;

    motor.call(posicao);
  }

  float rad2deg(float rad)
  {
    return 180.0/3.1415926535 * rad;
  }

  // Mantem o angulo de diferenca entre 180 da forma mais logica no algoritmo
  float wrap180(float atual, float apontar)
  {
    float delta = atual - apontar;

    if(delta >  180.0) delta = (delta - 360.0);
    if(delta < -180.0) delta = (delta + 360.0);
    // Pequeno ajuste para nao ficar atrasado na pratica
    if(delta > 0) delta+=5;
    if(delta < 0) delta-=5;

//    delta = (delta > 145) ? 145 : delta;
//    delta = (delta < -145) ? -145 : delta;

    return delta;
  }

  void escutarPixhawk(const mavros_msgs::VFR_HUDConstPtr& msg)
  {
    /// Relacoes obtidas da mensagem VFR_HUD vinda da placa
    ///
    estamos_dentro = msg->throttle*100; // Se estamos ou nao na regiao de interesse -> cancela offset (vem 0 ou 0.01 da placa)
//    ROS_INFO("ESTAMOS DENTRO: %f", estamos_dentro);
    pitch_para_apontar = rad2deg(msg->airspeed);     // [RAD] -> [DEGREES]
    yaw_atual          = msg->groundspeed;           // [DEGREES]
    // Ajusta chegada desse angulo que vai de -180 a +180
    yaw_para_apontar   = ((float)(msg->heading) >= 0) ? (float)(msg->heading) : (float)(msg->heading) + 360.0f; // [DEGREES]
    std_msgs::Int8 msg_estamosdentro;
    msg_estamosdentro.data = (int)estamos_dentro;
    pub_estamosdentro.publish(msg_estamosdentro); // Daqui vou ler la na janela principal
//    yaw_para_apontar = msg->heading;
    // Mostrando na tela se esta tudo ok
    //        ROS_INFO("Pitch: [%.2f]", pitch_para_apontar);
//    ROS_INFO("Yaw:   [%.2f]", yaw_atual-yaw_para_apontar);
//    ROS_INFO("YAW_ATUAL [%.2f]\tAPONTAR: [%.2f]\tDELTA: [%.2f]", yaw_atual, yaw_para_apontar, wrap180(yaw_atual, yaw_para_apontar));
    ROS_INFO("Perseguindo orientacao OK!!!");
  }

  void escutarOffset(const std_msgs::Int8& msg)
  {
    offset = msg.data;
  }

  void escutarOffset_tilt(const std_msgs::Int8& msg)
  {
    offset_tilt = msg.data;
  }

  void escutarEsquema(const std_msgs::Int8& msg)
  {
    esquema = msg.data;
  }

  void escutarPan(const dynamixel_workbench_msgs::DynamixelStateConstPtr msg){
    dyn_msg.header.stamp = ros::Time::now();
    dyn_msg.pose.pose.position.x = (double)msg->present_position; // O PAN estara na posicao X da mensagem
  }

  void escutarTilt(const dynamixel_workbench_msgs::DynamixelStateConstPtr msg){
    dyn_msg.pose.pose.position.y = (double)msg->present_position; // O TILT estara na posicao Y da mensagem
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controle_automatico");

  PixhawkeMotor pxm;

  ros::Rate rate(90);

  while(ros::ok())
  {
    pxm.ExecutarClasse(argc, argv);
//    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
