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

#include <std_msgs/Float32.h>

#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>

class PixhawkeMotor
{
private:
  // Dados vindos da placa
  float pitch_para_apontar, yaw_atual, yaw_para_apontar, estamos_dentro;
  // Ranges para alcance de pwm e angulo [DEGREES] de yaw e pitch
  int pwm_yaw_range[2]     = {0, 1023}; // [PWM]
  int pwm_pitch_range[2]   = {1900, 2250}; // [PWM]
  float ang_yaw_range[2]   = {0.0  , 300.0}; // [DEGREES]
  float ang_pitch_range[2] = {171.0, 198.0}; // [DEGREES]
  float ang_pitch_horizontal = 185.1; // [DEGREES] 1976 DE PWM
  int pwm_pitch_horizontal = 2156; // [PWM]
  int pwm_yaw_frente = 466; // apontar sempre para frente do veiculo caso nao precise virar [PWM]
  float yaw_mid_range, pitch_mid_range; // [DEGREES]
  // Relacao pwm/ang[DEGREES] para os dois casos
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
  }

  int ExecutarClasse(int argc, char **argv)
  {
    calcularAngulosMotores();
    enviarAngulosMotores();
  }

private:
  void calcularAngulosMotores()
  {
    // Analisando diferenca de yaw
    delta_yaw = wrap180(yaw_atual, yaw_para_apontar);
    // Inserindo o offset vindo da GUI
    offset_ang = -300.0f*((float)offset + 48.0f)/97.0f + 300 - yaw_mid_range; // Diferenca para o centro do range
    offset_tilt_ang = 41.0f*((float)offset_tilt + 59.0f)/98.0f + ang_pitch_range[0] - ang_pitch_horizontal; // Diferenca para o centro do range

//    ROS_INFO("offset: %.2f", offset_ang);
    //ROS_INFO("delta yaw: %.2f", yaw_mid_range);
    if(estamos_dentro == 0.0f){ // Se nao estamos dentro o offset vale, se estamos dentro so vale o automatico
      ang_pan = ((yaw_mid_range + offset_ang) < ang_yaw_range[1]) ? yaw_mid_range + offset_ang : ang_yaw_range[1]; // LImitando maximo
      ang_pan = (ang_pan > ang_yaw_range[0]) ? ang_pan : ang_yaw_range[0]; // LImitando minimo
//      ROS_INFO("ang pan: %.2f", ang_pan);
      // Analisando diferenca de pitch -> somente a mesma sobre o pwm para manter horizontal
//      ROS_INFO("offset tilt ang: %.2f", offset_tilt_ang);
      delta_pitch = ((ang_pitch_horizontal + offset_tilt_ang) < ang_pitch_range[1]) ? offset_tilt_ang : ang_pitch_range[1] - ang_pitch_horizontal;
      delta_pitch = ((ang_pitch_horizontal + delta_pitch) > ang_pitch_range[0]) ? delta_pitch : ang_pitch_range[0] - ang_pitch_horizontal;
//      ROS_INFO("delta tilt: %.2f", delta_pitch);
    } else { // de 60 em 60 graus aqui !!
//      delta_yaw = (int)(delta_yaw/60) * 60; // Aqui arredonda para multiplos de 60, creio eu
      ang_pan = ((yaw_mid_range + delta_yaw) < ang_yaw_range[1]) ? yaw_mid_range + delta_yaw : ang_yaw_range[1]; // LImitando maximo
      ang_pan = (ang_pan > ang_yaw_range[0]) ? ang_pan : ang_yaw_range[0]; // LImitando minimo
      // Analisando diferenca de pitch -> somente a mesma sobre o pwm para manter horizontal
//      ROS_INFO("pitch_para_apontar: %.2f", pitch_para_apontar);
      delta_pitch = ((ang_pitch_horizontal + pitch_para_apontar) < ang_pitch_range[1]) ? pitch_para_apontar : ang_pitch_range[1] - ang_pitch_horizontal;
      delta_pitch = ((ang_pitch_horizontal + delta_pitch) > ang_pitch_range[0]) ? delta_pitch : ang_pitch_range[0] - ang_pitch_horizontal;
//      ROS_INFO("delta_pitch: %.2f", delta_pitch);
    }
//    ROS_INFO("DELTA PITCH: %.2f", delta_pitch);
    // Uma vez todos os angulos calculados, converter para valor de pwm para enviar aos motores
    pwm_pan  = pwm_yaw_range[0] + (ang_pan - ang_yaw_range[0])*pwm_ang_yaw;
    pwm_tilt = pwm_pitch_horizontal + delta_pitch*pwm_ang_pitch;
//    ROS_INFO("pwm_tilt porra: %.2f", pwm_tilt);
  }

  void enviarAngulosMotores()
  {
//    ROS_INFO("Angulo PAN : %d", pwm_pan);
//    ROS_INFO("Angulo TILT: %d", pwm_tilt);

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
    if(delta >  180.0) {delta -= 360.0;}
    if(delta < -180.0) {delta += 360.0;}

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
    yaw_para_apontar   = ((float)(msg->heading)*0.01 >= 0) ? (float)(msg->heading)*0.01 : yaw_atual; // [DEGREES]
    std_msgs::Int8 msg_estamosdentro;
    msg_estamosdentro.data = (int)estamos_dentro;
    pub_estamosdentro.publish(msg_estamosdentro); // Daqui vou ler la na janela principal
    //        yaw_para_apontar = msg->heading*0.01;
    // Mostrando na tela se esta tudo ok
    //        ROS_INFO("Pitch: [%.2f]", pitch_para_apontar);
    //        ROS_INFO("Yaw:   [%.2f]", yaw_atual);
    //        ROS_INFO("ALvo:  [%.2f]", yaw_para_apontar);
  }

  void escutarOffset(const std_msgs::Int8& msg)
  {
    offset = msg.data;
  }

  void escutarOffset_tilt(const std_msgs::Int8& msg)
  {
    offset_tilt = msg.data;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controle_automatico");

  PixhawkeMotor pxm;

  ros::Rate rate(20);

  while(ros::ok())
  {
    pxm.ExecutarClasse(argc, argv);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
