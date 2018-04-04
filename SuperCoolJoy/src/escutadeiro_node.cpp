#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

#include "../../dynamixel-workbench/dynamixel_workbench_toolbox/include/dynamixel_workbench_toolbox/dynamixel_multi_driver.h"

#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>


class JoyAndMotor{

private:
  float pan_pos;
  float tilt_pos;
  float pan_min;
  float tilt_min;
  float pan_max;
  float tilt_max;

  double joy_pan;
  double joy_tilt;

  int trigger_bt;
  int shut_bt;
  int centro_bt;

  double deadzone; // O Joy tem que estar mais que isso em modulo para mexer o motor.

  int move_rate; // TODO: usar saida do controle para ditar intensidade

  ros::NodeHandle nh_; // Nodehandle aqui para ser usado em todos os procedimentos
  ros::ServiceClient posicaoMotor; // Chamara o servico contido no outro package dynamixel_controllers, lancado junto
  dynamixel_workbench_msgs::JointCommand posicao_objetivo; // Mensagem a ser preenchida com a posicao, para chamar o servico

  void limitarValor(float min, float max, float &val)
  {
    if (val < min)
    {
      val = min;
    }
    if (val > max)
    {
      val = max;
    }
  }

  void MandaProMotor(){
    if(centro_bt == 1)
    {
      pan_pos  =  466; //(pan_min  + pan_max )/2;
      tilt_pos = 2156; //(tilt_max + tilt_min)/2;
    }
    if(shut_bt == 1) // Botao 3
    {
      ros::shutdown();
    }
    if(trigger_bt == 1)
    {
      // Controle da entrada de pan
      if(fabs(joy_pan) > deadzone){
        if(joy_pan > 0){ // TODO: verificar sentido de giro do motor
          pan_pos = pan_pos + move_rate;
        } else {
          pan_pos = pan_pos - move_rate;
        }
      }
      limitarValor(pan_min, pan_max, pan_pos);
      // Controle da entrada de tilt
      if(fabs(joy_tilt) > deadzone){
        if(joy_tilt > 0){ // TODO: verificar sentido de giro do motor
          tilt_pos = tilt_pos + move_rate;
        } else {
          tilt_pos = tilt_pos - move_rate;
        }
      }
      limitarValor(tilt_min, tilt_max, tilt_pos);
    }
    ROS_INFO("PAN: %.0f         TILT: %.0f", pan_pos, tilt_pos);

    posicao_objetivo.request.pan_pos  = pan_pos;
    posicao_objetivo.request.tilt_pos = tilt_pos;
    posicao_objetivo.request.unit = "raw";

    posicao_objetivo.response.pan_pos  = pan_pos;
    posicao_objetivo.response.tilt_pos = tilt_pos;

    posicaoMotor.call(posicao_objetivo);
  }

public:

  // Construtor inicia tudo
  JoyAndMotor():deadzone(0.2), pan_min(0), tilt_min(1876.0), pan_max(1023.0), tilt_max(2342.0)
  {
    pan_pos  =  466; // Posicao pan frente na pratica //(pan_min  + pan_max )/2;
    tilt_pos = 2156; // Posicao horizontal tilt //(tilt_max + tilt_min)/2;
    posicaoMotor = nh_.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");
  }

  int main(int argc, char **argv)
  {
    // Vamos ler aqui a essa taxa o Joy
    ros::Subscriber sub = nh_.subscribe("joy", 100, &JoyAndMotor::EscutandoJoy, this);
    ros::Rate rate(40);

    while(ros::ok()){
      MandaProMotor();
      ros::spinOnce();

      rate.sleep();
    }

    return 0;
  }

private:

  void EscutandoJoy(const sensor_msgs::Joy& msg)
  {
    trigger_bt = msg.buttons[0];
    shut_bt = msg.buttons[1];
    centro_bt = msg.buttons[4];

    if(msg.buttons[0] == 1) // Se o gatilho esta pressionado
    {
      joy_pan = msg.axes[0];
      joy_tilt = msg.axes[1];
    }

    move_rate = (msg.axes[2]+1.1)*40; // Le o stick de gas entre o + e -
  }
}; // Fim da classe, agora o main de verdade do problema

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DoJoyPara_o_Motor");

  JoyAndMotor jm;

  return jm.main(argc, argv);
}
