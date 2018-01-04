You need to first source the set_puregev_gen file in install.

Then just do

roslaunch flir_gige node.launch <ip_address:=xxx.xxx.xxx.xxx>
<fps:=20>  <raw:=false>
