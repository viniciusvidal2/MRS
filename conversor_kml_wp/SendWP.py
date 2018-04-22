# -----------------------------------------------
# IMPORTS
# -----------------------------------------------
# import sys, struct, time, os
# import numpy as np, scipy.io as io
# import math
from time import sleep
#
from pymavlink import mavutil
# from pymavlink import DFReader
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

# from io import FileIO
import re
# import argparse

# -----------------------------------------------
# CONEXAO COM A PLACA
# -----------------------------------------------
device = '/dev/ttyACM1'
baudrate = 115200
master = mavutil.mavlink_connection(device, baud=baudrate)

# Waiting for heartbeat message from the APM board
conexao = master.wait_heartbeat()

# Requesting message transmissions
master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

# Creating one buffer class
f   = fifo()
mav = mavlink1.MAVLink(f)
mav2 = mavlink2.MAVLink(f)

# -----------------------------------------------
# RECEBENDO PARAMETROS
# -----------------------------------------------
# parser = argparse.ArgumentParser(description='Pegar nome do arquivo para converter e a altitude.')
# parser.add_argument('file', help='Arquivo, com caminho a procurar')
# args = parser.parse_args()
#
# format_rem = str(args.file).rsplit('.', 1)[0]
# wp_file = open(format_rem+".waypoint", "r")

wp_file = open("/home/vinicius/Desktop/Arquivo_de_waypoints_conversao/gpsstatus-180222-14627-Tiplan-3.waypoints", "r")

# -----------------------------------------------
# VARRENDO ARQUIVO E ENVIANDO PONTOS
# -----------------------------------------------
if conexao:
    # msg2 = mavlink2.MAVLink_mission_clear_all_message(master.target_system, master.target_component)
    # msg2.pack(mav2)
    # buf = msg2.get_msgbuf()
    # sleep(0.1)
    # master.write(buf)
    msg = mavlink1.MAVLink_mission_clear_all_message(master.target_system, master.target_component)
    msg.pack(mav)
    buf = msg.get_msgbuf()
    sleep(0.1)
    master.write(buf)
    for line in wp_file:
        if 'QGC' in line:
            continue
        else:
            id = re.findall(r"[-+]?\d*", str(line))
            coords = re.findall(r"[-+]?\d*\.\d+|\d+", str(line))
            lat = int(float(coords[8])*10000000)
            lon = int(float(coords[9])*10000000)
            alt = int(float(coords[10]))
            if id[0] == '0': #home aqui, setar home
                # msg2 = mavlink2.MAVLink_set_home_position_message(master.target_system, latitude=lat, longitude=lon,
                #                                                  altitude=alt,
                #                                                  x=0.0, y=0.0, z=0.0, q=(0.0, 0.0, 0.0, 0.0),
                #                                                  approach_x=0.0, approach_y=0.0, approach_z=0.0)
                # msg2.pack(mav2)
                # buf = msg2.get_msgbuf()
                # sleep(0.1)
                # master.write(buf)
                msg = mavlink1.MAVLink_set_home_position_message(master.target_system, latitude=lat, longitude=lon,
                                                                 altitude=alt,
                                                                 x=0.0, y=0.0, z=0.0, q=(0.0, 0.0, 0.0, 0.0),
                                                                 approach_x=0.0, approach_y=0.0, approach_z=0.0)
                msg.pack(mav)
                buf=msg.get_msgbuf()
                sleep(0.1)
                master.write(buf)
            else: # enviar waypoints
                # msg2 = mavlink2.MAVLink_mission_item_message(master.target_system, master.target_component,
                #                                             seq=int(id[0]), current=False, frame=3, command=16,
                #                                             autocontinue=True,
                #                                             param1=0.0, param2=0.0, param3=0.0, param4=0.0,
                #                                             x=float(lat) / 10000000, y=float(lon) / 10000000,
                #                                             z=float(alt))
                # msg2.pack(mav2)
                # buf = msg2.get_msgbuf()
                # sleep(0.1)
                # master.write(buf)
                msg = mavlink1.MAVLink_mission_item_message(master.target_system, master.target_component,
                                                            seq=int(id[0]), current=False, frame=3, command=16,
                                                            autocontinue=True,
                                                            param1=0.0, param2=0.0, param3=0.0, param4=0.0,
                                                            x=float(lat)/10000000, y=float(lon)/10000000, z=float(alt))
                msg.pack(mav)
                buf = msg.get_msgbuf()
                sleep(0.1)
                master.write(buf)