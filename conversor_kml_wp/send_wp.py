# -----------------------------------------------
# IMPORTS
# -----------------------------------------------
from dronekit import connect, Command, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import pymavlink
# import MAVProxy

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
connection_string = '/dev/ttyACM0'
baudrate = 115200

print 'Connecting to vehicle on: %s' % connection_string

vehicle = connect(connection_string, wait_ready=True, baud=baudrate)


# -----------------------------------------------
# RECEBENDO PARAMETROS
# -----------------------------------------------
parser = argparse.ArgumentParser(description='Pegar nome do arquivo para converter e a altitude.')
parser.add_argument('file', help='Arquivo, com caminho a procurar')
args = parser.parse_args()

format_rem = str(args.file).rsplit('.', 1)[0]
wp_file = open(format_rem+".waypoints", "r")

# wp_file = open("/home/vinicius/Desktop/Arquivo_de_waypoints_conversao/gpsstatus-180222-214627-Tiplan-3.waypoints", "r")

# -----------------------------------------------
# VARRENDO ARQUIVO E ENVIANDO PONTOS
# -----------------------------------------------
# Baixando comandos
cmds = vehicle.commands
cmds.wait_ready()

cmds = vehicle.commands
cmds.clear()

if vehicle.last_heartbeat:
    for line in wp_file:
        if 'QGC' in line:
            continue
        else:
            id = re.findall(r"[-+]?\d*", str(line))
            coords = re.findall(r"[-+]?\d*\.\d+|\d+", str(line))
            lat = float(coords[8])
            lon = float(coords[9])
            alt = float(coords[10])
            if id[0] == '0': #home aqui, setar home
                home = LocationGlobal(lat, lon, alt)
                vehicle.home_location = home
            else: # enviar waypoints
                wp = Command(0,
                              0,
                              0,
                              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                              0, 0, 0, 0, 0, 0,
                              lat, lon, alt)
                cmds.add(wp)
print "Enviando %d pontos para o veiculo..." % len(cmds)
cmds.upload()