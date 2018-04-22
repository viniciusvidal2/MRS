from io import FileIO
import re
import argparse

# Lendo os argumentos
parser = argparse.ArgumentParser(description='Pegar nome do arquivo para converter e a altitude.')
parser.add_argument('file', help='Arquivo, com caminho todo a procurar')
parser.add_argument('altitude', help='Arquivo, com caminho todo a procurar')
args = parser.parse_args()

# Lendo o arquivo kml passado como parametro no caso depois
kml_file = open(str(args.file), 'r')
altitude = str(args.altitude) # altitude aproximada do local, sendo que nao sabemos se eh necessario para a missao rodar na placa, vinda do usuario

# Criando novo arquivo no modelo .waypoint
format_rem = str(args.file).rsplit('.', 1)[0]
wp_file = open(format_rem+".waypoints", "w+")
wp_file.write("QGC WPL 110\n")
# Comandos constantes
home = "0\t1\t0\t16\t0\t0\t0\t0\t"
wp_common_1 = "\t0\t3\t16\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t"
wp_common_2 = "\t8.000000\t1\n"

# Varrendo arquivo de entrada kml para conseguir copiar um por um os pontos para o .waypoint
count_points = 1
for line in kml_file:
    if 'coordinates' in line:
        lat_lon = re.findall(r"[-+]?\d*\.\d+|\d+", str(line))
        lat = lat_lon[0]
        lon = lat_lon[1]
        if count_points == 1: # escrevendo o Home
            wp_file.write(home+lat+"\t"+lon+"\t"+altitude+"\t1\n")
        wp_file.write(str(count_points)+wp_common_1+lat+"\t"+lon+wp_common_2)
        count_points = count_points + 1

print("Completa conversao!")

wp_file.close()
