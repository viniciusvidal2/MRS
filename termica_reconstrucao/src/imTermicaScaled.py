#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Se inscreve no tópico 'thermal/image_raw'
# Cria colorbar e publica imagem no tópico 'thermal/image_scaled'
# Publica imagem em 8bits em escala termica (termica/thermal/image_8bit)


import rospy
import sys, time
import numpy as np
import cv2
import roslib
import matplotlib.pyplot as plt
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mpl_toolkits.axes_grid1 import make_axes_locatable

class imScale():

	def __init__(self):
		self.image_pub = rospy.Publisher("termica/thermal/image_scaled", Image, queue_size = 10)
                self.image_pub_8bit = rospy.Publisher("termica/thermal/image_8bit", Image, queue_size = 10)
                self.subscriber = rospy.Subscriber("termica/thermal/image_raw", Image, self.imCallback,  queue_size = 1)
		self.bridge = CvBridge()


        def imCallback(self, img_msg):
            ## msg -> opencv
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")


            ## Fator de ganho da camera termica
            #K = 0.04   #high resolution mode
            K = 0.4   #low resolution mode


            ## Escolhendo as temperaturas minimas e maximas a serem detectadas
            tempMin = 0  ;
            tempMax = 100;


            ## Escolhendo resolucao de temperatura
            resolucao = 5;


            ## Plotando imagem termica com escala fixa (tempMin --> tempMax)
            fig, ax = plt.subplots()
            fig.subplots_adjust(0,0,1,1)
            data = cv_image
            dataMin = (tempMin + 273.15)/K;
            dataMax = (tempMax + 273.15)/K;
            dataStep = resolucao/K;

            cax = ax.imshow(data, interpolation = 'none', cmap = 'jet_r', vmin = dataMin, vmax = dataMax)
            cbar = fig.colorbar(cax, ticks = np.arange(0, dataMax + dataStep, dataStep), orientation='vertical')
            cbar.ax.set_yticklabels(['{:.0f}'.format(x*K - 273.15) for x in np.arange(dataMin, dataMax+dataStep, dataStep)])
            ax.yaxis.set_major_locator(plt.NullLocator())
            ax.xaxis.set_major_locator(plt.NullLocator())
            fig.canvas.draw()

            ## Publicando imagem com escala acoplada
            data2 = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
            data2 = data2.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            self.image_pub.publish ( self.bridge.cv2_to_imgmsg(data2, "bgr8") )


            ## Publicando imagem termica pura
            data = cv_image
            m_dpi = 165;
            fig_raw, ax_raw = plt.subplots(figsize=(640.0/float(m_dpi) , 512.0/float(m_dpi)), dpi = m_dpi )
            fig_raw.subplots_adjust(0,0,1,1)
            plt.margins(0,0)
            ax_raw.set_axis_off()
            ax_raw.yaxis.set_major_locator(plt.NullLocator())
            ax_raw.xaxis.set_major_locator(plt.NullLocator())
            cax_raw = ax_raw.imshow(cv_image, interpolation = 'nearest', cmap = 'jet_r', aspect = 'auto', vmin = dataMin, vmax = dataMax)
            fig_raw.canvas.draw()
            data_raw = np.fromstring(fig_raw.canvas.tostring_rgb(), dtype=np.uint8, sep='')
            data_raw = data_raw.reshape(fig_raw.canvas.get_width_height()[::-1] + (3,))

            msg_im_out = self.bridge.cv2_to_imgmsg(data_raw, "bgr8");
            h_in = img_msg.header.stamp;
            h_out = h_in;
            self.image_pub_8bit.publish(msg_im_out)
            pass

def main(args):
	imS = imScale()
	rospy.init_node('escala' , anonymous = True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Terminando no"
	cv2.destroyAllWindows()


if __name__ == '__main__':
        print "Escala na imagem Termica ON"
	main(sys.argv)
