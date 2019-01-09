#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Se inscreve no tópico 'thermal/image_raw'
# Criar colorbar e publica imagem no tópico 'thermal/image_scaled


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
		self.subscriber = rospy.Subscriber("termica/thermal/image_raw", Image, self.imageCallback,  queue_size = 1)
		self.bridge = CvBridge()


	def imageCallback(self, data):
		#print "callback"

		# msg to opencv
                #cv_image = self.bridge.imgmsg_to_cv2(data, 'mono16')
                cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
                #cv_image = cv2.cvtColor(cv_image_cl, cv2.COLOR_BGR2GRAY)
                #temp_0 = cv_image[100, 100]*0.04 - 273.15;
                #print temp_0
                print cv_image[50, 300]*0.04-273.15
                print '---'

		# colorbar
		fig, ax = plt.subplots()
		data = cv_image
                dataMax = cv_image.max()
                dataMin = cv_image.min()
                print dataMax
                print dataMin
                dataStep = 12.5 # (0.5)/0.04  -- resolucao de 0.5 grau
		cax = ax.imshow(data, interpolation = 'nearest', cmap = 'jet_r')
                cbar = fig.colorbar(cax, ticks = np.arange(0, dataMax + dataStep, dataStep), orientation='vertical')
                cbar.ax.set_yticklabels(['{:.0f}'.format(x*0.04-273.15) for x in np.arange(dataMin, dataMax+dataStep, dataStep)])
                ax.yaxis.set_major_locator(plt.NullLocator())
                ax.xaxis.set_major_locator(plt.NullLocator())
                fig.canvas.draw()

		# fig to numpy
		data2 = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
		data2 = data2.reshape(fig.canvas.get_width_height()[::-1] + (3,))
		
		# publish image
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
                msg.data = np.array(cv2.imencode('.jpeg', data2)[1]).tostring()
	     
		# Publish new image
		self.image_pub.publish ( self.bridge.cv2_to_imgmsg(data2, "bgr8") ) 


############################################################################################################################################
############################################################################################################################################


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
