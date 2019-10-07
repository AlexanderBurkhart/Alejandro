#!/usr/bin/env python

#author: Alexander Burkhart
#date: 10/6/18
#version: 1.0

import os
import argparse
import csv
import time
import sys
import glob

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import logging

logging.basicConfig(level=logging.INFO)
bridge = CvBridge()

def main():
	"""Extract a folder of images from a rosbag.
	"""
	parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
	parser.add_argument("bag_file", help="Input ROS bag.")
	parser.add_argument("img_dir", help="Images directory.")
	parser.add_argument("image_topic", help="Image topic.")
	parser.add_argument("ackermann_topic", help="Ackermann topic.")
	
	args = parser.parse_args()
	
	logging.info("Extract images from %s on topic %s into %s" % (args.bag_file,
		                                                      args.image_topic, args.img_dir))
	bag_name = args.bag_file[8:args.bag_file.find('.bag')]
	
	"""
	Passes bag and returns an array that consists of
	image file name, speed, steering, and time
	"""
	with rosbag.Bag(args.bag_file, "r") as bag:
		data = get_data_from(bag, args.img_dir, args.bag_file, bag_name)
		create_csv_from(data, args.img_dir, bag_name+'.csv')

	logging.info("---Successfully created bag data!---")

def get_data_from(bag, img_dir, bag_file, bag_name): 
	logging.debug("in get_data")

	data = []

	imgCounter = 0	

	store_dir = img_dir+bag_name+"/"

	#check if img dir for specific bag exists
	if os.path.exists(store_dir):
		sys.exit('Error: The name of the bag file already exists.') 
	os.makedirs(store_dir)

	last_ackermann = None

	saving = False

	for topic, msg, t in bag.read_messages():
		if topic == "/ackermann_cmd":
			speed = msg.drive.speed
			#logging.debug(msg)
			if speed > 0 and not saving:
				saving = True
				logging.info("Starting to save data at %i..." % imgCounter) 
			if saving:
				last_ackermann = msg
		if topic == "/zed/rgb/image_rect_color":
			imgCounter += 1
			if saving:
				file_name = save_image(msg, imgCounter, bag_name, store_dir)
				row = {}
				row['file_name'] = store_dir+file_name
				row['speed'] = last_ackermann.drive.speed
				row['steering_angle'] = last_ackermann.drive.steering_angle				
				data.append(row)
				logging.info("Saved %s" % file_name)

				logging.debug("counter:{} speed: {} steering_angle:{}".format(imgCounter, 
										last_ackermann.drive.speed, 
										last_ackermann.drive.steering_angle))
		
	return data

def save_image(image, count, bag_name, store_dir):
	file_name = "%06i"%count + bag_name+".png"

	cv_img = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
	cv2.imwrite(os.path.join(store_dir, file_name), cv_img)
	return file_name

def create_csv_from(data, img_dir, file_name='data.csv'):
    #write data to a csv file
	with open(img_dir+file_name, 'w') as csvFile:
		writer = csv.writer(csvFile)
		for row in data:
			writer.writerow([row['file_name'], row['speed'], row['steering_angle']])
	logging.info("Created csv file called %s" % file_name)

if __name__ == '__main__':
	main()
