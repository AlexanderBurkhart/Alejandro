import numpy as np
import argparse
import os
import glob

from skimage.io import imsave, imread
from skimage import color
from skimage.filters import gaussian

from keras.callbacks import ModelCheckpoint

from model import preprocess, get_model

img_rows = 600
img_cols = 800

def train_and_predict():
	print('Loading and processing data...')
	print('-')

	imgs_train, imgs_train_mask = create_and_load_train_data()

	imgs_train = preprocess(imgs_train)
	imgs_train_mask = preprocess(imgs_train_mask)

	imgs_train = imgs_train.astype('float32')

	imgs_train_mask = imgs_train_mask.astype('float32')
	imgs_train_mask /= 255

	print('Creating model...')
	print('-')

	model = get_model()

	print('Training model...')
	print('-')
	
	model.fit(imgs_train, imgs_train_mask, batch_size=16, epochs=50, verbose=1, shuffle=True, validation_split=0.2)
	
	print('Saving model as detector_.h5...')	
	print('-')

	model.save('detector_.h5')

	print('Done saving...')

def brightness(img):
	hsv = color.rgb2hsv(img)
	value = hsv[:,:,2]
	hsv[:,:,2] *= value

	return (color.hsv2rgb(hsv)*255).astype('uint8')

def get_image_and_mask(image_name):
	image_mask_path = image_name.split('.')[0] + '_mask.jpg'
	img = imread(image_name)
	img_mask = np.array([])
	
	if os.path.exists(image_mask_path):
		img_mask = imread(image_mask_path)
#	else:
#		print('COULD NOT FIND MASK IMAGE')

	return np.array([img]), np.array([img_mask])

def get_total_images(path):
	total = 0
	for folder in os.listdir(path):
		path_subfolder = os.path.join(path, folder, '*.jpg')
		images = glob.glob(path_subfolder)
		total += len(images)
	return total

def motion_blur(img, size=3):
	kernel_motion_blur = np.zeros((size, size))
	kernel_motion_blur[int((size-1)/2), :] = np.ones(size)
	kernel_motion_blur = kernel_motion_blur / size

	return (gaussian(img, sigma=3, multichannel=True)*255).astype('uint8')

def create_and_load_train_data():
	train_path = 'data/train'

	inc = 4

	total = get_total_images(train_path)*inc//2

	imgs = np.ndarray((total, img_rows, img_cols, 3), dtype=np.uint8)
	imgs_mask = np.ndarray((total, img_rows, img_cols, 3), dtype=np.uint8)

	print('Creating training images...')
	print('-')

	i = 0
	for subfolder in os.listdir(train_path):
		train_path_subfolder = os.path.join(train_path, subfolder, '*.jpg')
		images = glob.glob(train_path_subfolder)
		for path in images:
			
			if 'mask' in path.split('\\')[-1]:
				continue

			imgs[i], imgs_mask[i] = get_image_and_mask(path)

			imgs[i+1] = brightness(imgs[i])
			imgs_mask[i+1] = imgs_mask[i]

			imgs[i+2] = motion_blur(imgs[i])
			imgs_mask[i+2] = imgs_mask[i]

			imgs[i+3] = motion_blur(imgs[i+1])
			imgs_mask[i+3] = imgs_mask[i]

			print('Loading...')
			i += inc			

	print('Loading done.')
	print('-')	

	print('Returning data...')
	return imgs, imgs_mask

if __name__ == '__main__':
	train_and_predict()