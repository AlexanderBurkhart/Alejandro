import cv2
import glob
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
import h5py

from keras.backend.tensorflow_backend import set_session
from keras.models import Sequential
from keras.layers import Conv2D, Flatten, Dense, MaxPooling2D, Dropout
from keras.utils.np_utils import to_categorical
from keras import losses, optimizers, regularizers

def _import(x_train, x_label, labels):
	print('Importing training data...')
	for img_class, directory in enumerate(['stop', 'yield', 'none']):
		for i, file_name in enumerate(glob.glob("{}/*.jpg".format(directory))):
			
			file = cv2.imread(file_name)

			print('Importing: {}'.format(file_name))	
	
			resized = cv2.resize(file, (32, 64))

			x_train.append(resized/255.)
			x_label.append(img_class)

			file = cv2.cvtColor(file, cv2.COLOR_BGR2RGB)
			blur = cv2.GaussianBlur(file, (7, 7), 0)

			resized = cv2.resize(blur, (32, 64))

			x_train.append(resized/255.)
			x_label.append(img_class)

	x_train = np.array(x_train)
	x_label = np.array(x_label)
	labels = to_categorical(x_label)

	print('Done importing')
	print()

	return x_train, x_label, labels

def implement_model():
	print('Implementing model...')
	print('-')

	num_classes = 4	

	model = Sequential()	

	model.add(Conv2D(32, (3, 3), input_shape=(64, 32, 3), padding='same', activation='relu', kernel_initializer='random_uniform', kernel_regularizer=regularizers.l2(0.01)))
	model.add(MaxPooling2D(2,2))
	
	Dropout(0.8)
	model.add(Conv2D(32, (3, 3), padding='same', activation='relu', kernel_initializer='random_uniform', kernel_regularizer=regularizers.l2(0.01)))
	model.add(MaxPooling2D(2,2))
	
	Dropout(0.8)
	model.add(Flatten())

	model.add(Dense(8, activation='relu', kernel_initializer='random_uniform', kernel_regularizer=regularizers.l2(0.01)))
	model.add(Dense(num_classes, activation='softmax'))
	
	loss = losses.categorical_crossentropy
	optimizer = optimizers.Adam()
	model.compile(loss=loss, optimizer=optimizer, metrics=['accuracy'])

	print('Done implementing')
	print()

	return model

def train(model, x_train, labels):
	print('Training model..')
	print('-')

	model.fit(x_train, labels, batch_size=64, epochs=15, verbose=True, validation_split=0.1, shuffle=True)
	
	print('Done training')

	print("Score: {}".format(model.evaluate(x_train, categorical_labels, verbose=0)))
	model.save('classifier.h5')

if __name__ == '__main__':
	
	model = None

	X_train = []
	X_label = []
	categorical_labels = None	

	X_train, X_label, categorical_labels = _import(X_train, X_label, categorical_labels)
	
	model = implement_model()

	train(model, X_train, categorical_labels)
