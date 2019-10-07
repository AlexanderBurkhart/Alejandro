import cv2
import tensorflow as tf
import numpy as np

class SignClassifier(object):
    def __init__(self):
		self.model = None
		self.width = 0
		self.height = 0
		self.channels = 3

	def setup_classifier(self, model, width, height, invalid_state_number, channels=3):
		self.width = width
		self.height = height
		self.model = model
		self.channels = channels
		self.invalid_state_number = invalid_state_number

		self.graph = tf.get_default_graph()

	def get_classification(self, image):
		resized = cv2.resize(image, (self.width, self.height))
		resized = resized / 255

		with self.graph.as_default():
			predictions = self.model.predict(resized.reshape((1, self.height, self.width, self.channels)))
			sign_state = predictions[0].tolist().index(np.max(predictions[0]))
			return sign_state

	    return self.invalid_state_number