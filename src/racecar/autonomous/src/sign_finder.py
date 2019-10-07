from keras.models import load_model
from keras import backend as K
from sign_classification.sign_classifier import SignClassifier
import tf
import cv2
import numpy as np

def dice_coef(y_true, y_pred):
	y_true_f = K.flatten(y_true)
	y_pred_f = K.flatten(y_pred)
	intersection = K.sum(y_true_f * y_pred_f)
	return (2. * intersection + SMOOTH) / (K.sum(y_true_f) + K.sum(y_pred_f) + SMOOTH)


def dice_coef_loss(y_true, y_pred):
	return -dice_coef(y_true, y_pred)

class SignFinder(object):
	def __init__(self):
		
		#setting up classifier
		self.sign_classifier = SignClassifier()
		model = load_model('models/classifier.h5')
		resize_width = 32
		resize_height = 64
		self.invalid_state_number = 2
		self.sign_classifier.setup_classifier(model, resize_width, resize_height, self.invalid_state_number)

		#setting up detector
		self.detector_model = load_model('models/detector.h5', custom_objects={'dice_coef_loss': dice_coef_loss, 'dice_coef': dice_coef})
		self.detector_model._make_predict_function()
		self.resize_width = 128
		self.resize_height = 96

		self.resize_height_ratio = 600/float(self.resize_height)
		self.resize_width_ratio = 800/float(self.resize_width)
		self.middle_col = self.resize_width/2
		self.projection_threshold = 2
		self.projection_min = 200

    def extract_img(self, mask, img):
		if(np.max(mask) < self.projection_min):
			return None


		row_projection = np.sum(mask, axis = 1)
		row_index = np.argmax(row_projection)

		if(np.max(row_projection) < self.projection_threshold):
			return None


		zero_row_indexes = np.argwhere(row_projection <= self.projection_threshold)

		#top
		top_part = zero_row_indexes[zero_row_indexes < row_index]
		top = np.max(top_part) if top_part.size > 0 else 0

		#bottom
		bottom_part = zero_row_indexes[zero_row_indexes > row_index]
		bottom = np.min(bottom_part) if bottom_part.size > 0 else self.resize_height


		roi = mask[top:bottom,:]
		column_projection = np.sum(roi, axis=0)

		if(np.max(column_projection) < self.projection_min):
			return None


		non_zero_column_index = np.argwhere(column_projection > self.projection_min)

		index_of_column_index = np.argmin(np.abs(non_zero_column_index - self.middle_col))
		column_index = non_zero_column_index[index_of_column_index][0]

		zero_column_indexes = np.argwhere(column_projection == 0)

		#left
		left_side = zero_column_indexes[zero_column_indexes < column_index]
		left = np.max(left_side) if left_side.size > 0 else 0

		#right
		right_side = zero_column_indexes[zero_column_indexes > column_index]
		right = np.min(right_side) if right_side.size > 0 else self.resize_width

		return img[int(top*self.resize_height_ratio):int(bottom*self.resize_height_ratio), int(left*self.resize_width_ratio):int(right*self.resize_width_ratio)]

	def detect_and_classify_sign(self, img):
		resize_img = cv2.cvtColor(cv2.resize(img, (self.resize_width, self.resize_height)), cv2.COLOR_RGB2GRAY)
		resize_img = resize_img[..., np.newaxis]
	
		img_mask = self.detector_model.predict(resize_img[None, :, :, :], batch_size=1)[0]
		img_mask = (img_mask[:,:,0]*255).astype(np.uint8)

		extract_img = self.extract_img(img_mask, img)
		if extract_img is None:
			return self.invalid_state_number

		state = self.sign_classifier.get_classification(extract_img)

		return state