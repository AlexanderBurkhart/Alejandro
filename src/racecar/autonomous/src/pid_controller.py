#! /usr/bin/env python

import numpy as np

class PID_Controller(object):
	def init(self, Kp, Ki, Kd):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd

		self.p_error = 0
		self.i_error = 0
		self.d_error = 0

		self.WINDOW_SIZE = 20

		self.window = np.zeros(self.WINDOW_SIZE)
		self.i = self.WINDOW_SIZE - 1
		self.sum = 0

	def updateError(self, cte, dt):
		self.d_error = (cte-self.p_error) / dt
		self.p_error = cte
		self.i_error = self.add_i(cte*dt)

	def totalError(self):
		return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error

	def resetError(self):
		self.p_error = 0
		self.i_error = 0
		self.d_error = 0
		self.i = 0
		self.sum = 0
	
	def add_i(self, err):
		self.i = (self.i+1) % self.WINDOW_SIZE
		self.sum = self.sum - self.window[self.i] + err
		return self.sum
