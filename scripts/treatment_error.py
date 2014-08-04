class TreatmentError(Exception):
	def __init__(self, msg, step):
		self.msg = msg
		self.step = step
	def __str__(self):
		return 'Error: %s. In step: %s' % (self.msg, self.step)
