import sys

class Model(object):

	def _init_(self):
		self.VALID_MODES = ['minimize', 'maximize']
		self.origin = None
		self.dest = None
		self.overhead = None
		self.mode = None
		self.graph_projection = None
		self.algo = None
		self.bbox = None
		self.impedence = None

	#setters    
	def set_origin(self, o):
		self.origin = o

	def set_dest(self, d):
		self.dest = d

	def set_mode(self, mode_str):
		self.mode = mode_str
	
	def set_overhead(self, x):
		self.overhead = x

	def set_algo(self, algo):
		self.algo = algo

	def set_bbox(self, bb):
		self.bbox = bb

	def set_graph_projection(self, graph_projection):
		self.graph_projection = graph_projection

	#getters

	def get_graph_projection(self):
		return self.graph_projection

	def get_origin(self):
		return self.origin

	def get_dest(self):
		return self.dest

	def get_mode(self):
		return self.mode
	
	def get_overhead(self):
		return self.overhead

	def get_algo(self):
		return self.algo

	def get_bbox(self):
		return self.bbox

