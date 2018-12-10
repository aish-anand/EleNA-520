import sys
from controller import Controller
from model import Model
from View import View
import argparse

class Main(object):

	def __init__(self, algo):
		self.algo = algo
		self.run()

	def run(self):
		model_object = Model()
		controller_object = Controller()
		view_object = View()
		controller_object.set_model(model_object)
		view_object.set_controller(controller_object)
		controller_object.set_view(view_object)
		#calling view to get user input
		controller_object.get_input()
		#setting algorithm option
		model_object.set_algo(self.algo)
		#calling function to get optimal route
		controller_object.get_route()

if _name_ == '_main_':
	parser = argparse.ArgumentParser(description='Choose an algorithm [1 (Exhaustive Search), 2(Modified Dijkstra), 3(Hybrid)] Note: 1 only minimizes, DEFAULT: 1')
	parser.add_argument('--algorithm', help='an integer for algorithm', default='1')
	args = parser.parse_args()
	option = args.algorithm
	if(option not in ['1', '2', '3']):
		raise ValueError("Please enter algorithm value of 1, 2 or 3")
	model_obj = Main(int(option))