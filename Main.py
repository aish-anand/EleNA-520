import sys
from controller import Controller
from model import Model
from View import View
import argparse

class Main(object):
	def __init__(self, algo):
		self.TEST_FLAG = False
		self.algo = algo
		self.run()

	def run(self):
		model_obj = Model() 
		controller_obj = Controller()
		view_obj = View()
		
		controller_obj.set_model(model_obj)
		view_obj.set_controller(controller_obj)
		
		controller_obj.set_view(view_obj)
		
		controller_obj.get_input()
		
		model_obj.set_algo(self.algo)

		controller_obj.get_route()

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Choose an algorithm [1 (Exhaustive Search), 2(Modified Dijkstra), 3(Hybrid)] Note: 1 only minimizes, DEFAULT: 1')
	parser.add_argument('--algorithm', help='an integer for algorithm', default='1')
	args = parser.parse_args()
	option = args.algorithm
	if(option not in ['1', '2', '3']):
		raise ValueError("Please enter algorithm value of 1, 2 or 3")
	model_obj = Main(int(option))

